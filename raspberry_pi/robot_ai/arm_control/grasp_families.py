"""Offline grasp-family routing around demonstrated endpoints.

One manually taught cola contact is useful for a small local family of similar
cylinders.  It is not a universal grasp policy.  This module makes that
boundary explicit: only an object within a deliberately conservative measured
envelope can reuse the taught side-route planner.  Cups, pens, flat objects,
and tissue are returned as separate families that need their own demonstration
and gripper measurements.

No UART, CAN, ROS, camera, gripper, or actuator API is imported here.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any, Sequence

import numpy as np

from .model import ArmModel
from .taught_grasp import TaughtGraspPlan, TaughtGraspPlanningError, build_taught_side_grasp_plan


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_REGISTRY_PATH = Path(__file__).resolve().parent / "config" / "grasp_family_registry.json"


class GraspFamilyRegistryError(ValueError):
    """The class-to-family contract is malformed or incomplete."""


@dataclass(frozen=True)
class GraspFamilyPreview:
    """A non-motion result describing whether a demonstrated route is eligible."""

    object_class: str
    family: str
    strategy: str
    status: str
    reason: str
    required_measurements: tuple[str, ...]
    plan: TaughtGraspPlan | None = None

    @property
    def preview_ready(self) -> bool:
        return self.status == "preview_ready" and self.plan is not None

    def as_dict(self) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "schema": "xiaou_grasp_family_preview_v1",
            "offline": True,
            "hardware_motion": False,
            "object_class": self.object_class,
            "family": self.family,
            "strategy": self.strategy,
            "status": self.status,
            "reason": self.reason,
            "required_measurements": list(self.required_measurements),
            "preview_ready": self.preview_ready,
        }
        if self.plan is not None:
            payload["plan"] = self.plan.as_dict()
        return payload


def _finite_positive(value: float | None, label: str) -> float:
    if value is None:
        raise ValueError(f"{label} is required")
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{label} must be finite and positive")
    return result


def _finite_vector(values: Sequence[float], size: int, label: str) -> np.ndarray:
    result = np.asarray(values, dtype=np.float64).reshape(-1)
    if result.size != size or not np.isfinite(result).all():
        raise ValueError(f"{label} must contain {size} finite values")
    return result


def _strings(values: Any, label: str) -> tuple[str, ...]:
    if not isinstance(values, list) or not all(isinstance(value, str) and value for value in values):
        raise GraspFamilyRegistryError(f"{label} must be a non-empty string list")
    return tuple(values)


def load_grasp_family_registry(path: str | Path = DEFAULT_REGISTRY_PATH) -> dict[str, Any]:
    """Load a small, versioned class-to-grasp-family contract."""

    registry_path = Path(path)
    try:
        payload = json.loads(registry_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise GraspFamilyRegistryError(f"cannot load grasp family registry: {registry_path}") from exc
    if not isinstance(payload, dict) or payload.get("schema_version") != 1:
        raise GraspFamilyRegistryError("grasp family registry schema_version must be 1")
    families = payload.get("families")
    classes = payload.get("classes")
    aliases = payload.get("aliases", {})
    if not isinstance(families, dict) or not isinstance(classes, dict) or not isinstance(aliases, dict):
        raise GraspFamilyRegistryError("grasp family registry requires families, classes, and aliases mappings")
    for name, family in families.items():
        if not isinstance(name, str) or not isinstance(family, dict):
            raise GraspFamilyRegistryError("family entries must be mappings")
        if not isinstance(family.get("strategy"), str) or family.get("route_status") not in {
            "preview_ready_if_local", "requires_dedicated_teach", "reject"
        }:
            raise GraspFamilyRegistryError(f"invalid family declaration: {name}")
        _strings(family.get("required_measurements"), f"families.{name}.required_measurements")
    for name, item in classes.items():
        if not isinstance(name, str) or not isinstance(item, dict) or item.get("family") not in families:
            raise GraspFamilyRegistryError(f"invalid class declaration: {name}")
    if not all(isinstance(name, str) and isinstance(target, str) and target in classes for name, target in aliases.items()):
        raise GraspFamilyRegistryError("aliases must point to declared classes")
    return payload


def _normalized_class(object_class: str, registry: dict[str, Any]) -> str:
    if not isinstance(object_class, str) or not object_class.strip():
        raise ValueError("object_class must be a non-empty string")
    normalized = object_class.strip().lower()
    aliases = registry["aliases"]
    return str(aliases.get(normalized, normalized))


def _family_preview(
    object_class: str,
    family_name: str,
    family: dict[str, Any],
    *,
    status: str,
    reason: str,
) -> GraspFamilyPreview:
    return GraspFamilyPreview(
        object_class=object_class,
        family=family_name,
        strategy=str(family["strategy"]),
        status=status,
        reason=reason,
        required_measurements=_strings(family["required_measurements"], f"families.{family_name}.required_measurements"),
    )


def build_grasp_family_preview(
    object_class: str,
    start_deg: Sequence[float],
    taught_deg: Sequence[float],
    *,
    model: ArmModel,
    lower_deg: Sequence[float],
    upper_deg: Sequence[float],
    object_radius_m: float | None = None,
    object_height_m: float | None = None,
    target_offset_base_m: Sequence[float] = (0.0, 0.0, 0.0),
    registry_path: str | Path = DEFAULT_REGISTRY_PATH,
) -> GraspFamilyPreview:
    """Return a no-I/O plan only when the current taught route is truly local.

    ``target_offset_base_m`` is a base-frame object-center displacement from
    the manually taught cola contact.  It is bounded by the all-corner offline
    characterization stored in the registry, not by a single-axis numerical
    failure edge. It is not a substitute for camera calibration or a new teach
    pose.
    """

    registry = load_grasp_family_registry(registry_path)
    normalized = _normalized_class(object_class, registry)
    class_entry = registry["classes"].get(normalized)
    if not isinstance(class_entry, dict):
        return GraspFamilyPreview(
            object_class=normalized,
            family="unknown",
            strategy="reject",
            status="rejected",
            reason="unknown object class has no demonstrated grasp family",
            required_measurements=("class_schema_update", "dedicated_grasp_demonstration"),
        )
    family_name = str(class_entry["family"])
    family = registry["families"][family_name]
    route_status = str(family["route_status"])
    if route_status == "reject":
        return _family_preview(
            normalized,
            family_name,
            family,
            status="rejected",
            reason="this object family is flexible or geometrically unmodeled; do not reuse a rigid-object route",
        )
    if route_status == "requires_dedicated_teach":
        return _family_preview(
            normalized,
            family_name,
            family,
            status="requires_dedicated_teach",
            reason="the current cola side-contact demonstration does not define this grasp family",
        )

    reference = family.get("reference_object")
    tolerance = family.get("local_dimension_tolerance_m")
    envelope = family.get("target_offset_envelope_m")
    if not isinstance(reference, dict) or not isinstance(tolerance, dict) or not isinstance(envelope, dict):
        raise GraspFamilyRegistryError(f"{family_name} is missing local taught-route bounds")
    reference_radius = _finite_positive(reference.get("radius_m"), "reference radius_m")
    reference_height = _finite_positive(reference.get("height_m"), "reference height_m")
    radius = reference_radius if object_radius_m is None and class_entry.get("reference_dimensions_allowed") else _finite_positive(object_radius_m, "object_radius_m")
    height = reference_height if object_height_m is None and class_entry.get("reference_dimensions_allowed") else _finite_positive(object_height_m, "object_height_m")
    radius_tolerance = _finite_positive(tolerance.get("radius"), "local radius tolerance")
    height_tolerance = _finite_positive(tolerance.get("height"), "local height tolerance")
    if abs(radius - reference_radius) > radius_tolerance or abs(height - reference_height) > height_tolerance:
        return _family_preview(
            normalized,
            family_name,
            family,
            status="requires_dedicated_teach",
            reason=(
                "object dimensions exceed the local cola demonstration envelope; "
                "capture a dedicated contact pose instead of retargeting this route"
            ),
        )
    offset = _finite_vector(target_offset_base_m, 3, "target_offset_base_m")
    lower_offset = _finite_vector(envelope.get("min") or (), 3, "target_offset_envelope_m.min")
    upper_offset = _finite_vector(envelope.get("max") or (), 3, "target_offset_envelope_m.max")
    if np.any(lower_offset >= upper_offset):
        raise GraspFamilyRegistryError("target offset envelope must have min < max")
    if np.any(offset < lower_offset) or np.any(offset > upper_offset):
        return _family_preview(
            normalized,
            family_name,
            family,
            status="requires_dedicated_teach",
            reason="target offset exceeds the conservative local taught-route envelope",
        )
    try:
        plan = build_taught_side_grasp_plan(
            start_deg,
            taught_deg,
            model=model,
            lower_deg=lower_deg,
            upper_deg=upper_deg,
            object_radius_m=radius,
            object_height_m=height,
            taught_contact_offset_base_m=offset,
        )
    except TaughtGraspPlanningError as exc:
        return _family_preview(
            normalized,
            family_name,
            family,
            status="requires_dedicated_teach",
            reason=f"local retargeting is not reachable in the current effective limits: {exc}",
        )
    return GraspFamilyPreview(
        object_class=normalized,
        family=family_name,
        strategy=str(family["strategy"]),
        status="preview_ready",
        reason="within the conservative local taught-cylinder envelope; planning preview only",
        required_measurements=_strings(family["required_measurements"], f"families.{family_name}.required_measurements"),
        plan=plan,
    )


__all__ = [
    "DEFAULT_REGISTRY_PATH",
    "GraspFamilyPreview",
    "GraspFamilyRegistryError",
    "build_grasp_family_preview",
    "load_grasp_family_registry",
]
