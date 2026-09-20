"""Versioned, no-I/O registry for extending taught grasp routes.

The current arm has one verified offline route: a very local cola-like side
grasp.  Future cup, pen, flat-object, and tissue routes must be captured as
separate data rather than inferred from that cola endpoint.  This module
validates the data contract on either the desktop or Pi without importing
UART, CAN, ROS, a camera, or a gripper driver.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np


DEFAULT_TEACH_REGISTRY_PATH = Path(__file__).resolve().parent / "config" / "object_teach_registry.json"
JOINT_COUNT = 6


class TeachRegistryError(ValueError):
    """The object-to-demonstration contract is incomplete or malformed."""


def _finite_vector(value: object, size: int, label: str, *, positive: bool = False) -> tuple[float, ...]:
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise TeachRegistryError(f"{label} must contain {size} values")
    try:
        values = tuple(float(item) for item in value)
    except (TypeError, ValueError) as exc:
        raise TeachRegistryError(f"{label} must contain numbers") from exc
    if not all(math.isfinite(item) for item in values):
        raise TeachRegistryError(f"{label} must contain finite values")
    if positive and any(item <= 0.0 for item in values):
        raise TeachRegistryError(f"{label} must contain positive values")
    return values


def _positive(value: object, label: str) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError) as exc:
        raise TeachRegistryError(f"{label} must be a finite positive number") from exc
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise TeachRegistryError(f"{label} must be a finite positive number")
    return parsed


def _strings(value: object, label: str) -> tuple[str, ...]:
    if not isinstance(value, list) or not value or not all(isinstance(item, str) and item.strip() for item in value):
        raise TeachRegistryError(f"{label} must be a non-empty string list")
    return tuple(item.strip() for item in value)


@dataclass(frozen=True)
class ObjectTeachSpec:
    """Validated declaration of one object family's future teach requirements."""

    object_class: str
    family: str
    strategy: str
    route_status: str
    required_pose_ids: tuple[str, ...]
    required_measurements: tuple[str, ...]
    gripper_measurements_required: tuple[str, ...]
    simulation_model: Mapping[str, object]

    @property
    def ready_for_real_teach(self) -> bool:
        return self.route_status == "requires_dedicated_teach"

    def as_dict(self) -> dict[str, Any]:
        return {
            "object_class": self.object_class,
            "family": self.family,
            "strategy": self.strategy,
            "route_status": self.route_status,
            "required_pose_ids": list(self.required_pose_ids),
            "required_measurements": list(self.required_measurements),
            "gripper_measurements_required": list(self.gripper_measurements_required),
            "simulation_model": dict(self.simulation_model),
            "ready_for_real_teach": self.ready_for_real_teach,
        }


def load_object_teach_registry(path: str | Path = DEFAULT_TEACH_REGISTRY_PATH) -> dict[str, Any]:
    """Load and structurally validate the registry without touching hardware."""

    registry_path = Path(path)
    try:
        payload = json.loads(registry_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise TeachRegistryError(f"cannot load object teach registry: {registry_path}") from exc
    if not isinstance(payload, dict) or payload.get("schema_version") != 1:
        raise TeachRegistryError("object teach registry schema_version must be 1")
    if payload.get("hardware_motion") is not False:
        raise TeachRegistryError("object teach registry must remain no-motion")
    objects = payload.get("objects")
    if not isinstance(objects, dict) or not objects:
        raise TeachRegistryError("object teach registry requires objects")
    for object_class, item in objects.items():
        if not isinstance(object_class, str) or not isinstance(item, dict):
            raise TeachRegistryError("object entries must be mappings")
        if item.get("route_status") not in {"current_local_preview", "requires_dedicated_teach", "reject"}:
            raise TeachRegistryError(f"{object_class}.route_status is invalid")
        if not isinstance(item.get("family"), str) or not isinstance(item.get("strategy"), str):
            raise TeachRegistryError(f"{object_class} requires family and strategy")
        _strings(item.get("required_pose_ids"), f"{object_class}.required_pose_ids")
        _strings(item.get("required_measurements"), f"{object_class}.required_measurements")
        _strings(item.get("gripper_measurements_required"), f"{object_class}.gripper_measurements_required")
        simulation = item.get("simulation_model")
        if not isinstance(simulation, dict):
            raise TeachRegistryError(f"{object_class}.simulation_model must be a mapping")
        if simulation.get("shape") not in {"cylinder", "capsule", "box", "sheet_stack"}:
            raise TeachRegistryError(f"{object_class}.simulation_model.shape is invalid")
        _positive(simulation.get("radius_m"), f"{object_class}.simulation_model.radius_m")
        _positive(simulation.get("height_m"), f"{object_class}.simulation_model.height_m")
        _positive(simulation.get("mass_kg"), f"{object_class}.simulation_model.mass_kg")
    return payload


def resolve_object_teach_spec(
    object_class: str,
    path: str | Path = DEFAULT_TEACH_REGISTRY_PATH,
) -> ObjectTeachSpec:
    """Resolve a known class to its no-I/O teach specification."""

    if not isinstance(object_class, str) or not object_class.strip():
        raise TeachRegistryError("object_class must be a non-empty string")
    payload = load_object_teach_registry(path)
    normalized = object_class.strip().lower()
    aliases = payload.get("aliases", {})
    if not isinstance(aliases, dict):
        raise TeachRegistryError("aliases must be a mapping")
    normalized = str(aliases.get(normalized, normalized))
    item = payload["objects"].get(normalized)
    if not isinstance(item, dict):
        raise TeachRegistryError(f"unknown object class: {normalized}")
    return ObjectTeachSpec(
        object_class=normalized,
        family=str(item["family"]),
        strategy=str(item["strategy"]),
        route_status=str(item["route_status"]),
        required_pose_ids=_strings(item["required_pose_ids"], f"{normalized}.required_pose_ids"),
        required_measurements=_strings(item["required_measurements"], f"{normalized}.required_measurements"),
        gripper_measurements_required=_strings(
            item["gripper_measurements_required"], f"{normalized}.gripper_measurements_required"
        ),
        simulation_model=dict(item["simulation_model"]),
    )


def validate_taught_object_record(
    record: Mapping[str, object],
    *,
    registry_path: str | Path = DEFAULT_TEACH_REGISTRY_PATH,
) -> dict[str, Any]:
    """Validate a future manually captured multi-pose demonstration record.

    A record may be produced by a future read-only teaching command, but this
    validator deliberately does not capture anything.  It prevents a later
    planner from accepting one six-axis pose as a universal grasp definition.
    """

    if not isinstance(record, Mapping) or record.get("schema") != "xiaou_object_teach_record_v1":
        raise TeachRegistryError("teach record schema must be xiaou_object_teach_record_v1")
    if record.get("read_only_capture") is not True or record.get("hardware_motion") is not False:
        raise TeachRegistryError("teach record must originate from read-only capture")
    spec = resolve_object_teach_spec(str(record.get("object_class") or ""), registry_path)
    if spec.route_status == "reject":
        raise TeachRegistryError(f"{spec.object_class} is intentionally not a taught rigid-object route")
    poses = record.get("poses_deg")
    if not isinstance(poses, Mapping):
        raise TeachRegistryError("teach record poses_deg must be a mapping")
    missing_pose_ids = [pose_id for pose_id in spec.required_pose_ids if pose_id not in poses]
    if missing_pose_ids:
        raise TeachRegistryError("teach record lacks required poses: " + ", ".join(missing_pose_ids))
    validated_poses = {pose_id: list(_finite_vector(poses[pose_id], JOINT_COUNT, f"poses_deg.{pose_id}")) for pose_id in spec.required_pose_ids}
    measurements = record.get("measurements")
    if not isinstance(measurements, Mapping):
        raise TeachRegistryError("teach record measurements must be a mapping")
    missing_measurements = [key for key in spec.required_measurements if key not in measurements]
    if missing_measurements:
        raise TeachRegistryError("teach record lacks measurements: " + ", ".join(missing_measurements))
    gripper = record.get("gripper")
    if not isinstance(gripper, Mapping):
        raise TeachRegistryError("teach record gripper must be a mapping")
    missing_gripper = [key for key in spec.gripper_measurements_required if key not in gripper]
    if missing_gripper:
        raise TeachRegistryError("teach record lacks gripper measurements: " + ", ".join(missing_gripper))
    for key in (*spec.required_measurements, *spec.gripper_measurements_required):
        container = measurements if key in measurements else gripper
        value = container[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
            raise TeachRegistryError(f"{key} must be finite")
    return {
        "schema": "xiaou_object_teach_validation_v1",
        "offline": True,
        "hardware_motion": False,
        "object": spec.as_dict(),
        "pose_count": len(validated_poses),
        "poses_deg": validated_poses,
        "validated": True,
    }


__all__ = [
    "DEFAULT_TEACH_REGISTRY_PATH",
    "ObjectTeachSpec",
    "TeachRegistryError",
    "load_object_teach_registry",
    "resolve_object_teach_spec",
    "validate_taught_object_record",
]
