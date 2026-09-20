"""Conservative TCP swept-path clearance checks for offline diagnostics.

This module deliberately validates only the tool-center path against a table
plane and explicitly configured keep-out boxes. It is useful before a ROS 2 /
MoveIt PlanningScene run, but it is not a substitute for full link geometry,
self-collision, or real-machine collision validation.
"""

from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass
import json
import math
from pathlib import Path

import numpy as np


DEFAULT_DIAGNOSTIC_SCENE_PATH = Path(__file__).resolve().parent / "config" / "offline_diagnostic_scene.json"


def _as_point(value: object, label: str) -> np.ndarray:
    point = np.asarray(value, dtype=np.float64).reshape(-1)
    if point.size != 3 or not np.isfinite(point).all():
        raise ValueError(f"{label} must be a finite 3D point")
    return point


def _as_finite_number(value: object, label: str, *, positive: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{label} must be numeric")
    number = float(value)
    if not math.isfinite(number) or (positive and number <= 0.0):
        qualifier = "positive finite" if positive else "finite"
        raise ValueError(f"{label} must be {qualifier}")
    return number


@dataclass(frozen=True)
class AxisAlignedBox:
    name: str
    minimum_m: np.ndarray
    maximum_m: np.ndarray

    def __post_init__(self) -> None:
        if not self.name.strip():
            raise ValueError("keep-out box name must not be empty")
        minimum = _as_point(self.minimum_m, f"{self.name}.minimum_m")
        maximum = _as_point(self.maximum_m, f"{self.name}.maximum_m")
        if np.any(minimum >= maximum):
            raise ValueError(f"{self.name} minimum_m must be below maximum_m on every axis")
        object.__setattr__(self, "minimum_m", minimum)
        object.__setattr__(self, "maximum_m", maximum)

    def clearance_to_point(self, point_m: np.ndarray, safety_radius_m: float) -> float:
        point = _as_point(point_m, "TCP point")
        outside_delta = np.maximum(np.maximum(self.minimum_m - point, 0.0), point - self.maximum_m)
        return float(np.linalg.norm(outside_delta) - safety_radius_m)


@dataclass(frozen=True)
class DiagnosticScene:
    frame: str
    table_z_m: float
    tcp_safety_radius_m: float
    minimum_table_clearance_m: float
    minimum_obstacle_clearance_m: float
    keep_out_boxes: tuple[AxisAlignedBox, ...]

    def __post_init__(self) -> None:
        if self.frame != "base_link":
            raise ValueError("offline diagnostic scene must use the base_link frame")
        for label, value, positive in (
            ("table_z_m", self.table_z_m, False),
            ("tcp_safety_radius_m", self.tcp_safety_radius_m, True),
            ("minimum_table_clearance_m", self.minimum_table_clearance_m, False),
            ("minimum_obstacle_clearance_m", self.minimum_obstacle_clearance_m, False),
        ):
            number = _as_finite_number(value, label, positive=positive)
            if not positive and number < 0.0:
                raise ValueError(f"{label} must be non-negative")


@dataclass(frozen=True)
class ClearanceReview:
    safe: bool
    sample_count: int
    minimum_table_clearance_m: float
    minimum_obstacle_clearance_m: float | None
    violations: tuple[str, ...]

    def to_dict(self) -> dict[str, object]:
        return {
            "safe": self.safe,
            "sample_count": self.sample_count,
            "minimum_table_clearance_m": self.minimum_table_clearance_m,
            "minimum_obstacle_clearance_m": self.minimum_obstacle_clearance_m,
            "violations": list(self.violations),
        }


def load_diagnostic_scene(path: str | Path = DEFAULT_DIAGNOSTIC_SCENE_PATH) -> DiagnosticScene:
    raw = json.loads(Path(path).read_text(encoding="utf-8"))
    if raw.get("schema_version") != 1 or raw.get("scene_type") != "offline_diagnostic":
        raise ValueError("unsupported offline diagnostic scene schema")
    boxes_raw = raw.get("keep_out_boxes")
    if not isinstance(boxes_raw, list):
        raise ValueError("keep_out_boxes must be a list")
    boxes = []
    for entry in boxes_raw:
        if not isinstance(entry, dict):
            raise ValueError("each keep-out box must be an object")
        boxes.append(
            AxisAlignedBox(
                name=str(entry.get("name", "")),
                minimum_m=entry.get("minimum_m"),
                maximum_m=entry.get("maximum_m"),
            )
        )
    return DiagnosticScene(
        frame=str(raw.get("frame", "")),
        table_z_m=_as_finite_number(raw.get("table_z_m"), "table_z_m"),
        tcp_safety_radius_m=_as_finite_number(raw.get("tcp_safety_radius_m"), "tcp_safety_radius_m", positive=True),
        minimum_table_clearance_m=_as_finite_number(raw.get("minimum_table_clearance_m"), "minimum_table_clearance_m"),
        minimum_obstacle_clearance_m=_as_finite_number(raw.get("minimum_obstacle_clearance_m"), "minimum_obstacle_clearance_m"),
        keep_out_boxes=tuple(boxes),
    )


def review_tcp_positions(points_m: Iterable[np.ndarray], scene: DiagnosticScene) -> ClearanceReview:
    points = [_as_point(point, "TCP point") for point in points_m]
    if not points:
        raise ValueError("at least one TCP point is required for scene review")

    table_clearances = [float(point[2] - scene.table_z_m - scene.tcp_safety_radius_m) for point in points]
    minimum_table = min(table_clearances)
    obstacle_clearances: list[float] = []
    violations: list[str] = []
    if minimum_table < scene.minimum_table_clearance_m:
        violations.append(
            "TCP table clearance "
            f"{minimum_table:.4f} m is below {scene.minimum_table_clearance_m:.4f} m"
        )

    for box in scene.keep_out_boxes:
        clearance = min(box.clearance_to_point(point, scene.tcp_safety_radius_m) for point in points)
        obstacle_clearances.append(clearance)
        if clearance < scene.minimum_obstacle_clearance_m:
            violations.append(
                f"TCP clearance to {box.name} is {clearance:.4f} m, below "
                f"{scene.minimum_obstacle_clearance_m:.4f} m"
            )

    return ClearanceReview(
        safe=not violations,
        sample_count=len(points),
        minimum_table_clearance_m=minimum_table,
        minimum_obstacle_clearance_m=min(obstacle_clearances) if obstacle_clearances else None,
        violations=tuple(violations),
    )
