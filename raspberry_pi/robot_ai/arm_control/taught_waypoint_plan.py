"""No-I/O joint trajectory previews from a validated multi-pose teach record.

Unlike the cola-specific side planner, this module accepts the explicitly
captured intermediate poses required by each object family.  It deliberately
does not derive a cup, pen, or tissue route from another object's endpoint.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Mapping, Sequence

import numpy as np

from .teach_registry import validate_taught_object_record
from .trajectory import JointLimits, TrajectoryPoint, plan_quintic_joint_trajectory


class TaughtWaypointPlanningError(ValueError):
    """A validated record cannot be converted into an offline joint preview."""


@dataclass(frozen=True)
class TaughtWaypointSegment:
    name: str
    start_deg: tuple[float, ...]
    goal_deg: tuple[float, ...]
    duration_s: float
    point_count: int
    peak_abs_velocity_deg_s: float
    peak_abs_acceleration_deg_s2: float

    def as_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "start_deg": list(self.start_deg),
            "goal_deg": list(self.goal_deg),
            "duration_s": self.duration_s,
            "point_count": self.point_count,
            "peak_abs_velocity_deg_s": self.peak_abs_velocity_deg_s,
            "peak_abs_acceleration_deg_s2": self.peak_abs_acceleration_deg_s2,
        }


def _six(values: Sequence[float], label: str, *, positive: bool = False) -> np.ndarray:
    array = np.asarray(values, dtype=np.float64).reshape(-1)
    if array.size != 6 or not np.isfinite(array).all() or (positive and np.any(array <= 0.0)):
        raise TaughtWaypointPlanningError(f"{label} must contain six finite" + (" positive" if positive else "") + " values")
    return array


def build_taught_waypoint_preview(
    record: Mapping[str, object],
    *,
    position_min_deg: Sequence[float],
    position_max_deg: Sequence[float],
    velocity_max_deg_s: Sequence[float],
    acceleration_max_deg_s2: Sequence[float],
    sample_period_s: float = 0.05,
) -> dict[str, Any]:
    """Generate piecewise quintic trajectory metadata from a teach record.

    The returned trajectory is intentionally preview-only.  It contains no
    transport frame, execution flag, or gripper command; the object-specific
    gripper values remain part of the measured teach record.
    """

    if not math.isfinite(float(sample_period_s)) or sample_period_s <= 0.0:
        raise TaughtWaypointPlanningError("sample_period_s must be finite and positive")
    try:
        validation = validate_taught_object_record(record)
    except ValueError as exc:
        raise TaughtWaypointPlanningError(str(exc)) from exc
    object_info = validation["object"]
    if object_info["route_status"] != "requires_dedicated_teach":
        raise TaughtWaypointPlanningError("only a dedicated multi-pose teach record may use this generic waypoint preview")
    lower = _six(position_min_deg, "position_min_deg")
    upper = _six(position_max_deg, "position_max_deg")
    velocity = _six(velocity_max_deg_s, "velocity_max_deg_s", positive=True)
    acceleration = _six(acceleration_max_deg_s2, "acceleration_max_deg_s2", positive=True)
    if np.any(lower >= upper):
        raise TaughtWaypointPlanningError("position limits are invalid")
    limits = JointLimits(
        position_min=np.radians(lower),
        position_max=np.radians(upper),
        velocity_max=np.radians(velocity),
        acceleration_max=np.radians(acceleration),
    )
    poses = validation["poses_deg"]
    pose_ids = tuple(object_info["required_pose_ids"])
    segments: list[TaughtWaypointSegment] = []
    total_points = 0
    total_duration_s = 0.0
    for start_name, goal_name in zip(pose_ids, pose_ids[1:]):
        start = _six(poses[start_name], f"poses_deg.{start_name}")
        goal = _six(poses[goal_name], f"poses_deg.{goal_name}")
        try:
            points: list[TrajectoryPoint] = plan_quintic_joint_trajectory(
                np.radians(start),
                np.radians(goal),
                limits,
                sample_period_s=sample_period_s,
                minimum_duration_s=0.25,
            )
        except ValueError as exc:
            raise TaughtWaypointPlanningError(f"{start_name}->{goal_name}: {exc}") from exc
        duration = float(points[-1].time_from_start_s)
        peak_velocity = max(float(np.max(np.abs(point.velocities))) for point in points)
        peak_acceleration = max(float(np.max(np.abs(point.accelerations))) for point in points)
        segments.append(
            TaughtWaypointSegment(
                name=f"{start_name}_to_{goal_name}",
                start_deg=tuple(float(value) for value in start),
                goal_deg=tuple(float(value) for value in goal),
                duration_s=duration,
                point_count=len(points),
                peak_abs_velocity_deg_s=float(math.degrees(peak_velocity)),
                peak_abs_acceleration_deg_s2=float(math.degrees(peak_acceleration)),
            )
        )
        total_points += len(points)
        total_duration_s += duration
    return {
        "schema": "xiaou_taught_waypoint_preview_v1",
        "offline": True,
        "hardware_motion": False,
        "execution": "planning_preview_only",
        "object": object_info,
        "sample_period_s": sample_period_s,
        "segment_count": len(segments),
        "total_duration_s": total_duration_s,
        "total_points": total_points,
        "segments": [segment.as_dict() for segment in segments],
        "limitations": [
            "The source poses must come from read-only real-arm teaching; this preview does not transmit them.",
            "MuJoCo/MoveIt replay remains an approximation until the camera, TCP, table, and gripper are measured.",
        ],
    }


__all__ = ["TaughtWaypointPlanningError", "TaughtWaypointSegment", "build_taught_waypoint_preview"]
