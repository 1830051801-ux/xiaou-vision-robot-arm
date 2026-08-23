"""Teach-pose anchored, approximate side-grasp route planning.

This module deliberately separates a demonstrated contact pose from the route
needed to reach it.  A taught pose is an endpoint, not an obstacle-avoiding
trajectory: direct joint interpolation can sweep laterally through a bottle.

For the project's current no-camera demonstration mode, the taught TCP is
treated as the centre-height side contact of an approximate cylinder.  The
planner derives an elevated transit plane, selects a reachable side approach,
then returns a sequence of IK-resolved Cartesian waypoints:

``raise -> overhead transfer -> overhead pregrasp -> segmented descend ->
segmented side approach -> taught contact``.

It is an offline model planner.  It performs no UART, CAN, ROS, gripper, or
motion I/O.  The caller owns execution and must keep hardware safety controls
independent from this approximate geometry.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Iterable, Sequence

import numpy as np

from .kinematics import fk_space, ik_space_multistart
from .model import ArmModel


JOINT_COUNT = 6


class TaughtGraspPlanningError(RuntimeError):
    """The approximated route could not be represented within joint limits."""


@dataclass(frozen=True)
class TaughtGraspStage:
    """One joint target generated from a Cartesian route waypoint."""

    name: str
    phase: str
    target_joint_rad: np.ndarray
    target_pose: np.ndarray
    ik_position_error_m: float
    ik_orientation_error_rad: float

    def as_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "phase": self.phase,
            "target_joint_deg": [round(float(value), 6) for value in np.degrees(self.target_joint_rad)],
            "target_tcp_xyz_m": [round(float(value), 6) for value in self.target_pose[:3, 3]],
            "target_tcp_rotation": [
                [round(float(value), 8) for value in row] for row in self.target_pose[:3, :3]
            ],
            "ik_position_error_m": float(self.ik_position_error_m),
            "ik_orientation_error_rad": float(self.ik_orientation_error_rad),
        }


@dataclass(frozen=True)
class TaughtGraspPlan:
    """A no-I/O plan tied to a real manually taught final joint pose."""

    model_name: str
    start_joint_rad: np.ndarray
    taught_joint_rad: np.ndarray
    object_center_m: np.ndarray
    table_z_m: float
    object_height_m: float
    object_radius_m: float
    taught_contact_offset_m: np.ndarray
    approach_direction_m: np.ndarray
    approach_distance_m: float
    contact_handoff_clearance_m: float
    overhead_clearance_m: float
    transit_sag_reserve_m: float
    transit_z_m: float
    sampled_transit_min_clearance_m: float
    stages: tuple[TaughtGraspStage, ...]
    candidate_label: str
    final_joint_is_exact_taught_pose: bool

    def as_dict(self) -> dict[str, Any]:
        return {
            "schema": "xiaou_taught_side_grasp_plan_v1",
            "planner": "teach_pose_anchored_poe_ik",
            "execution": "offline_only",
            "hardware_motion": False,
            "gripper_action": "not_included",
            "model_name": self.model_name,
            "start_joint_deg": [round(float(value), 6) for value in np.degrees(self.start_joint_rad)],
            "taught_grasp_joint_deg": [round(float(value), 6) for value in np.degrees(self.taught_joint_rad)],
            "taught_contact_offset_base_m": [
                round(float(value), 6) for value in self.taught_contact_offset_m
            ],
            "endpoint_mode": (
                "exact_taught_joint_pose"
                if self.final_joint_is_exact_taught_pose
                else "ik_retargeted_from_taught_pose"
            ),
            "object_model": {
                "shape": "upright_cylinder",
                "center_base_m": [round(float(value), 6) for value in self.object_center_m],
                "table_z_m": float(self.table_z_m),
                "height_m": float(self.object_height_m),
                "radius_m": float(self.object_radius_m),
                "anchor": "taught TCP is assumed to be side-center contact",
            },
            "approach": {
                "selection": self.candidate_label,
                "direction_base": [round(float(value), 8) for value in self.approach_direction_m],
                "pregrasp_distance_m": float(self.approach_distance_m),
                "contact_handoff_clearance_m": float(self.contact_handoff_clearance_m),
                "overhead_clearance_m": float(self.overhead_clearance_m),
                "transit_sag_reserve_m": float(self.transit_sag_reserve_m),
                "transit_z_m": float(self.transit_z_m),
                "sampled_transit_min_clearance_m": float(self.sampled_transit_min_clearance_m),
            },
            "stages": [stage.as_dict() for stage in self.stages],
            "limitations": [
                "Object dimensions and tabletop height are approximate and anchored to the taught contact pose.",
                "The final tool contact is a pose replay; gripper channel, jaw opening, closing angle, and gripping force are not modeled here.",
                "This POE route is not a substitute for measured camera calibration or a calibrated planning scene.",
            ],
        }


def _finite_array(values: Sequence[float] | np.ndarray, label: str) -> np.ndarray:
    array = np.asarray(values, dtype=np.float64).reshape(-1)
    if array.size != JOINT_COUNT or not np.isfinite(array).all():
        raise TaughtGraspPlanningError(f"{label} must contain six finite values")
    return array


def _finite_translation(values: Sequence[float] | np.ndarray, label: str) -> np.ndarray:
    array = np.asarray(values, dtype=np.float64).reshape(-1)
    if array.size != 3 or not np.isfinite(array).all():
        raise TaughtGraspPlanningError(f"{label} must contain three finite values")
    return array


def _validate_inputs(
    start_deg: Sequence[float],
    taught_deg: Sequence[float],
    lower_deg: Sequence[float],
    upper_deg: Sequence[float],
    *,
    object_height_m: float,
    object_radius_m: float,
    pregrasp_clearance_m: float,
    overhead_clearance_m: float,
    transit_sag_reserve_m: float,
    max_descent_step_m: float,
    max_side_step_m: float,
    contact_handoff_clearance_m: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    arrays = (
        _finite_array(start_deg, "start_deg"),
        _finite_array(taught_deg, "taught_deg"),
        _finite_array(lower_deg, "lower_deg"),
        _finite_array(upper_deg, "upper_deg"),
    )
    start, taught, lower, upper = arrays
    if np.any(lower >= upper):
        raise TaughtGraspPlanningError("each lower joint limit must be below its upper limit")
    for label, values in (("start", start), ("taught", taught)):
        if np.any(values < lower) or np.any(values > upper):
            raise TaughtGraspPlanningError(f"{label} joint pose exceeds effective limits")
    scalar_values = {
        "object_height_m": object_height_m,
        "object_radius_m": object_radius_m,
        "pregrasp_clearance_m": pregrasp_clearance_m,
        "overhead_clearance_m": overhead_clearance_m,
        "transit_sag_reserve_m": transit_sag_reserve_m,
        "max_descent_step_m": max_descent_step_m,
        "max_side_step_m": max_side_step_m,
        "contact_handoff_clearance_m": contact_handoff_clearance_m,
    }
    if any(not math.isfinite(float(value)) or float(value) <= 0.0 for value in scalar_values.values()):
        raise TaughtGraspPlanningError("object dimensions and route distances must be finite and positive")
    if object_radius_m * 2.0 >= object_height_m:
        raise TaughtGraspPlanningError("object height must exceed its diameter for an upright side grasp")
    return arrays


def _solve_ik(
    model: ArmModel,
    target_pose: np.ndarray,
    current_rad: np.ndarray,
    lower_rad: np.ndarray,
    upper_rad: np.ndarray,
) -> tuple[np.ndarray, float, float]:
    """Solve locally with deterministic branch seeds and continuity preference."""

    offsets = (
        np.zeros(JOINT_COUNT, dtype=np.float64),
        np.array((0.0, -0.30, 0.30, 0.0, 0.0, 0.0), dtype=np.float64),
        np.array((0.0, 0.30, -0.30, 0.0, 0.0, 0.0), dtype=np.float64),
        np.array((0.0, -0.70, 0.70, 0.0, 0.0, 0.0), dtype=np.float64),
        np.array((0.0, 0.70, -0.70, 0.0, 0.0, 0.0), dtype=np.float64),
    )
    result = ik_space_multistart(
        model.home_grasp_tcp,
        model.screw_axes,
        target_pose,
        [np.clip(current_rad + offset, lower_rad, upper_rad) for offset in offsets],
        preferred_angles=current_rad,
        joint_lower=lower_rad,
        joint_upper=upper_rad,
        orientation_tolerance_rad=2e-4,
        position_tolerance_m=2e-4,
        max_iterations=900,
        max_step_rad=0.07,
    )
    if not result.converged:
        raise TaughtGraspPlanningError(
            "IK did not converge: "
            f"position={result.position_error_m:.6f} m, "
            f"orientation={result.orientation_error_rad:.6f} rad"
        )
    return result.joint_angles, float(result.position_error_m), float(result.orientation_error_rad)


def _pose_with_xyz(template: np.ndarray, xyz: np.ndarray) -> np.ndarray:
    pose = np.asarray(template, dtype=np.float64).copy()
    pose[:3, 3] = np.asarray(xyz, dtype=np.float64)
    return pose


def _quintic_scale(tau: float) -> float:
    return 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5


def _sample_tcp_path(model: ArmModel, start_rad: np.ndarray, goal_rad: np.ndarray, count: int = 201) -> np.ndarray:
    points: list[np.ndarray] = []
    for tau in np.linspace(0.0, 1.0, count):
        joint_angles = start_rad + _quintic_scale(float(tau)) * (goal_rad - start_rad)
        pose = fk_space(model.home_grasp_tcp, model.screw_axes, joint_angles)
        points.append(pose[:3, 3])
    return np.asarray(points, dtype=np.float64)


def _horizontal_direction(axis: np.ndarray, label: str) -> tuple[str, np.ndarray] | None:
    direction = np.asarray(axis, dtype=np.float64).copy()
    direction[2] = 0.0
    norm = float(np.linalg.norm(direction))
    if norm < 1e-5:
        return None
    return label, direction / norm


def _side_direction_candidates(goal_pose: np.ndarray) -> Iterable[tuple[str, np.ndarray]]:
    # Tool local Z is tried first because the demonstrated branch has a
    # reachable side approach along that axis. The other horizontalized tool
    # axes are deterministic fallbacks for future taught poses.
    for axis_index, axis_label in ((2, "tool_z"), (0, "tool_x"), (1, "tool_y")):
        candidate = _horizontal_direction(goal_pose[:3, axis_index], axis_label)
        if candidate is None:
            continue
        label, direction = candidate
        yield label + "_positive", direction
        yield label + "_negative", -direction


def _build_candidate(
    *,
    model: ArmModel,
    start_rad: np.ndarray,
    taught_rad: np.ndarray,
    lower_rad: np.ndarray,
    upper_rad: np.ndarray,
    start_pose: np.ndarray,
    taught_pose: np.ndarray,
    direction: np.ndarray,
    candidate_label: str,
    object_height_m: float,
    object_radius_m: float,
    pregrasp_clearance_m: float,
    overhead_clearance_m: float,
    transit_sag_reserve_m: float,
    max_descent_step_m: float,
    max_side_step_m: float,
    contact_handoff_clearance_m: float,
    preserve_taught_joint_endpoint: bool,
    taught_contact_offset_m: np.ndarray,
) -> TaughtGraspPlan:
    object_center = taught_pose[:3, 3].copy()
    table_z_m = float(object_center[2] - object_height_m / 2.0)
    object_top_z_m = float(object_center[2] + object_height_m / 2.0)
    approach_distance_m = float(object_radius_m + pregrasp_clearance_m)
    requested_transit_z_m = max(
        float(start_pose[2, 3]),
        object_top_z_m + overhead_clearance_m + transit_sag_reserve_m,
    )

    last_error: str | None = None
    # IK branch changes can introduce several millimetres of endpoint-joint
    # sag. Raise both high waypoints when sampling reveals less clearance than
    # requested, while still respecting the measured joint envelope.
    for _ in range(8):
        current = start_rad.copy()
        stages: list[TaughtGraspStage] = []

        def add_ik_stage(name: str, phase: str, target_pose: np.ndarray) -> None:
            nonlocal current
            joints, position_error, orientation_error = _solve_ik(
                model, target_pose, current, lower_rad, upper_rad
            )
            stages.append(
                TaughtGraspStage(name, phase, joints, target_pose, position_error, orientation_error)
            )
            current = joints

        try:
            start_raise_pose = _pose_with_xyz(
                start_pose, np.array((start_pose[0, 3], start_pose[1, 3], requested_transit_z_m))
            )
            add_ik_stage("raise_to_transit", "raise", start_raise_pose)
            target_overhead_pose = _pose_with_xyz(
                taught_pose, np.array((taught_pose[0, 3], taught_pose[1, 3], requested_transit_z_m))
            )
            add_ik_stage("transfer_above_target", "transfer", target_overhead_pose)
            pregrasp_xy = object_center[:2] - direction[:2] * approach_distance_m
            pregrasp_overhead_pose = _pose_with_xyz(
                taught_pose, np.array((pregrasp_xy[0], pregrasp_xy[1], requested_transit_z_m))
            )
            add_ik_stage("transfer_above_pregrasp", "transfer", pregrasp_overhead_pose)

            descent_count = max(
                1,
                int(math.ceil((requested_transit_z_m - float(taught_pose[2, 3])) / max_descent_step_m)),
            )
            for index in range(1, descent_count + 1):
                fraction = index / descent_count
                z_m = requested_transit_z_m + fraction * (float(taught_pose[2, 3]) - requested_transit_z_m)
                pose = _pose_with_xyz(taught_pose, np.array((pregrasp_xy[0], pregrasp_xy[1], z_m)))
                add_ik_stage(f"descend_pregrasp_{index:02d}", "descend", pose)

            # Keep ordinary side-approach segments outside the object.  The
            # final, deliberately short contact segment is the only route
            # portion allowed to cross the approximate cylinder boundary.
            contact_handoff_distance_m = object_radius_m + contact_handoff_clearance_m
            if contact_handoff_distance_m >= approach_distance_m:
                raise TaughtGraspPlanningError(
                    "contact handoff distance must stay below the pregrasp distance"
                )
            side_travel_m = approach_distance_m - contact_handoff_distance_m
            approach_count = max(1, int(math.ceil(side_travel_m / max_side_step_m)))
            for index in range(1, approach_count + 1):
                fraction = index / approach_count
                distance_m = approach_distance_m - side_travel_m * fraction
                xyz = object_center - direction * distance_m
                pose = _pose_with_xyz(taught_pose, xyz)
                add_ik_stage(f"side_approach_{index:02d}", "side_approach", pose)

            if preserve_taught_joint_endpoint:
                # Preserve the manually demonstrated final joint branch,
                # instead of allowing another IK run to choose a nearby but
                # mechanically different endpoint.
                stages.append(
                    TaughtGraspStage(
                        "taught_side_contact",
                        "approach_contact",
                        taught_rad.copy(),
                        taught_pose.copy(),
                        0.0,
                        0.0,
                    )
                )
                current = taught_rad.copy()
            else:
                add_ik_stage("taught_side_contact", "approach_contact", taught_pose.copy())
        except TaughtGraspPlanningError as exc:
            last_error = str(exc)
            break

        previous = start_rad
        transit_min_z = math.inf
        for stage in stages:
            path = _sample_tcp_path(model, previous, stage.target_joint_rad)
            # The first raise begins far from the bottle and is allowed to
            # start below its top. Only the two target-area transfer stages
            # must clear the inferred object top.
            if stage.phase == "transfer":
                transit_min_z = min(transit_min_z, float(np.min(path[:, 2])))
            previous = stage.target_joint_rad
        sampled_clearance_m = transit_min_z - object_top_z_m
        if sampled_clearance_m >= overhead_clearance_m:
            return TaughtGraspPlan(
                model_name=model.name,
                start_joint_rad=start_rad.copy(),
                taught_joint_rad=taught_rad.copy(),
                object_center_m=object_center,
                table_z_m=table_z_m,
                object_height_m=float(object_height_m),
                object_radius_m=float(object_radius_m),
                taught_contact_offset_m=taught_contact_offset_m.copy(),
                approach_direction_m=direction.copy(),
                approach_distance_m=approach_distance_m,
                contact_handoff_clearance_m=float(contact_handoff_clearance_m),
                overhead_clearance_m=float(overhead_clearance_m),
                transit_sag_reserve_m=float(transit_sag_reserve_m),
                transit_z_m=float(requested_transit_z_m),
                sampled_transit_min_clearance_m=float(sampled_clearance_m),
                stages=tuple(stages),
                candidate_label=candidate_label,
                final_joint_is_exact_taught_pose=preserve_taught_joint_endpoint,
            )
        last_error = (
            f"sampled transit clearance {sampled_clearance_m:.4f} m is below "
            f"the requested {overhead_clearance_m:.4f} m"
        )
        requested_transit_z_m += max(0.003, overhead_clearance_m - sampled_clearance_m + 0.002)

    raise TaughtGraspPlanningError(last_error or "candidate route could not be generated")


def build_taught_side_grasp_plan(
    start_deg: Sequence[float],
    taught_deg: Sequence[float],
    *,
    model: ArmModel,
    lower_deg: Sequence[float],
    upper_deg: Sequence[float],
    object_height_m: float = 0.190,
    object_radius_m: float = 0.032,
    pregrasp_clearance_m: float = 0.120,
    overhead_clearance_m: float = 0.040,
    transit_sag_reserve_m: float = 0.020,
    max_descent_step_m: float = 0.030,
    max_side_step_m: float = 0.015,
    contact_handoff_clearance_m: float = 0.066,
    taught_contact_offset_base_m: Sequence[float] = (0.0, 0.0, 0.0),
) -> TaughtGraspPlan:
    """Plan a side grasp from a ready pose to a manually taught contact pose.

    The result is intentionally a *model-derived approximate demonstration*.
    It has no knowledge of camera calibration or real gripper force, and does
    not issue any command.  With the default zero contact offset, the final
    stage exactly preserves ``taught_deg``.  A nonzero
    ``taught_contact_offset_base_m`` retargets the demonstrated TCP in the
    base frame through IK.  It is useful only for offline coordinate-error
    regression; a real target offset still requires calibration before motion.
    """

    start_deg_array, taught_deg_array, lower_deg_array, upper_deg_array = _validate_inputs(
        start_deg,
        taught_deg,
        lower_deg,
        upper_deg,
        object_height_m=object_height_m,
        object_radius_m=object_radius_m,
        pregrasp_clearance_m=pregrasp_clearance_m,
        overhead_clearance_m=overhead_clearance_m,
        transit_sag_reserve_m=transit_sag_reserve_m,
        max_descent_step_m=max_descent_step_m,
        max_side_step_m=max_side_step_m,
        contact_handoff_clearance_m=contact_handoff_clearance_m,
    )
    start_rad = np.radians(start_deg_array)
    taught_rad = np.radians(taught_deg_array)
    lower_rad = np.radians(lower_deg_array)
    upper_rad = np.radians(upper_deg_array)
    start_pose = fk_space(model.home_grasp_tcp, model.screw_axes, start_rad)
    taught_contact_offset = _finite_translation(
        taught_contact_offset_base_m, "taught_contact_offset_base_m"
    )
    taught_pose = fk_space(model.home_grasp_tcp, model.screw_axes, taught_rad)
    taught_pose[:3, 3] += taught_contact_offset
    preserve_taught_joint_endpoint = bool(np.allclose(taught_contact_offset, 0.0, atol=1e-12))

    failures: list[str] = []
    for candidate_label, direction in _side_direction_candidates(taught_pose):
        try:
            return _build_candidate(
                model=model,
                start_rad=start_rad,
                taught_rad=taught_rad,
                lower_rad=lower_rad,
                upper_rad=upper_rad,
                start_pose=start_pose,
                taught_pose=taught_pose,
                direction=direction,
                candidate_label=candidate_label,
                object_height_m=object_height_m,
                object_radius_m=object_radius_m,
                pregrasp_clearance_m=pregrasp_clearance_m,
                overhead_clearance_m=overhead_clearance_m,
                transit_sag_reserve_m=transit_sag_reserve_m,
                max_descent_step_m=max_descent_step_m,
                max_side_step_m=max_side_step_m,
                contact_handoff_clearance_m=contact_handoff_clearance_m,
                preserve_taught_joint_endpoint=preserve_taught_joint_endpoint,
                taught_contact_offset_m=taught_contact_offset,
            )
        except TaughtGraspPlanningError as exc:
            failures.append(f"{candidate_label}: {exc}")
    raise TaughtGraspPlanningError(
        "no orientation-derived side approach is reachable within effective limits: " + "; ".join(failures)
    )
