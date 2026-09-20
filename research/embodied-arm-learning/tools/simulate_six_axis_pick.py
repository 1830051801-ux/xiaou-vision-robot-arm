"""Run a deterministic, transmit-free end-to-end six-axis pickup simulation.

The simulation uses the checked-in POE model and homography. It compares the
current perception orientation with a model-aligned orientation so that an IK
failure is reported as a real integration issue instead of being hidden by
invented hardware parameters.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import sys

import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[1]
ROBOT_AI = ROOT / "robot_ai"
if str(ROBOT_AI) not in sys.path:
    sys.path.insert(0, str(ROBOT_AI))

from arm_control import (
    JointLimits,
    fk_space,
    ik_space_multistart,
    load_default_model,
    plan_quintic_joint_trajectory,
)
from arm_control.can_loopback import CanLoopbackBus
from arm_control.can_protocol import encode_position_command
from arm_control.scene_review import DiagnosticScene, load_diagnostic_scene, review_tcp_positions


@dataclass
class SegmentResult:
    name: str
    ik_converged: bool
    ik_iterations: int
    position_error_m: float
    orientation_error_rad: float
    trajectory_points: int
    duration_s: float
    issue: str | None = None
    scene_review: dict[str, object] | None = None


def rotation_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def make_pose(rotation: np.ndarray, xyz_m: tuple[float, float, float]) -> np.ndarray:
    pose = np.eye(4, dtype=np.float64)
    pose[:3, :3] = rotation
    pose[:3, 3] = xyz_m
    return pose


def pixel_to_base(homography: np.ndarray, u: float, v: float) -> tuple[float, float]:
    projected = homography @ np.array([u, v, 1.0], dtype=np.float64)
    if abs(float(projected[2])) < 1e-12:
        raise ValueError("homography produced a point at infinity")
    xy_mm = projected[:2] / projected[2]
    return float(xy_mm[0]) * 1e-3, float(xy_mm[1]) * 1e-3


def run_sequence(
    model,
    limits: JointLimits,
    poses: list[tuple[str, np.ndarray]],
    *,
    scene: DiagnosticScene,
    max_ik_iterations: int = 400,
    ik_restarts: int = 1,
) -> tuple[list[SegmentResult], np.ndarray, list[str]]:
    if max_ik_iterations < 1:
        raise ValueError("max_ik_iterations must be positive")
    if ik_restarts not in (1, 3, 5):
        raise ValueError("ik_restarts must be one of 1, 3, or 5")
    current_angles = np.zeros(6, dtype=np.float64)
    all_results: list[SegmentResult] = []
    issues: list[str] = []
    for name, target in poses:
        offsets = [
            np.zeros(6, dtype=np.float64),
            np.array([0.0, -0.5, 0.5, 0.0, 0.0, 0.0]),
            np.array([0.0, 0.5, -0.5, 0.0, 0.0, 0.0]),
            np.array([0.0, -1.0, 1.0, 0.0, 0.0, 0.0]),
            np.array([0.0, 1.0, -1.0, 0.0, 0.0, 0.0]),
        ][:ik_restarts]
        seeds = [np.clip(current_angles + offset, limits.position_min, limits.position_max) for offset in offsets]
        result = ik_space_multistart(
            model.home_grasp_tcp,
            model.screw_axes,
            target,
            seeds,
            preferred_angles=current_angles,
            joint_lower=limits.position_min,
            joint_upper=limits.position_max,
            orientation_tolerance_rad=2e-4,
            position_tolerance_m=2e-4,
            max_iterations=max_ik_iterations,
            max_step_rad=0.15,
        )
        if not result.converged:
            issue = "IK did not converge within the assumed joint envelope"
            issues.append(f"{name}: {issue}")
            all_results.append(
                SegmentResult(
                    name,
                    False,
                    result.iterations,
                    result.position_error_m,
                    result.orientation_error_rad,
                    0,
                    0.0,
                    issue,
                )
            )
            continue
        points = plan_quintic_joint_trajectory(
            current_angles,
            result.joint_angles,
            limits,
            sample_period_s=0.01,
            minimum_duration_s=0.25,
        )
        for point in points:
            if np.any(point.positions < limits.position_min - 1e-9) or np.any(point.positions > limits.position_max + 1e-9):
                issues.append(f"{name}: trajectory exceeds position envelope")
            if np.any(np.abs(point.velocities) > limits.velocity_max + 1e-8):
                issues.append(f"{name}: trajectory exceeds velocity envelope")
            if np.any(np.abs(point.accelerations) > limits.acceleration_max + 1e-7):
                issues.append(f"{name}: trajectory exceeds acceleration envelope")
        tcp_positions = [
            fk_space(model.home_grasp_tcp, model.screw_axes, point.positions)[:3, 3]
            for point in points
        ]
        scene_review = review_tcp_positions(tcp_positions, scene)
        scene_issue = None
        if not scene_review.safe:
            scene_issue = "; ".join(scene_review.violations)
            issues.append(f"{name}: {scene_issue}")
        all_results.append(
            SegmentResult(
                name,
                True,
                result.iterations,
                result.position_error_m,
                result.orientation_error_rad,
                len(points),
                points[-1].time_from_start_s,
                issue=scene_issue,
                scene_review=scene_review.to_dict(),
            )
        )
        current_angles = result.joint_angles
    return all_results, current_angles, issues


def loopback_check() -> dict[str, object]:
    bus = CanLoopbackBus()
    command_frames = []
    for node_id in range(1, 7):
        can_id, payload = encode_position_command(node_id, 0.05 * node_id, 0.4, enable=True, sequence=node_id)
        command_frames.append((can_id, payload))
    for _ in range(45):
        for can_id, payload in command_frames:
            bus.send(can_id, payload)
        bus.step(0.02)
    positions = {str(node_id): bus.joints[node_id].position_rad for node_id in range(1, 7)}
    stop_id, stop_payload = encode_position_command(1, positions["1"], 0.0, quick_stop=True)
    bus.send(stop_id, stop_payload)
    quick_stop_ok = not bus.joints[1].enabled and bus.joints[1].velocity_rad_s == 0.0
    watchdog_bus = CanLoopbackBus()
    watchdog_id, watchdog_payload = encode_position_command(2, 0.1, 0.2, enable=True)
    watchdog_bus.send(watchdog_id, watchdog_payload)
    watchdog_bus.step(0.201)
    watchdog_ok = watchdog_bus.joints[2].fault and not watchdog_bus.joints[2].enabled
    return {
        "socketcan_opened": False,
        "all_joints_reached_command": all(abs(positions[str(i)] - 0.05 * i) < 1e-9 for i in range(1, 7)),
        "quick_stop_ok": quick_stop_ok,
        "watchdog_fault_ok": watchdog_ok,
        "positions_rad": positions,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "runtime" / "simulations" / "six_axis_pick_20260806.json")
    parser.add_argument("--pixel-u", type=float, default=318.0)
    parser.add_argument("--pixel-v", type=float, default=297.0)
    parser.add_argument("--target-x-m", type=float, help="diagnostic base_link X override; supply with --target-y-m")
    parser.add_argument("--target-y-m", type=float, help="diagnostic base_link Y override; supply with --target-x-m")
    parser.add_argument("--place-x-m", type=float, default=0.260, help="diagnostic base_link placement X")
    parser.add_argument("--place-y-m", type=float, default=-0.100, help="diagnostic base_link placement Y")
    parser.add_argument("--table-z-m", type=float, default=0.0)
    parser.add_argument("--grasp-height-m", type=float, default=0.03)
    parser.add_argument("--yaw-deg", type=float, default=0.0)
    parser.add_argument(
        "--orientation-mode",
        choices=("current_rpy", "cad_tcp"),
        default="current_rpy",
        help="target TCP orientation; cad_tcp is diagnostic until real TCP calibration",
    )
    parser.add_argument("--max-ik-iterations", type=int, default=400)
    parser.add_argument("--ik-restarts", type=int, choices=(1, 3, 5), default=1)
    parser.add_argument(
        "--scene",
        type=Path,
        default=ROOT / "robot_ai" / "arm_control" / "config" / "offline_diagnostic_scene.json",
        help="offline TCP clearance scene; never enables hardware motion",
    )
    args = parser.parse_args()

    model = load_default_model()
    if (args.target_x_m is None) != (args.target_y_m is None):
        parser.error("--target-x-m and --target-y-m must be supplied together")
    if args.target_x_m is not None and args.target_y_m is not None:
        x_m, y_m = float(args.target_x_m), float(args.target_y_m)
        if not math.isfinite(x_m) or not math.isfinite(y_m):
            parser.error("diagnostic target coordinates must be finite")
        target_source = "diagnostic_cli_override_not_camera_calibration"
    else:
        calibration = yaml.safe_load((ROOT / "codex_pickup_package" / "workspace_homography.yaml").read_text(encoding="utf-8"))
        homography = np.asarray(calibration["homography"], dtype=np.float64)
        x_m, y_m = pixel_to_base(homography, args.pixel_u, args.pixel_v)
        target_source = "saved_homography"
    scene = load_diagnostic_scene(args.scene)
    if not math.isfinite(args.place_x_m) or not math.isfinite(args.place_y_m):
        parser.error("diagnostic placement coordinates must be finite")
    yaw = math.radians(args.yaw_deg)
    current_rotation = rotation_from_rpy(math.pi, 0.0, yaw)
    aligned_rotation = rotation_from_rpy(0.0, 0.0, yaw) @ model.home_grasp_tcp[:3, :3]
    grasp_z = args.table_z_m + args.grasp_height_m
    approach_z = grasp_z + 0.09
    place_xyz = (args.place_x_m, args.place_y_m, approach_z)
    limits = JointLimits(
        position_min=np.full(6, -math.pi),
        position_max=np.full(6, math.pi),
        velocity_max=np.full(6, 0.4),
        acceleration_max=np.full(6, 0.8),
    )
    current_poses = [
        ("approach", make_pose(current_rotation, (x_m, y_m, approach_z))),
        ("descend", make_pose(current_rotation, (x_m, y_m, grasp_z))),
        ("lift", make_pose(current_rotation, (x_m, y_m, approach_z))),
        ("place", make_pose(current_rotation, place_xyz)),
        ("return_home", model.home_grasp_tcp.copy()),
    ]
    aligned_poses = [
        (name, make_pose(aligned_rotation, (x_m, y_m, approach_z)))
        for name in ("approach",)
    ] + [
        ("descend", make_pose(aligned_rotation, (x_m, y_m, grasp_z))),
        ("lift", make_pose(aligned_rotation, (x_m, y_m, approach_z))),
        ("place", make_pose(aligned_rotation, place_xyz)),
        ("return_home", model.home_grasp_tcp.copy()),
    ]
    current_results, _, current_issues = run_sequence(
        model,
        limits,
        current_poses,
        scene=scene,
        max_ik_iterations=args.max_ik_iterations,
        ik_restarts=args.ik_restarts,
    )
    aligned_results, _, aligned_issues = run_sequence(
        model,
        limits,
        aligned_poses,
        scene=scene,
        max_ik_iterations=args.max_ik_iterations,
        ik_restarts=args.ik_restarts,
    )
    selected_results, selected_issues = (
        (current_results, current_issues)
        if args.orientation_mode == "current_rpy"
        else (aligned_results, aligned_issues)
    )
    result = {
        "simulation": "offline_six_axis_pick",
        "real_can_opened": False,
        "real_motion_enabled": False,
        "synthetic_detection": {"class": "cup", "confidence": 0.92, "pixel": [args.pixel_u, args.pixel_v]},
        "target_source": target_source,
        "homography_output_m": [x_m, y_m],
        "diagnostic_place_xy_m": [args.place_x_m, args.place_y_m],
        "grasp_height_source": "command_line_assumption_not_measured",
        "assumed_grasp_height_m": args.grasp_height_m,
        "assumed_joint_limits": True,
        "selected_orientation_mode": args.orientation_mode,
        "max_ik_iterations": args.max_ik_iterations,
        "ik_restarts": args.ik_restarts,
        "diagnostic_scene": {
            "path": str(args.scene),
            "frame": scene.frame,
            "table_z_m": scene.table_z_m,
            "tcp_safety_radius_m": scene.tcp_safety_radius_m,
            "minimum_table_clearance_m": scene.minimum_table_clearance_m,
            "minimum_obstacle_clearance_m": scene.minimum_obstacle_clearance_m,
            "keep_out_box_count": len(scene.keep_out_boxes),
            "scope": "TCP swept-path review only; not full robot collision checking",
        },
        "selected_path": {
            "segments": [asdict(item) for item in selected_results],
            "issues": selected_issues,
            "complete_pick_path": not selected_issues and all(item.ik_converged for item in selected_results),
        },
        "current_algorithm": {
            "orientation": "RPY(pi,0,yaw) from target_pose_node",
            "segments": [asdict(item) for item in current_results],
            "issues": current_issues,
            "complete_pick_path": not current_issues and all(item.ik_converged for item in current_results),
        },
        "model_aligned_comparison": {
            "orientation": "Rz(yaw) * model.home_grasp_tcp.rotation",
            "segments": [asdict(item) for item in aligned_results],
            "issues": aligned_issues,
            "complete_pick_path": not aligned_issues and all(item.ik_converged for item in aligned_results),
        },
        "protocol_loopback": loopback_check(),
        "findings": [
            "Current object grasp profiles are null; production perception must remain locked until measured.",
            "Current RPY(pi,0,yaw) target orientation must be checked against the real TCP frame; this simulation does not treat IK failure as a planning success.",
            "The model-aligned comparison is diagnostic only and must not be copied into production without TCP orientation calibration.",
            "The diagnostic scene checks sampled TCP swept paths against a table plane and keep-out boxes; it does not replace MoveIt full-link or self-collision checking.",
        ],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
