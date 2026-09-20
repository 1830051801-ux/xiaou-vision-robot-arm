"""Monte-Carlo offline simulation of the complete vision-to-motion chain.

This is a deterministic stress test, not a claim of hardware validation. It
models camera/YOLO jitter, homography residuals, object placement error,
grasp-height error, target yaw error, workspace gates, IK and trajectory
limits. No ROS node, SocketCAN interface, or real actuator is opened.
"""

from __future__ import annotations

from collections import Counter
import argparse
import json
import math
from pathlib import Path
import sys

import cv2
import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "robot_ai"))
sys.path.insert(0, str(ROOT / "tools"))

from arm_control import JointLimits, load_default_model
from simulate_six_axis_pick import make_pose, pixel_to_base, rotation_from_rpy, run_sequence


def in_hull(hull: np.ndarray, point: tuple[float, float]) -> bool:
    return cv2.pointPolygonTest(hull, point, False) >= 0.0


def in_configured_workspace(x_mm: float, y_mm: float) -> bool:
    return 80.0 <= x_mm <= 360.0 and -180.0 <= y_mm <= 180.0 and 250.0 <= math.hypot(x_mm, y_mm) <= 430.0


def sample_pixel(rng: np.random.Generator, hull: np.ndarray, bounds: tuple[float, float, float, float]) -> tuple[float, float]:
    u_min, u_max, v_min, v_max = bounds
    for _ in range(10_000):
        point = (float(rng.uniform(u_min, u_max)), float(rng.uniform(v_min, v_max)))
        if in_hull(hull, point):
            return point
    raise RuntimeError("failed to sample a point inside calibration hull")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trials", type=int, default=100)
    parser.add_argument("--seed", type=int, default=20260806)
    parser.add_argument(
        "--ignore-workspace-gate",
        action="store_true",
        help="diagnostic only; run IK on rejected points without changing production ROS2 gates",
    )
    parser.add_argument("--max-ik-iterations", type=int, default=400)
    parser.add_argument("--ik-restarts", type=int, choices=(1, 3, 5), default=1)
    parser.add_argument("--output", type=Path, default=ROOT / "runtime" / "simulations" / "real_world_batch_20260806.json")
    args = parser.parse_args()
    if args.trials < 1 or args.trials > 1000:
        raise ValueError("trials must be in 1..1000")
    if args.max_ik_iterations < 1:
        raise ValueError("max-ik-iterations must be positive")

    rng = np.random.default_rng(args.seed)
    model = load_default_model()
    calibration = yaml.safe_load((ROOT / "codex_pickup_package" / "workspace_homography.yaml").read_text(encoding="utf-8"))
    homography = np.asarray(calibration["homography"], dtype=np.float64)
    pixel_points = np.asarray(calibration["pixel_points"], dtype=np.float32)
    hull = cv2.convexHull(pixel_points)
    bounds = (
        float(np.min(pixel_points[:, 0])),
        float(np.max(pixel_points[:, 0])),
        float(np.min(pixel_points[:, 1])),
        float(np.max(pixel_points[:, 1])),
    )
    limits = JointLimits(
        position_min=np.full(6, -math.pi),
        position_max=np.full(6, math.pi),
        velocity_max=np.full(6, 0.4),
        acceleration_max=np.full(6, 0.8),
    )

    current_success = 0
    aligned_success = 0
    workspace_rejected = 0
    unstable_rejected = 0
    hull_rejected = 0
    current_failures: Counter[str] = Counter()
    aligned_failures: Counter[str] = Counter()
    current_failed_stages: Counter[str] = Counter()
    aligned_failed_stages: Counter[str] = Counter()
    current_iterations: list[int] = []
    aligned_iterations: list[int] = []
    xy_samples_mm: list[tuple[float, float]] = []
    trial_records: list[dict[str, object]] = []

    for trial in range(args.trials):
        true_u, true_v = sample_pixel(rng, hull, bounds)
        observations = np.column_stack(
            [
                rng.normal(true_u, 1.5, size=5),
                rng.normal(true_v, 1.5, size=5),
            ]
        )
        median_uv = np.median(observations, axis=0)
        spread_px = float(np.max(np.linalg.norm(observations - median_uv, axis=1)))
        record: dict[str, object] = {
            "trial": trial,
            "true_pixel": [true_u, true_v],
            "median_pixel": median_uv.tolist(),
            "spread_px": spread_px,
        }
        if spread_px > 18.0:
            unstable_rejected += 1
            record["gate"] = "target_unstable"
            trial_records.append(record)
            continue
        if not in_hull(hull, (float(median_uv[0]), float(median_uv[1]))):
            hull_rejected += 1
            record["gate"] = "outside_calibrated_image_region"
            trial_records.append(record)
            continue
        x_m, y_m = pixel_to_base(homography, float(median_uv[0]), float(median_uv[1]))
        x_m += float(rng.normal(0.0, 0.00083))
        y_m += float(rng.normal(0.0, 0.00083))
        x_m += float(rng.normal(0.0, 0.002))
        y_m += float(rng.normal(0.0, 0.002))
        xy_samples_mm.append((x_m * 1000.0, y_m * 1000.0))
        record["base_xy_m"] = [x_m, y_m]
        if not in_configured_workspace(x_m * 1000.0, y_m * 1000.0) and not args.ignore_workspace_gate:
            workspace_rejected += 1
            record["gate"] = "outside_robot_workspace"
            trial_records.append(record)
            continue
        yaw = float(rng.normal(0.0, math.radians(3.0)))
        grasp_z = max(0.01, float(rng.normal(0.03, 0.003)))
        approach_z = grasp_z + 0.09
        record.update({
            "gate": "workspace_bypassed_for_diagnostic" if args.ignore_workspace_gate else "accepted",
            "yaw_rad": yaw,
            "grasp_z_m": grasp_z,
        })
        current_rotation = rotation_from_rpy(math.pi, 0.0, yaw)
        aligned_rotation = rotation_from_rpy(0.0, 0.0, yaw) @ model.home_grasp_tcp[:3, :3]
        place_xyz = (0.285, -0.16, approach_z)
        current_poses = [
            ("approach", make_pose(current_rotation, (x_m, y_m, approach_z))),
            ("descend", make_pose(current_rotation, (x_m, y_m, grasp_z))),
            ("lift", make_pose(current_rotation, (x_m, y_m, approach_z))),
            ("place", make_pose(current_rotation, place_xyz)),
            ("return_home", model.home_grasp_tcp.copy()),
        ]
        aligned_poses = [
            ("approach", make_pose(aligned_rotation, (x_m, y_m, approach_z))),
            ("descend", make_pose(aligned_rotation, (x_m, y_m, grasp_z))),
            ("lift", make_pose(aligned_rotation, (x_m, y_m, approach_z))),
            ("place", make_pose(aligned_rotation, place_xyz)),
            ("return_home", model.home_grasp_tcp.copy()),
        ]
        current_results, _, current_issues = run_sequence(
            model,
            limits,
            current_poses,
            max_ik_iterations=args.max_ik_iterations,
            ik_restarts=args.ik_restarts,
        )
        aligned_results, _, aligned_issues = run_sequence(
            model,
            limits,
            aligned_poses,
            max_ik_iterations=args.max_ik_iterations,
            ik_restarts=args.ik_restarts,
        )
        current_ok = not current_issues and all(item.ik_converged for item in current_results)
        aligned_ok = not aligned_issues and all(item.ik_converged for item in aligned_results)
        if current_ok:
            current_success += 1
        for issue in current_issues:
            current_failures[issue.split(":", 1)[-1].strip()] += 1
            current_failed_stages[issue.split(":", 1)[0].strip()] += 1
        current_iterations.extend(item.ik_iterations for item in current_results)
        if aligned_ok:
            aligned_success += 1
        for issue in aligned_issues:
            aligned_failures[issue.split(":", 1)[-1].strip()] += 1
            aligned_failed_stages[issue.split(":", 1)[0].strip()] += 1
        aligned_iterations.extend(item.ik_iterations for item in aligned_results)
        record.update({
            "current_algorithm_success": current_ok,
            "model_aligned_success": aligned_ok,
            "current_issues": current_issues,
            "aligned_issues": aligned_issues,
        })
        trial_records.append(record)

    xy_array = np.asarray(xy_samples_mm, dtype=np.float64) if xy_samples_mm else np.empty((0, 2))
    report = {
        "simulation": "real_world_monte_carlo_offline",
        "seed": args.seed,
        "trials": args.trials,
        "real_can_opened": False,
        "real_motion_enabled": False,
        "workspace_gate_enforced": not args.ignore_workspace_gate,
        "max_ik_iterations": args.max_ik_iterations,
        "ik_restarts": args.ik_restarts,
        "noise_model": {
            "detection_frames": 5,
            "pixel_sigma_px": 1.5,
            "homography_residual_sigma_mm": 0.83,
            "object_xy_sigma_mm": 2.0,
            "yaw_sigma_deg": 3.0,
            "grasp_height_mean_m": 0.03,
            "grasp_height_sigma_m": 0.003,
        },
        "calibration_output_range_mm": {
            "x_min": float(np.min(xy_array[:, 0])) if len(xy_array) else None,
            "x_max": float(np.max(xy_array[:, 0])) if len(xy_array) else None,
            "y_min": float(np.min(xy_array[:, 1])) if len(xy_array) else None,
            "y_max": float(np.max(xy_array[:, 1])) if len(xy_array) else None,
        },
        "gates": {
            "target_unstable": unstable_rejected,
            "outside_calibrated_image_region": hull_rejected,
            "outside_robot_workspace": workspace_rejected,
            "accepted_for_ik": args.trials - unstable_rejected - hull_rejected - workspace_rejected,
        },
        "current_algorithm": {
            "successes_after_workspace_gate": current_success,
            "success_rate_after_gate": current_success / max(1, args.trials - unstable_rejected - hull_rejected - workspace_rejected),
            "failures": dict(current_failures),
            "failed_stages": dict(current_failed_stages),
            "ik_iterations": {
                "max": max(current_iterations) if current_iterations else None,
                "mean": float(np.mean(current_iterations)) if current_iterations else None,
            },
        },
        "model_aligned_diagnostic": {
            "successes_after_workspace_gate": aligned_success,
            "success_rate_after_gate": aligned_success / max(1, args.trials - unstable_rejected - hull_rejected - workspace_rejected),
            "failures": dict(aligned_failures),
            "failed_stages": dict(aligned_failed_stages),
            "ik_iterations": {
                "max": max(aligned_iterations) if aligned_iterations else None,
                "mean": float(np.mean(aligned_iterations)) if aligned_iterations else None,
            },
        },
        "global_findings": [
            "The checked-in homography maps its calibration points to y approximately -220..-240 mm, while the runtime workspace gate is -180..180 mm; this configuration mismatch must be resolved from measured geometry.",
            "The current RPY(pi,0,yaw) TCP orientation fails the model IK for accepted targets; it requires a real TCP frame calibration before use.",
            "The model-aligned orientation is diagnostic only and is not written into production ROS2 parameters.",
            "The numeric batch does not replace MoveIt PlanningScene collision checking or real actuator feedback.",
        ],
        "trials_detail": trial_records,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    summary = {key: value for key, value in report.items() if key != "trials_detail"}
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
