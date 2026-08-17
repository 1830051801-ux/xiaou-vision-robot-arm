#!/usr/bin/env python3
"""Export a taught-pose grasp plan as ROS2/MoveIt waypoint data, without ROS I/O."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from robot_ai.arm_control import (  # noqa: E402
    build_grasp_family_preview,
    build_taught_side_grasp_plan,
    load_default_model,
)


def _matrix_to_xyzw(rotation: list[list[float]]) -> list[float]:
    """Convert a validated 3x3 rotation matrix to a ROS xyzw quaternion."""

    matrix = np.asarray(rotation, dtype=np.float64)
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all():
        raise ValueError("rotation must be a finite 3x3 matrix")
    if not np.allclose(matrix.T @ matrix, np.eye(3), atol=1e-6) or not np.isclose(np.linalg.det(matrix), 1.0, atol=1e-6):
        raise ValueError("rotation must be orthonormal with determinant +1")
    trace = float(np.trace(matrix))
    if trace > 0.0:
        root = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * root
        x = (matrix[2, 1] - matrix[1, 2]) / root
        y = (matrix[0, 2] - matrix[2, 0]) / root
        z = (matrix[1, 0] - matrix[0, 1]) / root
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            root = math.sqrt(max(1e-15, 1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2])) * 2.0
            x = 0.25 * root
            y = (matrix[0, 1] + matrix[1, 0]) / root
            z = (matrix[0, 2] + matrix[2, 0]) / root
            w = (matrix[2, 1] - matrix[1, 2]) / root
        elif index == 1:
            root = math.sqrt(max(1e-15, 1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2])) * 2.0
            x = (matrix[0, 1] + matrix[1, 0]) / root
            y = 0.25 * root
            z = (matrix[1, 2] + matrix[2, 1]) / root
            w = (matrix[0, 2] - matrix[2, 0]) / root
        else:
            root = math.sqrt(max(1e-15, 1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1])) * 2.0
            x = (matrix[0, 2] + matrix[2, 0]) / root
            y = (matrix[1, 2] + matrix[2, 1]) / root
            z = 0.25 * root
            w = (matrix[1, 0] - matrix[0, 1]) / root
    quaternion = np.asarray((x, y, z, w), dtype=np.float64)
    quaternion /= np.linalg.norm(quaternion)
    return [round(float(value), 10) for value in quaternion]


READY_DEG = [0.0, -50.0, -55.0, -70.0, 110.0, 0.0]
TAUGHT_COLA_GRASP_DEG = [-100.000244140625, -48.93047332763672, -71.99970245361328, -84.9998664855957, 110.00060272216797, 0.0]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--hardware-config",
        type=Path,
        default=ROOT / "robot_ai" / "arm_control" / "config" / "hardware_calibration_candidate_20260811.json",
    )
    parser.add_argument("--output", type=Path, default=ROOT / "runtime" / "teach_poses" / "cola_taught_grasp_ros2_waypoints.json")
    parser.add_argument("--object-class", default="cola")
    parser.add_argument("--object-radius-m", type=float)
    parser.add_argument("--object-height-m", type=float)
    parser.add_argument("--target-offset-base-m", default="0,0,0")
    args = parser.parse_args()
    config = json.loads(args.hardware_config.read_text(encoding="utf-8"))
    lower = [math.degrees(float(value)) for value in config["position_min_rad"]]
    upper = [math.degrees(float(value)) for value in config["position_max_rad"]]
    try:
        target_offset = [float(value.strip()) for value in args.target_offset_base_m.split(",")]
    except ValueError as exc:
        parser.error("target-offset-base-m must be three comma-separated numbers")
        raise AssertionError from exc
    if len(target_offset) != 3 or not all(math.isfinite(value) for value in target_offset):
        parser.error("target-offset-base-m must be three finite numbers")
    family_preview = build_grasp_family_preview(
        args.object_class,
        READY_DEG,
        TAUGHT_COLA_GRASP_DEG,
        model=load_default_model(),
        lower_deg=lower,
        upper_deg=upper,
        object_radius_m=args.object_radius_m,
        object_height_m=args.object_height_m,
        target_offset_base_m=target_offset,
    )
    if not family_preview.preview_ready or family_preview.plan is None:
        raise RuntimeError(
            f"grasp family {family_preview.family} is {family_preview.status}: {family_preview.reason}"
        )
    plan = family_preview.plan.as_dict()
    ros2_waypoints = []
    for stage in plan["stages"]:
        rotation = stage["target_tcp_rotation"]
        # C++ MoveIt receives a pose quaternion at runtime; matrix remains in
        # this artifact to keep a lossless, inspectable base_link contract.
        ros2_waypoints.append({
            "name": stage["name"],
            "phase": stage["phase"],
            "frame_id": "base_link",
            "position_m": stage["target_tcp_xyz_m"],
            "rotation_matrix": rotation,
            "orientation_xyzw": _matrix_to_xyzw(rotation),
            "joint_seed_deg": stage["target_joint_deg"],
        })
    payload = {
        "schema": "xiaou_ros2_moveit_waypoints_v1",
        "execution": "planning_preview_only",
        "planner_contract": {
            "planning_group": "arm",
            "end_effector_link": "grasp_tcp",
            "frame_id": "base_link",
            "moveit_cartesian_eef_step_m": 0.010,
            "moveit_collision_check": False,
            "hardware_execution": False,
            "output_topic": "/xiaou/cartesian_waypoints",
            "result_topic": "/xiaou/cartesian_preview_trajectory",
        },
        "start_state_contract": {
            "source": "taught_ready_pose",
            "joint_names": list(load_default_model().joint_names),
            "joint_positions_deg": [round(float(value), 6) for value in READY_DEG],
            "purpose": "MoveIt preview must start from the taught ready pose, not the joint_state_publisher zero pose.",
            "hardware_execution": False,
        },
        "source_plan": plan,
        "grasp_family_preview": family_preview.as_dict(),
        "waypoints": ros2_waypoints,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"planned": True, "waypoints": len(ros2_waypoints), "output": str(args.output)}, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
