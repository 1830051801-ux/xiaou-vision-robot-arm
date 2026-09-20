"""Static review of the ROS2 six-axis architecture and execution gates."""

from __future__ import annotations

import json
import argparse
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "ros2_ws" / "src"


def require(text: str, needle: str, label: str, failures: list[str]) -> None:
    if needle not in text:
        failures.append(f"{label}: missing {needle!r}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "runtime" / "simulations" / "ros2_architecture_audit_20260806.json",
    )
    args = parser.parse_args()
    failures: list[str] = []
    pipeline = (SRC / "xiaou_arm_planning" / "launch" / "pipeline.launch.py").read_text(encoding="utf-8")
    review = (SRC / "xiaou_arm_planning" / "launch" / "review_only.launch.py").read_text(encoding="utf-8")
    smoke = (SRC / "xiaou_arm_planning" / "launch" / "safety_smoke.launch.py").read_text(encoding="utf-8")
    planner = (SRC / "xiaou_arm_planning" / "src" / "target_planner_node.cpp").read_text(encoding="utf-8")
    perception = (SRC / "xiaou_arm_perception" / "xiaou_arm_perception" / "target_pose_node.py").read_text(encoding="utf-8")
    readiness = (SRC / "xiaou_arm_hardware" / "xiaou_arm_hardware" / "hardware_readiness_node.py").read_text(encoding="utf-8")

    for needle, label in (
        ('"allow_trajectory_execution": False', "MoveIt execution gate"),
        ('{"allow_execution": False}', "planner execution gate"),
        ('DeclareLaunchArgument(\n                "start_perception"', "optional perception gate"),
        ("condition=IfCondition(start_perception)", "perception condition"),
        ('"start_move_group"', "MoveIt start gate"),
        ('"start_planner"', "planner start gate"),
    ):
        require(pipeline, needle, "pipeline", failures)
    for needle, label in (
        ('"start_perception": "false"', "review-only perception disabled"),
        ('"table_z_m": "nan"', "review-only grasp height lock"),
    ):
        require(review, needle, "review launch", failures)
    for needle in ('"start_perception": "false"', '"start_move_group": "false"', '"start_planner": "false"'):
        require(smoke, needle, "safety smoke launch", failures)
    for needle, label in (
        ('if (!allow_execution_ || !hardware_ready_)', "planner hardware gate"),
        ('validate_trajectory', "trajectory preflight"),
        ('target->header.frame_id != "base_link"', "target frame gate"),
    ):
        require(planner, needle, "planner", failures)
    for needle in (
        "configuration_incomplete",
        "target_unstable",
        "outside_robot_workspace",
        "target_pose_published",
    ):
        require(perception, needle, "perception status", failures)
    for needle in ("/xiaou/hardware_ready", "TRANSIENT_LOCAL", "validate_motion_readiness"):
        require(readiness, needle, "readiness node", failures)

    report = {
        "architecture_verified": not failures,
        "review_only_launch_has_no_can_control_node": "can_control.launch.py" not in review,
        "required_layers": ["state_publisher", "MoveIt move_group", "readiness gate", "planner"],
        "failure_count": len(failures),
        "failures": failures,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if not failures else 1


if __name__ == "__main__":
    raise SystemExit(main())
