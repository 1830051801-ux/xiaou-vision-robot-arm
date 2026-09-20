#!/usr/bin/env python3
"""Replay the ROS2 observation/decision contract without ROS2 or hardware.

This is a message-level contract test.  It exercises the same 46-feature
observation conversion and Transformer safety gate used by the ROS2 decision
node, then checks the launch/source gates that keep MoveIt and hardware motion
disabled during offline work.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Any

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from robot_ai.decision.transformer_policy import SafetyGate, TASK_TO_ID, observation_dict_to_vector
from robot_ai.decision.onnx_policy import OnnxDecisionTransformerPolicy
from robot_ai.decision.ros2_message_contract import FreshnessBook, validate_observation_contract
from simulation.desktop_scene import DesktopScene, OBJECT_CLASSES


TOPIC_CONTRACT = {
    "/xiaou/sim/observation": "JSON observation from desktop scene",
    "/xiaou/sim/action_candidates": "JSON rule candidates",
    "/xiaou/sim/target_pose": "PoseStamped in base_link",
    "/xiaou/sim/status": "JSON simulation status",
    "/xiaou/target_pose": "PoseStamped from YOLO perception",
    "/xiaou/target_status": "JSON target status from YOLO perception",
    "/xiaou/decision_observation": "JSON bridge output for Transformer",
    "/xiaou/decision": "JSON Transformer decision candidate or blocked result",
    "/xiaou/hardware_ready": "latched Bool readiness gate",
}


def _check_source_gates() -> list[str]:
    failures: list[str] = []
    pipeline = (PROJECT_ROOT / "ros2_ws/src/xiaou_arm_planning/launch/pipeline.launch.py").read_text(encoding="utf-8")
    planner = (PROJECT_ROOT / "ros2_ws/src/xiaou_arm_planning/src/target_planner_node.cpp").read_text(encoding="utf-8")
    readiness = (PROJECT_ROOT / "ros2_ws/src/xiaou_arm_hardware/xiaou_arm_hardware/hardware_readiness_node.py").read_text(encoding="utf-8")
    required = (
        ('"allow_trajectory_execution": False', "MoveIt trajectory execution must be disabled"),
        ('{"allow_execution": False}', "planner execution must be disabled"),
        ("validate_trajectory", "planner trajectory validation is missing"),
        ("/xiaou/hardware_ready", "hardware readiness topic is missing"),
        ("validate_motion_readiness", "readiness validator is missing"),
    )
    for needle, message in required:
        if needle not in pipeline + planner + readiness:
            failures.append(message)
    if "can_control.launch.py" in (PROJECT_ROOT / "ros2_ws/src/xiaou_arm_planning/launch/review_only.launch.py").read_text(encoding="utf-8"):
        failures.append("review-only launch references CAN control")
    return failures


def replay(checkpoint: Path, scenarios: list[str], device: str) -> dict[str, Any]:
    if checkpoint.suffix.lower() == ".onnx":
        policy: Any = OnnxDecisionTransformerPolicy(checkpoint, max_seq_len=16)
        metadata = dict(getattr(policy, "metadata", {}))
        backend = "onnxruntime-cpu"
    else:
        from robot_ai.decision.transformer_policy import DecisionTransformerPolicy

        policy = DecisionTransformerPolicy(device=device, max_seq_len=16)
        metadata = policy.load(checkpoint)
        policy.net.eval()
        backend = str(policy.device)
    simulation_gate = SafetyGate(
        require_motion_enabled=False,
        require_hardware_ready=False,
        require_collision_free=True,
        require_feedback_verified=False,
    )
    hardware_gate = SafetyGate(
        require_motion_enabled=True,
        require_hardware_ready=True,
        require_collision_free=True,
        require_feedback_verified=True,
    )
    rows: list[dict[str, Any]] = []
    failures: list[str] = []
    for index, scenario in enumerate(scenarios):
        scene = DesktopScene(scenario=scenario, seed=20260809 + index)
        observation = scene.observation()
        contract_valid, contract_failures = validate_observation_contract(observation)
        if not contract_valid:
            failures.append(f"{scenario}: observation contract invalid: {contract_failures}")
        vector = observation_dict_to_vector(observation)
        if vector.shape != (46,) or not np.isfinite(vector).all():
            failures.append(f"{scenario}: observation vector is not finite 46D")
        simulation_decision = policy.predict(vector, TASK_TO_ID[scenario], simulation_gate, observation)
        blocked_context = dict(observation)
        blocked_context["safety"] = {
            "motion_enabled": False,
            "hardware_ready": False,
            "collision_free": True,
            "feedback_verified": False,
        }
        hardware_decision = policy.predict(vector, TASK_TO_ID[scenario], hardware_gate, blocked_context)
        expected_topic_fields = {
            "schema_version": observation.get("schema_version"),
            "yolo_class": (observation.get("yolo") or {}).get("class"),
            "target_class": (observation.get("target") or {}).get("class"),
            "target_frame": "base_link",
            "tf_child": "sim_" + scenario,
            "observation_dim": int(vector.size),
        }
        if simulation_decision.status not in {"candidate", "blocked_by_action_limits"}:
            failures.append(f"{scenario}: simulation decision unexpectedly {simulation_decision.status}")
        if hardware_decision.status != "blocked_by_safety_gate":
            failures.append(f"{scenario}: hardware gate did not block ({hardware_decision.status})")
        rows.append({
            "scenario": scenario,
            "topics": {
                "observation": "/xiaou/sim/observation",
                "candidate": "/xiaou/sim/action_candidates",
                "target_pose": "/xiaou/sim/target_pose",
                "tf": "base_link -> sim_" + scenario,
                "decision": "/xiaou/decision",
            },
            "contract": expected_topic_fields,
            "contract_validation": {"passed": contract_valid, "failures": contract_failures},
            "simulation_decision": simulation_decision.as_dict(),
            "hardware_gate_decision": hardware_decision.as_dict(),
            "hardware_motion": False,
        })
    source_failures = _check_source_gates()
    failures.extend(source_failures)
    freshness = FreshnessBook()
    freshness.update("target_pose", 1_000_000_000)
    freshness_cases = {
        "fresh_at_1_4s": freshness.is_fresh("target_pose", 1_400_000_000, 0.5),
        "stale_at_1_6s": freshness.is_fresh("target_pose", 1_600_000_000, 0.5),
        "missing_channel": freshness.is_fresh("joint_states", 1_400_000_000, 0.5),
    }
    if freshness_cases != {"fresh_at_1_4s": True, "stale_at_1_6s": False, "missing_channel": False}:
        failures.append(f"freshness edge case mismatch: {freshness_cases}")
    malformed_valid, malformed_failures = validate_observation_contract({"schema_version": 9})
    if malformed_valid or "schema_version" not in malformed_failures:
        failures.append("malformed observation was accepted")
    return {
        "offline": True,
        "hardware_motion": False,
        "ros2_process_started": False,
        "ros2_cli_required": False,
        "checkpoint": str(checkpoint),
        "backend": backend,
        "checkpoint_metadata": metadata,
        "topic_contract": TOPIC_CONTRACT,
        "source_gate_failures": source_failures,
        "resilience_cases": {
            "freshness": freshness_cases,
            "malformed_message_rejected": not malformed_valid,
            "malformed_failures": malformed_failures,
        },
        "scenarios": rows,
        "failure_count": len(failures),
        "failures": failures,
        "passed": not failures,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--checkpoint",
        type=Path,
        default=PROJECT_ROOT / "runtime/decision/transformer_policy_safety_final_20260813.int8.onnx",
    )
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--scenarios", default=",".join(OBJECT_CLASSES))
    parser.add_argument("--output", type=Path, default=PROJECT_ROOT / "runtime/simulations/ros2_offline_replay_20260809.json")
    args = parser.parse_args()
    checkpoint = args.checkpoint if args.checkpoint.is_absolute() else PROJECT_ROOT / args.checkpoint
    scenarios = [item.strip().lower() for item in args.scenarios.split(",") if item.strip()]
    if not checkpoint.is_file():
        raise FileNotFoundError(checkpoint)
    if not scenarios or any(item not in OBJECT_CLASSES for item in scenarios):
        raise ValueError(f"scenarios must be drawn from {OBJECT_CLASSES}")
    report = replay(checkpoint.resolve(), scenarios, args.device)
    output = args.output if args.output.is_absolute() else PROJECT_ROOT / args.output
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
