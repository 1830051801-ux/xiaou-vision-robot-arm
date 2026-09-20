#!/usr/bin/env python3
"""Stress YOLO -> calibration -> observation -> Transformer -> safety gates.

Real validation images exercise both registered OpenCV/ONNX detectors and the
checked-in calibration/profile preconditions.  Randomized synthetic detection
contracts then exercise downstream ROS-compatible observations and the INT8
Transformer under valid, low-confidence, unknown-class, collision, joint,
workspace, stale, and invalid-calibration cases.  No ROS process or hardware
transport is started.
"""

from __future__ import annotations

import argparse
from collections import Counter
import json
import math
from pathlib import Path
import sys
from typing import Any

import cv2
import numpy as np
import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from robot_ai.decision.onnx_policy import OnnxDecisionTransformerPolicy
from robot_ai.decision.transformer_policy import SafetyGate, TASK_TO_ID, observation_dict_to_vector
from robot_ai.vision.model_registry import resolve_profile
from robot_ai.vision.unified_yolo import UnifiedYolo
from simulation.desktop_scene import DesktopScene, OBJECT_CLASSES


IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
DETECTION_TO_TASK = {
    "pen": "pen",
    "bottle": "bottle",
    "cola": "cola",
    "earphone": "desktop_item",
    "cup": "cup",
    "tissue": "tissue_pull",
}
CASE_TYPES = (
    "valid",
    "low_confidence",
    "unknown_class",
    "collision_risk",
    "joint_offline",
    "outside_workspace",
    "stale_detection",
    "invalid_calibration",
)


def _pixel_to_base(homography: np.ndarray, u: float, v: float) -> tuple[float, float]:
    projected = homography @ np.asarray([u, v, 1.0], dtype=np.float64)
    if abs(float(projected[2])) < 1e-12:
        raise ValueError("homography produced a point at infinity")
    xy_mm = projected[:2] / projected[2]
    if not np.isfinite(xy_mm).all():
        raise ValueError("homography produced non-finite coordinates")
    return float(xy_mm[0]) * 0.001, float(xy_mm[1]) * 0.001


def _sample_calibrated_pixel(rng: np.random.Generator, hull: np.ndarray) -> tuple[float, float]:
    x, y, width, height = cv2.boundingRect(hull)
    for _ in range(1000):
        u = float(rng.uniform(x, x + width))
        v = float(rng.uniform(y, y + height))
        if cv2.pointPolygonTest(hull, (u, v), False) >= 0:
            return u, v
    raise RuntimeError("unable to sample inside calibration hull")


def _observation(
    scenario: str,
    *,
    x_m: float,
    y_m: float,
    u: float,
    v: float,
    confidence: float,
) -> dict[str, Any]:
    scene = DesktopScene(
        scenario=scenario,
        seed=20260809,
        object_position_m=(x_m, y_m, 0.0),
    )
    observation = scene.observation()
    observation["source"] = "offline_perception_decision_stress"
    observation["yolo"].update({
        "class": scenario,
        "confidence": confidence,
        "center_px": [u, v],
        "source": "synthetic_detection_contract",
    })
    observation["target"]["class"] = scenario
    observation["target"]["position_base_m"] = [x_m, y_m, 0.0]
    observation["perception"] = {
        "detection_fresh": True,
        "calibration_valid": True,
        "provisional_geometry": True,
    }
    return observation


def _actual_detector_audit(
    profile_name: str,
    images: list[Path],
    hull: np.ndarray,
    grasp_profiles: dict[str, Any],
) -> dict[str, Any]:
    profile = resolve_profile(profile_name)
    detector = UnifiedYolo.from_profile(profile_name)
    detections = 0
    inside_calibration = 0
    configuration_incomplete = 0
    class_counts: Counter[str] = Counter()
    for image_path in images:
        raw = np.fromfile(str(image_path), dtype=np.uint8)
        frame = cv2.imdecode(raw, cv2.IMREAD_COLOR) if raw.size else None
        if frame is None:
            raise ValueError(f"cannot decode {image_path}")
        for item in detector.detect(frame):
            detections += 1
            class_counts[item.name] += 1
            if cv2.pointPolygonTest(hull, (float(item.cx), float(item.cy)), False) < 0:
                continue
            inside_calibration += 1
            row = grasp_profiles.get(item.name) or {}
            if not isinstance(row.get("grasp_height_m"), (int, float)):
                configuration_incomplete += 1
    return {
        "profile": profile.as_dict(),
        "images": len(images),
        "detections": detections,
        "class_counts": dict(class_counts),
        "inside_calibration_region": inside_calibration,
        "configuration_incomplete": configuration_incomplete,
        "pose_candidates_published": inside_calibration - configuration_incomplete,
        "hardware_motion": False,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profiles", default="high_recall,high_precision")
    parser.add_argument(
        "--checkpoint",
        type=Path,
        default=PROJECT_ROOT / "runtime/decision/transformer_policy_safety_final_20260813.int8.onnx",
    )
    parser.add_argument(
        "--images-dir",
        type=Path,
        default=PROJECT_ROOT / "runtime/yolo_train/xiaou_objects_dataset_deep/images/val",
    )
    parser.add_argument("--trials", type=int, default=800)
    parser.add_argument("--seed", type=int, default=20260921)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.trials < len(CASE_TYPES):
        parser.error(f"trials must be at least {len(CASE_TYPES)}")

    checkpoint = args.checkpoint if args.checkpoint.is_absolute() else PROJECT_ROOT / args.checkpoint
    images_dir = args.images_dir if args.images_dir.is_absolute() else PROJECT_ROOT / args.images_dir
    images = sorted(path for path in images_dir.iterdir() if path.suffix.lower() in IMAGE_SUFFIXES)
    if not images:
        raise FileNotFoundError(images_dir)

    calibration = yaml.safe_load((PROJECT_ROOT / "codex_pickup_package/workspace_homography.yaml").read_text(encoding="utf-8"))
    homography = np.asarray(calibration["homography"], dtype=np.float64)
    pixel_points = np.asarray(calibration["pixel_points"], dtype=np.float32)
    hull = cv2.convexHull(pixel_points)
    if homography.shape != (3, 3) or np.linalg.matrix_rank(homography) != 3:
        raise ValueError("invalid checked-in homography")
    profile_config = json.loads((PROJECT_ROOT / "robot_ai/arm_control/config/object_grasp_profiles.json").read_text(encoding="utf-8"))
    grasp_profiles = profile_config.get("classes") or {}
    profile_names = [value.strip() for value in args.profiles.split(",") if value.strip()]
    actual = [_actual_detector_audit(name, images, hull, grasp_profiles) for name in profile_names]

    policy = OnnxDecisionTransformerPolicy(checkpoint)
    simulation_gate = SafetyGate(
        require_motion_enabled=False,
        require_hardware_ready=False,
        require_collision_free=True,
        require_feedback_verified=False,
        require_detection_fresh=True,
        require_calibration_valid=True,
        require_all_joints_online=True,
        min_detection_confidence=0.15,
    )
    hardware_gate = SafetyGate(
        require_motion_enabled=True,
        require_hardware_ready=True,
        require_collision_free=True,
        require_feedback_verified=True,
        require_detection_fresh=True,
        require_calibration_valid=True,
        require_all_joints_online=True,
        min_detection_confidence=0.15,
    )
    rng = np.random.default_rng(args.seed)
    case_counts: Counter[str] = Counter()
    policy_rejects: Counter[str] = Counter()
    gate_blocks: Counter[str] = Counter()
    precheck_blocks: Counter[str] = Counter()
    unsafe_candidates: Counter[str] = Counter()
    valid_candidates = 0
    hardware_candidates = 0
    examples = []

    for index in range(args.trials):
        case = CASE_TYPES[index % len(CASE_TYPES)]
        scenario = OBJECT_CLASSES[int(rng.integers(0, len(OBJECT_CLASSES)))]
        u, v = _sample_calibrated_pixel(rng, hull)
        x_m, y_m = _pixel_to_base(homography, u, v)
        observation = _observation(scenario, x_m=x_m, y_m=y_m, u=u, v=v, confidence=0.92)
        if case == "low_confidence":
            observation["yolo"]["confidence"] = 0.05
        elif case == "unknown_class":
            observation["yolo"]["class"] = "unknown"
            observation["target"]["class"] = "unknown"
        elif case == "collision_risk":
            observation["safety"]["collision_free"] = False
        elif case == "joint_offline":
            observation["arm"]["joint_online"][index % 6] = False
        elif case == "outside_workspace":
            observation["target"]["position_base_m"][0] = 0.65
        elif case == "stale_detection":
            observation["perception"]["detection_fresh"] = False
        elif case == "invalid_calibration":
            observation["perception"]["calibration_valid"] = False

        case_counts[case] += 1
        target_class = str(observation["target"]["class"])
        task_name = DETECTION_TO_TASK.get(target_class, target_class if target_class in TASK_TO_ID else None)
        precheck_failure = None
        if task_name is None or task_name not in TASK_TO_ID:
            precheck_failure = "unknown_object_class"
        else:
            target_position = observation["target"]["position_base_m"]
            if not (0.0 <= float(target_position[0]) <= 0.60 and -0.60 <= float(target_position[1]) <= 0.60):
                precheck_failure = "outside_policy_workspace"

        if precheck_failure:
            precheck_blocks[case] += 1
            simulation_status = "blocked_by_precheck"
            decision_dict = {"strategy": "reject", "phase": "abort", "blocked_reasons": [precheck_failure]}
        else:
            vector = observation_dict_to_vector(observation)
            decision = policy.predict(vector, TASK_TO_ID[task_name], simulation_gate, observation)
            simulation_status = decision.status
            decision_dict = decision.as_dict()
            allowed, failures = simulation_gate.evaluate(observation)
            if decision.status == "rejected_by_policy":
                policy_rejects[case] += 1
            if not allowed:
                gate_blocks[case] += 1
            unsafe = decision.strategy != "reject" and decision.status == "candidate" and allowed and case != "valid"
            if unsafe:
                unsafe_candidates[case] += 1
            if case == "valid" and decision.strategy != "reject" and decision.status == "candidate" and allowed:
                valid_candidates += 1

            hardware_allowed, _ = hardware_gate.evaluate(observation)
            if decision.strategy != "reject" and hardware_allowed:
                hardware_candidates += 1

        if len(examples) < len(CASE_TYPES):
            examples.append({
                "case": case,
                "scenario": scenario,
                "pixel": [round(u, 3), round(v, 3)],
                "base_xy_m": [round(x_m, 6), round(y_m, 6)],
                "simulation_status": simulation_status,
                "decision": decision_dict,
            })

    valid_total = case_counts["valid"]
    negative_total = args.trials - valid_total
    unsafe_total = sum(unsafe_candidates.values())
    report = {
        "offline": True,
        "hardware_motion": False,
        "camera_opened": False,
        "serial_opened": False,
        "can_opened": False,
        "ros2_process_started": False,
        "checkpoint": str(checkpoint.resolve()),
        "actual_yolo_calibration_audit": actual,
        "actual_chain_note": "Checked-in grasp heights are null, so actual image detections cannot publish a pose or motion candidate.",
        "synthetic_contract_stress": {
            "trials": args.trials,
            "seed": args.seed,
            "case_counts": dict(case_counts),
            "valid_candidates": valid_candidates,
            "valid_total": valid_total,
            "negative_total": negative_total,
            "policy_rejects": dict(policy_rejects),
            "gate_blocks": dict(gate_blocks),
            "precheck_blocks": dict(precheck_blocks),
            "unsafe_candidates": dict(unsafe_candidates),
            "unsafe_total": unsafe_total,
            "hardware_candidates": hardware_candidates,
            "examples": examples,
        },
        "passed": unsafe_total == 0 and hardware_candidates == 0 and valid_candidates == valid_total,
        "limitations": [
            "cup and tissue detections are synthetic contracts because the current YOLO schema has no labeled cup/tissue data",
            "provisional object geometry is used only for downstream message stress",
            "no ROS executor, camera, motor bus, MoveIt execution, or real grasp is tested",
        ],
    }
    output = args.output if args.output.is_absolute() else PROJECT_ROOT / args.output
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
