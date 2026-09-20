#!/usr/bin/env python3
"""Extract and smoke-test the Pi inference archive without touching hardware."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import tarfile
import tempfile
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ARCHIVE = PROJECT_ROOT / "runtime/deployment/xiaou_pi_inference_final_20260813.tar.gz"
DEFAULT_IMAGE = PROJECT_ROOT / "runtime/yolo_train/xiaou_objects_dataset_deep/images/val/pen_calib_001.jpg"
FORBIDDEN = (".pt", ".pth", "ultralytics", "mujoco", "dataset")


def verify(archive: Path, image: Path) -> dict[str, Any]:
    if not archive.is_file():
        raise FileNotFoundError(archive)
    if not image.is_file():
        raise FileNotFoundError(image)
    # Some optional GUI/logging dependencies keep a Windows file handle alive
    # until interpreter shutdown.  Cleanup errors must not turn a successful
    # read-only archive verification into a false deployment failure.
    with tempfile.TemporaryDirectory(prefix="xiaou_pi_verify_", ignore_cleanup_errors=True) as staging:
        stage = Path(staging)
        with tarfile.open(archive, "r:*") as bundle:
            members = bundle.getmembers()
            names = [item.name for item in members]
            forbidden = [name for name in names if any(token in name.lower() for token in FORBIDDEN)]
            try:
                bundle.extractall(stage, filter="data")
            except TypeError:  # Python < 3.12
                bundle.extractall(stage)
        root = stage / "xiaou_pi"
        sys.path.insert(0, str(root))
        missing: list[str] = []
        try:
            import cv2  # type: ignore
            import numpy as np  # type: ignore

            from robot_ai.decision.onnx_policy import OnnxDecisionTransformerPolicy
            from robot_ai.yolo_opencv import OpenCVDnnYolo
            from robot_ai.arm_control import load_grasp_family_registry, load_object_teach_registry

            raw = np.fromfile(str(image), dtype=np.uint8)
            frame = cv2.imdecode(raw, cv2.IMREAD_COLOR) if raw.size else None
            if frame is None:
                raise ValueError(f"cannot decode image: {image}")
            detector = OpenCVDnnYolo(root / "models/xiaou_objects_gpu_deep.onnx", imgsz=640, conf_thres=0.1)
            detections = detector.detect(frame)
            policy = OnnxDecisionTransformerPolicy(
                root / "runtime/decision/transformer_policy_safety_final_20260813.int8.onnx"
            )
            grasp_registry = load_grasp_family_registry()
            teach_registry = load_object_teach_registry()
            metadata = dict(policy.metadata)
            required = [
                "DEPLOYMENT_MANIFEST.json",
                "models/xiaou_objects_gpu_deep.onnx",
                "runtime/decision/transformer_policy_safety_final_20260813.int8.onnx",
                "robot_ai/arm_control/grasp_families.py",
                "robot_ai/arm_control/config/grasp_family_registry.json",
                "robot_ai/arm_control/config/object_teach_registry.json",
                "robot_ai/arm_control/teach_registry.py",
                "robot_ai/arm_control/taught_waypoint_plan.py",
                "tools/profile_pi_inference_runtime.py",
                "tools/preview_object_teach_record.py",
                "robot_ai/yolo_opencv.py",
                "robot_ai/decision/onnx_policy.py",
                "robot_ai/arm_control/taught_grasp.py",
                "robot_ai/arm_control/config/arm_model.json",
            ]
            missing = [path for path in required if not (root / path).is_file()]
            from tools.verify_pi_inference_release import verify as verify_release

            release_verification = verify_release(root, require_runtime_modules=True)
            dual_checks: dict[str, Any] = {"present": False, "passed": True}
            registry_path = root / "robot_ai/vision/config/yolo_model_registry.json"
            if registry_path.is_file():
                from robot_ai.vision.model_registry import resolve_profile

                profile_rows = []
                for profile_name in ("high_recall", "high_precision"):
                    profile = resolve_profile(profile_name, registry_path=registry_path, project_root=root)
                    profile_detector = OpenCVDnnYolo(profile.model_path, imgsz=640, conf_thres=profile.confidence)
                    profile_detections = profile_detector.detect(frame)
                    profile_rows.append({
                        "name": profile_name,
                        "classes": list(profile.classes),
                        "detection_count": len(profile_detections),
                        "sha256": profile.sha256,
                    })
                dual_checks = {
                    "present": True,
                    "profiles": profile_rows,
                    "safety_transformer_metadata": dict(policy.metadata),
                    "passed": all(row["detection_count"] > 0 for row in profile_rows)
                    and policy.metadata.get("hardware_motion") is False,
                }
        finally:
            sys.path.pop(0)
    return {
        "offline": True,
        "hardware_motion": False,
        "serial_opened": False,
        "can_opened": False,
        "archive": str(archive.resolve()),
        "file_count": len(names),
        "forbidden_entries": forbidden,
        "missing_required_files": missing,
        "yolo_names": list(detector.names),
        "yolo_detection_count": len(detections),
        "transformer_metadata": metadata,
        "grasp_family_registry_schema": grasp_registry.get("schema_version"),
        "grasp_family_classes": sorted(grasp_registry.get("classes", {})),
        "teach_registry_classes": sorted(teach_registry.get("objects", {})),
        "release_verification": release_verification,
        "dual_candidate_checks": dual_checks,
        "passed": not forbidden and not missing and bool(detections) and metadata.get("hardware_motion") is False and dual_checks["passed"] and release_verification["passed"],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive", type=Path, default=DEFAULT_ARCHIVE)
    parser.add_argument("--image", type=Path, default=DEFAULT_IMAGE)
    parser.add_argument("--output", type=Path, default=PROJECT_ROOT / "runtime/deployment/xiaou_pi_package_probe_20260809.json")
    args = parser.parse_args()
    archive = args.archive if args.archive.is_absolute() else PROJECT_ROOT / args.archive
    image = args.image if args.image.is_absolute() else PROJECT_ROOT / args.image
    report = verify(archive.resolve(), image.resolve())
    output = args.output if args.output.is_absolute() else PROJECT_ROOT / args.output
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
