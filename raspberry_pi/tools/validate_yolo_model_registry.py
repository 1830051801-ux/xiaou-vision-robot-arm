#!/usr/bin/env python3
"""Resolve and smoke-test every hash-checked YOLO profile offline."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import cv2
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from robot_ai.vision.model_registry import DEFAULT_REGISTRY, load_registry, resolve_profile
from robot_ai.vision.unified_yolo import UnifiedYolo


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--registry", type=Path, default=DEFAULT_REGISTRY)
    parser.add_argument(
        "--image",
        type=Path,
        default=PROJECT_ROOT / "runtime/yolo_train/xiaou_objects_dataset_deep/images/val/pen_calib_001.jpg",
    )
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    registry = load_registry(args.registry)
    image_path = args.image if args.image.is_absolute() else PROJECT_ROOT / args.image
    raw = np.fromfile(str(image_path), dtype=np.uint8)
    frame = cv2.imdecode(raw, cv2.IMREAD_COLOR) if raw.size else None
    if frame is None:
        raise FileNotFoundError(image_path)
    rows = []
    failures = []
    for name in registry["profiles"]:
        try:
            profile = resolve_profile(name, registry_path=args.registry)
            detector = UnifiedYolo.from_profile(name, registry_path=args.registry)
            detections = detector.detect(frame)
            row = {
                **profile.as_dict(),
                "backend_names": detector.backend_names,
                "detection_count": len(detections),
                "detected_classes": sorted({item.name for item in detections}),
                "passed": bool(detections),
            }
            if not row["passed"]:
                failures.append(f"{name}: no detections on smoke image")
            rows.append(row)
        except Exception as exc:
            failures.append(f"{name}: {type(exc).__name__}: {exc}")
    report = {
        "offline": True,
        "hardware_motion": False,
        "camera_opened": False,
        "serial_opened": False,
        "can_opened": False,
        "automatic_switching": registry["automatic_switching"],
        "default_profile": registry["default_profile"],
        "profiles": rows,
        "failures": failures,
        "passed": not failures,
    }
    output = args.output if args.output.is_absolute() else PROJECT_ROOT / args.output
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
