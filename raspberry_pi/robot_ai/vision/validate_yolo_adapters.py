#!/usr/bin/env python3
"""Offline smoke test for the unified YOLO model contract."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import cv2
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[2]
for import_root in (PROJECT_ROOT, PROJECT_ROOT / "robot_ai"):
    if str(import_root) not in sys.path:
        sys.path.insert(0, str(import_root))

from robot_ai.vision.unified_yolo import UnifiedYolo


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--models", default="xiaou_objects_gpu_deep.onnx")
    parser.add_argument("--images", default="my_objects_data/images/calib_000.jpg,my_data/penimages/calib_000.jpg")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--conf", type=float, default=0.10)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--union", action="store_true", help="keep all model detections instead of class fallback")
    args = parser.parse_args()

    images = [Path(item.strip()) for item in args.images.split(",") if item.strip()]
    report = {"offline": True, "hardware_motion": False, "models": args.models, "runs": []}
    detector = UnifiedYolo.from_config(
        args.models,
        imgsz=args.imgsz,
        conf=args.conf,
        device=args.device,
        fallback_uncovered_classes=not args.union,
    )
    for image_path in images:
        path = image_path if image_path.is_absolute() else PROJECT_ROOT / image_path
        raw = np.fromfile(str(path), dtype=np.uint8)
        frame = cv2.imdecode(raw, cv2.IMREAD_COLOR) if raw.size else None
        if frame is None:
            raise FileNotFoundError(f"Cannot read image: {path}")
        detections = detector.detect(frame)
        report["runs"].append({
            "image": str(path),
            "shape": list(frame.shape),
            "detections": [
                {
                    "name": item.name,
                    "confidence": round(float(item.conf), 6),
                    "box": [item.x1, item.y1, item.x2, item.y2],
                    "center_px": [item.cx, item.cy],
                }
                for item in detections
            ],
        })
    report["backend_names"] = detector.backend_names
    report["detection_count"] = sum(len(run["detections"]) for run in report["runs"])
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
