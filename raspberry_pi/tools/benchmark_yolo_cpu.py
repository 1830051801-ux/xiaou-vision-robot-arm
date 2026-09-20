#!/usr/bin/env python3
"""Benchmark the Raspberry Pi-compatible OpenCV/ONNX YOLO path offline.

The benchmark only reads local images and an ONNX file.  It never opens a
camera, ROS topic, serial port, CAN interface, or motion path.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time
from collections import Counter

import cv2
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from robot_ai.yolo_opencv import OpenCVDnnYolo


def _read_image(path: Path) -> np.ndarray:
    raw = np.fromfile(str(path), dtype=np.uint8)
    frame = cv2.imdecode(raw, cv2.IMREAD_COLOR) if raw.size else None
    if frame is None:
        raise FileNotFoundError(f"cannot decode image: {path}")
    return frame


def _resolve_images(raw: str) -> list[Path]:
    paths: list[Path] = []
    for item in raw.split(","):
        value = item.strip()
        if not value:
            continue
        path = Path(value)
        if not path.is_absolute():
            path = PROJECT_ROOT / path
        if not path.is_file():
            raise FileNotFoundError(path)
        paths.append(path.resolve())
    if not paths:
        raise ValueError("at least one image is required")
    return paths


def benchmark(model: Path, images: list[Path], *, imgsz: int, conf: float, warmup: int, iterations: int, cv_threads: int) -> dict:
    cv2.setNumThreads(max(1, int(cv_threads)))
    detector = OpenCVDnnYolo(model_path=model, imgsz=imgsz, conf_thres=conf)
    frames = [_read_image(path) for path in images]
    for _ in range(max(0, warmup)):
        for frame in frames:
            detector.detect(frame)

    latencies_ms: list[float] = []
    class_counts: Counter[str] = Counter()
    confidence_values: list[float] = []
    detection_counts: list[int] = []
    for _ in range(max(1, iterations)):
        for frame in frames:
            started = time.perf_counter()
            detections = detector.detect(frame)
            latencies_ms.append((time.perf_counter() - started) * 1000.0)
            detection_counts.append(len(detections))
            for item in detections:
                class_counts[str(item.name)] += 1
                confidence_values.append(float(item.conf))

    ordered = sorted(latencies_ms)
    p50 = ordered[len(ordered) // 2]
    p95 = ordered[min(len(ordered) - 1, int(round(len(ordered) * 0.95)) - 1)]
    return {
        "offline": True,
        "hardware_motion": False,
        "backend": "opencv-onnx-cpu",
        "opencv_threads": max(1, int(cv_threads)),
        "model": str(model.resolve()),
        "model_bytes": model.stat().st_size,
        "names": list(detector.names),
        "image_count": len(images),
        "warmup_passes": warmup,
        "iterations_per_image": max(1, iterations),
        "samples": len(latencies_ms),
        "latency_ms": {
            "min": round(min(latencies_ms), 4),
            "p50": round(p50, 4),
            "mean": round(sum(latencies_ms) / len(latencies_ms), 4),
            "p95": round(p95, 4),
            "max": round(max(latencies_ms), 4),
        },
        "detections": {
            "total": sum(detection_counts),
            "mean_per_image": round(sum(detection_counts) / len(detection_counts), 4),
            "classes": dict(sorted(class_counts.items())),
            "mean_confidence": round(sum(confidence_values) / len(confidence_values), 6)
            if confidence_values
            else 0.0,
        },
        "images": [str(path) for path in images],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=PROJECT_ROOT / "models" / "xiaou_objects_gpu_deep.onnx")
    parser.add_argument(
        "--images",
        default="runtime/yolo_train/xiaou_objects_dataset_deep/images/val/obj_calib_000.jpg,"
        "runtime/yolo_train/xiaou_objects_dataset_deep/images/val/obj_calib_009.jpg,"
        "runtime/yolo_train/xiaou_objects_dataset_deep/images/val/pen_calib_001.jpg",
    )
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--conf", type=float, default=0.10)
    parser.add_argument("--warmup", type=int, default=3)
    parser.add_argument("--iterations", type=int, default=10)
    parser.add_argument("--cv-threads", type=int, default=1)
    parser.add_argument("--output", type=Path, default=PROJECT_ROOT / "runtime" / "yolo_train" / "yolo_cpu_benchmark_deep.json")
    args = parser.parse_args()
    model = args.model if args.model.is_absolute() else PROJECT_ROOT / args.model
    model = model.resolve()
    if not model.is_file():
        raise FileNotFoundError(model)
    report = benchmark(
        model,
        _resolve_images(args.images),
        imgsz=args.imgsz,
        conf=args.conf,
        warmup=args.warmup,
        iterations=args.iterations,
        cv_threads=args.cv_threads,
    )
    output = args.output if args.output.is_absolute() else PROJECT_ROOT / args.output
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
