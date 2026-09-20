#!/usr/bin/env python3
"""Compare Pi-compatible ONNX detectors under offline photometric shifts.

The transformations preserve geometry, so the original YOLO boxes remain
valid.  Metrics use greedy class-aware IoU matching at 0.5.  This small local
holdout is useful for relative regression checks, not a substitute for new
camera data or a production benchmark.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time
from typing import Callable

import cv2
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from robot_ai.yolo_opencv import Detection, OpenCVDnnYolo


IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def _clip(frame: np.ndarray, alpha: float, beta: float) -> np.ndarray:
    return np.clip(frame.astype(np.float32) * alpha + beta, 0, 255).astype(np.uint8)


def _jpeg(frame: np.ndarray) -> np.ndarray:
    ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 40])
    if not ok:
        raise RuntimeError("JPEG robustness transform failed")
    decoded = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
    if decoded is None:
        raise RuntimeError("JPEG robustness transform decode failed")
    return decoded


TRANSFORMS: dict[str, Callable[[np.ndarray], np.ndarray]] = {
    "original": lambda frame: frame,
    "dark": lambda frame: _clip(frame, 0.65, -8.0),
    "bright": lambda frame: _clip(frame, 1.20, 15.0),
    "high_contrast": lambda frame: _clip(frame, 1.35, -45.0),
    "blur_5x5": lambda frame: cv2.GaussianBlur(frame, (5, 5), 0.0),
    "jpeg_q40": _jpeg,
}


def _load_ground_truth(label_path: Path, width: int, height: int, names: list[str]) -> list[tuple[str, tuple[float, float, float, float]]]:
    rows = []
    for line_no, raw in enumerate(label_path.read_text(encoding="utf-8").splitlines(), 1):
        fields = raw.split()
        if not fields:
            continue
        if len(fields) != 5:
            raise ValueError(f"{label_path}:{line_no}: expected 5 fields")
        class_id = int(fields[0])
        cx, cy, box_w, box_h = (float(value) for value in fields[1:])
        x1 = (cx - box_w / 2.0) * width
        y1 = (cy - box_h / 2.0) * height
        x2 = (cx + box_w / 2.0) * width
        y2 = (cy + box_h / 2.0) * height
        rows.append((names[class_id], (x1, y1, x2, y2)))
    return rows


def _iou(first: tuple[float, float, float, float], second: tuple[float, float, float, float]) -> float:
    x1 = max(first[0], second[0])
    y1 = max(first[1], second[1])
    x2 = min(first[2], second[2])
    y2 = min(first[3], second[3])
    intersection = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    first_area = max(0.0, first[2] - first[0]) * max(0.0, first[3] - first[1])
    second_area = max(0.0, second[2] - second[0]) * max(0.0, second[3] - second[1])
    union = first_area + second_area - intersection
    return intersection / union if union > 0.0 else 0.0


def _match(ground_truth: list[tuple[str, tuple[float, float, float, float]]], detections: list[Detection]) -> tuple[int, int, int]:
    candidates: list[tuple[float, int, int]] = []
    for truth_index, (truth_name, truth_box) in enumerate(ground_truth):
        for detection_index, detection in enumerate(detections):
            if detection.name != truth_name:
                continue
            score = _iou(truth_box, (detection.x1, detection.y1, detection.x2, detection.y2))
            if score >= 0.5:
                candidates.append((score, truth_index, detection_index))
    matched_truth: set[int] = set()
    matched_detections: set[int] = set()
    for _, truth_index, detection_index in sorted(candidates, reverse=True):
        if truth_index in matched_truth or detection_index in matched_detections:
            continue
        matched_truth.add(truth_index)
        matched_detections.add(detection_index)
    true_positive = len(matched_truth)
    return true_positive, len(detections) - true_positive, len(ground_truth) - true_positive


def _metrics(tp: int, fp: int, fn: int) -> dict[str, float | int]:
    precision = tp / (tp + fp) if tp + fp else 0.0
    recall = tp / (tp + fn) if tp + fn else 0.0
    f1 = 2.0 * precision * recall / (precision + recall) if precision + recall else 0.0
    return {"tp": tp, "fp": fp, "fn": fn, "precision": precision, "recall": recall, "f1": f1}


def evaluate(model_path: Path, images: list[Path], labels_dir: Path, conf: float) -> dict:
    detector = OpenCVDnnYolo(model_path, imgsz=640, conf_thres=conf)
    totals = {name: {"tp": 0, "fp": 0, "fn": 0, "latency_ms": []} for name in TRANSFORMS}
    for image_path in images:
        raw = np.fromfile(str(image_path), dtype=np.uint8)
        frame = cv2.imdecode(raw, cv2.IMREAD_COLOR) if raw.size else None
        if frame is None:
            raise ValueError(f"cannot decode {image_path}")
        truth = _load_ground_truth(labels_dir / f"{image_path.stem}.txt", frame.shape[1], frame.shape[0], detector.names)
        for variant, transform in TRANSFORMS.items():
            transformed = transform(frame)
            started = time.perf_counter()
            detections = detector.detect(transformed)
            elapsed_ms = (time.perf_counter() - started) * 1000.0
            tp, fp, fn = _match(truth, detections)
            totals[variant]["tp"] += tp
            totals[variant]["fp"] += fp
            totals[variant]["fn"] += fn
            totals[variant]["latency_ms"].append(elapsed_ms)
    variants = {}
    aggregate_tp = aggregate_fp = aggregate_fn = 0
    for name, row in totals.items():
        aggregate_tp += row["tp"]
        aggregate_fp += row["fp"]
        aggregate_fn += row["fn"]
        variants[name] = {
            **_metrics(row["tp"], row["fp"], row["fn"]),
            "mean_latency_ms": float(np.mean(row["latency_ms"])),
        }
    return {
        "model": str(model_path.resolve()),
        "names": detector.names,
        "image_count": len(images),
        "confidence_threshold": conf,
        "iou_threshold": 0.5,
        "variants": variants,
        "aggregate": _metrics(aggregate_tp, aggregate_fp, aggregate_fn),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--models", required=True, help="comma-separated label=path entries")
    parser.add_argument("--images-dir", type=Path, required=True)
    parser.add_argument("--labels-dir", type=Path, required=True)
    parser.add_argument("--conf", type=float, default=0.10)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not 0.0 < args.conf < 1.0:
        parser.error("conf must be in (0, 1)")

    images_dir = args.images_dir if args.images_dir.is_absolute() else PROJECT_ROOT / args.images_dir
    labels_dir = args.labels_dir if args.labels_dir.is_absolute() else PROJECT_ROOT / args.labels_dir
    images = sorted(path for path in images_dir.iterdir() if path.suffix.lower() in IMAGE_SUFFIXES)
    if not images:
        raise FileNotFoundError(f"no images in {images_dir}")

    models = {}
    for entry in (value.strip() for value in args.models.split(",") if value.strip()):
        label, separator, raw_path = entry.partition("=")
        if not separator or not label or not raw_path:
            parser.error("models must use label=path entries")
        path = Path(raw_path)
        path = path if path.is_absolute() else PROJECT_ROOT / path
        if not path.is_file():
            raise FileNotFoundError(path)
        models[label] = evaluate(path.resolve(), images, labels_dir.resolve(), args.conf)

    report = {
        "offline": True,
        "hardware_motion": False,
        "camera_opened": False,
        "serial_opened": False,
        "can_opened": False,
        "transformations": list(TRANSFORMS),
        "models": models,
        "warning": "Metrics are a relative 20-image photometric regression, not a new independent camera dataset.",
    }
    output = args.output if args.output.is_absolute() else PROJECT_ROOT / args.output
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
