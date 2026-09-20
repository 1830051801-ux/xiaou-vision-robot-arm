#!/usr/bin/env python3
"""Measure the no-motion YOLO plus INT8 policy path on a Raspberry Pi.

This tool intentionally accepts one local smoke image and runs only CPU ONNX
inference.  It neither opens a camera nor imports the arm UART/CAN modules.
Use it after staging a Pi release to replace the desktop RAM estimate with a
measured RSS and latency report.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import platform
import sys
import time
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


def _rss_mb() -> float | None:
    """Return current process RSS on Linux without adding a dependency."""

    status = Path("/proc/self/status")
    if status.is_file():
        for line in status.read_text(encoding="utf-8").splitlines():
            if line.startswith("VmRSS:"):
                fields = line.split()
                if len(fields) >= 2:
                    return float(fields[1]) / 1024.0
    return None


def _percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    if not ordered:
        raise ValueError("cannot calculate percentile of no values")
    index = max(0, min(len(ordered) - 1, int(round((len(ordered) - 1) * fraction))))
    return float(ordered[index])


def profile(
    model: Path,
    policy_path: Path,
    image: Path,
    *,
    iterations: int,
    warmup: int,
    cv_threads: int,
    max_rss_mb: float,
    task_class: str,
) -> dict[str, Any]:
    if iterations < 1 or warmup < 0 or cv_threads < 1 or max_rss_mb <= 0.0:
        raise ValueError("iterations, cv_threads, and max_rss_mb must be positive; warmup must be non-negative")
    if not model.is_file() or not policy_path.is_file() or not image.is_file():
        raise FileNotFoundError(f"model={model}, policy={policy_path}, image={image}")
    import cv2  # type: ignore
    import numpy as np  # type: ignore

    from robot_ai.decision.onnx_policy import OnnxDecisionTransformerPolicy
    from robot_ai.decision.transformer_policy import TASK_TO_ID
    from robot_ai.yolo_opencv import OpenCVDnnYolo

    if task_class not in TASK_TO_ID:
        raise ValueError(f"task_class must be one of {sorted(TASK_TO_ID)}")
    cv2.setNumThreads(cv_threads)
    raw = np.fromfile(str(image), dtype=np.uint8)
    frame = cv2.imdecode(raw, cv2.IMREAD_COLOR) if raw.size else None
    if frame is None:
        raise ValueError(f"cannot decode image: {image}")
    rss_before = _rss_mb()
    detector = OpenCVDnnYolo(model, imgsz=640, conf_thres=0.10)
    policy = OnnxDecisionTransformerPolicy(policy_path, max_seq_len=16)
    if policy.metadata.get("hardware_motion") is not False:
        raise RuntimeError("policy metadata must keep hardware_motion false")
    observation = np.zeros((1, 46), dtype=np.float32)
    observation[0, 0] = 1.0
    for _ in range(warmup):
        detector.detect(frame)
        policy.predict(observation, TASK_TO_ID[task_class])
    latencies: list[float] = []
    detections: list[int] = []
    peak_rss = rss_before or 0.0
    for _ in range(iterations):
        started = time.perf_counter()
        result = detector.detect(frame)
        policy.predict(observation, TASK_TO_ID[task_class])
        latencies.append((time.perf_counter() - started) * 1000.0)
        detections.append(len(result))
        current_rss = _rss_mb()
        if current_rss is not None:
            peak_rss = max(peak_rss, current_rss)
    rss_after = _rss_mb()
    if rss_after is not None:
        peak_rss = max(peak_rss, rss_after)
    p95 = _percentile(latencies, 0.95)
    rss_measured = rss_before is not None and rss_after is not None
    platform_info = {"system": platform.system(), "machine": platform.machine(), "python": sys.version.split()[0]}
    pi_linux_runtime = platform_info["system"] == "Linux" and rss_measured
    memory_budget_passed = bool(rss_measured and peak_rss <= max_rss_mb)
    return {
        "schema": "xiaou_pi_inference_profile_v1",
        "offline": True,
        "hardware_motion": False,
        "camera_opened": False,
        "serial_opened": False,
        "can_opened": False,
        "platform": platform_info,
        "model": {"path": str(model.resolve()), "bytes": model.stat().st_size, "classes": list(detector.names)},
        "policy": {"path": str(policy_path.resolve()), "bytes": policy_path.stat().st_size, "metadata": dict(policy.metadata)},
        "image": str(image.resolve()),
        "iterations": iterations,
        "warmup": warmup,
        "opencv_threads": cv_threads,
        "latency_ms": {
            "min": min(latencies),
            "p50": _percentile(latencies, 0.50),
            "mean": sum(latencies) / len(latencies),
            "p95": p95,
            "max": max(latencies),
        },
        "detections": {"min": min(detections), "max": max(detections), "mean": sum(detections) / len(detections)},
        "rss_mb": {
            "before": rss_before,
            "after": rss_after,
            "peak": peak_rss if rss_measured else None,
            "limit": max_rss_mb,
            "measured": rss_measured,
            "within_limit": memory_budget_passed if rss_measured else None,
        },
        "deployment_readiness": {
            "pi_linux_rss_measured": pi_linux_runtime,
            "status": "pi_measurement_complete" if pi_linux_runtime else "desktop_reference_only",
        },
        "passed": memory_budget_passed,
        "note": "Only a Linux run with measured /proc RSS can pass the Pi memory gate. Run this while the intended ROS2/background services are also running before approving a 2 GB deployment.",
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=PROJECT_ROOT / "models/xiaou_objects_gpu_deep.onnx")
    parser.add_argument(
        "--policy",
        type=Path,
        default=PROJECT_ROOT / "runtime/decision/transformer_policy_safety_final_20260813.int8.onnx",
    )
    parser.add_argument("--image", type=Path, required=True, help="local Pi smoke image; no camera device is opened")
    parser.add_argument("--task-class", default="cola")
    parser.add_argument("--iterations", type=int, default=20)
    parser.add_argument("--warmup", type=int, default=3)
    parser.add_argument("--cv-threads", type=int, default=1)
    parser.add_argument("--max-rss-mb", type=float, default=1024.0)
    parser.add_argument("--output", type=Path, default=Path("runtime/pi_inference_profile.json"))
    args = parser.parse_args()
    model = args.model if args.model.is_absolute() else PROJECT_ROOT / args.model
    policy = args.policy if args.policy.is_absolute() else PROJECT_ROOT / args.policy
    image = args.image if args.image.is_absolute() else PROJECT_ROOT / args.image
    output = args.output if args.output.is_absolute() else PROJECT_ROOT / args.output
    report = profile(
        model.resolve(), policy.resolve(), image.resolve(), iterations=args.iterations, warmup=args.warmup,
        cv_threads=args.cv_threads, max_rss_mb=args.max_rss_mb, task_class=args.task_class,
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
