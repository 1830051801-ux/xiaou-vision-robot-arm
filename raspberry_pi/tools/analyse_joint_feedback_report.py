#!/usr/bin/env python3
"""Analyse a saved J1..J6 read-only diagnostic report without opening devices."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT / "robot_ai") not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT / "robot_ai"))

from arm_control.feedback_quality import analyse_feedback_samples


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("report", type=Path)
    parser.add_argument("--freshness-threshold-s", type=float, default=0.50)
    parser.add_argument("--min-online-ratio", type=float, default=1.0)
    parser.add_argument("--max-p95-latency-ms", type=float)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report_path = args.report if args.report.is_absolute() else PROJECT_ROOT / args.report
    payload = json.loads(report_path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("diagnostic report must be a JSON object")
    samples = payload.get("samples")
    if not isinstance(samples, list):
        raise ValueError("diagnostic report must contain a samples list")
    result = analyse_feedback_samples(
        samples,
        now_monotonic_s=time.monotonic(),
        freshness_threshold_s=args.freshness_threshold_s,
        min_online_ratio=args.min_online_ratio,
        max_p95_latency_ms=args.max_p95_latency_ms,
    )
    result.update({"offline": True, "serial_opened": False, "can_opened": False, "source_report": str(report_path.resolve())})
    if args.output:
        output = args.output if args.output.is_absolute() else PROJECT_ROOT / args.output
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
