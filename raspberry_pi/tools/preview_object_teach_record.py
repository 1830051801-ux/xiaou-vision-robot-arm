#!/usr/bin/env python3
"""Turn a validated object-specific teach record into an offline trajectory preview.

The input must contain read-only captured poses for one object family.  This
tool does not open UART, CAN, ROS, a camera, or a gripper; it only validates
the record and writes piecewise quintic timing metadata for review.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from robot_ai.arm_control import build_taught_waypoint_preview  # noqa: E402


def _deg(values: object, label: str) -> list[float]:
    if not isinstance(values, list) or len(values) != 6:
        raise ValueError(f"{label} must be a six-value list")
    result = [math.degrees(float(value)) for value in values]
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{label} must be finite")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--record", type=Path, required=True, help="xiaou_object_teach_record_v1 JSON from read-only capture")
    parser.add_argument(
        "--hardware-config",
        type=Path,
        default=PROJECT_ROOT / "robot_ai/arm_control/config/hardware_calibration_candidate_20260811.json",
        help="measured/candidate effective joint limits used only for preview",
    )
    parser.add_argument("--sample-period-s", type=float, default=0.05)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not math.isfinite(args.sample_period_s) or args.sample_period_s <= 0.0:
        parser.error("--sample-period-s must be finite and positive")
    record_path = args.record if args.record.is_absolute() else PROJECT_ROOT / args.record
    config_path = args.hardware_config if args.hardware_config.is_absolute() else PROJECT_ROOT / args.hardware_config
    output = args.output if args.output.is_absolute() else PROJECT_ROOT / args.output
    record = json.loads(record_path.read_text(encoding="utf-8"))
    config = json.loads(config_path.read_text(encoding="utf-8"))
    preview = build_taught_waypoint_preview(
        record,
        position_min_deg=_deg(config.get("position_min_rad"), "position_min_rad"),
        position_max_deg=_deg(config.get("position_max_rad"), "position_max_rad"),
        velocity_max_deg_s=_deg(config.get("velocity_max_rad_s"), "velocity_max_rad_s"),
        acceleration_max_deg_s2=_deg(config.get("acceleration_max_rad_s2"), "acceleration_max_rad_s2"),
        sample_period_s=args.sample_period_s,
    )
    preview["source_record"] = str(record_path.resolve())
    preview["hardware_config"] = str(config_path.resolve())
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(preview, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({
        "planned": True,
        "execution": preview["execution"],
        "object_class": preview["object"]["object_class"],
        "segments": preview["segment_count"],
        "output": str(output),
    }, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
