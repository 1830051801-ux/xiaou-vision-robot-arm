"""Create and validate a six-axis calibration record without touching hardware."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def _six(values: str | None, cast, label: str):
    if values is None:
        return [None] * 6
    items = [cast(item.strip()) for item in values.split(",")]
    if len(items) != 6:
        raise ValueError(f"{label} requires exactly six comma-separated values")
    return items


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--ids", default="1,2,3,4,5,6")
    parser.add_argument("--zero-offset-rad")
    parser.add_argument("--direction")
    parser.add_argument("--position-min-rad")
    parser.add_argument("--position-max-rad")
    parser.add_argument("--velocity-max-rad-s")
    parser.add_argument("--acceleration-max-rad-s2")
    args = parser.parse_args()
    ids = _six(args.ids, int, "ids")
    if any(value < 1 or value > 63 for value in ids) or len(set(ids)) != 6:
        raise ValueError("ids must be six unique values in 1..63")
    directions = _six(args.direction, int, "direction")
    if any(value is not None and value not in (-1, 1) for value in directions):
        raise ValueError("direction values must be -1 or +1")
    record = {
        "schema_version": 1,
        "motion_enabled": False,
        "protocol_confirmed": False,
        "estop_verified": False,
        "feedback_verified": False,
        "protocol_family": "XiaoU_CAN_V1",
        "can_interface": None,
        "can_bitrate": None,
        "joint_node_ids": ids,
        "encoder_zero_offset_rad": _six(args.zero_offset_rad, float, "zero-offset-rad"),
        "encoder_direction": directions,
        "position_min_rad": _six(args.position_min_rad, float, "position-min-rad"),
        "position_max_rad": _six(args.position_max_rad, float, "position-max-rad"),
        "velocity_max_rad_s": _six(args.velocity_max_rad_s, float, "velocity-max-rad-s"),
        "acceleration_max_rad_s2": _six(args.acceleration_max_rad_s2, float, "acceleration-max-rad-s2"),
        "measurement_status": "draft_only_no_hardware_access",
        "notes": "Fill only from supervised single-axis measurements; keep motion locks false until review.",
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(record, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(record, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
