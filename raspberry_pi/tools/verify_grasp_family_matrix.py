#!/usr/bin/env python3
"""Exercise the offline class-to-grasp-family boundary without hardware I/O."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from robot_ai.arm_control import build_grasp_family_preview, load_default_model  # noqa: E402


READY_DEG = [0.0, -50.0, -55.0, -70.0, 110.0, 0.0]
TAUGHT_COLA_GRASP_DEG = [-100.000244140625, -48.93047332763672, -71.99970245361328, -84.9998664855957, 110.00060272216797, 0.0]


def _limits(path: Path) -> tuple[list[float], list[float]]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    return (
        [math.degrees(float(value)) for value in payload["position_min_rad"]],
        [math.degrees(float(value)) for value in payload["position_max_rad"]],
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--hardware-config",
        type=Path,
        default=PROJECT_ROOT / "robot_ai" / "arm_control" / "config" / "hardware_calibration_candidate_20260811.json",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=PROJECT_ROOT / "runtime" / "simulations" / "grasp_family_matrix_20260813.json",
    )
    args = parser.parse_args()
    lower, upper = _limits(args.hardware_config)
    cases = (
        {"object_class": "cola"},
        {"object_class": "bottle", "object_radius_m": 0.033, "object_height_m": 0.192},
        {"object_class": "bottle", "object_radius_m": 0.045, "object_height_m": 0.240},
        {"object_class": "cup", "object_radius_m": 0.045, "object_height_m": 0.090},
        {"object_class": "pen", "object_radius_m": 0.006, "object_height_m": 0.140},
        {"object_class": "desktop_item", "object_radius_m": 0.035, "object_height_m": 0.030},
        {"object_class": "tissue_pull", "object_radius_m": 0.060, "object_height_m": 0.040},
        {"object_class": "earphone", "object_radius_m": 0.010, "object_height_m": 0.020},
        {"object_class": "unknown_thing", "object_radius_m": 0.020, "object_height_m": 0.100},
        {"object_class": "cola", "target_offset_base_m": [0.004, 0.0, 0.0]},
    )
    rows = []
    for case in cases:
        preview = build_grasp_family_preview(
            case["object_class"],
            READY_DEG,
            TAUGHT_COLA_GRASP_DEG,
            model=load_default_model(),
            lower_deg=lower,
            upper_deg=upper,
            object_radius_m=case.get("object_radius_m"),
            object_height_m=case.get("object_height_m"),
            target_offset_base_m=case.get("target_offset_base_m", (0.0, 0.0, 0.0)),
        )
        row = {"case": case, **preview.as_dict()}
        if preview.plan is not None:
            row["stage_count"] = len(preview.plan.stages)
        rows.append(row)
    counts: dict[str, int] = {}
    for row in rows:
        counts[row["status"]] = counts.get(row["status"], 0) + 1
    report = {
        "schema": "xiaou_grasp_family_matrix_v1",
        "offline": True,
        "hardware_motion": False,
        "serial_opened": False,
        "can_opened": False,
        "ros_hardware_started": False,
        "cases": rows,
        "status_counts": counts,
        "acceptance": {
            "only_local_cylinder_route_ready": all(
                row["status"] != "preview_ready" or row["object_class"] in {"cola", "bottle"}
                for row in rows
            ),
            "non_cylinder_families_require_teach_or_reject": all(
                row["status"] != "preview_ready"
                for row in rows
                if row["object_class"] not in {"cola", "bottle"}
            ),
        },
    }
    output = args.output if args.output.is_absolute() else PROJECT_ROOT / args.output
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({
        "offline": True,
        "status_counts": counts,
        "acceptance": report["acceptance"],
        "output": str(output),
    }, ensure_ascii=False, indent=2))
    return 0 if all(report["acceptance"].values()) else 2


if __name__ == "__main__":
    raise SystemExit(main())
