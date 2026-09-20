#!/usr/bin/env python3
"""Map conservative local retargeting bounds for one taught side grasp.

The output is an offline planning characterization, not a camera calibration
and not a promise of physical grasp success.  It samples full Cartesian
envelope corners rather than only one-axis offsets, so a configured rectangular
reuse envelope can be backed by an explicit no-I/O IK check.
"""

from __future__ import annotations

import argparse
from itertools import product
import json
import math
from pathlib import Path
import sys

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from robot_ai.arm_control import TaughtGraspPlanningError, build_taught_side_grasp_plan, load_default_model  # noqa: E402


READY_DEG = [0.0, -50.0, -55.0, -70.0, 110.0, 0.0]
TAUGHT_COLA_GRASP_DEG = [-100.000244140625, -48.93047332763672, -71.99970245361328, -84.9998664855957, 110.00060272216797, 0.0]


def _limits(path: Path) -> tuple[list[float], list[float]]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    return (
        [math.degrees(float(value)) for value in payload["position_min_rad"]],
        [math.degrees(float(value)) for value in payload["position_max_rad"]],
    )


def _positive(value: float, label: str) -> float:
    value = float(value)
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"{label} must be finite and positive")
    return value


def _parse_envelopes(value: str) -> list[tuple[float, float, float]]:
    envelopes: list[tuple[float, float, float]] = []
    for item in value.split(";"):
        fields = [piece.strip() for piece in item.split(",") if piece.strip()]
        if len(fields) != 3:
            raise ValueError("each envelope must be x,y,z millimetres")
        envelopes.append(tuple(_positive(float(piece) / 1000.0, "envelope axis") for piece in fields))
    if not envelopes:
        raise ValueError("at least one envelope is required")
    return envelopes


def _offset_points(envelope: tuple[float, float, float]) -> list[tuple[float, float, float]]:
    # Zero plus all 8 corners catches coupled X/Y/Z branch failures that a
    # separate +/- axis scan would miss.
    corners = list(product(*((-axis, axis) for axis in envelope)))
    return [(0.0, 0.0, 0.0), *corners]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--hardware-config",
        type=Path,
        default=PROJECT_ROOT / "robot_ai" / "arm_control" / "config" / "hardware_calibration_candidate_20260811.json",
    )
    parser.add_argument(
        "--envelopes-mm",
        default="0.25,0.25,0.25;0.5,0.5,0.5;1,1,0.5;2,1,0.5;3,2,1",
        help="semicolon-separated symmetric x,y,z envelope candidates in mm",
    )
    parser.add_argument("--radius-variation-mm", type=float, default=0.0)
    parser.add_argument("--height-variation-mm", type=float, default=0.0)
    parser.add_argument(
        "--output",
        type=Path,
        default=PROJECT_ROOT / "runtime" / "simulations" / "taught_side_grasp_envelope_20260813.json",
    )
    args = parser.parse_args()
    envelopes = _parse_envelopes(args.envelopes_mm)
    radius_variation = _positive(abs(args.radius_variation_mm) / 1000.0 + 1e-12, "radius variation") - 1e-12
    height_variation = _positive(abs(args.height_variation_mm) / 1000.0 + 1e-12, "height variation") - 1e-12
    lower, upper = _limits(args.hardware_config)
    model = load_default_model()
    rows = []
    for envelope in envelopes:
        cases = []
        for offset in _offset_points(envelope):
            for radius_delta, height_delta in product(
                ((0.0,) if radius_variation == 0.0 else (-radius_variation, radius_variation)),
                ((0.0,) if height_variation == 0.0 else (-height_variation, height_variation)),
            ):
                radius = 0.032 + radius_delta
                height = 0.190 + height_delta
                record = {
                    "offset_base_m": list(offset),
                    "object_radius_m": radius,
                    "object_height_m": height,
                }
                try:
                    plan = build_taught_side_grasp_plan(
                        READY_DEG,
                        TAUGHT_COLA_GRASP_DEG,
                        model=model,
                        lower_deg=lower,
                        upper_deg=upper,
                        object_radius_m=radius,
                        object_height_m=height,
                        taught_contact_offset_base_m=offset,
                    )
                    record.update({
                        "passed": True,
                        "stage_count": len(plan.stages),
                        "candidate": plan.candidate_label,
                        "sampled_transit_min_clearance_m": plan.sampled_transit_min_clearance_m,
                    })
                except TaughtGraspPlanningError as exc:
                    record.update({"passed": False, "reason": str(exc)})
                cases.append(record)
        rows.append({
            "envelope_half_extent_m": list(envelope),
            "case_count": len(cases),
            "passed": sum(bool(case["passed"]) for case in cases),
            "failed": sum(not bool(case["passed"]) for case in cases),
            "all_corners_reachable": all(bool(case["passed"]) for case in cases),
            "cases": cases,
        })
    passing = [row for row in rows if row["all_corners_reachable"]]
    report = {
        "schema": "xiaou_taught_side_grasp_envelope_v1",
        "offline": True,
        "hardware_motion": False,
        "serial_opened": False,
        "can_opened": False,
        "ros_hardware_started": False,
        "method": "zero-plus-eight-corners Cartesian retargeting with the exact ready pose and current effective joint limits",
        "dimensions_tested": {
            "reference_radius_m": 0.032,
            "reference_height_m": 0.190,
            "radius_variation_m": radius_variation,
            "height_variation_m": height_variation,
        },
        "results": rows,
        "largest_tested_all_corner_envelope_m": passing[-1]["envelope_half_extent_m"] if passing else None,
        "limitations": [
            "This checks planner reachability only, not camera error, gripper force, actuator tracking, or real collision clearance.",
            "The start state is the exact taught ready pose. A different start pose needs a new plan and validation.",
        ],
    }
    output = args.output if args.output.is_absolute() else PROJECT_ROOT / args.output
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({
        "offline": True,
        "largest_tested_all_corner_envelope_m": report["largest_tested_all_corner_envelope_m"],
        "results": [{
            "envelope_half_extent_m": row["envelope_half_extent_m"],
            "passed": row["passed"],
            "failed": row["failed"],
            "all_corners_reachable": row["all_corners_reachable"],
        } for row in rows],
        "output": str(output),
    }, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
