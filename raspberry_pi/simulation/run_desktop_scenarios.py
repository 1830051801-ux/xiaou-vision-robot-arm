#!/usr/bin/env python3
"""Run deterministic desktop-object task simulations without ROS or hardware."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from desktop_scene import OBJECT_SPECS, run_scenario


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenarios", default=",".join(OBJECT_SPECS), help="comma-separated scenarios")
    parser.add_argument("--seed", type=int, default=20260809)
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()
    scenarios = [value.strip() for value in args.scenarios.split(",") if value.strip()]
    reports = [run_scenario(scenario, args.seed + index) for index, scenario in enumerate(scenarios)]
    report = {"offline": True, "hardware_motion": False, "scenarios": reports}
    payload = json.dumps(report, ensure_ascii=False, indent=2)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload + "\n", encoding="utf-8")
    print(payload)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
