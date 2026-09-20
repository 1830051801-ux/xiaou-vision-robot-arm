#!/usr/bin/env python3
"""Run several seeded physical grasp regressions and merge their summaries.

``simulate_physical_grasp.py`` already samples table positions and tool yaw for
each trial.  This wrapper runs independent seeds so a single fixed trajectory
cannot hide a collision or IK boundary.  The underlying MuJoCo runner remains
offline and keeps serial/CAN/ROS hardware disabled.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[1]
PHYSICAL_RUNNER = PROJECT_ROOT / "tools" / "simulate_physical_grasp.py"


def _summary(report: dict[str, Any]) -> dict[str, Any]:
    scenarios = []
    for item in report.get("scenarios", []):
        scenarios.append({
            "scenario": item.get("scenario"),
            "trials": item.get("trials", 0),
            "passed": item.get("passed", 0),
            "success_rate": item.get("success_rate", 0.0),
            "failure_counts": item.get("failure_counts", {}),
            "warning_counts": item.get("warning_counts", {}),
            "max_sampling_attempts_used": max(
                (int(row.get("sampling_attempts", 0)) for row in item.get("trial_records", [])),
                default=0,
            ),
        })
    return {
        "seed": report.get("seed"),
        "total_trials": report.get("total_trials", 0),
        "total_passed": report.get("total_passed", 0),
        "success_rate": report.get("success_rate", 0.0),
        "hardware_motion": report.get("hardware_motion", True),
        "serial_opened": report.get("serial_opened", True),
        "can_opened": report.get("can_opened", True),
        "ros_hardware_started": report.get("ros_hardware_started", True),
        "scenarios": scenarios,
    }


def run_seed(
    seed: int,
    *,
    scenarios: str,
    trials: int,
    max_sampling_attempts: int,
    geometry_jitter_pct: float,
    fp32: Path,
    int8: Path,
    hardware_config: Path,
    run_output: Path,
    ascii_asset_root: Path,
) -> dict[str, Any]:
    command = [
        sys.executable,
        str(PHYSICAL_RUNNER),
        "--trials-per-scenario",
        str(trials),
        "--max-sampling-attempts",
        str(max_sampling_attempts),
        "--geometry-jitter-pct",
        str(geometry_jitter_pct),
        "--seed",
        str(seed),
        "--scenarios",
        scenarios,
        "--fp32-checkpoint",
        str(fp32),
        "--int8-model",
        str(int8),
        "--hardware-config",
        str(hardware_config),
        "--output",
        str(run_output),
        "--ascii-asset-dir",
        str(ascii_asset_root / f"seed_{seed}"),
        "--asset-dir",
        str(PROJECT_ROOT / "runtime/simulations/randomized_assets" / f"seed_{seed}"),
    ]
    completed = subprocess.run(command, cwd=PROJECT_ROOT, capture_output=True, text=True, check=False)
    if not run_output.is_file():
        raise RuntimeError(
            f"physical runner did not write {run_output}; exit={completed.returncode}\n"
            f"stderr={completed.stderr[-2000:]}"
        )
    report = json.loads(run_output.read_text(encoding="utf-8"))
    summary = _summary(report)
    summary["runner_exit_code"] = completed.returncode
    summary["runner_stderr_tail"] = completed.stderr[-1000:]
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seeds", default="20260809,20260810")
    parser.add_argument("--scenarios", default="bottle,cup,pen,desktop_item,tissue_pull,cola")
    parser.add_argument("--trials-per-scenario", type=int, default=4)
    parser.add_argument("--max-sampling-attempts", type=int, default=200)
    parser.add_argument("--geometry-jitter-pct", type=float, default=0.05)
    parser.add_argument("--fp32", type=Path, default=PROJECT_ROOT / "runtime/decision/transformer_policy_safety_seed20260832.pt")
    parser.add_argument("--int8", type=Path, default=PROJECT_ROOT / "runtime/decision/transformer_policy_safety_final_20260813.int8.onnx")
    parser.add_argument(
        "--hardware-config",
        type=Path,
        default=PROJECT_ROOT / "robot_ai" / "arm_control" / "config" / "hardware_calibration_candidate_20260811.json",
        help="no-motion Pi/F407 candidate configuration passed to every MuJoCo seed",
    )
    parser.add_argument("--output", type=Path, default=PROJECT_ROOT / "runtime/simulations/randomized_physical_regression_20260809.json")
    parser.add_argument("--run-dir", type=Path, default=PROJECT_ROOT / "runtime/simulations/randomized_runs")
    parser.add_argument("--ascii-asset-root", type=Path, default=Path("D:/xiaou_mujoco_randomized"))
    args = parser.parse_args()
    seeds = [int(item.strip()) for item in args.seeds.split(",") if item.strip()]
    if not seeds or args.trials_per_scenario < 1 or args.max_sampling_attempts < 1:
        raise ValueError("seeds, trials-per-scenario, and max-sampling-attempts must be positive")
    fp32 = args.fp32 if args.fp32.is_absolute() else PROJECT_ROOT / args.fp32
    int8 = args.int8 if args.int8.is_absolute() else PROJECT_ROOT / args.int8
    hardware_config = args.hardware_config if args.hardware_config.is_absolute() else PROJECT_ROOT / args.hardware_config
    if not fp32.is_file() or not int8.is_file():
        raise FileNotFoundError(f"missing policy: {fp32} / {int8}")
    if not hardware_config.is_file():
        raise FileNotFoundError(f"missing hardware config: {hardware_config}")
    run_dir = args.run_dir if args.run_dir.is_absolute() else PROJECT_ROOT / args.run_dir
    run_dir.mkdir(parents=True, exist_ok=True)
    runs = []
    for seed in seeds:
        run_output = run_dir / f"physical_seed_{seed}.json"
        runs.append(
            run_seed(
                seed,
                scenarios=args.scenarios,
                trials=args.trials_per_scenario,
                max_sampling_attempts=args.max_sampling_attempts,
                geometry_jitter_pct=args.geometry_jitter_pct,
                fp32=fp32.resolve(),
                int8=int8.resolve(),
                hardware_config=hardware_config.resolve(),
                run_output=run_output,
                ascii_asset_root=args.ascii_asset_root,
            )
        )
    total_trials = sum(int(item["total_trials"]) for item in runs)
    total_passed = sum(int(item["total_passed"]) for item in runs)
    hardware_safe = all(
        item["hardware_motion"] is False
        and item["serial_opened"] is False
        and item["can_opened"] is False
        and item["ros_hardware_started"] is False
        for item in runs
    )
    report = {
        "offline": True,
        "hardware_motion": False,
        "randomization": {
            "independent_seeds": seeds,
            "placement_and_yaw_sampling": "delegated to simulate_physical_grasp.py per trial",
            "position_range_m": {"x": [0.235, 0.315], "y": [-0.155, -0.085]},
            "yaw_range_deg": [-3.0, 3.0],
            "geometry_jitter": True,
            "geometry_jitter_pct": args.geometry_jitter_pct,
            "geometry_note": "Provisional +/- jitter is used for offline robustness only; replace with measured object dimensions before real-arm use.",
        },
        "hardware_motion_profile": str(hardware_config.resolve()),
        "seeds": runs,
        "total_trials": total_trials,
        "total_passed": total_passed,
        "success_rate": total_passed / max(1, total_trials),
        "hardware_safe": hardware_safe,
        "passed": total_passed == total_trials and hardware_safe,
    }
    output = args.output if args.output.is_absolute() else PROJECT_ROOT / args.output
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
