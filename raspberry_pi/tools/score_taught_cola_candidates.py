#!/usr/bin/env python3
"""Rank every reachable taught-cola side approach with offline MuJoCo replay.

The geometric planner can identify several IK-reachable approach directions,
but only physics replay can expose collision-mesh contacts along their joint
interpolations.  This tool deliberately performs no UART, CAN, ROS, gripper,
or actuator I/O.  It writes a truthful ``selected_candidate`` only when the
best candidate has no sampled forbidden or premature object contact.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from collections import Counter
from pathlib import Path
from typing import Any

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import simulate_physical_grasp as physics
from simulate_taught_cola_grasp import TAUGHT_COLA_GRASP_DEG

from robot_ai.arm_control import (
    TaughtGraspPlan,
    build_taught_side_grasp_candidates,
    load_default_model,
)

DEFAULT_OUTPUT = PROJECT_ROOT / "runtime" / "simulations" / "taught_cola_candidate_score_20260921.json"
DEFAULT_ASCII_ROOT = Path("D:/xiaou_mujoco_taught_candidate_score")


def _finite_positive(value: float, label: str) -> float:
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{label} must be finite and positive")
    return result


def _score_candidate(
    simulator: Any,
    plan: TaughtGraspPlan,
) -> dict[str, Any]:
    """Replay one candidate and expose all collision evidence for ranking."""

    simulator.reset(
        float(plan.object_center_m[0]),
        float(plan.object_center_m[1]),
        robot_joint_rad=plan.start_joint_rad,
    )
    forbidden_pairs: Counter[str] = Counter()
    premature_target_pairs: Counter[str] = Counter()
    stage_records: list[dict[str, Any]] = []
    forbidden_samples = 0
    premature_target_samples = 0
    for index, stage in enumerate(plan.stages):
        motion = simulator.move_to(np.asarray(stage.target_joint_rad, dtype=np.float64))
        forbidden_samples += int(motion["forbidden_contact_samples"])
        forbidden_pairs.update(motion["forbidden_contact_pairs"])
        if index < len(plan.stages) - 1:
            premature_target_samples += int(motion["target_contact_samples"])
            premature_target_pairs.update(motion["target_contact_pairs"])
        stage_records.append(
            {
                "name": stage.name,
                "phase": stage.phase,
                "motion": motion,
            }
        )
    admissible = forbidden_samples == 0 and premature_target_samples == 0
    return {
        "candidate_label": plan.candidate_label,
        "admissible": admissible,
        "ranking": {
            "forbidden_contact_samples": forbidden_samples,
            "premature_target_contact_samples": premature_target_samples,
            "sampled_transit_min_clearance_m": plan.sampled_transit_min_clearance_m,
            "stage_count": len(plan.stages),
        },
        "forbidden_contact_pairs": dict(forbidden_pairs),
        "premature_target_contact_pairs": dict(premature_target_pairs),
        "plan": plan.as_dict(),
        "stages": stage_records,
    }


def _rank_key(item: dict[str, Any]) -> tuple[int, int, int, float, int, str]:
    ranking = item["ranking"]
    return (
        0 if item["admissible"] else 1,
        int(ranking["forbidden_contact_samples"]),
        int(ranking["premature_target_contact_samples"]),
        -float(ranking["sampled_transit_min_clearance_m"]),
        int(ranking["stage_count"]),
        str(item["candidate_label"]),
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--hardware-config",
        type=Path,
        default=PROJECT_ROOT / "robot_ai" / "arm_control" / "config" / "hardware_calibration_candidate_20260811.json",
    )
    parser.add_argument("--cola-height-m", type=float, default=0.190)
    parser.add_argument("--cola-radius-m", type=float, default=0.032)
    parser.add_argument("--pregrasp-clearance-m", type=float, default=0.120)
    parser.add_argument("--contact-handoff-clearance-m", type=float, default=0.066)
    parser.add_argument("--overhead-clearance-m", type=float, default=0.040)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--ascii-asset-root", type=Path, default=DEFAULT_ASCII_ROOT)
    args = parser.parse_args()
    for label, value in (
        ("cola-height-m", args.cola_height_m),
        ("cola-radius-m", args.cola_radius_m),
        ("pregrasp-clearance-m", args.pregrasp_clearance_m),
        ("contact-handoff-clearance-m", args.contact_handoff_clearance_m),
        ("overhead-clearance-m", args.overhead_clearance_m),
    ):
        _finite_positive(value, label)

    config_path = args.hardware_config if args.hardware_config.is_absolute() else PROJECT_ROOT / args.hardware_config
    profile = physics.load_simulation_motion_profile(config_path.resolve())
    runtime = physics._require_mujoco()
    lower_deg = [math.degrees(float(value)) for value in profile.joint_limits.position_min]
    upper_deg = [math.degrees(float(value)) for value in profile.joint_limits.position_max]
    ready_deg = [math.degrees(float(value)) for value in profile.ready_pose_rad]
    candidates = build_taught_side_grasp_candidates(
        ready_deg,
        TAUGHT_COLA_GRASP_DEG,
        model=load_default_model(),
        lower_deg=lower_deg,
        upper_deg=upper_deg,
        object_height_m=args.cola_height_m,
        object_radius_m=args.cola_radius_m,
        pregrasp_clearance_m=args.pregrasp_clearance_m,
        contact_handoff_clearance_m=args.contact_handoff_clearance_m,
        overhead_clearance_m=args.overhead_clearance_m,
    )
    description = physics.parse_robot_description()
    staged = physics.stage_ascii_assets(description, "taught_candidate_score", args.ascii_asset_root)
    object_spec: dict[str, float | str] = {
        "shape": "cylinder",
        "radius_m": float(args.cola_radius_m),
        "height_m": float(args.cola_height_m),
        "mass_kg": 0.40,
        "friction": 0.55,
        "strategy": "side_wrap",
        "grasp_mode": "side",
    }
    first_plan = candidates[0]
    xml_path = physics.build_mjcf(
        staged,
        "taught_cola_candidate_score",
        args.ascii_asset_root / "cola",
        object_spec,
        table_top_m=first_plan.table_z_m,
    )
    rows: list[dict[str, Any]] = []
    for plan in candidates:
        simulator = physics.PhysicalEpisode(
            xml_path,
            "cola",
            object_spec,
            table_top_m=plan.table_z_m,
            joint_limits=profile.joint_limits,
        )
        rows.append(_score_candidate(simulator, plan))
    ranked = sorted(rows, key=_rank_key)
    selected = ranked[0] if ranked and bool(ranked[0]["admissible"]) else None
    report = {
        "schema": "xiaou_taught_cola_candidate_score_v1",
        "offline": True,
        "hardware_motion": False,
        "serial_opened": False,
        "can_opened": False,
        "ros_hardware_started": False,
        "physics_engine": {"name": "MuJoCo", "version": getattr(runtime, "__version__", "unknown")},
        "candidate_count": len(rows),
        "selection_status": "selected_offline_candidate" if selected is not None else "blocked_by_self_collision",
        "selection_rule": [
            "zero forbidden contact samples",
            "zero premature target-contact samples",
            "highest sampled transit clearance",
            "fewest stages",
        ],
        "selected_candidate": None if selected is None else selected["candidate_label"],
        "best_candidate_diagnostics": ranked[0] if ranked else None,
        "candidates_ranked": ranked,
        "limitations": [
            "This ranks approximate CAD collision meshes and a primitive cola cylinder only.",
            "It does not verify jaw force, object retention, camera calibration, or real actuator tracking.",
            "A blocked result requires a new teach pose or corrected collision model; it is not converted into a motion command.",
        ],
    }
    output = args.output if args.output.is_absolute() else PROJECT_ROOT / args.output
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({
        "offline": True,
        "candidate_count": report["candidate_count"],
        "selection_status": report["selection_status"],
        "selected_candidate": report["selected_candidate"],
        "output": str(output),
    }, ensure_ascii=False, indent=2))
    return 0 if selected is not None else 2


if __name__ == "__main__":
    raise SystemExit(main())
