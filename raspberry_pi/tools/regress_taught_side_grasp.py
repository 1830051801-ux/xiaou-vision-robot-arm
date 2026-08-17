#!/usr/bin/env python3
"""Randomized offline MuJoCo regression for a taught cylindrical side grasp.

This tool deliberately stays on the development PC.  It perturbs only the
approximate tabletop/object geometry around one demonstrated final contact,
then verifies that a fresh POE/IK route can be produced and replayed using the
checked-in collision meshes.  It never opens UART, CAN, ROS hardware, or a
gripper channel.

It is not a claim that one cola demonstration can pick every object.  A cup or
bottle needs its own taught endpoint; a pen, tissue, or flat object needs a
different grasp-family planner and demonstration.
"""

from __future__ import annotations

import argparse
from collections import Counter
import json
import math
from pathlib import Path
import sys
from typing import Any

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from robot_ai.arm_control import (  # noqa: E402
    TaughtGraspPlanningError,
    build_taught_side_grasp_plan,
    load_default_model,
)

import simulate_physical_grasp as physics  # noqa: E402
from simulate_taught_cola_grasp import (  # noqa: E402
    TAUGHT_COLA_GRASP_DEG,
)


DEFAULT_OUTPUT = PROJECT_ROOT / "runtime" / "simulations" / "taught_side_grasp_regression_20260813.json"
DEFAULT_ASCII_ROOT = Path("D:/xiaou_mujoco_taught_regression")


def _finite_nonnegative(value: float, label: str) -> float:
    result = float(value)
    if not math.isfinite(result) or result < 0.0:
        raise ValueError(f"{label} must be finite and non-negative")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trials", type=int, default=24)
    parser.add_argument("--seed", type=int, default=20260813)
    parser.add_argument("--radius-jitter-m", type=float, default=0.001)
    parser.add_argument("--height-jitter-m", type=float, default=0.003)
    parser.add_argument("--contact-xy-jitter-m", type=float, default=0.001)
    parser.add_argument("--contact-z-jitter-m", type=float, default=0.0005)
    parser.add_argument("--start-joint-jitter-deg", type=float, default=0.0)
    parser.add_argument("--pregrasp-clearance-m", type=float, default=0.120)
    parser.add_argument("--contact-handoff-clearance-m", type=float, default=0.066)
    parser.add_argument(
        "--hardware-config",
        type=Path,
        default=PROJECT_ROOT / "robot_ai" / "arm_control" / "config" / "hardware_calibration_candidate_20260811.json",
        help="no-motion Pi/F407 candidate configuration that supplies limits and ready pose",
    )
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--ascii-asset-root", type=Path, default=DEFAULT_ASCII_ROOT)
    parser.add_argument(
        "--record-only-collisions",
        action="store_true",
        help="Keep CAD self-contact as an observation and do not count it as a regression failure.",
    )
    args = parser.parse_args()
    if args.trials < 1 or args.trials > 500:
        parser.error("trials must be in 1..500")
    radius_jitter = _finite_nonnegative(args.radius_jitter_m, "radius-jitter-m")
    height_jitter = _finite_nonnegative(args.height_jitter_m, "height-jitter-m")
    contact_xy_jitter = _finite_nonnegative(args.contact_xy_jitter_m, "contact-xy-jitter-m")
    contact_z_jitter = _finite_nonnegative(args.contact_z_jitter_m, "contact-z-jitter-m")
    start_joint_jitter = _finite_nonnegative(args.start_joint_jitter_deg, "start-joint-jitter-deg")
    pregrasp_clearance = _finite_nonnegative(args.pregrasp_clearance_m, "pregrasp-clearance-m")
    contact_handoff_clearance = _finite_nonnegative(
        args.contact_handoff_clearance_m, "contact-handoff-clearance-m"
    )
    if pregrasp_clearance <= contact_handoff_clearance:
        parser.error("pregrasp-clearance-m must exceed contact-handoff-clearance-m")

    config_path = args.hardware_config if args.hardware_config.is_absolute() else PROJECT_ROOT / args.hardware_config
    profile = physics.load_simulation_motion_profile(config_path.resolve())
    runtime = physics._require_mujoco()
    lower_deg = [math.degrees(float(value)) for value in profile.joint_limits.position_min]
    upper_deg = [math.degrees(float(value)) for value in profile.joint_limits.position_max]
    ready_deg = np.degrees(profile.ready_pose_rad)
    description = physics.parse_robot_description()
    staged = physics.stage_ascii_assets(description, "taught_side_regression", args.ascii_asset_root)
    arm_model = load_default_model()
    rng = np.random.default_rng(args.seed)

    trials: list[dict[str, Any]] = []
    failures: Counter[str] = Counter()
    selected_candidates: Counter[str] = Counter()
    self_contact_counts: Counter[str] = Counter()

    for index in range(args.trials):
        radius_m = 0.032 + float(rng.uniform(-radius_jitter, radius_jitter))
        height_m = 0.190 + float(rng.uniform(-height_jitter, height_jitter))
        contact_offset_m = np.asarray(
            (
                rng.uniform(-contact_xy_jitter, contact_xy_jitter),
                rng.uniform(-contact_xy_jitter, contact_xy_jitter),
                rng.uniform(-contact_z_jitter, contact_z_jitter),
            ),
            dtype=np.float64,
        )
        start_deg = ready_deg.copy() + rng.uniform(
            -start_joint_jitter, start_joint_jitter, size=6
        )
        start_deg[5] = ready_deg[5]
        record: dict[str, Any] = {
            "trial": index,
            "object_radius_m": radius_m,
            "object_height_m": height_m,
            "taught_contact_offset_base_m": contact_offset_m.tolist(),
            "start_joint_deg": start_deg.tolist(),
        }
        try:
            plan = build_taught_side_grasp_plan(
                start_deg,
                TAUGHT_COLA_GRASP_DEG,
                model=arm_model,
                lower_deg=lower_deg,
                upper_deg=upper_deg,
                object_radius_m=radius_m,
                object_height_m=height_m,
                pregrasp_clearance_m=pregrasp_clearance,
                contact_handoff_clearance_m=contact_handoff_clearance,
                taught_contact_offset_base_m=contact_offset_m,
            )
        except TaughtGraspPlanningError as exc:
            record.update({"passed": False, "reason": "plan_unreachable", "detail": str(exc)})
            failures["plan_unreachable"] += 1
            trials.append(record)
            continue

        object_spec: dict[str, float | str] = {
            "shape": "cylinder",
            "radius_m": radius_m,
            "height_m": height_m,
            "mass_kg": 0.40,
            "friction": 0.55,
            "strategy": "side_wrap",
            "grasp_mode": "side",
        }
        xml_path = physics.build_mjcf(
            staged,
            f"taught_side_trial_{index:03d}",
            args.ascii_asset_root / "taught_side_regression",
            object_spec,
            table_top_m=plan.table_z_m,
        )
        simulator = physics.PhysicalEpisode(
            xml_path,
            "cola",
            object_spec,
            table_top_m=plan.table_z_m,
            joint_limits=profile.joint_limits,
        )
        simulator.reset(
            float(plan.object_center_m[0]),
            float(plan.object_center_m[1]),
            robot_joint_rad=plan.start_joint_rad,
        )

        playback: list[dict[str, Any]] = []
        premature_target_contacts: Counter[str] = Counter()
        self_contacts: Counter[str] = Counter()
        for stage_index, stage in enumerate(plan.stages):
            motion = simulator.move_to(np.asarray(stage.target_joint_rad, dtype=np.float64))
            forbidden_contacts = simulator.forbidden_contacts()
            if stage_index < len(plan.stages) - 1:
                premature_target_contacts.update(motion["target_contact_pairs"])
            for contact in forbidden_contacts:
                key = f"{contact['geom1']}<->{contact['geom2']}"
                self_contacts[key] += 1
            playback.append(
                {
                    "name": stage.name,
                    "phase": stage.phase,
                    "target_contact_samples": motion["target_contact_samples"],
                    "forbidden_contact_count": len(forbidden_contacts),
                    "motion": motion,
                }
            )

        joint_limit_ok = all(
            np.all(np.degrees(stage.target_joint_rad) >= np.asarray(lower_deg) - 1e-9)
            and np.all(np.degrees(stage.target_joint_rad) <= np.asarray(upper_deg) + 1e-9)
            for stage in plan.stages
        )
        no_premature_target_contact = not premature_target_contacts
        collision_ok = args.record_only_collisions or not self_contacts
        passed = bool(joint_limit_ok and no_premature_target_contact and collision_ok)
        if not joint_limit_ok:
            failures["joint_limit"] += 1
        if not no_premature_target_contact:
            failures["premature_target_contact"] += 1
        if self_contacts and not args.record_only_collisions:
            failures["cad_self_or_table_contact"] += 1
        selected_candidates[plan.candidate_label] += 1
        self_contact_counts.update(self_contacts)
        record.update(
            {
                "passed": passed,
                "candidate": plan.candidate_label,
                "stage_count": len(plan.stages),
                "endpoint_mode": "exact_taught_pose" if plan.final_joint_is_exact_taught_pose else "ik_retargeted",
                "sampled_transit_min_clearance_m": plan.sampled_transit_min_clearance_m,
                "contact_handoff_clearance_m": plan.contact_handoff_clearance_m,
                "premature_target_contacts": dict(premature_target_contacts),
                "forbidden_contact_summary": dict(self_contacts),
                "playback": playback,
            }
        )
        trials.append(record)

    passed_count = sum(bool(row.get("passed")) for row in trials)
    report = {
        "schema": "xiaou_taught_side_grasp_regression_v1",
        "offline": True,
        "hardware_motion": False,
        "serial_opened": False,
        "can_opened": False,
        "ros_hardware_started": False,
        "physics_engine": {"name": "MuJoCo", "version": getattr(runtime, "__version__", "unknown")},
        "model": {
            "poe": str(PROJECT_ROOT / "robot_ai" / "arm_control" / "config" / "arm_model.json"),
            "urdf": str(physics.XACRO_PATH),
            "collision_mesh_count": len(description.collision_meshes),
            "hardware_motion_profile": str(profile.source_path),
            "hardware_motion_profile_schema_version": profile.source_schema_version,
            "ready_pose_deg": [float(value) for value in ready_deg],
        },
        "randomization": {
            "seed": args.seed,
            "trials": args.trials,
            "radius_jitter_m": radius_jitter,
            "height_jitter_m": height_jitter,
            "taught_contact_xy_jitter_m": contact_xy_jitter,
            "taught_contact_z_jitter_m": contact_z_jitter,
            "start_joint_jitter_deg": start_joint_jitter,
            "pregrasp_clearance_m": pregrasp_clearance,
            "contact_handoff_clearance_m": contact_handoff_clearance,
        },
        "acceptance": {
            "joint_limits": "must remain within the current effective Pi envelope",
            "pre_contact_object_collision": "must have no MuJoCo target contact before the final taught contact segment",
            "cad_contact_policy": "record_only" if args.record_only_collisions else "fail",
        },
        "passed": passed_count,
        "failed": len(trials) - passed_count,
        "success_rate": passed_count / max(1, len(trials)),
        "failure_counts": dict(failures),
        "candidate_counts": dict(selected_candidates),
        "cad_contact_summary": dict(self_contact_counts),
        "trials": trials,
        "limitations": [
            "This regression applies small coordinate/dimension perturbations around one cola side-grasp demonstration, not arbitrary object-class grasping.",
            "MuJoCo uses approximate cylinder geometry and uncalibrated static jaw proxies; it does not validate real gripping force.",
            "Only offline planning is exercised. The Pi/F407 UART executor remains outside this tool.",
        ],
    }
    output = args.output if args.output.is_absolute() else PROJECT_ROOT / args.output
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    summary_keys = (
        "offline", "physics_engine", "passed", "failed", "success_rate", "failure_counts", "candidate_counts"
    )
    summary = {key: report[key] for key in summary_keys}
    summary["output"] = str(output)
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0 if passed_count == len(trials) else 2


if __name__ == "__main__":
    raise SystemExit(main())
