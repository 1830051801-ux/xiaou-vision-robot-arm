#!/usr/bin/env python3
"""Run a MuJoCo mesh playback for the approximate taught-cola side grasp.

This is the no-camera rehearsal path for the next hardware session.  It uses
the real checked-in arm collision meshes, a primitive upright cola cylinder,
and the POE plan generated from the existing taught joint pose.  Collision
samples are reported for inspection but intentionally do not reject the
toy-demonstration route.  No serial, CAN, ROS hardware, or gripper command is
opened by this program.
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

from robot_ai.arm_control import build_taught_side_grasp_plan, load_default_model  # noqa: E402

import simulate_physical_grasp as physics  # noqa: E402


TAUGHT_COLA_GRASP_DEG = [
    -100.000244140625,
    -48.93047332763672,
    -71.99970245361328,
    -84.9998664855957,
    110.00060272216797,
    0.0,
]
DEFAULT_OUTPUT = PROJECT_ROOT / "runtime" / "simulations" / "taught_cola_grasp_mujoco.json"
DEFAULT_ASCII_ROOT = Path("D:/xiaou_mujoco_taught_cola")


def _load_motion_profile(path: Path) -> tuple[list[float], list[float], np.ndarray]:
    """Return the no-motion effective envelope and recorded ready pose."""

    profile = physics.load_simulation_motion_profile(path)
    return (
        [math.degrees(float(value)) for value in profile.joint_limits.position_min],
        [math.degrees(float(value)) for value in profile.joint_limits.position_max],
        profile.ready_pose_rad.copy(),
    )


def _contact_summary(simulator: Any) -> list[dict[str, str]]:
    return simulator._contacts()


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
    parser.add_argument("--target-offset-base-m", default="0,0,0")
    parser.add_argument("--overhead-clearance-m", type=float, default=0.040)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--ascii-asset-root", type=Path, default=DEFAULT_ASCII_ROOT)
    args = parser.parse_args()
    if any(not math.isfinite(float(value)) or float(value) <= 0.0 for value in (
        args.cola_height_m,
        args.cola_radius_m,
        args.pregrasp_clearance_m,
        args.contact_handoff_clearance_m,
        args.overhead_clearance_m,
    )):
        parser.error("all dimensions must be finite and positive")
    try:
        target_offset = [float(value.strip()) for value in args.target_offset_base_m.split(",")]
    except ValueError:
        parser.error("target-offset-base-m must be three comma-separated numbers")
    if len(target_offset) != 3 or not all(math.isfinite(value) for value in target_offset):
        parser.error("target-offset-base-m must be three finite numbers")

    config_path = args.hardware_config if args.hardware_config.is_absolute() else PROJECT_ROOT / args.hardware_config
    profile = physics.load_simulation_motion_profile(config_path.resolve())
    runtime = physics._require_mujoco()
    lower_deg = [math.degrees(float(value)) for value in profile.joint_limits.position_min]
    upper_deg = [math.degrees(float(value)) for value in profile.joint_limits.position_max]
    ready_deg = [math.degrees(float(value)) for value in profile.ready_pose_rad]
    arm_model = load_default_model()
    plan = build_taught_side_grasp_plan(
        ready_deg,
        TAUGHT_COLA_GRASP_DEG,
        model=arm_model,
        lower_deg=lower_deg,
        upper_deg=upper_deg,
        object_height_m=args.cola_height_m,
        object_radius_m=args.cola_radius_m,
        pregrasp_clearance_m=args.pregrasp_clearance_m,
        contact_handoff_clearance_m=args.contact_handoff_clearance_m,
        overhead_clearance_m=args.overhead_clearance_m,
        taught_contact_offset_base_m=target_offset,
    )

    description = physics.parse_robot_description()
    staged = physics.stage_ascii_assets(description, "cola", args.ascii_asset_root)
    object_spec: dict[str, float | str] = {
        "shape": "cylinder",
        "radius_m": float(args.cola_radius_m),
        "height_m": float(args.cola_height_m),
        "mass_kg": 0.40,
        "friction": 0.55,
        "strategy": "side_wrap",
        "grasp_mode": "side",
    }
    xml_path = physics.build_mjcf(
        staged,
        "taught_cola",
        args.ascii_asset_root / "cola",
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
    object_center = plan.object_center_m
    simulator.reset(
        float(object_center[0]),
        float(object_center[1]),
        robot_joint_rad=plan.start_joint_rad,
    )
    initial_joint_deg = np.degrees(simulator.data.qpos[:6]).copy()

    motion_records: list[dict[str, Any]] = []
    all_forbidden: Counter[str] = Counter()
    all_contacts: Counter[str] = Counter()
    for stage in plan.stages:
        motion = simulator.move_to(np.asarray(stage.target_joint_rad, dtype=np.float64))
        contacts = _contact_summary(simulator)
        forbidden = simulator.forbidden_contacts()
        for item in contacts:
            all_contacts[f"{item['geom1']}<->{item['geom2']}"] += 1
        for item in forbidden:
            all_forbidden[f"{item['geom1']}<->{item['geom2']}"] += 1
        motion_records.append(
            {
                "name": stage.name,
                "phase": stage.phase,
                "target_joint_deg": [round(float(value), 6) for value in np.degrees(stage.target_joint_rad)],
                "motion": motion,
                "contact_count_at_endpoint": len(contacts),
                "forbidden_contact_count_at_endpoint": len(forbidden),
            }
        )

    report = {
        "schema": "xiaou_taught_cola_mujoco_replay_v1",
        "offline": True,
        "hardware_motion": False,
        "serial_opened": False,
        "can_opened": False,
        "ros_hardware_started": False,
        "physics_engine": {"name": "MuJoCo", "version": getattr(runtime, "__version__", "unknown")},
        "execution_policy": {
            "collision_mode": "record_only",
            "gripper_action": "not_included",
            "note": "This report does not prove physical collision clearance or gripping force.",
        },
        "model_sources": {
            "poe": str(PROJECT_ROOT / "robot_ai" / "arm_control" / "config" / "arm_model.json"),
            "urdf": str(physics.XACRO_PATH),
            "collision_meshes": len(description.collision_meshes),
            "mjcf": str(xml_path),
            "hardware_motion_profile": str(profile.source_path),
            "hardware_motion_profile_schema_version": profile.source_schema_version,
        },
        "world_alignment": {
            "table_top_m": plan.table_z_m,
            "alignment": "derived from taught TCP side-center minus half approximate cola height",
            "object_center_m": [float(value) for value in object_center],
        },
        "initial_robot_state": {
            "source": "ready_pose_replay_start",
            "joint_deg": [float(value) for value in initial_joint_deg],
            "planned_joint_deg": [float(value) for value in np.degrees(plan.start_joint_rad)],
        },
        "plan": plan.as_dict(),
        "playback": motion_records,
        "contact_summary": dict(all_contacts),
        "forbidden_contact_summary": dict(all_forbidden),
        "stage_count": len(motion_records),
        "final_stage": motion_records[-1]["name"] if motion_records else None,
    }
    output = args.output if args.output.is_absolute() else PROJECT_ROOT / args.output
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({
        "offline": True,
        "physics_engine": report["physics_engine"],
        "stage_count": report["stage_count"],
        "final_stage": report["final_stage"],
        "contact_summary": report["contact_summary"],
        "forbidden_contact_summary": report["forbidden_contact_summary"],
        "output": str(output),
    }, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
