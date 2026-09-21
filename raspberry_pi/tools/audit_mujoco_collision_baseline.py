#!/usr/bin/env python3
"""Report baseline MuJoCo contacts for ready and taught joint poses.

This is a read-only geometry diagnostic.  It does not change collision
filters, limits, or a teach pose, and it never opens a robot transport.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))
if str(PROJECT_ROOT / "tools") not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT / "tools"))

import simulate_physical_grasp as physics
from simulate_taught_cola_grasp import TAUGHT_COLA_GRASP_DEG

DEFAULT_HARDWARE = PROJECT_ROOT / "robot_ai" / "arm_control" / "config" / "hardware_calibration_candidate_20260811.json"
DEFAULT_OUTPUT = PROJECT_ROOT / "runtime" / "simulations" / "mujoco_collision_baseline_20260921.json"
ASCII_ASSET_ROOT = Path("D:/xiaou_mujoco_collision_baseline_20260921")


def _pose_contacts(simulator: Any, pose_deg: list[float], *, x: float, y: float) -> dict[str, Any]:
    pose_rad = np.radians(np.asarray(pose_deg, dtype=np.float64))
    simulator.reset(x, y, robot_joint_rad=pose_rad)
    contacts = simulator._contacts()
    return {
        "joint_deg": [float(value) for value in pose_deg],
        "contact_count": len(contacts),
        "forbidden_contacts": simulator.forbidden_contacts(),
        "target_contacts": simulator.target_robot_contacts(),
        "self_collision_only": bool(simulator.forbidden_contacts()) and not any(
            "target_object" in {row["body1"], row["body2"]} for row in simulator.forbidden_contacts()
        ),
        "contacts": contacts,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--hardware-config", type=Path, default=DEFAULT_HARDWARE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--x-m", type=float, default=0.17279)
    parser.add_argument("--y-m", type=float, default=0.016666)
    parser.add_argument("--table-z-m", type=float, default=-0.10783322127283833)
    args = parser.parse_args()
    if not all(math.isfinite(float(value)) for value in (args.x_m, args.y_m, args.table_z_m)):
        parser.error("position arguments must be finite")

    profile = physics.load_simulation_motion_profile(args.hardware_config.resolve())
    description = physics.parse_robot_description()
    staged = physics.stage_ascii_assets(description, "collision_baseline", ASCII_ASSET_ROOT)
    object_spec: dict[str, float | str] = {
        "shape": "cylinder",
        "radius_m": 0.032,
        "height_m": 0.190,
        "mass_kg": 0.40,
        "friction": 0.55,
        "strategy": "side_wrap",
        "grasp_mode": "side",
    }
    xml_path = physics.build_mjcf(
        staged,
        "collision_baseline",
        ASCII_ASSET_ROOT / "collision_baseline" / "cola",
        object_spec,
        table_top_m=args.table_z_m,
    )
    simulator = physics.PhysicalEpisode(
        xml_path,
        "cola",
        object_spec,
        table_top_m=args.table_z_m,
        joint_limits=profile.joint_limits,
    )
    ready_deg = [math.degrees(float(value)) for value in profile.ready_pose_rad]
    taught_deg = [float(value) for value in TAUGHT_COLA_GRASP_DEG]
    report = {
        "schema": "xiaou_mujoco_collision_baseline_v1",
        "offline": True,
        "hardware_motion": False,
        "serial_opened": False,
        "can_opened": False,
        "ros_hardware_started": False,
        "xml": str(xml_path),
        "object_model": object_spec,
        "ready": _pose_contacts(simulator, ready_deg, x=args.x_m, y=args.y_m),
        "taught": _pose_contacts(simulator, taught_deg, x=args.x_m, y=args.y_m),
        "interpretation": (
            "The taught pose has baseline self-contact in the current collision model; "
            "do not select a route until the teach pose or collision mesh is corrected."
            if _pose_contacts(simulator, taught_deg, x=args.x_m, y=args.y_m)["self_collision_only"]
            else "No baseline self-contact was found at the tested taught pose."
        ),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({
        "offline": True,
        "ready_contact_count": report["ready"]["contact_count"],
        "taught_contact_count": report["taught"]["contact_count"],
        "taught_self_collision_only": report["taught"]["self_collision_only"],
        "output": str(args.output),
    }, ensure_ascii=False, indent=2))
    return 0 if not report["taught"]["self_collision_only"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
