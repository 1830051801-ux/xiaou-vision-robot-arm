#!/usr/bin/env python3
"""Verify future-object teach requirements and MuJoCo shape alignment offline.

This tool checks that every object class used by the desktop Transformer has a
matching teach contract and a matching primitive geometry declaration.  It is
an integration guard for new image data or demonstrations; it never captures a
pose and never opens hardware transport.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from robot_ai.arm_control import load_object_teach_registry, resolve_object_teach_spec  # noqa: E402
from simulation.desktop_scene import OBJECT_CLASSES, OBJECT_SPECS  # noqa: E402


def verify() -> dict[str, Any]:
    registry = load_object_teach_registry()
    rows: list[dict[str, Any]] = []
    failures: list[str] = []
    for object_class in OBJECT_CLASSES:
        spec = resolve_object_teach_spec(object_class)
        model = dict(spec.simulation_model)
        scene = OBJECT_SPECS[object_class]
        shape_match = model.get("shape") == scene.get("shape")
        dimensions_match = all(
            abs(float(model[key]) - float(scene[key])) < 1e-12
            for key in ("radius_m", "height_m", "mass_kg")
        )
        row = {
            **spec.as_dict(),
            "scene_shape": scene.get("shape"),
            "shape_match": shape_match,
            "dimensions_match": dimensions_match,
        }
        rows.append(row)
        if not shape_match or not dimensions_match:
            failures.append(f"{object_class}: simulation model drift")
        if object_class == "cola":
            if spec.route_status != "current_local_preview":
                failures.append("cola: current local route status drift")
        elif spec.route_status != "requires_dedicated_teach":
            failures.append(f"{object_class}: must require dedicated teach")
    rejected = resolve_object_teach_spec("earphone")
    if rejected.route_status != "reject":
        failures.append("earphone: flexible-item rejection drift")
    return {
        "schema": "xiaou_object_teach_registry_audit_v1",
        "offline": True,
        "hardware_motion": False,
        "serial_opened": False,
        "can_opened": False,
        "camera_opened": False,
        "registry_status": registry.get("status"),
        "objects": rows,
        "rejected_object": rejected.as_dict(),
        "failure_count": len(failures),
        "failures": failures,
        "passed": not failures,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=PROJECT_ROOT / "runtime/simulations/object_teach_registry_audit_20260813.json",
    )
    args = parser.parse_args()
    output = args.output if args.output.is_absolute() else PROJECT_ROOT / args.output
    report = verify()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
