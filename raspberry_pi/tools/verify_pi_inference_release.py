#!/usr/bin/env python3
"""Verify one extracted XiaoU Pi inference release without opening hardware."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
from pathlib import Path, PurePosixPath
from typing import Any


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _safe_relative(value: object) -> Path | None:
    if not isinstance(value, str) or not value:
        return None
    pure = PurePosixPath(value)
    if pure.is_absolute() or ".." in pure.parts:
        return None
    return Path(*pure.parts)


def verify(root: Path, *, require_runtime_modules: bool = True) -> dict[str, Any]:
    root = root.resolve()
    manifest_path = root / "DEPLOYMENT_MANIFEST.json"
    failures: list[str] = []
    if not manifest_path.is_file():
        return {"root": str(root), "passed": False, "failures": ["manifest_missing"]}
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        return {"root": str(root), "passed": False, "failures": [f"manifest_invalid:{exc}"]}
    rows = manifest.get("files") if isinstance(manifest, dict) else None
    if manifest.get("schema_version") != 1 or not isinstance(rows, list):
        failures.append("manifest_schema")
        rows = []
    checked: list[str] = []
    for row in rows:
        if not isinstance(row, dict):
            failures.append("manifest_row")
            continue
        relative = _safe_relative(row.get("path"))
        if relative is None:
            failures.append("unsafe_manifest_path")
            continue
        candidate = (root / relative).resolve()
        try:
            candidate.relative_to(root)
        except ValueError:
            failures.append("manifest_path_escape")
            continue
        if not candidate.is_file():
            failures.append(f"missing:{relative.as_posix()}")
            continue
        if _sha256(candidate) != row.get("sha256"):
            failures.append(f"hash:{relative.as_posix()}")
            continue
        checked.append(relative.as_posix())
    config_path = root / "robot_ai/arm_control/config/hardware_calibration.json"
    config: dict[str, Any] = {}
    try:
        config = json.loads(config_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        failures.append(f"hardware_config:{exc}")
    locked_fields = ("motion_enabled", "protocol_confirmed", "uart_link_verified", "f407_firmware_verified", "estop_verified", "feedback_verified")
    unlocked = [field for field in locked_fields if config.get(field) is not False]
    if unlocked:
        failures.extend(f"unlocked:{field}" for field in unlocked)
    required_files = (
        "models/xiaou_objects_gpu_deep.onnx",
        "models/xiaou_objects_gpu_deep.names",
        "runtime/decision/transformer_policy_safety_final_20260813.int8.onnx",
        "runtime/decision/transformer_policy_safety_final_20260813.int8.json",
        "robot_ai/arm_control/grasp_families.py",
        "robot_ai/arm_control/config/grasp_family_registry.json",
        "robot_ai/arm_control/config/object_teach_registry.json",
        "robot_ai/arm_control/teach_registry.py",
        "robot_ai/arm_control/taught_waypoint_plan.py",
        "tools/profile_pi_inference_runtime.py",
        "tools/preview_object_teach_record.py",
        "scripts/install_pi_inference_release.sh",
        "scripts/verify_pi_inference_release.sh",
        "robot_ai/xiaou_status_dashboard.py",
        "robot_ai/face_display.py",
        "robot_ai/face_animation.py",
        "robot_ai/emote_assets/gif/idle.gif",
        "scripts/run_xiaou_face_screen.sh",
        "scripts/setup_pi_inference_env.sh",
    )
    missing_required = [item for item in required_files if not (root / item).is_file()]
    failures.extend(f"required:{item}" for item in missing_required)
    modules = {name: bool(importlib.util.find_spec(name)) for name in ("cv2", "numpy", "onnxruntime", "PIL")}
    if require_runtime_modules:
        failures.extend(f"module:{name}" for name, present in modules.items() if not present)
    return {
        "offline": True,
        "hardware_motion": False,
        "serial_opened": False,
        "can_opened": False,
        "root": str(root),
        "manifest_profile_set": manifest.get("profile_set"),
        "manifest_file_count": len(rows),
        "verified_file_count": len(checked),
        "missing_required": missing_required,
        "runtime_modules": modules,
        "locked_fields": locked_fields,
        "unlocked_fields": unlocked,
        "failures": failures,
        "passed": not failures,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--skip-runtime-imports", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = verify(args.root, require_runtime_modules=not args.skip_runtime_imports)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
