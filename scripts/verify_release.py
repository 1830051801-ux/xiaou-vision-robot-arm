#!/usr/bin/env python3
"""Check that the public XiaoU repository is self-contained and source-only.

The checker deliberately does not open a serial port, camera, CAN interface or
network socket. It validates file layout, model registry references, the
motion-lock default, and Keil source references.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import subprocess
import sys
from typing import Iterable


ROOT = Path(__file__).resolve().parents[1]
REQUIRED_PATHS = (
    "README.md",
    "LICENSE",
    "THIRD_PARTY_NOTICES.md",
    "docs/PUBLICATION.md",
    "raspberry_pi/robot_ai/arm_control/uart_protocol.py",
    "raspberry_pi/robot_ai/decision/transformer_policy.py",
    "raspberry_pi/robot_ai/vision/model_registry.py",
    "raspberry_pi/tests/test_uart_protocol.py",
    "raspberry_pi/simulation/desktop_scene.py",
    "raspberry_pi/ros2_ws/src/xiaou_arm_description",
    "raspberry_pi/models/xiaou_objects_gpu_deep.onnx",
    "raspberry_pi/runtime/decision/transformer_policy_safety_final_20260813.int8.onnx",
    "stm32_keil/BasicSetting_DaRanRobot/MDK-ARM/BasicSetting_DaRanRobot.uvprojx",
    "stm32_keil/BasicSetting_DaRanRobot/Src/trajectory.c",
    "stm32_keil/BasicSetting_DaRanRobot/Src/comm_protocol.c",
)


def _tracked_files(root: Path) -> set[str]:
    result = subprocess.run(
        ["git", "-C", str(root), "ls-files"],
        check=True,
        capture_output=True,
        text=True,
    )
    return {
        line.strip()
        for line in result.stdout.splitlines()
        if line.strip() and (root / line.strip()).is_file()
    }


def _forbidden_paths(paths: Iterable[str]) -> list[str]:
    failures: list[str] = []
    for path in paths:
        lower = path.lower()
        if (
            path == "raspberry_pi/config.env"
            or path == "raspberry_pi/scripts/enable_passwordless_sudo.sh"
            or lower.endswith(".uvoptx")
            or ".uvguix." in lower
        ):
            failures.append(path)
        elif "/objects/" in lower or "/listings/" in lower:
            failures.append(path)
        elif "/jlink" in lower or lower.endswith(".axf") or lower.endswith(".hex"):
            failures.append(path)
        elif lower.endswith((".pt", ".pth", ".tar.gz", ".whl")):
            failures.append(path)
        elif path.startswith(("raspberry_pi/runs/", "raspberry_pi/output/", "raspberry_pi/tmp/")):
            failures.append(path)
        elif path.startswith("raspberry_pi/runtime/deployment/"):
            failures.append(path)
    return sorted(set(failures))


def _keil_source_failures(root: Path) -> list[str]:
    project = root / "stm32_keil/BasicSetting_DaRanRobot/MDK-ARM/BasicSetting_DaRanRobot.uvprojx"
    if not project.is_file():
        return ["Keil project file is missing"]
    content = project.read_text(encoding="utf-8-sig", errors="replace")
    references = re.findall(r"<FilePath>(.*?)</FilePath>", content)
    failures: list[str] = []
    for raw in references:
        relative = raw.strip().replace("\\", "/")
        if not relative or relative.startswith("$("):
            continue
        candidate = (project.parent / relative).resolve()
        try:
            candidate.relative_to(root.resolve())
        except ValueError:
            failures.append(f"Keil reference escapes repository: {raw}")
            continue
        if not candidate.is_file():
            failures.append(f"Keil source missing: {raw}")
    return failures


def _model_registry_failures(root: Path) -> list[str]:
    registry_path = root / "raspberry_pi/robot_ai/vision/config/yolo_model_registry.json"
    try:
        registry = json.loads(registry_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        return [f"model registry is unreadable: {exc}"]
    failures: list[str] = []
    profiles = registry.get("profiles")
    if not isinstance(profiles, dict) or not profiles:
        return ["model registry has no profiles"]
    for name, profile in profiles.items():
        if not isinstance(profile, dict):
            failures.append(f"profile {name} is not an object")
            continue
        model = profile.get("model")
        schema = profile.get("class_schema")
        if not isinstance(model, str) or not (root / "raspberry_pi" / model).is_file():
            failures.append(f"profile {name} model is missing: {model}")
        if not isinstance(schema, str) or not (root / "raspberry_pi" / schema).is_file():
            failures.append(f"profile {name} schema is missing: {schema}")
    if registry.get("automatic_switching") is not False:
        failures.append("automatic model switching must remain disabled")
    return failures


def _motion_lock_failures(root: Path) -> list[str]:
    config_path = root / "raspberry_pi/robot_ai/arm_control/config/hardware_calibration.json"
    try:
        config = json.loads(config_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        return [f"hardware calibration config is unreadable: {exc}"]
    if config.get("motion_enabled") is True:
        return ["public hardware_calibration.json must stay motion-locked"]
    return []


def verify(root: Path) -> dict[str, object]:
    root = root.resolve()
    missing = [path for path in REQUIRED_PATHS if not (root / path).exists()]
    try:
        tracked = _tracked_files(root)
        forbidden = _forbidden_paths(tracked)
    except (OSError, subprocess.CalledProcessError) as exc:
        tracked = set()
        forbidden = [f"cannot enumerate tracked files: {exc}"]
    keil_failures = _keil_source_failures(root)
    registry_failures = _model_registry_failures(root)
    motion_failures = _motion_lock_failures(root)
    failures = missing + forbidden + keil_failures + registry_failures + motion_failures
    return {
        "passed": not failures,
        "tracked_file_count": len(tracked),
        "missing_required": missing,
        "forbidden_tracked": forbidden,
        "keil_reference_failures": keil_failures,
        "model_registry_failures": registry_failures,
        "motion_lock_failures": motion_failures,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=ROOT)
    args = parser.parse_args()
    report = verify(args.root)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
