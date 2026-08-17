#!/usr/bin/env python3
"""Run the read-only preflight used before reconnecting the real arm.

This command is intentionally a desktop audit.  It checks that the checked-in
hardware gate is still locked, replays UART/CAN and ROS2 message contracts,
and audits the Pi archive.  It never opens a serial/CAN device and never
starts ROS2, MoveIt, or a hardware node.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys
import tarfile
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[1]
CONFIG = PROJECT_ROOT / "robot_ai/arm_control/config/hardware_calibration.json"
ARCHIVE = PROJECT_ROOT / "runtime/deployment/xiaou_pi_inference_final_20260813.tar.gz"
ARCHIVE_REPORT = PROJECT_ROOT / "runtime/deployment/xiaou_pi_inference_final_20260813.json"
ARCHIVE_PROBE = PROJECT_ROOT / "runtime/deployment/xiaou_pi_inference_final_20260813_probe.json"
PROTOCOL_REPORT = PROJECT_ROOT / "runtime/simulations/protocol_offline_replay_final_20260813.json"
ROS_REPORT = PROJECT_ROOT / "runtime/simulations/ros2_offline_replay_final_20260813.json"
INT8_POLICY = PROJECT_ROOT / "runtime/decision/transformer_policy_safety_final_20260813.int8.onnx"


def _run(command: list[str], output: Path) -> dict[str, Any]:
    completed = subprocess.run(command, cwd=PROJECT_ROOT, capture_output=True, text=True, check=False)
    row: dict[str, Any] = {
        "command": command,
        "exit_code": completed.returncode,
        "stdout_tail": completed.stdout[-1200:],
        "stderr_tail": completed.stderr[-1200:],
        "report": str(output),
    }
    if output.is_file():
        try:
            row["passed"] = bool(json.loads(output.read_text(encoding="utf-8")).get("passed"))
        except (OSError, json.JSONDecodeError):
            row["passed"] = False
    else:
        row["passed"] = False
    return row


def _config_audit() -> dict[str, Any]:
    config = json.loads(CONFIG.read_text(encoding="utf-8"))
    required_false = [
        "motion_enabled",
        "protocol_confirmed",
        "uart_link_verified",
        "f407_firmware_verified",
        "estop_verified",
        "feedback_verified",
        "mcu_calibration_verified",
    ]
    violations = [name for name in required_false if config.get(name) is not False]
    transport = config.get("transport") or {}
    return {
        "path": str(CONFIG),
        "motion_enabled": config.get("motion_enabled"),
        "required_locked_fields": required_false,
        "violations": violations,
        "transport": {
            "kind": transport.get("kind"),
            "port": transport.get("port"),
            "baud": transport.get("baud"),
        },
        "passed": not violations,
    }


def _archive_audit() -> dict[str, Any]:
    if not ARCHIVE.is_file():
        return {"archive": str(ARCHIVE), "passed": False, "error": "archive is missing"}
    forbidden_tokens = (".pt", ".pth", "ultralytics", "mujoco", "dataset", "camera")
    with tarfile.open(ARCHIVE, "r:*") as bundle:
        names = [item.name for item in bundle.getmembers()]
    forbidden = [name for name in names if any(token in name.lower() for token in forbidden_tokens)]
    return {
        "archive": str(ARCHIVE),
        "file_count": len(names),
        "forbidden_entries": forbidden,
        "report_exists": ARCHIVE_REPORT.is_file(),
        "passed": not forbidden and ARCHIVE_REPORT.is_file(),
    }


def preflight() -> dict[str, Any]:
    config = _config_audit()
    protocol = _run(
        [sys.executable, str(PROJECT_ROOT / "tools/protocol_offline_replay.py"), "--output", str(PROTOCOL_REPORT)],
        PROTOCOL_REPORT,
    )
    ros = _run(
        [
            sys.executable,
            str(PROJECT_ROOT / "tools/ros2_offline_replay.py"),
            "--checkpoint",
            str(INT8_POLICY),
            "--device",
            "cpu",
            "--output",
            str(ROS_REPORT),
        ],
        ROS_REPORT,
    )
    deployment_probe = _run(
        [
            sys.executable,
            str(PROJECT_ROOT / "tools/verify_pi_deployment_package.py"),
            "--archive",
            str(ARCHIVE),
            "--output",
            str(ARCHIVE_PROBE),
        ],
        ARCHIVE_PROBE,
    )
    archive = _archive_audit()
    static_files = [
        PROJECT_ROOT / "robot_ai/arm_control/joint_status_probe.py",
        PROJECT_ROOT / "robot_ai/xiaou_status_dashboard.py",
        PROJECT_ROOT / "ros2_ws/src/xiaou_arm_planning/launch/review_only.launch.py",
        INT8_POLICY,
    ]
    missing_files = [str(path) for path in static_files if not path.is_file()]
    checks = {
        "hardware_config_locked": config["passed"],
        "protocol_replay": protocol["passed"] and protocol["exit_code"] == 0,
        "ros2_contract_replay": ros["passed"] and ros["exit_code"] == 0,
        "deployment_archive": archive["passed"] and deployment_probe["passed"] and deployment_probe["exit_code"] == 0,
        "required_files": not missing_files,
    }
    return {
        "offline": True,
        "hardware_motion": False,
        "serial_opened": False,
        "can_opened": False,
        "ros2_process_started": False,
        "moveit_started": False,
        "config": config,
        "protocol_replay": protocol,
        "ros2_replay": ros,
        "deployment_probe": deployment_probe,
        "archive": archive,
        "missing_files": missing_files,
        "checks": checks,
        "passed": all(checks.values()),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=PROJECT_ROOT / "runtime/simulations/hardware_return_preflight_final_20260813.json")
    args = parser.parse_args()
    report = preflight()
    output = args.output if args.output.is_absolute() else PROJECT_ROOT / args.output
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
