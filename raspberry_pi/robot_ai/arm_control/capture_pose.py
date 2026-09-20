#!/usr/bin/env python3
"""Capture stationary zero/ready poses from F407 telemetry.

The tool never sends a trajectory or CAN command.  It reads CMD_GET_STATE,
converts the six reported joint angles from degrees to radians, and optionally
records them in hardware_calibration.json.  Use this only with the arm
stationary and an accessible physical emergency stop.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import sys
import tempfile

try:
    from .uart_protocol import (
        CMD_GET_STATE,
        DEFAULT_BAUD,
        DEFAULT_PORT,
        Frame,
        RSP_ACK,
        SYS_STATE_TRANSPORT_ONLY,
        decode_state_payload,
        exchange,
    )
except ImportError:
    from uart_protocol import CMD_GET_STATE, DEFAULT_BAUD, DEFAULT_PORT, Frame, RSP_ACK, SYS_STATE_TRANSPORT_ONLY, decode_state_payload, exchange


DEFAULT_CONFIG = Path(__file__).resolve().parent / "config" / "hardware_calibration.json"


def _read_state(*, port: str, baud: int, timeout: float, sequence: int) -> dict[str, object]:
    response = exchange(Frame(CMD_GET_STATE, sequence), port=port, baud=baud, timeout_s=timeout)
    if response.cmd != RSP_ACK:
        raise RuntimeError(f"GET_STATE returned response 0x{response.cmd:02X}")
    state = decode_state_payload(response.payload)
    if state["state"] == SYS_STATE_TRANSPORT_ONLY:
        raise RuntimeError("F407 is transport-only; it has no real actuator feedback to capture")
    if state["estop"] or state["motion_busy"]:
        raise RuntimeError("refusing capture while E-stop is active or motion is busy")
    joints = state["joints"]
    if not isinstance(joints, list) or len(joints) != 6:
        raise RuntimeError("F407 did not return six joint telemetry records")
    if not all(bool(joint.get("online")) for joint in joints if isinstance(joint, dict)):
        raise RuntimeError("all six joints must report online before capture")
    angles_deg = [float(joint["angle_deg"]) for joint in joints]
    if not all(math.isfinite(value) for value in angles_deg):
        raise RuntimeError("telemetry contains a non-finite joint angle")
    return {"state": state, "angles_deg": angles_deg}


def _write_pose(config_path: Path, key: str, values: list[float]) -> None:
    config = json.loads(config_path.read_text(encoding="utf-8"))
    config[key] = [math.radians(value) for value in values]
    config[f"{key}_captured_from_telemetry"] = True
    with tempfile.NamedTemporaryFile("w", encoding="utf-8", dir=config_path.parent, delete=False) as temp:
        temp.write(json.dumps(config, ensure_ascii=False, indent=2) + "\n")
        temp_path = Path(temp.name)
    os.replace(temp_path, config_path)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pose", choices=("zero", "ready"), help="pose to capture")
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--timeout", type=float, default=0.5)
    parser.add_argument("--sequence", type=int, default=200, choices=range(256))
    parser.add_argument("--hardware-config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--write", action="store_true", help="write captured values to the JSON file")
    parser.add_argument("--confirm", default="", help="required exact confirmation when --write is used")
    args = parser.parse_args()
    if args.write and args.confirm != f"CAPTURE-{args.pose.upper()}":
        parser.error(f"--write requires --confirm CAPTURE-{args.pose.upper()}")
    try:
        result = _read_state(port=args.port, baud=args.baud, timeout=args.timeout, sequence=args.sequence)
    except Exception as exc:  # Keep terminal diagnostics concise on Pi.
        print(f"capture failed: {exc}", file=sys.stderr)
        return 1
    values = result["angles_deg"]
    print("Stationary six-axis telemetry (deg): " + ", ".join(f"{v:.6f}" for v in values))
    print("Command positive direction: counter-clockwise from each motor gear side")
    if not args.write:
        print("Dry run: JSON was not changed. Add --write --confirm CAPTURE-%s to record." % args.pose.upper())
        return 0
    key = "zero_pose_reference_rad" if args.pose == "zero" else "ready_pose_rad"
    _write_pose(args.hardware_config, key, values)
    print(f"Recorded {key} in {args.hardware_config}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
