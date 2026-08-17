#!/usr/bin/env python3
"""Preview or, after all gates pass, send the configured six-axis ready pose."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys

try:
    from .safety import load_hardware_config, require_motion_ready
    from .uart_protocol import CMD_TRAJ_POINT, DEFAULT_BAUD, DEFAULT_PORT, Frame, RSP_ACK, encode_frame, exchange, pack_trajectory_payload
except ImportError:
    from safety import load_hardware_config, require_motion_ready
    from uart_protocol import CMD_TRAJ_POINT, DEFAULT_BAUD, DEFAULT_PORT, Frame, RSP_ACK, encode_frame, exchange, pack_trajectory_payload


DEFAULT_CONFIG = Path(__file__).resolve().parent / "config" / "hardware_calibration.json"


def build_ready_frame(config: dict[str, object], sequence: int, duration_ms: int) -> Frame:
    values = config.get("ready_pose_rad")
    if not isinstance(values, list) or len(values) != 6 or any(value is None for value in values):
        raise ValueError("ready_pose_rad must contain six captured values")
    angles_deg = [math.degrees(float(value)) for value in values]
    if not all(math.isfinite(value) for value in angles_deg):
        raise ValueError("ready_pose_rad contains a non-finite value")
    return Frame(CMD_TRAJ_POINT, sequence, pack_trajectory_payload(angles_deg, duration_ms))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--hardware-config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--duration-ms", type=int, default=2000)
    parser.add_argument("--sequence", type=int, default=220, choices=range(256))
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--timeout", type=float, default=0.8)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm", default="")
    args = parser.parse_args()
    try:
        config = load_hardware_config(args.hardware_config)
        frame = build_ready_frame(config, args.sequence, args.duration_ms)
    except Exception as exc:
        print(f"ready pose invalid: {exc}", file=sys.stderr)
        return 2
    print(f"TX ready pose frame: {encode_frame(frame.cmd, frame.seq, frame.payload).hex(' ').upper()}")
    if not args.execute:
        print("Dry run: no UART opened and no motion command sent")
        return 0
    if args.confirm != "READY-POSE":
        print("--execute requires --confirm READY-POSE", file=sys.stderr)
        return 2
    try:
        require_motion_ready(config)
        response = exchange(frame, port=args.port, baud=args.baud, timeout_s=args.timeout, hardware_config=args.hardware_config)
    except Exception as exc:
        print(f"ready pose rejected: {exc}", file=sys.stderr)
        return 1
    if response.cmd != RSP_ACK:
        print(f"ready pose rejected by F407: response=0x{response.cmd:02X}", file=sys.stderr)
        return 1
    print("F407 accepted the ready pose command; verify feedback and physical stop path.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
