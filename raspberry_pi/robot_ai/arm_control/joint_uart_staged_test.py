#!/usr/bin/env python3
"""Staged one-joint UART test for the six-axis F407 integration.

Without --probe-locked or --execute this tool only constructs and prints the
six-axis CMD_TRAJ_POINT frame. It never opens /dev/serial0 in that default
mode. The locked probe is for a flashed transport-only F407 only; real motion
requires every calibration gate plus an explicit per-joint confirmation.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys

try:
    from .safety import MotionLockedError, load_hardware_config, require_motion_ready
    from .uart_protocol import (
        CMD_TRAJ_POINT,
        DEFAULT_BAUD,
        DEFAULT_PORT,
        RSP_ACK,
        RSP_MOTION_LOCKED,
        Frame,
        ProtocolError,
        ResponseError,
        encode_frame,
        exchange,
        pack_trajectory_payload,
        probe_motion_lock,
    )
except ImportError:  # Supports direct invocation from robot_ai/arm_control.
    from safety import MotionLockedError, load_hardware_config, require_motion_ready
    from uart_protocol import (
        CMD_TRAJ_POINT,
        DEFAULT_BAUD,
        DEFAULT_PORT,
        RSP_ACK,
        RSP_MOTION_LOCKED,
        Frame,
        ProtocolError,
        ResponseError,
        encode_frame,
        exchange,
        pack_trajectory_payload,
        probe_motion_lock,
    )


JOINT_IDS = tuple(range(1, 7))


def build_joint_trajectory_frame(
    joint_id: int, target_deg: float, duration_ms: int, sequence: int
) -> Frame:
    """Create an absolute six-axis target with exactly one non-zero joint."""

    if joint_id not in JOINT_IDS:
        raise ProtocolError("joint_id must be in 1..6")
    if not math.isfinite(target_deg):
        raise ProtocolError("target_deg must be finite")
    angles = [0.0] * 6
    angles[joint_id - 1] = float(target_deg)
    return Frame(
        CMD_TRAJ_POINT,
        sequence,
        pack_trajectory_payload(angles, duration_ms),
    )


def _load_config(path: Path | None) -> dict[str, object]:
    return load_hardware_config(path) if path is not None else load_hardware_config()


def _require_target_within_measured_limits(
    config: dict[str, object], joint_id: int, target_deg: float
) -> None:
    mins = config.get("position_min_rad")
    maxs = config.get("position_max_rad")
    if not isinstance(mins, list) or not isinstance(maxs, list):
        raise ProtocolError("hardware calibration limits are missing")
    min_deg = math.degrees(float(mins[joint_id - 1]))
    max_deg = math.degrees(float(maxs[joint_id - 1]))
    if not min_deg <= target_deg <= max_deg:
        raise ProtocolError(
            f"J{joint_id} target {target_deg:.3f} deg is outside measured limits "
            f"[{min_deg:.3f}, {max_deg:.3f}] deg"
        )


def _print_dry_run(frame: Frame, joint_id: int, target_deg: float, duration_ms: int) -> None:
    print(f"J{joint_id} target: {target_deg:.3f} deg, duration: {duration_ms} ms")
    print("Mode: dry-run; no serial device is opened")
    print(f"TX CMD_TRAJ_POINT payload: {frame.payload.hex(' ').upper()}")
    print(f"TX frame: {encode_frame(frame.cmd, frame.seq, frame.payload).hex(' ').upper()}")
    print("Current transport-only F407 response after a valid frame: RSP_MOTION_LOCKED (0x06).")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--joint", type=int, required=True, choices=JOINT_IDS)
    parser.add_argument("--target-deg", type=float, default=0.0)
    parser.add_argument("--duration-ms", type=int, default=1500)
    parser.add_argument("--sequence", type=int, default=16, choices=range(0, 256), metavar="0..255")
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, choices=(9600, 19200, 38400, 57600, 115200))
    parser.add_argument("--timeout", type=float, default=0.5)
    parser.add_argument("--hardware-config", type=Path)
    parser.add_argument("--confirm", default="")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument(
        "--probe-locked",
        action="store_true",
        help="verify a flashed transport-only F407 returns RSP_MOTION_LOCKED",
    )
    mode.add_argument(
        "--execute",
        action="store_true",
        help="send the trajectory only after every measured-hardware gate passes",
    )
    args = parser.parse_args()

    try:
        frame = build_joint_trajectory_frame(
            args.joint, args.target_deg, args.duration_ms, args.sequence
        )
    except (ProtocolError, ValueError) as exc:
        print(f"Invalid staged test: {exc}", file=sys.stderr)
        return 2

    if not args.probe_locked and not args.execute:
        _print_dry_run(frame, args.joint, args.target_deg, args.duration_ms)
        return 0

    if args.probe_locked:
        expected_confirmation = f"LOCKED-J{args.joint}"
        if args.confirm != expected_confirmation:
            parser.error(f"--probe-locked requires --confirm {expected_confirmation}")
        print("Checking locked F407 state before sending the single probe frame...")
        try:
            response = probe_motion_lock(
                frame, port=args.port, baud=args.baud, timeout_s=args.timeout
            )
        except (OSError, ProtocolError, ResponseError, RuntimeError, TimeoutError) as exc:
            print(f"Locked probe failed: {exc}", file=sys.stderr)
            return 1
        print(
            f"Locked probe passed: J{args.joint}, response=0x{response.cmd:02X}, "
            f"payload={response.payload.hex().upper()}"
        )
        return 0

    expected_confirmation = f"MOVE-J{args.joint}"
    if args.confirm != expected_confirmation:
        parser.error(f"--execute requires --confirm {expected_confirmation}")
    try:
        config = _load_config(args.hardware_config)
        require_motion_ready(config)
        _require_target_within_measured_limits(config, args.joint, args.target_deg)
        response = exchange(
            frame,
            port=args.port,
            baud=args.baud,
            timeout_s=args.timeout,
            hardware_config=args.hardware_config,
        )
    except (MotionLockedError, OSError, ProtocolError, ResponseError, RuntimeError, TimeoutError) as exc:
        print(f"Motion command was not sent or was rejected: {exc}", file=sys.stderr)
        return 1

    if response.cmd == RSP_MOTION_LOCKED:
        print("F407 remains transport-only locked; no motion was authorized.")
        return 3
    if response.cmd != RSP_ACK:
        print(f"F407 rejected J{args.joint}: response=0x{response.cmd:02X}", file=sys.stderr)
        return 1
    print(
        f"F407 accepted the J{args.joint} command. This is not completion evidence; "
        "verify feedback and the physical stop path before proceeding."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
