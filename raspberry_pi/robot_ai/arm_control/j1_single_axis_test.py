#!/usr/bin/env python3
"""Supervised absolute-angle test for one connected J1 actuator.

The formal F407 command is `CMD_TRAJ_POINT (0x50)`, a fixed 26-byte payload
`<6fH`: six absolute joint angles in degrees plus duration in milliseconds.
This tool keeps J2..J6 at zero, uses only the trajectory-point frame, and is
locked behind measured test bounds plus explicit confirmations.
"""

from __future__ import annotations

import argparse
import math
import sys
import time

try:
    from .uart_protocol import (
        CMD_GET_JOINT,
        CMD_TRAJ_POINT,
        CMD_STOP,
        DEFAULT_BAUD,
        DEFAULT_PORT,
        Frame,
        RSP_ACK,
        RSP_TRAJ_ACK,
        pack_trajectory_payload,
        _exchange_wire,
        decode_joint_payload,
    )
except ImportError:  # Direct invocation from robot_ai/arm_control.
    from uart_protocol import (
        CMD_GET_JOINT,
        CMD_TRAJ_POINT,
        CMD_STOP,
        DEFAULT_BAUD,
        DEFAULT_PORT,
        Frame,
        RSP_ACK,
        RSP_TRAJ_ACK,
        pack_trajectory_payload,
        _exchange_wire,
        decode_joint_payload,
    )


J1 = 1
CONFIRM = "MOVE-J1-ABS"


def build_j1_frame(target_deg: float, duration_ms: int, sequence: int) -> Frame:
    """Build the formal six-axis trajectory point with only J1 populated."""

    if not math.isfinite(target_deg):
        raise ValueError("target must be finite")
    if isinstance(duration_ms, bool) or not isinstance(duration_ms, int) or not 1 <= duration_ms <= 10000:
        raise ValueError("duration_ms must be an integer in 1..10000")
    if not 0 <= sequence <= 255:
        raise ValueError("sequence must be in 0..255")
    return Frame(
        CMD_TRAJ_POINT,
        sequence,
        pack_trajectory_payload([target_deg, 0.0, 0.0, 0.0, 0.0, 0.0], duration_ms),
    )


def _read_j1(port: str, baud: int, sequence: int, timeout_s: float) -> dict[str, object]:
    response = _exchange_wire(
        Frame(CMD_GET_JOINT, sequence, bytes((J1,))),
        port=port,
        baud=baud,
        timeout_s=timeout_s,
    )
    if response.cmd != RSP_ACK:
        raise RuntimeError(f"GET_JOINT returned response 0x{response.cmd:02X}")
    return decode_joint_payload(response.payload)


def _stop_best_effort(port: str, baud: int, sequence: int) -> None:
    try:
        _exchange_wire(Frame(CMD_STOP, sequence, b""), port=port, baud=baud, timeout_s=1.0)
        print("STOP sent")
    except Exception as exc:  # The physical E-stop remains the final fallback.
        print(f"STOP UART failed: {exc}; use the physical E-stop", file=sys.stderr)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target-deg", type=float, required=True, help="J1 absolute target")
    parser.add_argument("--duration-ms", type=int, default=2500)
    parser.add_argument("--min-deg", type=float, required=True, help="measured conservative J1 minimum")
    parser.add_argument("--max-deg", type=float, required=True, help="measured conservative J1 maximum")
    parser.add_argument("--max-delta-deg", type=float, default=30.0)
    parser.add_argument("--tolerance-deg", type=float, default=0.8)
    parser.add_argument("--settle-timeout-s", type=float, default=45.0)
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--sequence", type=int, default=32, choices=range(0, 256))
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm", default="")
    parser.add_argument("--only-j1-connected", action="store_true")
    parser.add_argument("--physical-estop-confirmed", action="store_true")
    args = parser.parse_args()

    if not all(math.isfinite(value) for value in (args.target_deg, args.min_deg, args.max_deg)):
        parser.error("target and measured bounds must be finite")
    if args.min_deg >= args.max_deg:
        parser.error("min-deg must be less than max-deg")
    if args.target_deg < args.min_deg or args.target_deg > args.max_deg:
        parser.error("target is outside the supplied measured J1 bounds")
    if not math.isfinite(args.max_delta_deg) or args.max_delta_deg <= 0.0:
        parser.error("max-delta-deg must be finite and positive")
    try:
        frame = build_j1_frame(args.target_deg, args.duration_ms, args.sequence)
    except ValueError as exc:
        parser.error(str(exc))

    print(f"J1 absolute target={args.target_deg:.3f} deg, duration={args.duration_ms} ms")
    print(f"TX CMD_TRAJ_POINT payload: {frame.payload.hex(' ').upper()}")
    if not args.execute:
        print("Dry run: no serial device opened and no motion command sent")
        print(
            f"Real execution requires --execute --confirm {CONFIRM} "
            "--only-j1-connected --physical-estop-confirmed"
        )
        return 0
    if args.confirm != CONFIRM:
        parser.error(f"--execute requires --confirm {CONFIRM}")
    if not args.only_j1_connected:
        parser.error("real J1 test requires --only-j1-connected")
    if not args.physical_estop_confirmed:
        parser.error("real J1 test requires --physical-estop-confirmed")

    try:
        before = _read_j1(args.port, args.baud, args.sequence, 2.0)
        if before["joint_id"] != J1 or not before["online"]:
            raise RuntimeError(f"J1 is not online: {before}")
        current = float(before["angle_deg"])
        current_speed = abs(float(before["speed_rpm"]))
        if current < args.min_deg or current > args.max_deg:
            raise RuntimeError(f"current J1 angle {current:.3f} is outside measured bounds")
        if current_speed > 1.0:
            raise RuntimeError(f"J1 is not stationary enough: speed={current_speed:.3f} rpm")
        delta = abs(args.target_deg - current)
        if delta > args.max_delta_deg:
            raise RuntimeError(f"requested move {delta:.3f} deg exceeds max-delta-deg")
        print(f"Preflight OK: current={current:.3f} deg, speed={current_speed:.3f} rpm, delta={delta:.3f} deg, online=True")

        response = _exchange_wire(frame, port=args.port, baud=args.baud, timeout_s=2.0)
        if response.cmd not in (RSP_ACK, RSP_TRAJ_ACK):
            raise RuntimeError(f"STM32 rejected CMD_TRAJ_POINT: response=0x{response.cmd:02X}")
        print(f"STM32 trajectory response=0x{response.cmd:02X}; polling J1 feedback")

        deadline = time.monotonic() + args.settle_timeout_s
        sequence = (args.sequence + 1) & 0xFF
        while time.monotonic() < deadline:
            time.sleep(0.25)
            state = _read_j1(args.port, args.baud, sequence, 1.0)
            sequence = (sequence + 1) & 0xFF
            if not state["online"]:
                raise RuntimeError("J1 went offline during motion")
            angle = float(state["angle_deg"])
            speed = abs(float(state["speed_rpm"]))
            print(f"feedback angle={angle:.3f} deg speed={speed:.3f} rpm")
            if abs(angle - args.target_deg) <= args.tolerance_deg and speed <= 0.2:
                print("J1 target reached and settled")
                return 0
        raise RuntimeError("J1 did not settle before timeout")
    except Exception as exc:
        print(f"J1 test aborted: {exc}", file=sys.stderr)
        _stop_best_effort(args.port, args.baud, (args.sequence + 200) & 0xFF)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
