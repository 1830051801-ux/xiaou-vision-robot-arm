#!/usr/bin/env python3
"""Read-only UART health and J1-J6 feedback probe.

This utility only sends CMD_PING, CMD_GET_JOINT, and optionally CMD_GET_STATE.
It never sends motion, enable, calibration, error-clear, configuration, CAN,
or PWM commands.
"""

from __future__ import annotations

import argparse
import json
import time
from typing import Any

try:
    from .uart_protocol import (
        CMD_GET_JOINT,
        CMD_GET_STATE,
        CMD_PING,
        DEFAULT_BAUD,
        DEFAULT_PORT,
        JOINT_COUNT,
        RSP_ACK,
        Frame,
        decode_joint_payload,
        decode_state_payload,
        exchange,
    )
except ImportError:  # Direct invocation from robot_ai/arm_control.
    from uart_protocol import (
        CMD_GET_JOINT,
        CMD_GET_STATE,
        CMD_PING,
        DEFAULT_BAUD,
        DEFAULT_PORT,
        JOINT_COUNT,
        RSP_ACK,
        Frame,
        decode_joint_payload,
        decode_state_payload,
        exchange,
    )


def _positive_int(value: str) -> int:
    parsed = int(value)
    if parsed < 1:
        raise argparse.ArgumentTypeError("value must be at least 1")
    return parsed


def _request(frame: Frame, *, port: str, baud: int, timeout_s: float) -> tuple[dict[str, Any] | None, str | None]:
    try:
        response = exchange(frame, port=port, baud=baud, timeout_s=timeout_s)
    except Exception as exc:
        return None, f"{type(exc).__name__}: {exc}"
    if response.cmd != RSP_ACK:
        return None, f"unexpected response command 0x{response.cmd:02X}"
    return {
        "response_cmd": f"0x{response.cmd:02X}",
        "response_seq": response.seq,
        "payload": response.payload,
    }, None


def _ping(seq: int, *, port: str, baud: int, timeout_s: float) -> dict[str, Any]:
    response, error = _request(Frame(CMD_PING, seq), port=port, baud=baud, timeout_s=timeout_s)
    if error is not None:
        return {"seq": seq, "ok": False, "error": error}
    assert response is not None
    return {
        "seq": seq,
        "ok": True,
        "response_cmd": response["response_cmd"],
        "response_seq": response["response_seq"],
    }


def _joint_status(joint_id: int, seq: int, *, port: str, baud: int, timeout_s: float) -> dict[str, Any]:
    response, error = _request(
        Frame(CMD_GET_JOINT, seq, bytes((joint_id,))),
        port=port,
        baud=baud,
        timeout_s=timeout_s,
    )
    if error is not None:
        return {"joint_id": joint_id, "seq": seq, "ok": False, "error": error}
    assert response is not None
    try:
        feedback = decode_joint_payload(response["payload"])
    except Exception as exc:
        return {
            "joint_id": joint_id,
            "seq": seq,
            "ok": False,
            "error": f"{type(exc).__name__}: {exc}",
        }
    return {
        "joint_id": joint_id,
        "seq": seq,
        "ok": True,
        "response_cmd": response["response_cmd"],
        "response_seq": response["response_seq"],
        "feedback": feedback,
    }


def _state_status(seq: int, *, port: str, baud: int, timeout_s: float) -> dict[str, Any]:
    response, error = _request(Frame(CMD_GET_STATE, seq), port=port, baud=baud, timeout_s=timeout_s)
    if error is not None:
        return {"seq": seq, "ok": False, "error": error}
    assert response is not None
    try:
        state = decode_state_payload(response["payload"])
    except Exception as exc:
        return {"seq": seq, "ok": False, "error": f"{type(exc).__name__}: {exc}"}
    return {
        "seq": seq,
        "ok": True,
        "response_cmd": response["response_cmd"],
        "response_seq": response["response_seq"],
        "state": state,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=_positive_int, default=DEFAULT_BAUD)
    parser.add_argument("--timeout-s", type=float, default=1.0)
    parser.add_argument("--ping-count", type=_positive_int, default=5)
    parser.add_argument("--rounds", type=_positive_int, default=3)
    parser.add_argument("--interval-s", type=float, default=0.15)
    parser.add_argument("--include-state", action="store_true", help="also send one read-only GET_STATE request")
    args = parser.parse_args()

    if args.timeout_s <= 0.0:
        parser.error("--timeout-s must be positive")
    if args.interval_s < 0.0:
        parser.error("--interval-s must be non-negative")

    sequence = 1
    pings = []
    for _ in range(args.ping_count):
        pings.append(_ping(sequence, port=args.port, baud=args.baud, timeout_s=args.timeout_s))
        sequence = (sequence + 1) & 0xFF
        time.sleep(args.interval_s)

    state = None
    if args.include_state:
        state = _state_status(sequence, port=args.port, baud=args.baud, timeout_s=args.timeout_s)
        sequence = (sequence + 1) & 0xFF
        time.sleep(args.interval_s)

    scans = []
    for round_index in range(1, args.rounds + 1):
        joints = []
        for joint_id in range(1, JOINT_COUNT + 1):
            joints.append(_joint_status(joint_id, sequence, port=args.port, baud=args.baud, timeout_s=args.timeout_s))
            sequence = (sequence + 1) & 0xFF
            time.sleep(args.interval_s)
        scans.append({"round": round_index, "joints": joints})

    failed_pings = sum(not item["ok"] for item in pings)
    failed_joint_queries = sum(not item["ok"] for scan in scans for item in scan["joints"])
    online_joint_ids = sorted(
        {
            item["feedback"]["joint_id"]
            for scan in scans
            for item in scan["joints"]
            if item["ok"] and bool(item["feedback"]["online"])
        }
    )
    report = {
        "read_only": True,
        "allowed_commands": ["CMD_PING", "CMD_GET_JOINT"] + (["CMD_GET_STATE"] if args.include_state else []),
        "port": args.port,
        "baud": args.baud,
        "pings": pings,
        "state": state,
        "scans": scans,
        "summary": {
            "ping_pass": len(pings) - failed_pings,
            "ping_fail": failed_pings,
            "joint_query_pass": args.rounds * JOINT_COUNT - failed_joint_queries,
            "joint_query_fail": failed_joint_queries,
            "online_joint_ids": online_joint_ids,
        },
    }
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if failed_pings == 0 and failed_joint_queries == 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
