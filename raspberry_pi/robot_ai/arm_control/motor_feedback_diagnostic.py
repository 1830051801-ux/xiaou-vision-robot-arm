#!/usr/bin/env python3
"""Read-only J1-J6 UART feedback diagnostic.

Only CMD_PING and CMD_GET_JOINT are transmitted.  This utility never enables,
moves, stops, clears errors on, calibrates, or configures the arm.
"""

from __future__ import annotations

import argparse
import json
import time
from typing import Any

try:
    from .feedback_quality import (
        analyse_feedback_samples,
        analyse_transport_events,
        latest_successful_observation_times,
        summarise_feedback_signature,
    )
except ImportError:  # Direct invocation from robot_ai/arm_control.
    from feedback_quality import (
        analyse_feedback_samples,
        analyse_transport_events,
        latest_successful_observation_times,
        summarise_feedback_signature,
    )

try:
    from .uart_protocol import (
        CMD_GET_JOINT,
        CMD_PING,
        DEFAULT_BAUD,
        DEFAULT_PORT,
        JOINT_COUNT,
        RSP_ACK,
        Frame,
        decode_joint_payload,
        exchange,
    )
except ImportError:  # Direct invocation from robot_ai/arm_control.
    from uart_protocol import (
        CMD_GET_JOINT,
        CMD_PING,
        DEFAULT_BAUD,
        DEFAULT_PORT,
        JOINT_COUNT,
        RSP_ACK,
        Frame,
        decode_joint_payload,
        exchange,
    )


def _positive_int(value: str) -> int:
    parsed = int(value)
    if parsed < 1:
        raise argparse.ArgumentTypeError("value must be at least 1")
    return parsed


def _joint_ids(value: str) -> list[int]:
    try:
        joint_ids = [int(part.strip()) for part in value.split(",") if part.strip()]
    except ValueError as exc:
        raise argparse.ArgumentTypeError("joint ids must be comma-separated integers") from exc
    if not joint_ids or len(set(joint_ids)) != len(joint_ids):
        raise argparse.ArgumentTypeError("joint ids must be non-empty and unique")
    if any(joint_id < 1 or joint_id > JOINT_COUNT for joint_id in joint_ids):
        raise argparse.ArgumentTypeError(f"joint ids must be in 1..{JOINT_COUNT}")
    return joint_ids


def _request(frame: Frame, *, port: str, baud: int, timeout_s: float) -> tuple[dict[str, Any] | None, str | None]:
    try:
        response = exchange(frame, port=port, baud=baud, timeout_s=timeout_s)
    except Exception as exc:
        return None, f"{type(exc).__name__}: {exc}"
    if response.cmd != RSP_ACK:
        return None, f"unexpected response command 0x{response.cmd:02X}"
    return {"response_cmd": f"0x{response.cmd:02X}", "response_seq": response.seq, "payload": response.payload}, None


def _ping(sequence: int, *, port: str, baud: int, timeout_s: float) -> dict[str, Any]:
    started_s = time.monotonic()
    response, error = _request(Frame(CMD_PING, sequence), port=port, baud=baud, timeout_s=timeout_s)
    completed_s = time.monotonic()
    if error is not None:
        return {
            "request_type": "ping",
            "seq": sequence,
            "ok": False,
            "error": error,
            "observed_at_monotonic_s": completed_s,
            "round_trip_ms": round((completed_s - started_s) * 1000.0, 3),
        }
    assert response is not None
    return {
        "request_type": "ping",
        "seq": sequence,
        "ok": True,
        "response_cmd": response["response_cmd"],
        "response_seq": response["response_seq"],
        "observed_at_monotonic_s": completed_s,
        "round_trip_ms": round((completed_s - started_s) * 1000.0, 3),
    }


def _read_joint(joint_id: int, sequence: int, *, port: str, baud: int, timeout_s: float) -> dict[str, Any]:
    started_s = time.monotonic()
    response, error = _request(
        Frame(CMD_GET_JOINT, sequence, bytes((joint_id,))),
        port=port,
        baud=baud,
        timeout_s=timeout_s,
    )
    completed_s = time.monotonic()
    if error is not None:
        return {
            "request_type": "get_joint",
            "joint_id": joint_id,
            "seq": sequence,
            "ok": False,
            "error": error,
            "observed_at_monotonic_s": completed_s,
            "round_trip_ms": round((completed_s - started_s) * 1000.0, 3),
        }
    assert response is not None
    try:
        feedback = decode_joint_payload(response["payload"])
    except Exception as exc:
        return {
            "request_type": "get_joint",
            "joint_id": joint_id,
            "seq": sequence,
            "ok": False,
            "error": f"{type(exc).__name__}: {exc}",
            "observed_at_monotonic_s": completed_s,
            "round_trip_ms": round((completed_s - started_s) * 1000.0, 3),
        }
    return {
        "request_type": "get_joint",
        "joint_id": joint_id,
        "seq": sequence,
        "ok": True,
        "response_cmd": response["response_cmd"],
        "response_seq": response["response_seq"],
        "feedback": feedback,
        "observed_at_monotonic_s": completed_s,
        "round_trip_ms": round((completed_s - started_s) * 1000.0, 3),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=_positive_int, default=DEFAULT_BAUD)
    parser.add_argument("--timeout-s", type=float, default=3.0)
    parser.add_argument("--ping-count", type=_positive_int, default=2)
    parser.add_argument("--samples-per-joint", type=_positive_int, default=5)
    parser.add_argument("--joint-ids", type=_joint_ids, default=list(range(1, JOINT_COUNT + 1)))
    parser.add_argument("--interval-s", type=float, default=0.15)
    parser.add_argument(
        "--startup-wait-s",
        type=float,
        default=0.0,
        help="read-only wait before the first PING; use after F407 power-up without hiding recovery evidence",
    )
    parser.add_argument("--freshness-threshold-s", type=float, default=0.50)
    parser.add_argument("--min-online-ratio", type=float, default=1.0)
    parser.add_argument("--max-p95-latency-ms", type=float)
    parser.add_argument(
        "--fast-response-threshold-ms",
        type=float,
        default=100.0,
        help="successful response above this duration is diagnosed as delayed",
    )
    parser.add_argument("--strict", action="store_true", help="return exit code 2 when transport or feedback quality fails")
    args = parser.parse_args()
    if args.timeout_s <= 0.0:
        parser.error("--timeout-s must be positive")
    if args.interval_s < 0.0:
        parser.error("--interval-s must be non-negative")
    if args.startup_wait_s < 0.0:
        parser.error("--startup-wait-s must be non-negative")
    if args.freshness_threshold_s <= 0.0:
        parser.error("--freshness-threshold-s must be positive")
    if not 0.0 <= args.min_online_ratio <= 1.0:
        parser.error("--min-online-ratio must be in [0, 1]")
    if args.max_p95_latency_ms is not None and args.max_p95_latency_ms <= 0.0:
        parser.error("--max-p95-latency-ms must be positive")
    if args.fast_response_threshold_ms <= 0.0:
        parser.error("--fast-response-threshold-ms must be positive")

    if args.startup_wait_s:
        time.sleep(args.startup_wait_s)

    sequence = 1
    pings = []
    for _ in range(args.ping_count):
        pings.append(_ping(sequence, port=args.port, baud=args.baud, timeout_s=args.timeout_s))
        sequence = (sequence + 1) & 0xFF
        time.sleep(args.interval_s)

    samples = []
    for sample_index in range(1, args.samples_per_joint + 1):
        for joint_id in args.joint_ids:
            result = _read_joint(joint_id, sequence, port=args.port, baud=args.baud, timeout_s=args.timeout_s)
            result["sample"] = sample_index
            samples.append(result)
            sequence = (sequence + 1) & 0xFF
            time.sleep(args.interval_s)

    per_joint: dict[str, dict[str, Any]] = {}
    for joint_id in args.joint_ids:
        joint_samples = [item for item in samples if item["joint_id"] == joint_id]
        successful = [item for item in joint_samples if item["ok"]]
        online = [item for item in successful if bool(item["feedback"]["online"])]
        per_joint[str(joint_id)] = {
            "attempts": len(joint_samples),
            "query_pass": len(successful),
            "query_fail": len(joint_samples) - len(successful),
            "online_samples": len(online),
            "offline_samples": len(successful) - len(online),
            "last_feedback": successful[-1]["feedback"] if successful else None,
        }

    failed_pings = sum(not item["ok"] for item in pings)
    failed_queries = sum(not item["ok"] for item in samples)
    collection_completed_s = time.monotonic()
    freshness_reference_s_by_joint = latest_successful_observation_times(
        samples, joint_ids=args.joint_ids
    )
    quality = analyse_feedback_samples(
        samples,
        now_monotonic_s=collection_completed_s,
        freshness_threshold_s=args.freshness_threshold_s,
        min_online_ratio=args.min_online_ratio,
        max_p95_latency_ms=args.max_p95_latency_ms,
        joint_ids=args.joint_ids,
        freshness_reference_s_by_joint=freshness_reference_s_by_joint,
    )
    transport = analyse_transport_events(
        [*pings, *samples], fast_response_threshold_ms=args.fast_response_threshold_ms
    )
    feedback_signature = summarise_feedback_signature(samples, joint_ids=args.joint_ids)
    report = {
        "read_only": True,
        "allowed_commands": ["CMD_PING", "CMD_GET_JOINT"],
        "port": args.port,
        "baud": args.baud,
        "joint_ids": args.joint_ids,
        "diagnostic_settings": {
            "startup_wait_s": args.startup_wait_s,
            "timeout_s": args.timeout_s,
            "interval_s": args.interval_s,
            "fast_response_threshold_ms": args.fast_response_threshold_ms,
            "collection_completed_monotonic_s": collection_completed_s,
            "freshness_reference": "per_joint_latest_successful_sample_completion",
        },
        "pings": pings,
        "samples": samples,
        "summary": {
            "ping_pass": len(pings) - failed_pings,
            "ping_fail": failed_pings,
            "joint_query_pass": len(samples) - failed_queries,
            "joint_query_fail": failed_queries,
            "per_joint": per_joint,
        },
        "transport_quality": transport,
        "feedback_quality": quality,
        "feedback_signature": feedback_signature,
    }
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 2 if args.strict and (not transport["passed"] or not quality["passed"]) else 0


if __name__ == "__main__":
    raise SystemExit(main())
