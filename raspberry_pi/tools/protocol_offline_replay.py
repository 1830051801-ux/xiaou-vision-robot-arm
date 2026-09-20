#!/usr/bin/env python3
"""Replay UART/CAN/J5 freshness cases without touching a device.

The generated frames are decoded by the same Pi-side protocol modules used on
the real arm.  CRC errors, fragmented UART input, invalid IDs, stale J5 data,
and the locked hardware configuration are all checked locally.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import struct
import sys
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT / "robot_ai") not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT / "robot_ai"))

from arm_control import can_protocol, uart_protocol
from arm_control.feedback_quality import analyse_feedback_samples
from arm_control.safety import load_hardware_config, validate_motion_readiness


def _uart_payload(joint_id: int, *, online: bool = True) -> bytes:
    return bytes((joint_id,)) + struct.pack("<fffB", -5.0 + joint_id, 0.25, 0.10, int(online))


def _fragment(data: bytes, widths: tuple[int, ...] = (1, 2, 3, 1, 5)) -> list[bytes]:
    chunks: list[bytes] = []
    offset = 0
    index = 0
    while offset < len(data):
        width = widths[index % len(widths)]
        chunks.append(data[offset : offset + width])
        offset += width
        index += 1
    return chunks


def _freshness(samples: list[dict[str, Any]], *, now_s: float, timeout_s: float) -> dict[str, Any]:
    ages = {int(item["joint_id"]): now_s - float(item["timestamp_s"]) for item in samples}
    return {
        "timeout_s": timeout_s,
        "ages_s": ages,
        "online_joint_ids": sorted(
            joint_id for joint_id, age in ages.items() if 0.0 <= age <= timeout_s
        ),
        "stale_joint_ids": sorted(
            joint_id for joint_id, age in ages.items() if age < 0.0 or age > timeout_s
        ),
    }


def replay() -> dict[str, Any]:
    failures: list[str] = []

    # UART frame round-trip, fragmentation, and CRC rejection.
    seq = 31
    wire = uart_protocol.encode_frame(uart_protocol.CMD_GET_JOINT, seq, _uart_payload(5))
    parser = uart_protocol.FrameParser()
    decoded = []
    for chunk in _fragment(wire):
        decoded.extend(parser.feed(chunk))
    if len(decoded) != 1 or decoded[0].seq != seq:
        failures.append("UART fragmented frame was not decoded")
    joint = uart_protocol.decode_joint_payload(decoded[0].payload) if decoded else {}
    if joint.get("joint_id") != 5 or joint.get("online") is not True:
        failures.append("UART J5 payload decode mismatch")
    corrupted = bytearray(wire)
    corrupted[-2] ^= 0xFF
    if uart_protocol.FrameParser().feed(corrupted):
        failures.append("UART CRC corruption was accepted")

    # Six-joint freshness gate: J5 is deliberately stale in the first case.
    samples = [
        {"joint_id": joint_id, "timestamp_s": 10.0 if joint_id != 5 else 8.0}
        for joint_id in range(1, 7)
    ]
    freshness = _freshness(samples, now_s=10.25, timeout_s=0.50)
    if freshness["stale_joint_ids"] != [5]:
        failures.append(f"J5 freshness classification mismatch: {freshness}")
    quality_samples = [
        {
            "joint_id": item["joint_id"],
            "ok": True,
            "seq": item["joint_id"],
            "response_seq": item["joint_id"],
            "observed_at_monotonic_s": item["timestamp_s"],
            "round_trip_ms": 20.0 + item["joint_id"],
            "feedback": {"joint_id": item["joint_id"], "online": item["joint_id"] != 6},
        }
        for item in samples
    ]
    quality = analyse_feedback_samples(
        quality_samples,
        now_monotonic_s=10.25,
        freshness_threshold_s=0.50,
        min_online_ratio=1.0,
    )
    if quality["per_joint"]["5"]["freshness_state"] != "stale":
        failures.append("J5 detailed freshness classification mismatch")
    if quality["per_joint"]["6"]["freshness_state"] != "offline_reported":
        failures.append("J6 controller-offline classification mismatch")

    # CAN command/feedback/diagnostic round-trip and invalid-ID rejection.
    can_id, payload = can_protocol.encode_position_command(5, 0.25, 0.10, sequence=7)
    command = can_protocol.decode_position_command(5, can_id, payload)
    if command.sequence != 7 or abs(command.position_rad - 0.25) > 1e-6:
        failures.append("CAN position command round-trip mismatch")
    feedback_payload = struct.pack("<BBih", can_protocol.FEEDBACK_OPCODE_STATUS, can_protocol.STATUS_HEARTBEAT, 250000, 100)
    feedback = can_protocol.decode_feedback(5, can_protocol.feedback_can_id(5), feedback_payload)
    if abs(feedback.position_rad - 0.25) > 1e-6 or feedback.status != can_protocol.STATUS_HEARTBEAT:
        failures.append("CAN feedback round-trip mismatch")
    diagnostic_id, diagnostic_payload = can_protocol.encode_diagnostic(5, 3, 0x1234)
    if diagnostic_id != can_protocol.diagnostic_can_id(5) or len(diagnostic_payload) != 8:
        failures.append("CAN diagnostic frame mismatch")
    try:
        can_protocol.decode_feedback(4, can_protocol.feedback_can_id(5), feedback_payload)
    except ValueError:
        invalid_can_rejected = True
    else:
        invalid_can_rejected = False
        failures.append("CAN wrong-node feedback was accepted")

    # The checked-in configuration must continue to reject motion readiness.
    config = load_hardware_config(PROJECT_ROOT / "robot_ai/arm_control/config/hardware_calibration.json")
    readiness = validate_motion_readiness(config)
    if readiness.ready:
        failures.append("hardware configuration unexpectedly reports motion ready")
    locked_fields = list(readiness.missing_or_invalid)

    return {
        "offline": True,
        "hardware_motion": False,
        "serial_opened": False,
        "can_opened": False,
        "uart": {
            "command": "CMD_GET_JOINT",
            "sequence": seq,
            "fragment_count": len(_fragment(wire)),
            "decoded_joint": joint,
            "crc_corruption_rejected": not bool(uart_protocol.FrameParser().feed(corrupted)),
        },
        "j5_freshness": freshness,
        "feedback_quality": quality,
        "can": {
            "command_id": hex(can_id),
            "feedback_id": hex(can_protocol.feedback_can_id(5)),
            "diagnostic_id": hex(diagnostic_id),
            "decoded_command": {
                "position_rad": command.position_rad,
                "velocity_rad_s": command.velocity_rad_s,
                "sequence": command.sequence,
            },
            "decoded_feedback": {
                "position_rad": feedback.position_rad,
                "velocity_rad_s": feedback.velocity_rad_s,
                "status": feedback.status,
            },
            "wrong_node_rejected": invalid_can_rejected,
        },
        "motion_gate": {"ready": readiness.ready, "locked_fields": locked_fields},
        "failure_count": len(failures),
        "failures": failures,
        "passed": not failures,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=PROJECT_ROOT / "runtime/simulations/protocol_offline_replay_20260809.json")
    args = parser.parse_args()
    report = replay()
    output = args.output if args.output.is_absolute() else PROJECT_ROOT / args.output
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
