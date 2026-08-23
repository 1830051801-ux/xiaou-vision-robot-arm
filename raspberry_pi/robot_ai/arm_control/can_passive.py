from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import socket
import struct
import subprocess
import time
from typing import Any


CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_EFF_MASK = 0x1FFFFFFF
CAN_SFF_MASK = 0x000007FF
CAN_FRAME_STRUCT = struct.Struct("=IB3x8s")


def parse_can_frame(frame: bytes, *, received_at: str | None = None) -> dict[str, Any]:
    if len(frame) < CAN_FRAME_STRUCT.size:
        raise ValueError(f"classic CAN frame must be at least 16 bytes, got {len(frame)}")
    can_id_raw, dlc, payload = CAN_FRAME_STRUCT.unpack(frame[: CAN_FRAME_STRUCT.size])
    dlc = min(int(dlc), 8)
    is_extended = bool(can_id_raw & CAN_EFF_FLAG)
    identifier = can_id_raw & (CAN_EFF_MASK if is_extended else CAN_SFF_MASK)
    data = payload[:dlc]
    protocol_hint = None
    if not is_extended:
        protocol_hint = {
            "family": "DrEmpower_CAN_v2.1_candidate",
            "node_id_candidate": (identifier >> 5) & 0x3F,
            "opcode_candidate": identifier & 0x1F,
            "confirmed": False,
        }
    return {
        "event": "can_frame",
        "received_at": received_at or datetime.now(timezone.utc).isoformat(),
        "can_id": identifier,
        "can_id_hex": f"0x{identifier:08X}" if is_extended else f"0x{identifier:03X}",
        "extended": is_extended,
        "remote": bool(can_id_raw & CAN_RTR_FLAG),
        "error": bool(can_id_raw & CAN_ERR_FLAG),
        "dlc": dlc,
        "data_hex": data.hex(" ").upper(),
        "data_bytes": list(data),
        "protocol_hint": protocol_hint,
    }


def read_interface_status(interface: str) -> dict[str, Any]:
    try:
        result = subprocess.run(
            ["ip", "-details", "-statistics", "link", "show", interface],
            check=False,
            capture_output=True,
            text=True,
            timeout=3,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
        return {"available": False, "error": str(exc)}
    return {
        "available": result.returncode == 0,
        "returncode": result.returncode,
        "stdout": result.stdout.strip(),
        "stderr": result.stderr.strip(),
    }


def listen(interface: str, log_path: Path, *, duration_s: float = 30.0, max_frames: int | None = None) -> int:
    if duration_s <= 0.0:
        raise ValueError("duration_s must be positive")
    log_path.parent.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    frame_count = 0
    error_count = 0
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.settimeout(0.25)
    sock.bind((interface,))
    with log_path.open("a", encoding="utf-8", buffering=1) as log_file:
        header = {
            "event": "listener_start",
            "time": datetime.now(timezone.utc).isoformat(),
            "interface": interface,
            "duration_s": duration_s,
            "interface_status": read_interface_status(interface),
            "safety": "receive_only_no_can_transmit_calls",
        }
        log_file.write(json.dumps(header, ensure_ascii=False) + "\n")
        try:
            while time.monotonic() - started < duration_s:
                if max_frames is not None and frame_count >= max_frames:
                    break
                try:
                    frame = sock.recv(CAN_FRAME_STRUCT.size)
                except socket.timeout:
                    continue
                record = parse_can_frame(frame)
                frame_count += 1
                error_count += int(record["error"])
                log_file.write(json.dumps(record, ensure_ascii=False) + "\n")
                print(f"{record['received_at']} {record['can_id_hex']} [{record['dlc']}] {record['data_hex']}")
        finally:
            sock.close()
            footer = {
                "event": "listener_stop",
                "time": datetime.now(timezone.utc).isoformat(),
                "frames": frame_count,
                "error_frames": error_count,
                "elapsed_s": round(time.monotonic() - started, 6),
            }
            log_file.write(json.dumps(footer, ensure_ascii=False) + "\n")
    return frame_count


def main() -> int:
    parser = argparse.ArgumentParser(description="Receive-only SocketCAN logger for the six-axis arm")
    parser.add_argument("--interface", default="can0")
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--max-frames", type=int)
    parser.add_argument(
        "--log",
        type=Path,
        default=Path("runtime/logs/arm_can_raw.jsonl"),
    )
    args = parser.parse_args()
    print("Receive-only mode: this program has no CAN transmit operation.")
    try:
        count = listen(args.interface, args.log, duration_s=args.duration, max_frames=args.max_frames)
    except (OSError, ValueError) as exc:
        print(f"CAN listener failed: {exc}")
        return 1
    print(f"Captured {count} frame(s); log: {args.log}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
