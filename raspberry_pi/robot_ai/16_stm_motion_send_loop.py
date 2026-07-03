from __future__ import annotations

import argparse
import time

import serial

from robot_protocol import decode_motion_frame, encode_motion_frame, frame_to_hex


def build_payload(args: argparse.Namespace) -> dict:
    return {
        "cmd": "move_cart",
        "x_base_mm": args.x,
        "y_base_mm": args.y,
        "z_mm": args.z,
        "rx_deg": args.rx,
        "ry_deg": args.ry,
        "rz_deg": args.rz,
        "speed": args.speed,
        "accel": args.accel,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Send STM32 cartesian motion frame periodically.")
    parser.add_argument("--port", default="/dev/serial0", help="serial port")
    parser.add_argument("--baud", type=int, default=115200, help="serial baud rate")
    parser.add_argument("--interval", type=float, default=5.0, help="send interval seconds")
    parser.add_argument("--seq", type=int, default=2, help="frame sequence id")
    parser.add_argument("--x", type=float, default=200.0, help="cartesian x in mm")
    parser.add_argument("--y", type=float, default=30.0, help="cartesian y in mm")
    parser.add_argument("--z", type=float, default=25.0, help="cartesian z in mm")
    parser.add_argument("--rx", type=float, default=0.0, help="cartesian rx in degrees")
    parser.add_argument("--ry", type=float, default=0.0, help="cartesian ry in degrees")
    parser.add_argument("--rz", type=float, default=0.0, help="cartesian rz in degrees")
    parser.add_argument("--speed", type=float, default=50.0, help="motion speed")
    parser.add_argument("--accel", type=float, default=100.0, help="motion acceleration")
    parser.add_argument("--read-window", type=float, default=0.3, help="seconds to print bytes returned by STM after each TX")
    parser.add_argument("--count", type=int, default=0, help="0 means keep sending until Ctrl+C")
    args = parser.parse_args()

    interval = max(0.1, args.interval)
    payload = build_payload(args)
    frame = encode_motion_frame(payload, seq=args.seq)
    parsed = decode_motion_frame(frame)

    print(f"port: {args.port} @ {args.baud}")
    print(f"interval: {interval:.1f}s")
    print(f"payload: {payload}")
    print(f"frame check: cmd=0x{parsed['cmd']:02X}, payload_len={len(parsed['payload'])}, seq={parsed['seq']}")
    print(f"motion: {frame_to_hex(frame)}")
    print("Press Ctrl+C to stop.")

    sent = 0
    with serial.Serial(args.port, args.baud, timeout=1, write_timeout=1) as ser:
        time.sleep(0.5)
        while args.count <= 0 or sent < args.count:
            ser.write(frame)
            ser.flush()
            sent += 1
            print(f"sent {sent}: {frame_to_hex(frame)}")
            deadline = time.monotonic() + max(0.0, args.read_window)
            rx = bytearray()
            while time.monotonic() < deadline:
                waiting = ser.in_waiting
                if waiting:
                    rx.extend(ser.read(waiting))
                else:
                    time.sleep(0.02)
            if rx:
                print(f"rx {len(rx)} byte(s): {frame_to_hex(bytes(rx))}")
            time.sleep(interval)


if __name__ == "__main__":
    main()
