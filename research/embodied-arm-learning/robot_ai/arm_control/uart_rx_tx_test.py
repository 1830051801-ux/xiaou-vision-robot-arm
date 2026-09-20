#!/usr/bin/env python3
"""Full-duplex UART loopback test for Raspberry Pi GPIO TXD/RXD.

Safety boundary: this tool sends printable ASCII diagnostics only.  It never
creates the binary arm protocol or CAN frames, so it is suitable for proving
the UART electrical path before any control protocol is enabled.

For a local loopback test, temporarily connect GPIO14/TXD (physical pin 8) to
GPIO15/RXD (physical pin 10).  Remove that jumper before connecting an MCU.
"""

from __future__ import annotations

import argparse
import os
import sys

try:  # Package invocation: python -m robot_ai.arm_control.uart_rx_tx_test
    from .uart_txd_test import (
        DEFAULT_BAUD,
        DEFAULT_PORT,
        build_payload,
        configure_uart,
        read_for,
        termios,
        write_all,
    )
except ImportError:  # Direct invocation: python3 robot_ai/arm_control/uart_rx_tx_test.py
    from uart_txd_test import (
        DEFAULT_BAUD,
        DEFAULT_PORT,
        build_payload,
        configure_uart,
        read_for,
        termios,
        write_all,
    )


DEFAULT_TEXT = "UART_RX_TX_LOOPBACK"


def exchange(port: str, *, baud: int, text: str, read_seconds: float) -> tuple[bytes, bytes]:
    """Configure the UART, send printable text, then return TX and RX bytes."""
    if termios is None:
        raise RuntimeError("this UART utility must run on Linux (for example, Raspberry Pi OS)")
    if read_seconds <= 0:
        raise ValueError("read-seconds must be positive")

    payload = build_payload(text)
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        configure_uart(fd, baud=baud)
        termios.tcflush(fd, termios.TCIOFLUSH)
        write_all(fd, payload)
        return payload, read_for(fd, read_seconds)
    finally:
        os.close(fd)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Safe UART TX/RX text loopback test; does not send arm command frames."
    )
    parser.add_argument("--port", default=DEFAULT_PORT, help="UART device, normally /dev/serial0")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, choices=(9600, 19200, 38400, 57600, 115200))
    parser.add_argument("--text", default=DEFAULT_TEXT, help="printable ASCII test payload")
    parser.add_argument("--read-seconds", type=float, default=2.0, help="time to wait for a reply or loopback")
    args = parser.parse_args()

    try:
        tx, rx = exchange(args.port, baud=args.baud, text=args.text, read_seconds=args.read_seconds)
    except (OSError, RuntimeError, ValueError, UnicodeError) as exc:
        print(f"UART RX/TX test failed: {exc}", file=sys.stderr)
        return 1

    print(f"UART: {args.port}, {args.baud} 8N1, no flow control")
    print(f"TX HEX: {tx.hex(' ').upper()}")
    print(f"RX HEX: {rx.hex(' ').upper() if rx else '<none>'}")
    if rx == tx:
        print("PASS: exact TXD->RXD loopback received.")
        return 0
    if rx:
        print("RX received but differs from TX; this is expected with an external responder.")
        return 0
    print("NO RX: check the TXD/RXD loopback jumper, shared ground, port, and baud rate.")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
