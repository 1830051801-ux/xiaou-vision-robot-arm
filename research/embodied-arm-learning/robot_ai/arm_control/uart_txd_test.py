#!/usr/bin/env python3
"""Safe Raspberry Pi UART/TXD text test.

This utility configures a Linux serial device as 115200 8N1 with no flow
control, sends an ASCII diagnostic line, and optionally prints received bytes.
It deliberately accepts text only: it is not an arm command sender and does
not construct CAN or motion-protocol frames.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import select
import sys
import time

try:
    import termios
except ImportError:  # Allows source checks on non-Linux development hosts.
    termios = None  # type: ignore[assignment]


DEFAULT_PORT = "/dev/serial0"
DEFAULT_BAUD = 115200
DEFAULT_TEXT = "HELLO_FROM_PI_TXD"


def build_payload(text: str, *, append_crlf: bool = True) -> bytes:
    """Return a printable, non-protocol diagnostic payload."""
    if not text:
        raise ValueError("text must not be empty")
    if any(ord(ch) < 0x20 and ch not in "\r\n\t" for ch in text):
        raise ValueError("text contains a non-printable control character")
    suffix = "\r\n" if append_crlf else ""
    return (text + suffix).encode("ascii", "strict")


def _baud_constant(baud: int) -> int:
    if termios is None:
        raise RuntimeError("UART configuration requires Linux termios")
    supported = {
        9600: termios.B9600,
        19200: termios.B19200,
        38400: termios.B38400,
        57600: termios.B57600,
        115200: termios.B115200,
    }
    try:
        return supported[baud]
    except KeyError as exc:
        raise ValueError(f"unsupported baud rate: {baud}") from exc


def configure_uart(fd: int, *, baud: int) -> None:
    """Configure an already-open Linux TTY as raw 8N1 without flow control."""
    if termios is None:
        raise RuntimeError("UART configuration requires Linux termios")

    speed = _baud_constant(baud)
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0  # input flags: no software flow control or byte translation
    attrs[1] = 0  # output flags: raw bytes
    attrs[2] &= ~(termios.PARENB | termios.CSTOPB | termios.CSIZE)
    if hasattr(termios, "CRTSCTS"):
        attrs[2] &= ~termios.CRTSCTS
    attrs[2] |= termios.CS8 | termios.CREAD | termios.CLOCAL
    attrs[3] = 0  # local flags: no canonical mode, echo, or signals
    attrs[4] = speed
    attrs[5] = speed
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)


def write_all(fd: int, payload: bytes) -> None:
    """Write every byte, handling short writes from the TTY driver."""
    offset = 0
    while offset < len(payload):
        written = os.write(fd, payload[offset:])
        if written <= 0:
            raise OSError("UART write returned no progress")
        offset += written
    if termios is not None:
        termios.tcdrain(fd)


def read_for(fd: int, duration_s: float) -> bytes:
    """Collect any reply without extending the requested timeout."""
    if duration_s < 0:
        raise ValueError("read duration must not be negative")
    deadline = time.monotonic() + duration_s
    received = bytearray()
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        readable, _, _ = select.select([fd], [], [], remaining)
        if not readable:
            break
        chunk = os.read(fd, 256)
        if not chunk:
            break
        received.extend(chunk)
    return bytes(received)


def send_text(port: str, *, baud: int, text: str, append_crlf: bool, read_seconds: float) -> bytes:
    """Open, configure, send a text line, and collect an optional reply."""
    if termios is None:
        raise RuntimeError("this UART utility must run on Linux (for example, Raspberry Pi OS)")

    payload = build_payload(text, append_crlf=append_crlf)
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        configure_uart(fd, baud=baud)
        write_all(fd, payload)
        return read_for(fd, read_seconds)
    finally:
        os.close(fd)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Send a safe ASCII diagnostic line through Raspberry Pi TXD (no arm command frames)."
    )
    parser.add_argument("--port", default=DEFAULT_PORT, help="UART device, normally /dev/serial0")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, choices=(9600, 19200, 38400, 57600, 115200))
    parser.add_argument("--text", default=DEFAULT_TEXT, help="ASCII diagnostic text to send")
    parser.add_argument("--no-crlf", action="store_true", help="do not append CRLF to the text")
    parser.add_argument("--read-seconds", type=float, default=1.5, help="reply collection time after TX")
    args = parser.parse_args()

    try:
        payload = build_payload(args.text, append_crlf=not args.no_crlf)
        print(f"UART TX test: port={args.port} baud={args.baud} 8N1 no-flow-control")
        print(f"TX ASCII: {payload.decode('ascii').rstrip()!r}")
        print(f"TX HEX:   {payload.hex(' ').upper()}")
        reply = send_text(
            args.port,
            baud=args.baud,
            text=args.text,
            append_crlf=not args.no_crlf,
            read_seconds=args.read_seconds,
        )
    except (OSError, RuntimeError, ValueError, UnicodeError) as exc:
        print(f"UART TX test failed: {exc}", file=sys.stderr)
        return 1

    print(f"RX HEX:   {reply.hex(' ').upper() if reply else '<none>'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
