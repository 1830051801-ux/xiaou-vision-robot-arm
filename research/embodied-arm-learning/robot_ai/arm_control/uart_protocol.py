#!/usr/bin/env python3
"""F407 UART framing and safe Pi-side transport for the formal arm baseline.

This module is deliberately a byte transport. It does not open CAN, calculate
joint motion, or let a VLA write joint commands. The command-line interface
only exposes PING, which is safe with the F407 transport-only firmware build.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
import os
from pathlib import Path
import select
import struct
import sys
import time
from typing import Iterable

try:
    import termios
except ImportError:  # Allows codec tests on Windows.
    termios = None  # type: ignore[assignment]


FRAME_START = 0xAA
FRAME_END = 0x55
ESCAPE = 0xBB
MAX_PAYLOAD = 248
JOINT_COUNT = 6
JOINT_WIRE_SIZE = 13
STATE_PAYLOAD_LEN = 4 + JOINT_COUNT * JOINT_WIRE_SIZE
TELEMETRY_PAYLOAD_LEN = 4 + STATE_PAYLOAD_LEN
GET_JOINT_RESPONSE_LEN = 1 + JOINT_WIRE_SIZE

CMD_PING = 0x01
CMD_ESTOP = 0x02
CMD_CLEAR_ERROR = 0x03
CMD_STOP = 0x13
CMD_GET_STATE = 0x20
CMD_GET_JOINT = 0x21
CMD_STREAM_START = 0x22
CMD_STREAM_STOP = 0x23
CMD_SET_ZERO = 0x30
CMD_SET_PID = 0x31
CMD_SET_LIMIT = 0x32
CMD_REBOOT_JOINT = 0x33
CMD_SAVE_CONFIG = 0x34
CMD_CALIB_START = 0x35
CMD_CALIB_END = 0x36
CMD_GET_CALIB_STATUS = 0x39
CMD_TRAJ_POINT = 0x50
CMD_TRAJ_BUFFER_CLEAR = 0x51
CMD_GRIPPER = 0x52

RSP_ACK = 0x00
RSP_NACK = 0x01
RSP_BUSY = 0x02
RSP_ESTOP_ACTIVE = 0x03
RSP_INVALID_PARAM = 0x04
RSP_QUEUE_FULL = 0x05
RSP_MOTION_LOCKED = 0x06
RSP_TELEMETRY = 0x80

MOTION_COMMANDS = frozenset({CMD_TRAJ_POINT, CMD_GRIPPER})
DEFAULT_PORT = "/dev/serial0"
DEFAULT_BAUD = 115200


class ProtocolError(ValueError):
    """A frame is malformed or violates the formal F407 UART protocol."""


class ResponseError(RuntimeError):
    """The F407 returned a valid response that is not the expected ACK."""


@dataclass(frozen=True)
class Frame:
    """One decoded or outbound F407 UART frame."""

    cmd: int
    seq: int
    payload: bytes = b""

    def __post_init__(self) -> None:
        for label, value in (("cmd", self.cmd), ("seq", self.seq)):
            if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= 0xFF:
                raise ProtocolError(f"{label} must be one byte")
        if not isinstance(self.payload, bytes):
            object.__setattr__(self, "payload", bytes(self.payload))
        if len(self.payload) > MAX_PAYLOAD:
            raise ProtocolError(f"payload exceeds {MAX_PAYLOAD} bytes")


def crc16_modbus(data: bytes | bytearray | memoryview) -> int:
    """CRC-16/MODBUS: init 0xFFFF, reflected polynomial 0xA001."""

    crc = 0xFFFF
    for byte in data:
        crc ^= int(byte)
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else (crc >> 1)
    return crc & 0xFFFF


def _escape_payload(payload: bytes) -> bytes:
    out = bytearray()
    for byte in payload:
        if byte == FRAME_START:
            out.extend((ESCAPE, 0x0A))
        elif byte == FRAME_END:
            out.extend((ESCAPE, 0x05))
        elif byte == ESCAPE:
            out.extend((ESCAPE, 0x0B))
        else:
            out.append(byte)
    return bytes(out)


def encode_frame(cmd: int, seq: int, payload: bytes = b"") -> bytes:
    """Encode payload-only escaping; header and CRC remain raw bytes."""

    frame = Frame(cmd=cmd, seq=seq, payload=payload)
    header = bytes((frame.cmd, len(frame.payload), frame.seq))
    crc = crc16_modbus(header + frame.payload)
    return bytes((FRAME_START,)) + header + _escape_payload(frame.payload) + struct.pack("<H", crc) + bytes((FRAME_END,))


class FrameParser:
    """Streaming parser matching the F407 state machine."""

    _WAIT_START = 0
    _CMD = 1
    _LEN = 2
    _SEQ = 3
    _PAYLOAD = 4
    _CRC_LO = 5
    _CRC_HI = 6
    _WAIT_END = 7

    def __init__(self) -> None:
        self._state = self._WAIT_START
        self._cmd = 0
        self._length = 0
        self._seq = 0
        self._payload = bytearray()
        self._crc = 0xFFFF
        self._expected_crc = 0
        self._escape_next = False

    def _reset(self) -> None:
        self._state = self._WAIT_START
        self._cmd = 0
        self._length = 0
        self._seq = 0
        self._payload.clear()
        self._crc = 0xFFFF
        self._expected_crc = 0
        self._escape_next = False

    def _begin(self) -> None:
        self._reset()
        self._state = self._CMD

    def _crc_append(self, byte: int) -> None:
        self._crc ^= byte
        for _ in range(8):
            self._crc = (self._crc >> 1) ^ 0xA001 if (self._crc & 1) else (self._crc >> 1)
        self._crc &= 0xFFFF

    def feed(self, data: Iterable[int] | bytes | bytearray | memoryview) -> list[Frame]:
        """Return each valid frame while accepting arbitrarily fragmented data."""

        frames: list[Frame] = []
        for raw_byte in data:
            byte = int(raw_byte)
            if not 0 <= byte <= 0xFF:
                raise ProtocolError("input contains a non-byte value")

            if self._state == self._WAIT_START:
                if byte == FRAME_START:
                    self._begin()
                continue

            if self._state == self._CMD:
                self._cmd = byte
                self._crc_append(byte)
                self._state = self._LEN
                continue

            if self._state == self._LEN:
                if byte > MAX_PAYLOAD:
                    if byte == FRAME_START:
                        self._begin()
                    else:
                        self._reset()
                    continue
                self._length = byte
                self._crc_append(byte)
                self._state = self._SEQ
                continue

            if self._state == self._SEQ:
                self._seq = byte
                self._crc_append(byte)
                self._state = self._PAYLOAD if self._length else self._CRC_LO
                continue

            if self._state == self._PAYLOAD:
                if self._escape_next:
                    self._escape_next = False
                    substitutions = {0x0A: FRAME_START, 0x05: FRAME_END, 0x0B: ESCAPE}
                    if byte not in substitutions:
                        self._reset()
                        continue
                    byte = substitutions[byte]
                elif byte == ESCAPE:
                    self._escape_next = True
                    continue
                elif byte == FRAME_START:
                    self._begin()
                    continue
                elif byte == FRAME_END:
                    self._reset()
                    continue

                if len(self._payload) >= self._length:
                    self._reset()
                    continue
                self._payload.append(byte)
                self._crc_append(byte)
                if len(self._payload) == self._length:
                    self._state = self._CRC_LO
                continue

            if self._state == self._CRC_LO:
                self._expected_crc = byte
                self._state = self._CRC_HI
                continue

            if self._state == self._CRC_HI:
                self._expected_crc |= byte << 8
                self._state = self._WAIT_END
                continue

            if self._state == self._WAIT_END:
                if byte == FRAME_END and self._crc == self._expected_crc:
                    frames.append(Frame(self._cmd, self._seq, bytes(self._payload)))
                    self._reset()
                elif byte == FRAME_START:
                    self._begin()
                else:
                    self._reset()
        return frames


def is_motion_command(cmd: int) -> bool:
    return cmd in MOTION_COMMANDS


def pack_trajectory_payload(joint_angles_deg: Iterable[float], duration_ms: int) -> bytes:
    """Pack six joint targets in degrees and a uint16 duration for CMD_TRAJ_POINT."""

    angles = tuple(float(value) for value in joint_angles_deg)
    if len(angles) != JOINT_COUNT or not all(math.isfinite(value) for value in angles):
        raise ProtocolError("trajectory requires exactly six finite joint angles")
    if isinstance(duration_ms, bool) or not isinstance(duration_ms, int) or not 1 <= duration_ms <= 10000:
        raise ProtocolError("duration_ms must be an integer in 1..10000")
    return struct.pack("<6fH", *angles, duration_ms)


def pack_gripper_payload(pwm_angle_deg: float) -> bytes:
    """Pack the fixed F407 gripper PWM angle after syntax/range checks."""

    value = float(pwm_angle_deg)
    if not math.isfinite(value) or not 0.0 <= value <= 180.0:
        raise ProtocolError("gripper PWM angle must be finite and within 0..180 degrees")
    return struct.pack("<f", value)


def decode_state_payload(payload: bytes) -> dict[str, object]:
    """Decode the fixed 82-byte CMD_GET_STATE ACK payload."""

    if len(payload) != STATE_PAYLOAD_LEN:
        raise ProtocolError(f"state payload must be {STATE_PAYLOAD_LEN} bytes, got {len(payload)}")
    state, error_code, estop, motion_busy = payload[:4]
    joints: list[dict[str, object]] = []
    offset = 4
    for joint_index in range(JOINT_COUNT):
        angle_deg, speed_rpm, torque_nm, online = struct.unpack_from("<fffB", payload, offset)
        joints.append(
            {
                "joint_id": joint_index + 1,
                "angle_deg": angle_deg,
                "speed_rpm": speed_rpm,
                "torque_nm": torque_nm,
                "online": bool(online),
            }
        )
        offset += JOINT_WIRE_SIZE
    return {
        "state": state,
        "error_code": error_code,
        "estop": bool(estop),
        "motion_busy": bool(motion_busy),
        "joints": joints,
    }


def decode_joint_payload(payload: bytes) -> dict[str, object]:
    if len(payload) != GET_JOINT_RESPONSE_LEN:
        raise ProtocolError(f"joint payload must be {GET_JOINT_RESPONSE_LEN} bytes, got {len(payload)}")
    joint_id = payload[0]
    if not 1 <= joint_id <= JOINT_COUNT:
        raise ProtocolError("joint payload contains an invalid joint ID")
    angle_deg, speed_rpm, torque_nm, online = struct.unpack_from("<fffB", payload, 1)
    return {
        "joint_id": joint_id,
        "angle_deg": angle_deg,
        "speed_rpm": speed_rpm,
        "torque_nm": torque_nm,
        "online": bool(online),
    }


def _baud_constant(baud: int) -> int:
    if termios is None:
        raise RuntimeError("UART configuration requires Linux termios")
    values = {
        9600: termios.B9600,
        19200: termios.B19200,
        38400: termios.B38400,
        57600: termios.B57600,
        115200: termios.B115200,
    }
    try:
        return values[baud]
    except KeyError as exc:
        raise ProtocolError(f"unsupported baud rate: {baud}") from exc


def configure_uart(fd: int, *, baud: int) -> None:
    """Configure an already-open Linux TTY as raw 8N1 with no flow control."""

    if termios is None:
        raise RuntimeError("UART configuration requires Linux termios")
    speed = _baud_constant(baud)
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] &= ~(termios.PARENB | termios.CSTOPB | termios.CSIZE)
    if hasattr(termios, "CRTSCTS"):
        attrs[2] &= ~termios.CRTSCTS
    attrs[2] |= termios.CS8 | termios.CREAD | termios.CLOCAL
    attrs[3] = 0
    attrs[4] = speed
    attrs[5] = speed
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)


def _write_all(fd: int, data: bytes) -> None:
    offset = 0
    while offset < len(data):
        written = os.write(fd, data[offset:])
        if written <= 0:
            raise OSError("UART write made no progress")
        offset += written
    if termios is not None:
        termios.tcdrain(fd)


def _read_until_response(fd: int, *, expected_seq: int, timeout_s: float) -> Frame:
    parser = FrameParser()
    deadline = time.monotonic() + timeout_s
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            raise TimeoutError(f"no F407 UART response for sequence {expected_seq}")
        readable, _, _ = select.select([fd], [], [], remaining)
        if not readable:
            continue
        chunk = os.read(fd, 256)
        for frame in parser.feed(chunk):
            if frame.cmd != RSP_TELEMETRY and frame.seq == expected_seq:
                return frame


def _require_motion_ready(config_path: str | Path | None) -> None:
    try:
        from .safety import load_hardware_config, require_motion_ready
    except ImportError:  # Direct script invocation on the Pi.
        from safety import load_hardware_config, require_motion_ready

    config = load_hardware_config(config_path) if config_path is not None else load_hardware_config()
    require_motion_ready(config)


def exchange(
    frame: Frame,
    *,
    port: str = DEFAULT_PORT,
    baud: int = DEFAULT_BAUD,
    timeout_s: float = 0.3,
    hardware_config: str | Path | None = None,
) -> Frame:
    """Send one complete frame and return its matching response.

    Motion frames cannot enter the transport unless the existing Pi safety file
    passes every measured-hardware gate. PING, state queries, STOP, and ESTOP
    stay available for staged link validation.
    """

    if timeout_s <= 0.0 or not math.isfinite(timeout_s):
        raise ProtocolError("timeout_s must be finite and positive")
    if is_motion_command(frame.cmd):
        _require_motion_ready(hardware_config)

    if termios is None:
        raise RuntimeError("this UART transport must run on Linux, for example Raspberry Pi OS")
    wire = encode_frame(frame.cmd, frame.seq, frame.payload)
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        configure_uart(fd, baud=baud)
        termios.tcflush(fd, termios.TCIOFLUSH)
        _write_all(fd, wire)
        return _read_until_response(fd, expected_seq=frame.seq, timeout_s=timeout_s)
    finally:
        os.close(fd)


def ping(
    *,
    port: str = DEFAULT_PORT,
    baud: int = DEFAULT_BAUD,
    sequence: int = 1,
    timeout_s: float = 0.3,
) -> Frame:
    """Send a no-motion PING and require the formal empty ACK."""

    response = exchange(Frame(CMD_PING, sequence), port=port, baud=baud, timeout_s=timeout_s)
    if response.cmd != RSP_ACK or response.payload:
        raise ResponseError(
            f"PING sequence {sequence} returned cmd=0x{response.cmd:02X}, payload={response.payload.hex()}"
        )
    return response


def main() -> int:
    parser = argparse.ArgumentParser(
        description="F407 UART link test. Only --ping is exposed; no motion frame can be sent."
    )
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--ping", action="store_true", help="send one safe PING and require ACK")
    parser.add_argument("--port", default=DEFAULT_PORT, help="UART path, normally /dev/serial0")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, choices=(9600, 19200, 38400, 57600, 115200))
    parser.add_argument("--sequence", type=int, default=1, choices=range(0, 256), metavar="0..255")
    parser.add_argument("--timeout", type=float, default=0.3, help="response timeout in seconds")
    args = parser.parse_args()

    wire = encode_frame(CMD_PING, args.sequence)
    print(f"UART: {args.port}, {args.baud} 8N1, no flow control")
    print(f"TX PING HEX: {wire.hex(' ').upper()}")
    try:
        response = ping(port=args.port, baud=args.baud, sequence=args.sequence, timeout_s=args.timeout)
    except (OSError, ProtocolError, ResponseError, RuntimeError, TimeoutError) as exc:
        print(f"PING failed: {exc}", file=sys.stderr)
        return 1
    print(f"RX ACK: cmd=0x{response.cmd:02X} seq={response.seq} payload=<empty>")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
