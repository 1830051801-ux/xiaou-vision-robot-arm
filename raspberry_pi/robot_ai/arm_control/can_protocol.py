"""Reference implementation of the XiaoU six-axis CAN protocol.

The wire format is intentionally small and deterministic so the STM32 side can
implement it without depending on ROS2 or Python.  The active hardware driver
uses the same layout in C++.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import struct


CAN_BITRATE = 500_000
COMMAND_ID_BASE = 0x100
FEEDBACK_ID_BASE = 0x180
DIAGNOSTIC_ID_BASE = 0x1C0
COMMAND_OPCODE_POSITION = 0x01
FEEDBACK_OPCODE_STATUS = 0x81
DIAGNOSTIC_OPCODE = 0xE0

FLAG_ENABLE = 1 << 0
FLAG_CLEAR_FAULT = 1 << 1
FLAG_QUICK_STOP = 1 << 2

STATUS_ENABLED = 1 << 0
STATUS_FAULT = 1 << 1
STATUS_ESTOP = 1 << 2
STATUS_HOMED = 1 << 3
STATUS_HEARTBEAT = 1 << 4

_COMMAND = struct.Struct("<BBih")
_FEEDBACK = struct.Struct("<BBih")
_DIAGNOSTIC = struct.Struct("<BBH4x")


def _node_id(node_id: int) -> int:
    if isinstance(node_id, bool) or not isinstance(node_id, int) or not 1 <= node_id <= 0x3F:
        raise ValueError("node_id must be an integer in 1..63")
    return node_id


def command_can_id(node_id: int) -> int:
    return COMMAND_ID_BASE + _node_id(node_id)


def feedback_can_id(node_id: int) -> int:
    return FEEDBACK_ID_BASE + _node_id(node_id)


def diagnostic_can_id(node_id: int) -> int:
    return DIAGNOSTIC_ID_BASE + _node_id(node_id)


def _scaled_position(position_rad: float) -> int:
    if not math.isfinite(position_rad):
        raise ValueError("position_rad must be finite")
    value = int(round(position_rad * 1_000_000.0))
    if not -(2**31) <= value <= 2**31 - 1:
        raise ValueError("position_rad is outside the int32 micro-radian range")
    return value


def _scaled_velocity(velocity_rad_s: float) -> int:
    if not math.isfinite(velocity_rad_s):
        raise ValueError("velocity_rad_s must be finite")
    value = int(round(velocity_rad_s * 1_000.0))
    if not -(2**15) <= value <= 2**15 - 1:
        raise ValueError("velocity_rad_s is outside the int16 milli-radian/s range")
    return value


def encode_position_command(
    node_id: int,
    position_rad: float,
    velocity_rad_s: float,
    *,
    enable: bool = False,
    clear_fault: bool = False,
    quick_stop: bool = False,
    sequence: int = 0,
) -> tuple[int, bytes]:
    """Return ``(arbitration_id, 8-byte payload)`` for one joint command.

    The sequence is a four-bit rolling counter stored in the high nibble of
    the flags byte.  It helps the STM32 detect stale/replayed commands without
    adding another byte to the fixed eight-byte CAN payload.
    """

    _node_id(node_id)
    if isinstance(sequence, bool) or not isinstance(sequence, int) or not 0 <= sequence <= 0x0F:
        raise ValueError("sequence must be an integer in 0..15")
    flags = (sequence << 4)
    flags |= FLAG_ENABLE if enable else 0
    flags |= FLAG_CLEAR_FAULT if clear_fault else 0
    flags |= FLAG_QUICK_STOP if quick_stop else 0
    payload = _COMMAND.pack(
        COMMAND_OPCODE_POSITION,
        flags,
        _scaled_position(position_rad),
        _scaled_velocity(velocity_rad_s),
    )
    return command_can_id(node_id), payload


@dataclass(frozen=True)
class JointCommand:
    position_rad: float
    velocity_rad_s: float
    enable: bool
    clear_fault: bool
    quick_stop: bool
    sequence: int


def decode_position_command(node_id: int, can_id: int, payload: bytes) -> JointCommand:
    """Decode one XiaoU V1 command frame for firmware and loopback tests."""

    if can_id != command_can_id(node_id):
        raise ValueError("command CAN ID does not match node_id")
    if len(payload) != _COMMAND.size:
        raise ValueError("command payload must be exactly 8 bytes")
    opcode, flags, position_urad, velocity_mrad_s = _COMMAND.unpack(payload)
    if opcode != COMMAND_OPCODE_POSITION:
        raise ValueError(f"unexpected command opcode 0x{opcode:02X}")
    return JointCommand(
        position_rad=position_urad / 1_000_000.0,
        velocity_rad_s=velocity_mrad_s / 1_000.0,
        enable=bool(flags & FLAG_ENABLE),
        clear_fault=bool(flags & FLAG_CLEAR_FAULT),
        quick_stop=bool(flags & FLAG_QUICK_STOP),
        sequence=(flags >> 4) & 0x0F,
    )


@dataclass(frozen=True)
class JointFeedback:
    position_rad: float
    velocity_rad_s: float
    status: int

    @property
    def enabled(self) -> bool:
        return bool(self.status & STATUS_ENABLED)

    @property
    def fault(self) -> bool:
        return bool(self.status & STATUS_FAULT)

    @property
    def estop(self) -> bool:
        return bool(self.status & STATUS_ESTOP)


def decode_feedback(node_id: int, can_id: int, payload: bytes) -> JointFeedback:
    if can_id != feedback_can_id(node_id):
        raise ValueError("feedback CAN ID does not match node_id")
    if len(payload) != _FEEDBACK.size:
        raise ValueError("feedback payload must be exactly 8 bytes")
    opcode, status, position_urad, velocity_mrad_s = _FEEDBACK.unpack(payload)
    if opcode != FEEDBACK_OPCODE_STATUS:
        raise ValueError(f"unexpected feedback opcode 0x{opcode:02X}")
    return JointFeedback(position_urad / 1_000_000.0, velocity_mrad_s / 1_000.0, status)


def encode_diagnostic(node_id: int, error_code: int, detail: int = 0) -> tuple[int, bytes]:
    _node_id(node_id)
    if not 0 <= error_code <= 0xFF or not 0 <= detail <= 0xFFFF:
        raise ValueError("diagnostic fields are outside their unsigned ranges")
    return diagnostic_can_id(node_id), _DIAGNOSTIC.pack(DIAGNOSTIC_OPCODE, error_code, detail)
