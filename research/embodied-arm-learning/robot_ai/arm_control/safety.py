from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any
import math


DEFAULT_HARDWARE_CONFIG = Path(__file__).resolve().parent / "config" / "hardware_calibration.json"


class MotionLockedError(RuntimeError):
    pass


@dataclass(frozen=True)
class MotionReadiness:
    ready: bool
    missing_or_invalid: tuple[str, ...]


_SIX_VALUE_FIELDS = (
    "joint_node_ids",
    "encoder_zero_offset_rad",
    "encoder_direction",
    "position_min_rad",
    "position_max_rad",
    "velocity_max_rad_s",
    "acceleration_max_rad_s2",
)

_REQUIRED_TRUE_FIELDS = (
    "protocol_confirmed",
    "uart_link_verified",
    "f407_firmware_verified",
    "estop_verified",
    "feedback_verified",
)


def load_hardware_config(path: str | Path = DEFAULT_HARDWARE_CONFIG) -> dict[str, Any]:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def validate_motion_readiness(config: dict[str, Any]) -> MotionReadiness:
    invalid: list[str] = []
    if config.get("motion_enabled") is not True:
        invalid.append("motion_enabled")
    for field in _REQUIRED_TRUE_FIELDS:
        if config.get(field) is not True:
            invalid.append(field)
    transport = config.get("transport")
    if not isinstance(transport, dict):
        invalid.append("transport")
    else:
        if transport.get("kind") != "uart":
            invalid.append("transport.kind")
        port = transport.get("port")
        if not isinstance(port, str) or not port.strip():
            invalid.append("transport.port")
        baud = transport.get("baud")
        if isinstance(baud, bool) or not isinstance(baud, int) or baud <= 0:
            invalid.append("transport.baud")
        if transport.get("data_bits") != 8:
            invalid.append("transport.data_bits")
        if transport.get("stop_bits") != 1:
            invalid.append("transport.stop_bits")
        if transport.get("parity") != "none":
            invalid.append("transport.parity")
        if transport.get("flow_control") is not False:
            invalid.append("transport.flow_control")
        if transport.get("direct_passthrough") is not True:
            invalid.append("transport.direct_passthrough")
        protocol_version = transport.get("protocol_version")
        if not isinstance(protocol_version, str) or not protocol_version.strip():
            invalid.append("transport.protocol_version")

    for field in _SIX_VALUE_FIELDS:
        values = config.get(field)
        if not isinstance(values, list) or len(values) != 6 or any(value is None for value in values):
            invalid.append(field)

    numeric_fields = (
        "encoder_zero_offset_rad",
        "position_min_rad",
        "position_max_rad",
        "velocity_max_rad_s",
        "acceleration_max_rad_s2",
    )
    for field in numeric_fields:
        values = config.get(field)
        if isinstance(values, list) and len(values) == 6 and all(value is not None for value in values):
            if any(isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) for value in values):
                invalid.append(f"{field}_values")

    directions = config.get("encoder_direction")
    if (
        isinstance(directions, list)
        and len(directions) == 6
        and all(value is not None for value in directions)
        and any(
            isinstance(value, bool) or not isinstance(value, int) or value not in (-1, 1)
            for value in directions
        )
    ):
        invalid.append("encoder_direction_values")

    node_ids = config.get("joint_node_ids")
    if isinstance(node_ids, list) and len(node_ids) == 6 and all(value is not None for value in node_ids):
        if any(isinstance(value, bool) or not isinstance(value, int) or value < 1 or value > 0x3F for value in node_ids):
            invalid.append("joint_node_ids_values")
        if len(set(node_ids)) != 6:
            invalid.append("joint_node_ids_unique")

    mins = config.get("position_min_rad")
    maxs = config.get("position_max_rad")
    if isinstance(mins, list) and isinstance(maxs, list) and len(mins) == len(maxs) == 6:
        if all(value is not None for value in mins + maxs) and any(lo >= hi for lo, hi in zip(mins, maxs)):
            invalid.append("position_limit_order")

    for field in ("velocity_max_rad_s", "acceleration_max_rad_s2"):
        values = config.get(field)
        if isinstance(values, list) and len(values) == 6 and all(value is not None for value in values):
            if any(isinstance(value, bool) or not isinstance(value, (int, float)) or value <= 0 for value in values):
                invalid.append(f"{field}_positive")

    invalid = list(dict.fromkeys(invalid))
    return MotionReadiness(not invalid, tuple(invalid))


def require_motion_ready(config: dict[str, Any]) -> None:
    readiness = validate_motion_readiness(config)
    if not readiness.ready:
        fields = ", ".join(readiness.missing_or_invalid)
        raise MotionLockedError(f"six-axis motion is locked; unresolved fields: {fields}")
