from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any
import math
import time


DEFAULT_HARDWARE_CONFIG = Path(__file__).resolve().parent / "config" / "hardware_calibration.json"
F407_UART_PROTOCOL_FAMILY = "DaRanRobot_F407_UART_V1"


class MotionLockedError(RuntimeError):
    pass


@dataclass(frozen=True)
class MotionReadiness:
    ready: bool
    missing_or_invalid: tuple[str, ...]


_SIX_VALUE_FIELDS = (
    "joint_node_ids",
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
    "mcu_calibration_verified",
)


def load_hardware_config(path: str | Path = DEFAULT_HARDWARE_CONFIG) -> dict[str, Any]:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def validate_motion_readiness(config: dict[str, Any]) -> MotionReadiness:
    invalid: list[str] = []
    if config.get("motion_enabled") is not True:
        invalid.append("motion_enabled")
    if config.get("protocol_family") != F407_UART_PROTOCOL_FAMILY:
        invalid.append("protocol_family")
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
        if protocol_version != F407_UART_PROTOCOL_FAMILY:
            invalid.append("transport.protocol_version")

    for field in _SIX_VALUE_FIELDS:
        values = config.get(field)
        if not isinstance(values, list) or len(values) != 6 or any(value is None for value in values):
            invalid.append(field)

    numeric_fields = (
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

    mcu_calibration = config.get("mcu_calibration")
    if not isinstance(mcu_calibration, dict) or mcu_calibration.get("authority") != "stm32_f407":
        invalid.append("mcu_calibration.authority")
    elif config.get("mcu_calibration_verified") is True:
        verification_method = mcu_calibration.get("verification_method")
        if verification_method in {"live_get_calib_status", "operator_confirmed_f407_zeros"}:
            verified_unix_s = mcu_calibration.get("verified_unix_s")
            zero_valid = mcu_calibration.get("zero_valid")
            if (
                isinstance(verified_unix_s, bool)
                or not isinstance(verified_unix_s, (int, float))
                or not math.isfinite(verified_unix_s)
                or verified_unix_s > time.time() + 5.0
                or time.time() - verified_unix_s > 30.0
            ):
                invalid.append("mcu_calibration.live_timestamp")
            if not isinstance(zero_valid, list) or len(zero_valid) != 6 or not all(
                value is True for value in zero_valid
            ):
                invalid.append("mcu_calibration.live_zero_valid")
        else:
            version = mcu_calibration.get("record_version")
            crc32 = mcu_calibration.get("record_crc32")
            if isinstance(version, bool) or not isinstance(version, int) or version < 1:
                invalid.append("mcu_calibration.record_version")
            if not isinstance(crc32, str) or len(crc32) != 8 or any(char not in "0123456789abcdefABCDEF" for char in crc32):
                invalid.append("mcu_calibration.record_crc32")

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
