"""Dependency-free validation shared by the ROS2 observation bridge and tests.

The contract rejects malformed messages before they reach the Transformer.  It
does not decide whether motion is allowed; that remains the independent safety
gate.  Keeping this module free of ``rclpy`` lets Windows and the Pi replay the
same edge cases without starting ROS2.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any

from .transformer_policy import OBJECT_CLASSES


SCHEMA_VERSION = 1
JOINT_COUNT = 6


def _finite_vector(value: object, size: int) -> bool:
    if not isinstance(value, (list, tuple)) or len(value) != size:
        return False
    try:
        return all(math.isfinite(float(item)) for item in value)
    except (TypeError, ValueError):
        return False


def validate_observation_contract(observation: object) -> tuple[bool, list[str]]:
    """Validate shape and scalar sanity; semantic gates still run afterwards."""
    if not isinstance(observation, dict):
        return False, ["observation_not_object"]
    failures: list[str] = []
    if observation.get("schema_version") != SCHEMA_VERSION:
        failures.append("schema_version")
    yolo = observation.get("yolo")
    target = observation.get("target")
    arm = observation.get("arm")
    phase = observation.get("phase")
    perception = observation.get("perception")
    safety = observation.get("safety")
    if not isinstance(yolo, dict) or not isinstance(yolo.get("class"), str):
        failures.append("yolo_class")
    else:
        try:
            confidence = float(yolo.get("confidence"))
            if not math.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
                failures.append("yolo_confidence")
        except (TypeError, ValueError):
            failures.append("yolo_confidence")
    if not isinstance(target, dict) or not isinstance(target.get("class"), str):
        failures.append("target_class")
    elif str(target.get("class")).lower() not in OBJECT_CLASSES:
        failures.append("target_class_unknown")
    if not isinstance(target, dict) or not _finite_vector(target.get("position_base_m"), 3):
        failures.append("target_position")
    if not isinstance(arm, dict) or not _finite_vector(arm.get("joint_position_rad"), JOINT_COUNT):
        failures.append("joint_position")
    if not isinstance(arm, dict) or not _finite_vector(arm.get("joint_velocity_rad_s"), JOINT_COUNT):
        failures.append("joint_velocity")
    online = arm.get("joint_online") if isinstance(arm, dict) else None
    if not isinstance(online, (list, tuple)) or len(online) != JOINT_COUNT or any(not isinstance(item, bool) for item in online):
        failures.append("joint_online")
    if not isinstance(phase, dict) or not isinstance(phase.get("name"), str):
        failures.append("phase")
    if not isinstance(perception, dict):
        failures.append("perception")
    else:
        for key in ("detection_fresh", "calibration_valid"):
            if not isinstance(perception.get(key), bool):
                failures.append(key)
    if not isinstance(safety, dict):
        failures.append("safety")
    else:
        for key in ("motion_enabled", "hardware_ready", "collision_free", "feedback_verified"):
            if not isinstance(safety.get(key), bool):
                failures.append(f"safety_{key}")
    return not failures, failures


@dataclass
class FreshnessBook:
    """Local-reception timestamps; untrusted remote message stamps are ignored."""

    received_at_ns: dict[str, int] = field(default_factory=dict)

    def update(self, channel: str, received_at_ns: int) -> None:
        if not channel:
            raise ValueError("channel must not be empty")
        if not isinstance(received_at_ns, int) or received_at_ns < 0:
            raise ValueError("received_at_ns must be a non-negative integer")
        previous = self.received_at_ns.get(channel)
        if previous is not None and received_at_ns < previous:
            raise ValueError("local reception time must not move backwards")
        self.received_at_ns[channel] = received_at_ns

    def age_s(self, channel: str, now_ns: int) -> float | None:
        received = self.received_at_ns.get(channel)
        if received is None or now_ns < received:
            return None
        return (now_ns - received) / 1_000_000_000.0

    def is_fresh(self, channel: str, now_ns: int, max_age_s: float) -> bool:
        if not math.isfinite(max_age_s) or max_age_s <= 0.0:
            raise ValueError("max_age_s must be finite and positive")
        age = self.age_s(channel, now_ns)
        return age is not None and age <= max_age_s
