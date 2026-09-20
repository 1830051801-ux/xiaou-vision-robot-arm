"""Deterministic, transmit-free CAN V1 loopback model for offline testing.

This module never opens SocketCAN. It models only the wire contract and the
minimum joint state needed to exercise watchdog, fault, estop and quick-stop
handling before STM32 firmware exists.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

from .can_protocol import (
    FEEDBACK_OPCODE_STATUS,
    FLAG_QUICK_STOP,
    STATUS_ENABLED,
    STATUS_ESTOP,
    STATUS_FAULT,
    STATUS_HEARTBEAT,
    STATUS_HOMED,
    command_can_id,
    decode_position_command,
    encode_position_command,
    feedback_can_id,
)


@dataclass
class SimulatedJoint:
    node_id: int
    position_rad: float = 0.0
    velocity_rad_s: float = 0.0
    target_position_rad: float = 0.0
    target_velocity_rad_s: float = 0.0
    enabled: bool = False
    fault: bool = False
    estop: bool = False
    homed: bool = True
    last_command_s: float = 0.0
    sequence: int = 0

    def apply(self, payload: bytes, now_s: float) -> None:
        command = decode_position_command(self.node_id, command_can_id(self.node_id), payload)
        if command.clear_fault:
            self.fault = False
        self.sequence = command.sequence
        self.last_command_s = now_s
        if command.quick_stop:
            self.target_position_rad = self.position_rad
            self.target_velocity_rad_s = 0.0
            self.velocity_rad_s = 0.0
            self.enabled = False
            return
        if self.estop or self.fault:
            self.enabled = False
            return
        self.enabled = command.enable
        self.target_position_rad = command.position_rad
        self.target_velocity_rad_s = abs(command.velocity_rad_s)

    def step(self, dt_s: float, now_s: float, watchdog_s: float = 0.2) -> None:
        if not math.isfinite(dt_s) or dt_s <= 0.0:
            raise ValueError("dt_s must be finite and positive")
        if now_s - self.last_command_s > watchdog_s:
            self.fault = True
            self.enabled = False
            self.velocity_rad_s = 0.0
            return
        if not self.enabled or self.fault or self.estop:
            self.velocity_rad_s = 0.0
            return
        delta = self.target_position_rad - self.position_rad
        speed = max(0.0, self.target_velocity_rad_s)
        step = min(abs(delta), speed * dt_s)
        self.velocity_rad_s = math.copysign(step / dt_s, delta) if step else 0.0
        self.position_rad += math.copysign(step, delta) if step else 0.0

    def feedback(self) -> bytes:
        status = STATUS_HEARTBEAT
        status |= STATUS_ENABLED if self.enabled else 0
        status |= STATUS_FAULT if self.fault else 0
        status |= STATUS_ESTOP if self.estop else 0
        status |= STATUS_HOMED if self.homed else 0
        _, payload = encode_position_command(
            self.node_id, self.position_rad, self.velocity_rad_s, sequence=self.sequence
        )
        return bytes(
            [FEEDBACK_OPCODE_STATUS, status]
        ) + payload[2:6] + payload[6:8]


class CanLoopbackBus:
    """Six-joint CAN V1 model with no operating-system CAN access."""

    def __init__(self, node_ids: tuple[int, ...] = (1, 2, 3, 4, 5, 6)) -> None:
        if len(node_ids) != 6 or len(set(node_ids)) != 6:
            raise ValueError("loopback requires six unique node IDs")
        self.joints = {node_id: SimulatedJoint(node_id) for node_id in node_ids}
        self.now_s = 0.0

    def send(self, can_id: int, payload: bytes) -> None:
        node_id = can_id - 0x100
        if node_id not in self.joints:
            raise ValueError(f"unknown command CAN ID 0x{can_id:03X}")
        self.joints[node_id].apply(payload, self.now_s)

    def step(self, dt_s: float) -> None:
        self.now_s += dt_s
        for joint in self.joints.values():
            joint.step(dt_s, self.now_s)

    def feedback_frames(self) -> list[tuple[int, bytes]]:
        return [(feedback_can_id(node_id), joint.feedback()) for node_id, joint in self.joints.items()]

    def inject_estop(self, node_id: int, active: bool = True) -> None:
        self.joints[node_id].estop = active
        if active:
            self.joints[node_id].enabled = False
            self.joints[node_id].velocity_rad_s = 0.0
