from __future__ import annotations

import math
import sys
from pathlib import Path
import unittest


PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR / "robot_ai"))

from arm_control.can_protocol import (  # noqa: E402
    FLAG_ENABLE,
    FLAG_QUICK_STOP,
    STATUS_ESTOP,
    STATUS_ENABLED,
    command_can_id,
    decode_feedback,
    encode_position_command,
    decode_position_command,
)
from arm_control.can_loopback import CanLoopbackBus  # noqa: E402


class CanProtocolTests(unittest.TestCase):
    def test_position_command_has_stable_id_and_units(self) -> None:
        can_id, payload = encode_position_command(
            3,
            math.pi / 2.0,
            -1.25,
            enable=True,
            sequence=9,
        )
        self.assertEqual(can_id, command_can_id(3))
        self.assertEqual(len(payload), 8)
        self.assertEqual(payload[0], 0x01)
        self.assertEqual(payload[1] & FLAG_ENABLE, FLAG_ENABLE)
        self.assertEqual(payload[1] >> 4, 9)
        self.assertAlmostEqual(int.from_bytes(payload[2:6], "little", signed=True) / 1e6, math.pi / 2.0, places=5)
        self.assertAlmostEqual(int.from_bytes(payload[6:8], "little", signed=True) / 1e3, -1.25, places=3)

    def test_feedback_decodes_position_velocity_and_status(self) -> None:
        payload = bytes([0x81, STATUS_ENABLED | STATUS_ESTOP])
        payload += int(250_000).to_bytes(4, "little", signed=True)
        payload += int(-500).to_bytes(2, "little", signed=True)
        feedback = decode_feedback(2, 0x182, payload)
        self.assertAlmostEqual(feedback.position_rad, 0.25)
        self.assertAlmostEqual(feedback.velocity_rad_s, -0.5)
        self.assertTrue(feedback.enabled)
        self.assertTrue(feedback.estop)

    def test_quick_stop_is_explicit(self) -> None:
        _, payload = encode_position_command(1, 0.0, 0.0, quick_stop=True)
        self.assertEqual(payload[1] & FLAG_QUICK_STOP, FLAG_QUICK_STOP)

    def test_invalid_node_id_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            command_can_id(0)
        with self.assertRaises(ValueError):
            command_can_id(64)

    def test_command_round_trip_decodes_flags_and_sequence(self) -> None:
        can_id, payload = encode_position_command(
            4, -0.25, 0.75, enable=True, clear_fault=True, sequence=13
        )
        command = decode_position_command(4, can_id, payload)
        self.assertAlmostEqual(command.position_rad, -0.25)
        self.assertAlmostEqual(command.velocity_rad_s, 0.75)
        self.assertTrue(command.enable)
        self.assertTrue(command.clear_fault)
        self.assertEqual(command.sequence, 13)

    def test_loopback_moves_and_quick_stops_without_socketcan(self) -> None:
        bus = CanLoopbackBus()
        can_id, payload = encode_position_command(1, 0.2, 0.5, enable=True)
        for _ in range(8):
            bus.send(can_id, payload)
            bus.step(0.05)
        self.assertAlmostEqual(bus.joints[1].position_rad, 0.2, places=6)
        _, stop_payload = encode_position_command(1, 0.2, 0.0, quick_stop=True)
        bus.send(can_id, stop_payload)
        self.assertFalse(bus.joints[1].enabled)
        self.assertEqual(bus.joints[1].velocity_rad_s, 0.0)

    def test_loopback_watchdog_faults_when_commands_stop(self) -> None:
        bus = CanLoopbackBus()
        can_id, payload = encode_position_command(2, 0.1, 0.2, enable=True)
        bus.send(can_id, payload)
        bus.step(0.201)
        self.assertTrue(bus.joints[2].fault)
        self.assertFalse(bus.joints[2].enabled)
