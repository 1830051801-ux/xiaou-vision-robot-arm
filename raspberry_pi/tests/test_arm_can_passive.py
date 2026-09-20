from __future__ import annotations

import inspect
import struct
import sys
import unittest
from pathlib import Path


PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR / "robot_ai"))

from arm_control import can_passive


class PassiveCanTests(unittest.TestCase):
    def test_standard_frame_is_decoded_without_protocol_guessing(self) -> None:
        frame = struct.pack("=IB3x8s", 0x123, 3, bytes([0xAA, 0x55, 0x01, 0, 0, 0, 0, 0]))
        record = can_passive.parse_can_frame(frame, received_at="test")
        self.assertEqual(record["can_id"], 0x123)
        self.assertEqual(record["data_bytes"], [0xAA, 0x55, 0x01])
        self.assertFalse(record["extended"])
        self.assertFalse(record["error"])
        self.assertEqual(record["protocol_hint"]["node_id_candidate"], 9)
        self.assertEqual(record["protocol_hint"]["opcode_candidate"], 3)
        self.assertFalse(record["protocol_hint"]["confirmed"])

    def test_error_and_extended_flags_are_preserved(self) -> None:
        raw_id = can_passive.CAN_EFF_FLAG | can_passive.CAN_ERR_FLAG | 0x1ABCDE
        frame = struct.pack("=IB3x8s", raw_id, 0, bytes(8))
        record = can_passive.parse_can_frame(frame, received_at="test")
        self.assertTrue(record["extended"])
        self.assertTrue(record["error"])
        self.assertEqual(record["can_id"], 0x1ABCDE)

    def test_listener_source_contains_no_transmit_calls(self) -> None:
        source = inspect.getsource(can_passive)
        self.assertNotIn(".send(", source)
        self.assertNotIn(".sendto(", source)
        self.assertNotIn("cansend", source.lower())


if __name__ == "__main__":
    unittest.main()
