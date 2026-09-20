from __future__ import annotations

import struct
import sys
import unittest
from pathlib import Path


PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR / "robot_ai"))

from arm_control.uart_protocol import (
    CMD_PING,
    CMD_TRAJ_POINT,
    FRAME_END,
    FRAME_START,
    RSP_ACK,
    Frame,
    FrameParser,
    ProtocolError,
    STATE_PAYLOAD_LEN,
    decode_state_payload,
    encode_frame,
    is_motion_command,
    pack_trajectory_payload,
)


class UartProtocolTests(unittest.TestCase):
    def test_ping_and_ack_match_the_f407_wire_examples(self) -> None:
        self.assertEqual(encode_frame(CMD_PING, 1), bytes.fromhex("AA010001E1C055"))
        self.assertEqual(encode_frame(RSP_ACK, 1), bytes.fromhex("AA000001B00055"))

    def test_parser_preserves_a_frame_split_at_every_possible_boundary(self) -> None:
        wire = encode_frame(0x50, 9, bytes((FRAME_START, FRAME_END, 0xBB, 0x00)))
        expected = [Frame(0x50, 9, bytes((FRAME_START, FRAME_END, 0xBB, 0x00)))]
        for split in range(1, len(wire)):
            parser = FrameParser()
            self.assertEqual(parser.feed(wire[:split]), [])
            self.assertEqual(parser.feed(wire[split:]), expected)

    def test_parser_rejects_crc_bypass_and_bad_escape(self) -> None:
        parser = FrameParser()
        self.assertEqual(parser.feed(bytes.fromhex("AA010001FFFF55")), [])
        self.assertEqual(parser.feed(bytes.fromhex("AA500109BBFF000055")), [])

    def test_trajectory_payload_is_fixed_six_floats_and_little_endian_duration(self) -> None:
        payload = pack_trajectory_payload([0.0, 1.0, 2.0, 3.0, 4.0, 5.0], 500)
        self.assertEqual(len(payload), 26)
        self.assertEqual(struct.unpack("<6fH", payload), (0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 500))
        self.assertTrue(is_motion_command(CMD_TRAJ_POINT))

    def test_state_payload_has_six_fixed_joint_records(self) -> None:
        payload = bytearray((1, 0, 0, 0))
        for index in range(6):
            payload.extend(struct.pack("<fffB", float(index), 10.0 + index, 0.5 + index, index % 2))
        self.assertEqual(len(payload), STATE_PAYLOAD_LEN)
        decoded = decode_state_payload(bytes(payload))
        self.assertEqual(decoded["state"], 1)
        self.assertEqual(len(decoded["joints"]), 6)
        self.assertEqual(decoded["joints"][4]["angle_deg"], 4.0)
        with self.assertRaises(ProtocolError):
            decode_state_payload(bytes(payload[:-1]))


if __name__ == "__main__":
    unittest.main()
