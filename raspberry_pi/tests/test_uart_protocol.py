from __future__ import annotations

import struct
import sys
import unittest
from pathlib import Path


PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR / "robot_ai"))

from arm_control.uart_protocol import (
    CMD_GRIPPER,
    CMD_GET_CALIB_STATUS,
    CMD_PING,
    CMD_SET_ZERO,
    CMD_TRAJ_BATCH,
    CMD_TRAJ_POINT,
    FRAME_END,
    FRAME_START,
    RSP_ACK,
    RSP_MOTION_LOCKED,
    Frame,
    FrameParser,
    ProtocolError,
    STATE_PAYLOAD_LEN,
    CALIB_STATUS_PAYLOAD_LEN,
    TELEMETRY_PAYLOAD_LEN,
    decode_state_payload,
    decode_calib_status_payload,
    encode_frame,
    is_motion_command,
    pack_trajectory_payload,
    pack_trajectory_batch_payload,
    pack_gripper_payload,
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

    def test_payload_end_marker_is_stuffed_before_uart_transmit(self) -> None:
        payload = bytes((0x10, FRAME_END, FRAME_START, 0xBB))
        wire = encode_frame(CMD_TRAJ_POINT, 9, payload)
        self.assertIn(bytes((0xBB, 0x05, 0xBB, 0x0A, 0xBB, 0x0B)), wire)
        self.assertEqual(FrameParser().feed(wire), [Frame(CMD_TRAJ_POINT, 9, payload)])

    def test_parser_rejects_crc_bypass_and_bad_escape(self) -> None:
        parser = FrameParser()
        self.assertEqual(parser.feed(bytes.fromhex("AA010001FFFF55")), [])
        self.assertEqual(parser.feed(bytes.fromhex("AA500109BBFF000055")), [])

    def test_trajectory_payload_is_fixed_six_floats_and_little_endian_duration(self) -> None:
        payload = pack_trajectory_payload([0.0, 1.0, 2.0, 3.0, 4.0, 5.0], 500)
        self.assertEqual(len(payload), 26)
        self.assertEqual(struct.unpack("<6fH", payload), (0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 500))
        self.assertTrue(is_motion_command(CMD_TRAJ_POINT))

    def test_trajectory_batch_contains_one_to_nine_complete_points(self) -> None:
        payload = pack_trajectory_batch_payload(
            [([0.0] * 6, 500), ([1.0] * 6, 1000)]
        )
        self.assertEqual(len(payload), 52)
        self.assertEqual(struct.unpack_from("<6fH", payload, 26)[-1], 1000)
        self.assertTrue(is_motion_command(CMD_TRAJ_BATCH))
        with self.assertRaises(ProtocolError):
            pack_trajectory_batch_payload([])
        with self.assertRaises(ProtocolError):
            pack_trajectory_batch_payload([([0.0] * 6, 100)] * 10)

    def test_gripper_payload_has_explicit_id_and_float32_angle(self) -> None:
        payload = pack_gripper_payload(1, 42.5)
        self.assertEqual(len(payload), 5)
        self.assertEqual(struct.unpack("<Bf", payload), (1, 42.5))
        self.assertTrue(is_motion_command(CMD_GRIPPER))
        self.assertTrue(is_motion_command(CMD_SET_ZERO))
        self.assertFalse(is_motion_command(CMD_GET_CALIB_STATUS))
        with self.assertRaises(ProtocolError):
            pack_gripper_payload(0, 42.5)

    def test_motion_locked_response_round_trips_with_original_command_payload(self) -> None:
        frame = Frame(RSP_MOTION_LOCKED, 17, bytes((CMD_TRAJ_POINT,)))
        parser = FrameParser()
        self.assertEqual(parser.feed(encode_frame(frame.cmd, frame.seq, frame.payload)), [frame])

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

    def test_telemetry_uses_the_same_fixed_state_payload_length(self) -> None:
        self.assertEqual(TELEMETRY_PAYLOAD_LEN, STATE_PAYLOAD_LEN)

    def test_calibration_status_is_read_only_and_decodes_six_zero_flags(self) -> None:
        payload = bytes((2, 1, 1, 1, 1, 1, 1))
        payload += struct.pack("<6f", 0.0, -1.0, -2.0, -3.0, 4.0, 5.0)
        payload += struct.pack("<6f", 0.1, 0.2, 0.3, 0.4, 0.5, 0.6)
        self.assertEqual(len(payload), CALIB_STATUS_PAYLOAD_LEN)
        decoded = decode_calib_status_payload(payload)
        self.assertEqual(decoded["state"], 2)
        self.assertEqual(decoded["zero_valid"], [True] * 6)
        self.assertAlmostEqual(decoded["angles_deg"][4], 4.0)
        with self.assertRaises(ProtocolError):
            decode_calib_status_payload(payload[:-1])


if __name__ == "__main__":
    unittest.main()
