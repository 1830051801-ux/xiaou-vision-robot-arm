from __future__ import annotations

import struct
import sys
import unittest
from pathlib import Path


PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR / "robot_ai"))

from arm_control.joint_uart_staged_test import build_joint_trajectory_frame
from arm_control.uart_protocol import CMD_TRAJ_POINT, ProtocolError


class JointUartStagedTestTests(unittest.TestCase):
    def test_each_joint_uses_the_same_six_axis_frame_shape(self) -> None:
        for joint_id in range(1, 7):
            frame = build_joint_trajectory_frame(joint_id, 2.5, 1500, joint_id)
            self.assertEqual(frame.cmd, CMD_TRAJ_POINT)
            angles = struct.unpack("<6fH", frame.payload)
            self.assertEqual(angles[-1], 1500)
            self.assertEqual(angles[joint_id - 1], 2.5)
            self.assertEqual(sum(value != 0.0 for value in angles[:6]), 1)

    def test_invalid_joint_is_rejected_without_opening_a_uart(self) -> None:
        with self.assertRaises(ProtocolError):
            build_joint_trajectory_frame(7, 0.0, 1500, 1)


if __name__ == "__main__":
    unittest.main()
