import json
import math
from pathlib import Path
import tempfile
import unittest

from robot_ai.arm_control.capture_pose import _write_pose
from robot_ai.arm_control.ready_pose import build_ready_frame
from robot_ai.arm_control.uart_protocol import CMD_TRAJ_POINT


class CapturePoseTests(unittest.TestCase):
    def test_zero_capture_writes_pi_reference_snapshot_in_radians(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "hardware.json"
            path.write_text("{}\n", encoding="utf-8")
            _write_pose(path, "zero_pose_reference_rad", [0.0, 90.0, -90.0, 180.0, 45.0, -45.0])
            data = json.loads(path.read_text(encoding="utf-8"))
        self.assertEqual(len(data["zero_pose_reference_rad"]), 6)
        self.assertAlmostEqual(data["zero_pose_reference_rad"][1], math.pi / 2.0)
        self.assertTrue(data["zero_pose_reference_rad_captured_from_telemetry"])

    def test_ready_capture_writes_ready_pose_in_radians(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "hardware.json"
            path.write_text("{}\n", encoding="utf-8")
            _write_pose(path, "ready_pose_rad", [1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
            data = json.loads(path.read_text(encoding="utf-8"))
        self.assertAlmostEqual(data["ready_pose_rad"][5], math.radians(6.0))
        self.assertTrue(data["ready_pose_rad_captured_from_telemetry"])

    def test_ready_frame_requires_a_captured_six_axis_pose(self):
        with self.assertRaises(ValueError):
            build_ready_frame({"ready_pose_rad": [None] * 6}, 1, 1000)
        frame = build_ready_frame({"ready_pose_rad": [0.0] * 6}, 1, 1000)
        self.assertEqual(frame.cmd, CMD_TRAJ_POINT)
        self.assertEqual(len(frame.payload), 26)


if __name__ == "__main__":
    unittest.main()
