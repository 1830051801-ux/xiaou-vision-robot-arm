from __future__ import annotations

import sys
import unittest
from pathlib import Path


PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR / "robot_ai"))
sys.path.insert(0, str(PROJECT_DIR / "tools"))

from verify_six_axis_stack import validate_grasp_profiles


class StackVerificationTests(unittest.TestCase):
    def test_v2_unmeasured_profiles_are_valid_and_keep_execution_locked(self) -> None:
        report = validate_grasp_profiles()
        self.assertEqual(report["schema_version"], 2)
        self.assertEqual(report["calibrated"], [])
        self.assertEqual(report["unmeasured"], ["bottle", "cola", "cup", "earphone", "pen"])


if __name__ == "__main__":
    unittest.main()
