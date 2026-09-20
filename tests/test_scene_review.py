from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np


PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR / "robot_ai"))

from arm_control.scene_review import AxisAlignedBox, DiagnosticScene, review_tcp_positions


class SceneReviewTests(unittest.TestCase):
    def setUp(self) -> None:
        self.scene = DiagnosticScene(
            frame="base_link",
            table_z_m=0.0,
            tcp_safety_radius_m=0.02,
            minimum_table_clearance_m=0.01,
            minimum_obstacle_clearance_m=0.015,
            keep_out_boxes=(
                AxisAlignedBox(
                    "fixture",
                    np.array([0.10, -0.05, 0.00]),
                    np.array([0.16, 0.05, 0.12]),
                ),
            ),
        )

    def test_clear_path_is_safe(self) -> None:
        review = review_tcp_positions(
            [np.array([0.25, 0.10, 0.20]), np.array([0.28, 0.10, 0.20])],
            self.scene,
        )
        self.assertTrue(review.safe, review)
        self.assertGreater(review.minimum_table_clearance_m, 0.01)
        self.assertGreater(review.minimum_obstacle_clearance_m, 0.015)

    def test_keep_out_and_table_clearance_are_reported(self) -> None:
        review = review_tcp_positions(
            [np.array([0.12, 0.00, 0.02]), np.array([0.13, 0.00, 0.02])],
            self.scene,
        )
        self.assertFalse(review.safe)
        self.assertEqual(review.sample_count, 2)
        self.assertLess(review.minimum_table_clearance_m, 0.01)
        self.assertTrue(any("fixture" in item for item in review.violations))


if __name__ == "__main__":
    unittest.main()
