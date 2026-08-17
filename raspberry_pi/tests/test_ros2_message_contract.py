from __future__ import annotations

import unittest

from robot_ai.decision.ros2_message_contract import FreshnessBook, validate_observation_contract
from simulation.desktop_scene import DesktopScene


class Ros2MessageContractTests(unittest.TestCase):
    def test_simulation_observation_matches_the_cross_process_contract(self) -> None:
        observation = DesktopScene(scenario="bottle", seed=20260809).observation()
        self.assertEqual(validate_observation_contract(observation), (True, []))

    def test_unknown_class_and_non_finite_joint_are_rejected_before_inference(self) -> None:
        observation = DesktopScene(scenario="pen", seed=1).observation()
        observation["target"]["class"] = "unknown"
        observation["arm"]["joint_position_rad"][2] = float("nan")
        valid, failures = validate_observation_contract(observation)
        self.assertFalse(valid)
        self.assertIn("target_class_unknown", failures)
        self.assertIn("joint_position", failures)

    def test_freshness_uses_local_monotonic_reception_time(self) -> None:
        book = FreshnessBook()
        book.update("target_pose", 1_000_000_000)
        self.assertTrue(book.is_fresh("target_pose", 1_400_000_000, 0.5))
        self.assertFalse(book.is_fresh("target_pose", 1_600_000_000, 0.5))
        self.assertFalse(book.is_fresh("missing", 1_400_000_000, 0.5))
        with self.assertRaises(ValueError):
            book.update("target_pose", 999_999_999)


if __name__ == "__main__":
    unittest.main()
