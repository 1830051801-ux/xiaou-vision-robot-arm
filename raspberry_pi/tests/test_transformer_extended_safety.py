from __future__ import annotations

import unittest

from robot_ai.decision.transformer_policy import SafetyGate


def complete_context() -> dict:
    return {
        "yolo": {"confidence": 0.9},
        "perception": {"detection_fresh": True, "calibration_valid": True},
        "arm": {"joint_online": [True] * 6},
        "safety": {
            "motion_enabled": True,
            "hardware_ready": True,
            "collision_free": True,
            "feedback_verified": True,
        },
    }


class TransformerExtendedSafetyTests(unittest.TestCase):
    def setUp(self) -> None:
        self.gate = SafetyGate(
            require_motion_enabled=True,
            require_hardware_ready=True,
            require_collision_free=True,
            require_feedback_verified=True,
            require_detection_fresh=True,
            require_calibration_valid=True,
            require_all_joints_online=True,
            min_detection_confidence=0.2,
        )

    def test_complete_context_passes(self) -> None:
        self.assertEqual(self.gate.evaluate(complete_context()), (True, []))

    def test_stale_low_confidence_uncalibrated_and_offline_joint_are_blocked(self) -> None:
        context = complete_context()
        context["yolo"]["confidence"] = 0.1
        context["perception"] = {"detection_fresh": False, "calibration_valid": False}
        context["arm"]["joint_online"][4] = False
        allowed, failures = self.gate.evaluate(context)
        self.assertFalse(allowed)
        self.assertEqual(
            failures,
            ["detection_fresh", "calibration_valid", "joint_online", "detection_confidence"],
        )


if __name__ == "__main__":
    unittest.main()
