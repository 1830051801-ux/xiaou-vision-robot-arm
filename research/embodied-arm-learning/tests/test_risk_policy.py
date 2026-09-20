from __future__ import annotations

import sys
import unittest
from pathlib import Path


PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR / "robot_ai"))

from arm_control.risk_policy import CandidateOutcome, RiskThresholds, choose_candidate, summarize_candidate


class RiskPolicyTests(unittest.TestCase):
    def setUp(self) -> None:
        self.thresholds = RiskThresholds(
            minimum_success_rate=0.8,
            minimum_perception_coverage=0.8,
            minimum_clearance_m=0.015,
        )

    def test_lowest_risk_eligible_candidate_is_selected(self) -> None:
        safe = summarize_candidate(
            "center_bin",
            [CandidateOutcome(True, True, 0.035, 4.0) for _ in range(10)],
            self.thresholds,
        )
        marginal = summarize_candidate(
            "rear_bin",
            [CandidateOutcome(True, True, 0.017, 3.0) for _ in range(10)],
            self.thresholds,
        )
        decision = choose_candidate([marginal, safe], self.thresholds)
        self.assertEqual(decision.action, "offline_plan_preview_ready")
        self.assertEqual(decision.selected_candidate, "center_bin")

    def test_low_perception_coverage_requires_recheck(self) -> None:
        summary = summarize_candidate(
            "center_bin",
            [CandidateOutcome(False, False, None, None, "perception_recheck") for _ in range(10)],
            self.thresholds,
        )
        decision = choose_candidate([summary], self.thresholds)
        self.assertEqual(decision.action, "vision_recheck_required")
        self.assertIsNone(decision.selected_candidate)

    def test_scene_or_trajectory_failure_blocks_candidate(self) -> None:
        summary = summarize_candidate(
            "rear_bin",
            [CandidateOutcome(True, False, 0.005, 3.0, "scene_clearance") for _ in range(10)],
            self.thresholds,
        )
        self.assertFalse(summary.eligible)
        decision = choose_candidate([summary], self.thresholds)
        self.assertEqual(decision.action, "blocked_by_risk_gate")

    def test_rejected_scene_rollouts_reduce_robust_clearance(self) -> None:
        outcomes = [CandidateOutcome(True, True, 0.040, 4.0) for _ in range(8)]
        outcomes.extend(
            [
                CandidateOutcome(True, False, 0.005, 4.0, "scene_clearance"),
                CandidateOutcome(True, False, 0.005, 4.0, "scene_clearance"),
            ]
        )
        summary = summarize_candidate("mixed_scene", outcomes, self.thresholds)
        self.assertLess(summary.clearance_p05_m or 1.0, self.thresholds.minimum_clearance_m)
        self.assertFalse(summary.eligible)


if __name__ == "__main__":
    unittest.main()
