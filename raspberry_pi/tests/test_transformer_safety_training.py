from __future__ import annotations

import unittest

import numpy as np

from robot_ai.decision.train_transformer_policy import _build_dataset
from simulation.desktop_scene import OBSERVATION_DIM, STRATEGIES


class TransformerSafetyTrainingTests(unittest.TestCase):
    def test_all_negative_dataset_targets_reject_and_abort(self) -> None:
        features, _, strategy, phase, actions, risk = _build_dataset(
            60,
            8,
            20260831,
            safety_negative_ratio=1.0,
        )
        self.assertEqual(features.shape, (60, 8, OBSERVATION_DIM))
        self.assertTrue(np.all(strategy == STRATEGIES.index("reject")))
        self.assertTrue(np.all(phase == 5))
        self.assertTrue(np.all(actions == 0.0))
        self.assertTrue(np.allclose(risk, 0.95))

    def test_zero_negative_ratio_preserves_positive_demonstrations(self) -> None:
        _, _, strategy, _, _, risk = _build_dataset(60, 8, 20260831, safety_negative_ratio=0.0)
        self.assertFalse(np.any(strategy == STRATEGIES.index("reject")))
        self.assertTrue(np.all(risk < 0.5))


if __name__ == "__main__":
    unittest.main()
