from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np


PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR / "robot_ai"))
sys.path.insert(0, str(PROJECT_DIR / "tools"))

from constraint_diffusion_twin import (
    ARCHITECTURE_NAME,
    CONTEXT_DIM,
    CONTEXT_TOKEN_NAMES,
    GRASP_INDEX,
    PHASE_NAMES,
    PLACE_INDEX,
    TASK_COUNT,
    TASK_NAMES,
    TASK_PHASE_IDS,
    TRAJECTORY_STEPS,
    balanced_episode_subset,
    build_expert_trajectory,
    build_task_trajectory,
    counterfactual_sensor_scales,
    fuse_pose_belief,
    pose_to_context,
    rigidify_pose,
    rotation_z,
    synthesize_multiview_observations,
)


class ConstraintDiffusionTwinTests(unittest.TestCase):
    def test_action_chunk_transformer_contract_is_explicit(self) -> None:
        self.assertEqual(ARCHITECTURE_NAME, "process_graph_multitask_action_chunk_transformer_diffusion")
        self.assertEqual(
            CONTEXT_TOKEN_NAMES,
            ("visual_grasp_pose", "visual_place_goal", "observation_uncertainty", "task_profile", "learned_task_token"),
        )
        self.assertEqual(TASK_COUNT, 5)
        self.assertEqual(CONTEXT_DIM, 21 + TASK_COUNT)
        self.assertEqual(PHASE_NAMES, ("home", "approach", "contact", "transfer", "retreat", "dwell"))
        self.assertEqual(TASK_PHASE_IDS.shape, (TASK_COUNT, TRAJECTORY_STEPS))
        precision_phases = TASK_PHASE_IDS[TASK_NAMES.index("precision_insert")]
        self.assertEqual(int(precision_phases[24]), PHASE_NAMES.index("dwell"))

    def test_quintic_trajectory_preserves_keyframe_anchors(self) -> None:
        home = np.zeros(6)
        grasp = np.array([0.20, -0.10, 0.15, -0.05, 0.08, -0.03])
        place = np.array([-0.12, 0.11, -0.14, 0.06, -0.07, 0.04])
        trajectory = build_expert_trajectory(home, grasp, place)
        self.assertEqual(trajectory.shape, (TRAJECTORY_STEPS, 6))
        np.testing.assert_allclose(trajectory[0], home)
        np.testing.assert_allclose(trajectory[GRASP_INDEX], grasp)
        np.testing.assert_allclose(trajectory[PLACE_INDEX], place)
        np.testing.assert_allclose(trajectory[-1], home)

    def test_rigidify_pose_recovers_rotation_after_float32_serialization(self) -> None:
        pose = np.eye(4, dtype=np.float64)
        pose[:3, :3] = np.array(
            [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64
        )
        pose[:3, 3] = [0.25, -0.12, 0.30]
        restored = rigidify_pose(pose.astype(np.float32))
        np.testing.assert_allclose(restored[:3, :3].T @ restored[:3, :3], np.eye(3), atol=1e-10)
        self.assertAlmostEqual(float(np.linalg.det(restored[:3, :3])), 1.0, places=10)
        np.testing.assert_allclose(restored[:3, 3], pose[:3, 3])

    def test_visual_context_has_stable_fixed_dimension(self) -> None:
        grasp = np.eye(4)
        place = np.eye(4)
        grasp[:3, 3] = [0.25, -0.12, 0.30]
        place[:3, 3] = [0.28, -0.08, 0.30]
        context = pose_to_context(grasp, place, 0.92, 0.003, np.deg2rad(2.0), "precision_insert")
        self.assertEqual(context.shape, (26,))
        self.assertEqual(int(np.argmax(context[21:])), TASK_NAMES.index("precision_insert"))
        self.assertAlmostEqual(float(context[21:].sum()), 1.0)
        self.assertTrue(np.isfinite(context).all())

    def test_multitask_trajectory_preserves_contact_keyframes(self) -> None:
        home = np.zeros(6)
        grasp = np.array([0.20, -0.10, 0.15, -0.05, 0.08, -0.03])
        place = np.array([-0.12, 0.11, -0.14, 0.06, -0.07, 0.04])
        approach_grasp = grasp * 0.7
        approach_place = place * 0.7
        trajectory = build_task_trajectory(home, grasp, place, approach_grasp, approach_place, "precision_insert")
        np.testing.assert_allclose(trajectory[GRASP_INDEX], grasp)
        np.testing.assert_allclose(trajectory[PLACE_INDEX], place)
        np.testing.assert_allclose(trajectory[-1], home)

    def test_counterfactual_scales_are_conservative_and_finite(self) -> None:
        grasp = np.eye(4)
        place = np.eye(4)
        context = pose_to_context(grasp, place, 0.92, 0.003, np.deg2rad(2.0), "transfer")
        xy_sigma_m, z_sigma_m, yaw_sigma_rad = counterfactual_sensor_scales(context, 0.5)
        self.assertAlmostEqual(xy_sigma_m, 0.0015, places=7)
        self.assertGreaterEqual(z_sigma_m, 0.0005)
        self.assertGreater(yaw_sigma_rad, 0.0)

    def test_robust_pose_belief_rejects_a_large_translation_and_yaw_outlier(self) -> None:
        def observed(x_offset: float, yaw_rad: float) -> np.ndarray:
            pose = np.eye(4)
            pose[:3, :3] = rotation_z(yaw_rad)
            pose[:3, 3] = [0.25 + x_offset, -0.12, 0.30]
            return pose

        belief = fuse_pose_belief(
            (
                observed(0.000, 0.000),
                observed(0.001, 0.010),
                observed(-0.001, -0.010),
                observed(0.050, 0.500),
            ),
            xy_sigma_m=0.003,
            z_sigma_m=0.002,
            yaw_sigma_rad=np.deg2rad(2.0),
        )
        self.assertLess(abs(float(belief.pose[0, 3]) - 0.25), 0.004)
        self.assertLess(abs(float(np.arctan2(belief.pose[1, 0], belief.pose[0, 0]))), 0.08)
        self.assertLess(belief.inlier_fraction, 1.0)
        self.assertGreater(belief.maximum_normalized_residual, 2.5)
        self.assertGreater(belief.effective_view_count, 3.0)

    def test_multiview_synthesis_preserves_primary_observation_and_count(self) -> None:
        clean = np.eye(4)
        clean[:3, 3] = [0.25, -0.12, 0.30]
        primary = clean.copy()
        primary[0, 3] += 0.002
        observations = synthesize_multiview_observations(
            clean,
            primary,
            view_count=5,
            xy_sigma_m=0.003,
            z_sigma_m=0.002,
            yaw_sigma_rad=np.deg2rad(2.0),
            outlier_probability=0.2,
            rng=np.random.default_rng(123),
        )
        self.assertEqual(len(observations), 5)
        np.testing.assert_allclose(observations[0], primary)
        self.assertTrue(all(pose.shape == (4, 4) for pose in observations))

    def test_balanced_episode_subset_keeps_an_equal_task_count(self) -> None:
        task_ids = np.repeat(np.arange(TASK_COUNT, dtype=np.int8), 3)
        data = {
            "task_ids": task_ids,
            "contexts": np.arange(len(task_ids) * CONTEXT_DIM, dtype=np.float32).reshape(len(task_ids), CONTEXT_DIM),
        }
        subset = balanced_episode_subset(data, episodes_per_task=2)
        self.assertEqual(len(subset["task_ids"]), TASK_COUNT * 2)
        for task_id in range(TASK_COUNT):
            self.assertEqual(int(np.count_nonzero(subset["task_ids"] == task_id)), 2)


if __name__ == "__main__":
    unittest.main()
