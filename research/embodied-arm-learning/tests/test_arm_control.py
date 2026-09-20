from __future__ import annotations

import sys
import unittest
from pathlib import Path
import copy

import numpy as np


PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR / "robot_ai"))

from arm_control import (
    JointLimits,
    fk_space,
    ik_space,
    ik_space_multistart,
    jacobian_space,
    load_default_model,
)
from arm_control.safety import load_hardware_config, validate_motion_readiness


class ArmControlTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.model = load_default_model()

    def test_home_pose_matches_extracted_model(self) -> None:
        actual = fk_space(self.model.home_grasp_tcp, self.model.screw_axes, np.zeros(6))
        np.testing.assert_allclose(actual, self.model.home_grasp_tcp, atol=1e-12)

    def test_space_jacobian_translation_matches_finite_difference(self) -> None:
        angles = np.array([0.21, -0.34, 0.27, -0.19, 0.16, -0.23])
        transform = fk_space(self.model.home_grasp_tcp, self.model.screw_axes, angles)
        jacobian = jacobian_space(self.model.screw_axes, angles)
        epsilon = 1e-7
        for joint in range(6):
            perturbed = angles.copy()
            perturbed[joint] += epsilon
            shifted = fk_space(self.model.home_grasp_tcp, self.model.screw_axes, perturbed)
            numeric_velocity = (shifted[:3, 3] - transform[:3, 3]) / epsilon
            spatial_velocity = jacobian[3:, joint] + np.cross(jacobian[:3, joint], transform[:3, 3])
            np.testing.assert_allclose(numeric_velocity, spatial_velocity, rtol=2e-5, atol=2e-6)

    def test_inverse_kinematics_returns_a_pose_solution(self) -> None:
        target_angles = np.array([0.10, -0.18, 0.14, -0.08, 0.11, -0.07])
        target = fk_space(self.model.home_grasp_tcp, self.model.screw_axes, target_angles)
        result = ik_space(
            self.model.home_grasp_tcp,
            self.model.screw_axes,
            target,
            target_angles + np.array([0.02, -0.01, 0.01, -0.01, 0.01, -0.01]),
        )
        self.assertTrue(result.converged, result)
        recovered = fk_space(self.model.home_grasp_tcp, self.model.screw_axes, result.joint_angles)
        np.testing.assert_allclose(recovered, target, atol=2e-4)

    def test_inverse_kinematics_respects_supplied_measured_limits(self) -> None:
        lower = np.full(6, -0.5)
        upper = np.full(6, 0.5)
        target_angles = np.array([0.18, -0.24, 0.12, -0.10, 0.14, -0.08])
        target = fk_space(self.model.home_grasp_tcp, self.model.screw_axes, target_angles)
        result = ik_space(
            self.model.home_grasp_tcp,
            self.model.screw_axes,
            target,
            np.zeros(6),
            joint_lower=lower,
            joint_upper=upper,
        )
        self.assertTrue(result.converged, result)
        self.assertTrue(np.all(result.joint_angles >= lower))
        self.assertTrue(np.all(result.joint_angles <= upper))

    def test_inverse_kinematics_rejects_non_rigid_target(self) -> None:
        target = self.model.home_grasp_tcp.copy()
        target[0, 0] = 2.0
        with self.assertRaisesRegex(ValueError, "orthonormal"):
            ik_space(
                self.model.home_grasp_tcp,
                self.model.screw_axes,
                target,
                np.zeros(6),
            )

    def test_inverse_kinematics_local_recovery_over_deterministic_pose_set(self) -> None:
        rng = np.random.default_rng(20260805)
        lower = np.full(6, -0.8)
        upper = np.full(6, 0.8)
        for case_index in range(40):
            target_angles = rng.uniform(-0.45, 0.45, size=6)
            initial_angles = np.clip(
                target_angles + rng.uniform(-0.12, 0.12, size=6), lower, upper
            )
            target = fk_space(self.model.home_grasp_tcp, self.model.screw_axes, target_angles)
            result = ik_space(
                self.model.home_grasp_tcp,
                self.model.screw_axes,
                target,
                initial_angles,
                joint_lower=lower,
                joint_upper=upper,
            )
            self.assertTrue(result.converged, f"case {case_index}: {result}")
            recovered = fk_space(
                self.model.home_grasp_tcp, self.model.screw_axes, result.joint_angles
            )
            np.testing.assert_allclose(recovered, target, atol=2e-4)

    def test_inverse_kinematics_multistart_prefers_converged_seed(self) -> None:
        target_angles = np.array([-0.50025, 0.5789, -0.2176, 1.2034, 0.5695, -1.5698])
        target = fk_space(self.model.home_grasp_tcp, self.model.screw_axes, target_angles)
        lower = np.full(6, -np.pi)
        upper = np.full(6, np.pi)
        result = ik_space_multistart(
            self.model.home_grasp_tcp,
            self.model.screw_axes,
            target,
            [np.zeros(6), np.array([0.0, -0.5, 0.5, 0.0, 0.0, 0.0])],
            joint_lower=lower,
            joint_upper=upper,
            max_iterations=150,
        )
        self.assertTrue(result.converged, result)
        self.assertLess(result.position_error_m, 2e-4)

    def test_quintic_trajectory_obeys_supplied_test_limits(self) -> None:
        from arm_control import plan_quintic_joint_trajectory

        limits = JointLimits(
            position_min=np.full(6, -1.0),
            position_max=np.full(6, 1.0),
            velocity_max=np.full(6, 0.5),
            acceleration_max=np.full(6, 1.0),
        )
        start = np.zeros(6)
        goal = np.array([0.4, -0.3, 0.2, -0.1, 0.05, -0.02])
        points = plan_quintic_joint_trajectory(start, goal, limits, sample_period_s=0.005)
        np.testing.assert_allclose(points[0].positions, start, atol=1e-12)
        np.testing.assert_allclose(points[-1].positions, goal, atol=1e-12)
        self.assertLessEqual(max(np.max(np.abs(p.velocities)) for p in points), 0.5 + 1e-10)
        self.assertLessEqual(max(np.max(np.abs(p.accelerations)) for p in points), 1.0 + 1e-8)

    def test_quintic_trajectory_obeys_randomized_joint_limits(self) -> None:
        from arm_control import plan_quintic_joint_trajectory

        rng = np.random.default_rng(20260805)
        limits = JointLimits(
            position_min=np.full(6, -1.2),
            position_max=np.full(6, 1.2),
            velocity_max=np.array([0.25, 0.30, 0.35, 0.40, 0.45, 0.50]),
            acceleration_max=np.array([0.4, 0.5, 0.6, 0.7, 0.8, 0.9]),
        )
        for _ in range(40):
            start = rng.uniform(-0.8, 0.8, size=6)
            goal = rng.uniform(-0.8, 0.8, size=6)
            points = plan_quintic_joint_trajectory(
                start, goal, limits, sample_period_s=0.004
            )
            self.assertLessEqual(
                max(np.max(np.abs(point.velocities)) for point in points),
                float(np.max(limits.velocity_max)) + 1e-10,
            )
            for point in points:
                self.assertTrue(np.all(np.abs(point.velocities) <= limits.velocity_max + 1e-10))
                self.assertTrue(
                    np.all(np.abs(point.accelerations) <= limits.acceleration_max + 1e-8)
                )

    def test_quintic_trajectory_rejects_non_finite_timing(self) -> None:
        from arm_control import plan_quintic_joint_trajectory

        limits = JointLimits(
            position_min=np.full(6, -1.0),
            position_max=np.full(6, 1.0),
            velocity_max=np.full(6, 0.5),
            acceleration_max=np.full(6, 1.0),
        )
        with self.assertRaisesRegex(ValueError, "finite and positive"):
            plan_quintic_joint_trajectory(
                np.zeros(6), np.zeros(6), limits, sample_period_s=float("nan")
            )

    def test_default_hardware_configuration_keeps_motion_locked(self) -> None:
        config = load_hardware_config()
        readiness = validate_motion_readiness(config)
        self.assertFalse(readiness.ready)
        self.assertIn("motion_enabled", readiness.missing_or_invalid)
        self.assertNotIn("joint_node_ids", readiness.missing_or_invalid)
        self.assertIn("protocol_confirmed", readiness.missing_or_invalid)
        self.assertIn("uart_link_verified", readiness.missing_or_invalid)
        self.assertIn("f407_firmware_verified", readiness.missing_or_invalid)
        self.assertIn("estop_verified", readiness.missing_or_invalid)
        self.assertIn("feedback_verified", readiness.missing_or_invalid)

    def test_hardware_gate_rejects_node_ids_outside_xiaou_protocol_range(self) -> None:
        config = copy.deepcopy(load_hardware_config())
        config["joint_node_ids"] = [0, 1, 2, 3, 4, 5]
        readiness = validate_motion_readiness(config)
        self.assertIn("joint_node_ids_values", readiness.missing_or_invalid)

    def test_complete_measured_hardware_configuration_can_pass_gate(self) -> None:
        config = copy.deepcopy(load_hardware_config())
        config.update(
            {
                "motion_enabled": True,
                "protocol_confirmed": True,
                "uart_link_verified": True,
                "f407_firmware_verified": True,
                "estop_verified": True,
                "feedback_verified": True,
                "joint_node_ids": [1, 2, 3, 4, 5, 6],
                "encoder_zero_offset_rad": [0.0] * 6,
                "encoder_direction": [1, -1, 1, -1, 1, -1],
                "position_min_rad": [-1.0] * 6,
                "position_max_rad": [1.0] * 6,
                "velocity_max_rad_s": [0.2] * 6,
                "acceleration_max_rad_s2": [0.4] * 6,
            }
        )
        readiness = validate_motion_readiness(config)
        self.assertTrue(readiness.ready, readiness)

    def test_non_finite_or_non_positive_hardware_limits_are_rejected(self) -> None:
        config = copy.deepcopy(load_hardware_config())
        config["velocity_max_rad_s"] = [0.2, 0.2, 0.0, 0.2, 0.2, float("nan")]
        readiness = validate_motion_readiness(config)
        self.assertIn("velocity_max_rad_s_values", readiness.missing_or_invalid)
        self.assertIn("velocity_max_rad_s_positive", readiness.missing_or_invalid)


if __name__ == "__main__":
    unittest.main()
