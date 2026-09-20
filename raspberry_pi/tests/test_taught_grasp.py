from __future__ import annotations

import json
import math
from pathlib import Path
import unittest

import numpy as np


PROJECT_DIR = Path(__file__).resolve().parents[1]
import sys

if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))

from robot_ai.arm_control import build_taught_side_grasp_plan, load_default_model
from robot_ai.arm_control.kinematics import fk_space


READY_DEG = [0.0, -50.0, -55.0, -70.0, 110.0, 0.0]
TAUGHT_DEG = [-100.000244140625, -48.93047332763672, -71.99970245361328, -84.9998664855957, 110.00060272216797, 0.0]


def effective_limits_deg() -> tuple[list[float], list[float]]:
    path = PROJECT_DIR / "robot_ai" / "arm_control" / "config" / "hardware_calibration_candidate_20260811.json"
    data = json.loads(path.read_text(encoding="utf-8"))
    return (
        [math.degrees(float(value)) for value in data["position_min_rad"]],
        [math.degrees(float(value)) for value in data["position_max_rad"]],
    )


class TaughtGraspPlannerTests(unittest.TestCase):
    def test_plan_uses_raise_transfer_descend_and_side_approach(self) -> None:
        lower, upper = effective_limits_deg()
        plan = build_taught_side_grasp_plan(
            READY_DEG,
            TAUGHT_DEG,
            model=load_default_model(),
            lower_deg=lower,
            upper_deg=upper,
        )
        stages = plan.stages
        self.assertEqual(stages[0].phase, "raise")
        self.assertEqual(stages[1].phase, "transfer")
        self.assertEqual(stages[2].phase, "transfer")
        self.assertTrue(any(stage.phase == "descend" for stage in stages))
        self.assertTrue(any(stage.phase == "side_approach" for stage in stages))
        self.assertEqual(stages[-1].name, "taught_side_contact")
        np.testing.assert_allclose(stages[-1].target_joint_rad, np.radians(TAUGHT_DEG), atol=1e-12)
        self.assertGreaterEqual(plan.sampled_transit_min_clearance_m, plan.overhead_clearance_m)
        for stage in stages[:-1]:
            if stage.phase == "side_approach":
                distance = np.linalg.norm(stage.target_pose[:2, 3] - plan.object_center_m[:2])
                self.assertGreaterEqual(
                    distance,
                    plan.object_radius_m + plan.contact_handoff_clearance_m - 1e-9,
                )

    def test_plan_stays_inside_effective_joint_limits(self) -> None:
        lower, upper = effective_limits_deg()
        plan = build_taught_side_grasp_plan(
            READY_DEG,
            TAUGHT_DEG,
            model=load_default_model(),
            lower_deg=lower,
            upper_deg=upper,
        )
        for stage in plan.stages:
            values = np.degrees(stage.target_joint_rad)
            self.assertTrue(np.all(values >= np.asarray(lower) - 1e-9))
            self.assertTrue(np.all(values <= np.asarray(upper) + 1e-9))

    def test_plan_is_explicitly_gripper_free(self) -> None:
        lower, upper = effective_limits_deg()
        payload = build_taught_side_grasp_plan(
            READY_DEG,
            TAUGHT_DEG,
            model=load_default_model(),
            lower_deg=lower,
            upper_deg=upper,
        ).as_dict()
        self.assertEqual(payload["gripper_action"], "not_included")
        self.assertFalse(payload["hardware_motion"])

    def test_small_contact_offset_retargets_the_taught_tcp_by_ik(self) -> None:
        lower, upper = effective_limits_deg()
        model = load_default_model()
        offset = np.asarray((0.003, -0.002, 0.001), dtype=np.float64)
        plan = build_taught_side_grasp_plan(
            READY_DEG,
            TAUGHT_DEG,
            model=model,
            lower_deg=lower,
            upper_deg=upper,
            taught_contact_offset_base_m=offset,
        )
        final_stage = plan.stages[-1]
        self.assertEqual(final_stage.name, "taught_side_contact")
        self.assertFalse(plan.final_joint_is_exact_taught_pose)
        np.testing.assert_allclose(plan.taught_contact_offset_m, offset, atol=1e-12)
        source_pose = fk_space(model.home_grasp_tcp, model.screw_axes, np.radians(TAUGHT_DEG))
        np.testing.assert_allclose(final_stage.target_pose[:3, 3], source_pose[:3, 3] + offset, atol=1e-12)
        self.assertLessEqual(final_stage.ik_position_error_m, 2e-4)

    def test_contact_handoff_must_be_inside_the_pregrasp_envelope(self) -> None:
        lower, upper = effective_limits_deg()
        with self.assertRaisesRegex(RuntimeError, "contact handoff distance"):
            build_taught_side_grasp_plan(
                READY_DEG,
                TAUGHT_DEG,
                model=load_default_model(),
                lower_deg=lower,
                upper_deg=upper,
                pregrasp_clearance_m=0.060,
                contact_handoff_clearance_m=0.060,
            )


if __name__ == "__main__":
    unittest.main()
