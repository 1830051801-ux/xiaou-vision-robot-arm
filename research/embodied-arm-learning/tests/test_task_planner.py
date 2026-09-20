from __future__ import annotations

import copy
import json
import sys
import tempfile
import unittest
from pathlib import Path

import yaml


PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR / "robot_ai"))

from arm_control.task_planner import DetectionCenter, plan_task


def complete_profile() -> dict:
    strategy = {
        "grasp_height_m": 0.03,
        "approach_height_m": 0.09,
        "gripper_open_pwm_deg": 40.0,
        "gripper_close_pwm_deg": 110.0,
        "placement_pose_id": "bin_a",
        "grasp_mode": "top_down",
        "failure_policy": "vision_recheck_then_safe_return_then_report",
    }
    return {
        "schema_version": 2,
        "camera_contract": {
            "width_px": 1920,
            "height_px": 1080,
            "mount": "fixed_external_overhead",
            "camera_motion": "forbidden_during_pick",
        },
        "classes": {"cup": copy.deepcopy(strategy), "pen": copy.deepcopy(strategy)},
    }


def fixed_homography() -> dict:
    return {
        "image_width_px": 1920,
        "image_height_px": 1080,
        "output_frame": "robot_base_table",
        "output_unit": "m",
        "homography": [[0.001, 0.0, 0.0], [0.0, 0.001, 0.0], [0.0, 0.0, 1.0]],
    }


class TaskPlannerTests(unittest.TestCase):
    def _write_inputs(self, root: Path, profile: dict | None = None, homography: dict | None = None) -> tuple[Path, Path]:
        profile_path = root / "profiles.json"
        homography_path = root / "workspace.yaml"
        profile_path.write_text(json.dumps(profile or complete_profile()), encoding="utf-8")
        homography_path.write_text(yaml.safe_dump(homography or fixed_homography()), encoding="utf-8")
        return profile_path, homography_path

    def test_wrong_camera_resolution_is_rejected_before_any_mapping(self) -> None:
        plan = plan_task(
            "pick",
            [DetectionCenter("cup", 300.0, 200.0, 0.9)],
            image_width_px=1280,
            image_height_px=720,
            requested_class="cup",
        )
        self.assertFalse(plan.transmittable)
        self.assertIn("1920x1080", plan.reason)

    def test_default_profile_keeps_unmeasured_class_locked(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            _, homography_path = self._write_inputs(root)
            plan = plan_task(
                "pick",
                [DetectionCenter("cup", 300.0, 200.0, 0.9)],
                image_width_px=1920,
                image_height_px=1080,
                requested_class="cup",
                homography_path=homography_path,
            )
        self.assertFalse(plan.transmittable)
        self.assertIn("uncalibrated", plan.reason)

    def test_complete_preview_still_obeys_the_hardware_motion_gate(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            profile_path, homography_path = self._write_inputs(root)
            plan = plan_task(
                "pick",
                [DetectionCenter("cup", 300.0, 200.0, 0.9)],
                image_width_px=1920,
                image_height_px=1080,
                requested_class="cup",
                profile_path=profile_path,
                homography_path=homography_path,
            )
        self.assertFalse(plan.transmittable)
        self.assertEqual(len(plan.steps), 1)
        self.assertIn("motion locked", plan.reason)

    def test_tidy_all_has_deterministic_center_order_and_stop_is_non_motion(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            profile_path, homography_path = self._write_inputs(root)
            plan = plan_task(
                "tidy_all",
                [
                    DetectionCenter("pen", 900.0, 800.0, 0.8),
                    DetectionCenter("cup", 400.0, 300.0, 0.7),
                ],
                image_width_px=1920,
                image_height_px=1080,
                profile_path=profile_path,
                homography_path=homography_path,
            )
        self.assertEqual([step.object_class for step in plan.steps], ["cup", "pen"])
        stop = plan_task("stop", [], image_width_px=1, image_height_px=1)
        self.assertTrue(stop.transmittable)

    def test_homography_without_1920x1080_metadata_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            profile_path, homography_path = self._write_inputs(
                root,
                homography={"homography": fixed_homography()["homography"]},
            )
            plan = plan_task(
                "pick",
                [DetectionCenter("cup", 300.0, 200.0, 0.9)],
                image_width_px=1920,
                image_height_px=1080,
                requested_class="cup",
                profile_path=profile_path,
                homography_path=homography_path,
            )
        self.assertFalse(plan.transmittable)
        self.assertIn("recalibration", plan.reason)


if __name__ == "__main__":
    unittest.main()
