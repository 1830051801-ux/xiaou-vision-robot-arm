from __future__ import annotations

from pathlib import Path
import sys
import unittest


PROJECT_DIR = Path(__file__).resolve().parents[1]
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))

from robot_ai.arm_control import (  # noqa: E402
    TaughtWaypointPlanningError,
    build_taught_waypoint_preview,
    resolve_object_teach_spec,
)


LOWER = [-165.0, -125.0, -135.0, -175.0, -85.0, -175.0]
UPPER = [165.0, 125.0, 135.0, 175.0, 115.0, 175.0]
VELOCITY = [20.0] * 6
ACCELERATION = [12.0] * 6


def record_for(object_class: str) -> dict:
    spec = resolve_object_teach_spec(object_class)
    poses = {}
    for index, name in enumerate(spec.required_pose_ids):
        poses[name] = [-5.0 * index, -50.0, -55.0, -70.0, 100.0, 0.0]
    return {
        "schema": "xiaou_object_teach_record_v1",
        "read_only_capture": True,
        "hardware_motion": False,
        "object_class": object_class,
        "poses_deg": poses,
        "measurements": {name: 0.10 for name in spec.required_measurements},
        "gripper": {name: 0.05 for name in spec.gripper_measurements_required},
    }


class TaughtWaypointPreviewTests(unittest.TestCase):
    def test_cup_record_becomes_preview_only_piecewise_quintic_plan(self) -> None:
        preview = build_taught_waypoint_preview(
            record_for("cup"),
            position_min_deg=LOWER,
            position_max_deg=UPPER,
            velocity_max_deg_s=VELOCITY,
            acceleration_max_deg_s2=ACCELERATION,
        )
        self.assertEqual(preview["execution"], "planning_preview_only")
        self.assertFalse(preview["hardware_motion"])
        self.assertEqual(preview["segment_count"], 5)
        self.assertGreater(preview["total_duration_s"], 0.0)
        self.assertTrue(all(item["point_count"] >= 2 for item in preview["segments"]))

    def test_missing_intermediate_pose_is_rejected(self) -> None:
        record = record_for("pen")
        del record["poses_deg"]["overhead"]
        with self.assertRaisesRegex(TaughtWaypointPlanningError, "lacks required poses"):
            build_taught_waypoint_preview(
                record,
                position_min_deg=LOWER,
                position_max_deg=UPPER,
                velocity_max_deg_s=VELOCITY,
                acceleration_max_deg_s2=ACCELERATION,
            )

    def test_cola_local_route_is_not_misrepresented_as_generic_waypoint_teach(self) -> None:
        with self.assertRaisesRegex(TaughtWaypointPlanningError, "dedicated multi-pose"):
            build_taught_waypoint_preview(
                record_for("cola"),
                position_min_deg=LOWER,
                position_max_deg=UPPER,
                velocity_max_deg_s=VELOCITY,
                acceleration_max_deg_s2=ACCELERATION,
            )


if __name__ == "__main__":
    unittest.main()
