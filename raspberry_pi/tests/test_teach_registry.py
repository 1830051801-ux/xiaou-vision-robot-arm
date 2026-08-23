from __future__ import annotations

from pathlib import Path
import sys
import unittest


PROJECT_DIR = Path(__file__).resolve().parents[1]
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))

from robot_ai.arm_control import (  # noqa: E402
    TeachRegistryError,
    load_object_teach_registry,
    resolve_object_teach_spec,
    validate_taught_object_record,
)


class TeachRegistryTests(unittest.TestCase):
    def test_known_object_families_have_explicit_teach_contracts(self) -> None:
        registry = load_object_teach_registry()
        self.assertFalse(registry["hardware_motion"])
        self.assertEqual(
            set(registry["objects"]),
            {"cola", "bottle", "cup", "pen", "desktop_item", "tissue_pull", "earphone"},
        )
        for name in ("bottle", "cup", "pen", "desktop_item", "tissue_pull"):
            with self.subTest(name=name):
                spec = resolve_object_teach_spec(name)
                self.assertTrue(spec.ready_for_real_teach)
                self.assertGreaterEqual(len(spec.required_pose_ids), 6)

    def test_aliases_resolve_without_creating_a_generic_route(self) -> None:
        self.assertEqual(resolve_object_teach_spec("water_bottle").object_class, "bottle")
        self.assertEqual(resolve_object_teach_spec("tissue").object_class, "tissue_pull")
        with self.assertRaisesRegex(TeachRegistryError, "unknown object class"):
            resolve_object_teach_spec("mystery_object")

    def test_reject_family_cannot_be_turned_into_a_taught_record(self) -> None:
        record = {
            "schema": "xiaou_object_teach_record_v1",
            "read_only_capture": True,
            "hardware_motion": False,
            "object_class": "earphone",
            "poses_deg": {"separate_family_design": [0.0] * 6},
            "measurements": {"deformable_geometry_model": 1.0},
            "gripper": {"separate_end_effector_characterization": 1.0},
        }
        with self.assertRaisesRegex(TeachRegistryError, "intentionally not a taught"):
            validate_taught_object_record(record)

    def test_multi_pose_record_requires_every_measurement(self) -> None:
        spec = resolve_object_teach_spec("cup")
        record = {
            "schema": "xiaou_object_teach_record_v1",
            "read_only_capture": True,
            "hardware_motion": False,
            "object_class": "cup",
            "poses_deg": {name: [0.0] * 6 for name in spec.required_pose_ids},
            "measurements": {name: 0.10 for name in spec.required_measurements},
            "gripper": {name: 0.05 for name in spec.gripper_measurements_required},
        }
        report = validate_taught_object_record(record)
        self.assertTrue(report["validated"])
        self.assertEqual(report["pose_count"], len(spec.required_pose_ids))
        del record["measurements"]["handle_yaw_rad"]
        with self.assertRaisesRegex(TeachRegistryError, "lacks measurements"):
            validate_taught_object_record(record)


if __name__ == "__main__":
    unittest.main()
