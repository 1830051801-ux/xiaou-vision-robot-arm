from __future__ import annotations

import json
import math
from pathlib import Path
import sys
import unittest


PROJECT_DIR = Path(__file__).resolve().parents[1]
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))

from robot_ai.arm_control import build_grasp_family_preview, load_default_model, load_grasp_family_registry


READY_DEG = [0.0, -50.0, -55.0, -70.0, 110.0, 0.0]
TAUGHT_DEG = [-100.000244140625, -48.93047332763672, -71.99970245361328, -84.9998664855957, 110.00060272216797, 0.0]


def effective_limits_deg() -> tuple[list[float], list[float]]:
    path = PROJECT_DIR / "robot_ai" / "arm_control" / "config" / "hardware_calibration_candidate_20260811.json"
    data = json.loads(path.read_text(encoding="utf-8"))
    return (
        [math.degrees(float(value)) for value in data["position_min_rad"]],
        [math.degrees(float(value)) for value in data["position_max_rad"]],
    )


class GraspFamilyTests(unittest.TestCase):
    def setUp(self) -> None:
        self.lower, self.upper = effective_limits_deg()
        self.model = load_default_model()

    def preview(self, object_class: str, **kwargs):
        return build_grasp_family_preview(
            object_class,
            READY_DEG,
            TAUGHT_DEG,
            model=self.model,
            lower_deg=self.lower,
            upper_deg=self.upper,
            **kwargs,
        )

    def test_cola_reuses_the_exact_taught_route_only_inside_local_envelope(self) -> None:
        preview = self.preview("cola")
        self.assertTrue(preview.preview_ready)
        self.assertIsNotNone(preview.plan)
        self.assertEqual(preview.plan.stages[-1].name, "taught_side_contact")

    def test_similar_bottle_is_previewable_but_oversize_bottle_needs_new_teach(self) -> None:
        local = self.preview("bottle", object_radius_m=0.033, object_height_m=0.192)
        self.assertTrue(local.preview_ready)
        oversized = self.preview("bottle", object_radius_m=0.045, object_height_m=0.240)
        self.assertEqual(oversized.status, "requires_dedicated_teach")
        self.assertIsNone(oversized.plan)

    def test_non_cylinder_families_do_not_reuse_cola_contact(self) -> None:
        for object_class in ("cup", "pen", "desktop_item", "tissue_pull"):
            with self.subTest(object_class=object_class):
                preview = self.preview(object_class, object_radius_m=0.020, object_height_m=0.100)
                self.assertEqual(preview.status, "requires_dedicated_teach")
                self.assertFalse(preview.preview_ready)

    def test_unknown_and_flexible_classes_are_rejected(self) -> None:
        self.assertEqual(self.preview("earphone").status, "rejected")
        self.assertEqual(self.preview("unseen_object").status, "rejected")

    def test_outside_coordinate_envelope_requires_new_teach(self) -> None:
        preview = self.preview("cola", target_offset_base_m=(0.004, 0.0, 0.0))
        self.assertEqual(preview.status, "requires_dedicated_teach")

    def test_registry_has_no_implicit_generic_class(self) -> None:
        registry = load_grasp_family_registry()
        self.assertNotIn("generic", registry["classes"])
        self.assertEqual(registry["status"], "offline_preview_only")


if __name__ == "__main__":
    unittest.main()
