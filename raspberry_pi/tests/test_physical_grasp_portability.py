from __future__ import annotations

from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch


PROJECT_DIR = Path(__file__).resolve().parents[1]
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))

import tools.simulate_physical_grasp as physical  # noqa: E402


class PhysicalGraspPortabilityTests(unittest.TestCase):
    def test_checked_in_collision_model_runs_without_external_cad_world(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            missing_world = Path(temporary) / "robot_world_model.urdf"
            with patch.object(physical, "DEFAULT_WORLD_URDF_PATH", missing_world):
                description = physical.parse_robot_description()

        self.assertEqual(len(description.joints), 6)
        self.assertEqual(len(description.collision_meshes), 8)
        self.assertTrue(all(path.is_file() for path in description.collision_meshes))
        self.assertEqual(description.world_meshes, ())
        self.assertIsNone(description.world_urdf)

    def test_explicit_missing_external_world_is_reported(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            missing_world = Path(temporary) / "robot_world_model.urdf"
            with self.assertRaises(FileNotFoundError):
                physical.parse_robot_description(missing_world)


if __name__ == "__main__":
    unittest.main()
