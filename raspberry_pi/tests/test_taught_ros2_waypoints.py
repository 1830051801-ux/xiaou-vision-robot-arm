from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


PROJECT_DIR = Path(__file__).resolve().parents[1]
VALIDATOR_PATH = PROJECT_DIR / "tools" / "validate_taught_ros2_waypoints.py"


def _load_validator():
    spec = importlib.util.spec_from_file_location("taught_waypoint_validator", VALIDATOR_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load waypoint validator")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class TaughtRos2WaypointTests(unittest.TestCase):
    def test_exported_waypoints_are_valid_preview_only_contract(self) -> None:
        validator = _load_validator()
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "waypoints.json"
            result = subprocess.run(
                [
                    sys.executable,
                    str(PROJECT_DIR / "tools" / "export_taught_grasp_ros2_waypoints.py"),
                    "--output",
                    str(output),
                ],
                cwd=PROJECT_DIR,
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(result.returncode, 0, result.stderr)
            payload = json.loads(output.read_text(encoding="utf-8"))
        report = validator.validate(payload)
        self.assertTrue(report["validated"])
        self.assertEqual(report["final_waypoint"], "taught_side_contact")
        self.assertFalse(report["hardware_execution"])
        self.assertGreaterEqual(report["waypoint_count"], 2)
        self.assertEqual(report["grasp_family"], "taught_cylinder_side_v1")

    def test_live_execution_contract_is_rejected(self) -> None:
        validator = _load_validator()
        with self.assertRaisesRegex(ValueError, "planning_preview_only"):
            validator.validate({"schema": "xiaou_ros2_moveit_waypoints_v1", "execution": "live", "waypoints": []})


if __name__ == "__main__":
    unittest.main()
