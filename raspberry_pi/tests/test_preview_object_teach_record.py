from __future__ import annotations

import contextlib
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch


PROJECT_DIR = Path(__file__).resolve().parents[1]
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))

from tools.preview_object_teach_record import main  # noqa: E402


def _cup_record() -> dict:
    return {
        "schema": "xiaou_object_teach_record_v1",
        "read_only_capture": True,
        "hardware_motion": False,
        "object_class": "cup",
        "poses_deg": {
            "ready": [0.0, -50.0, -55.0, -70.0, 110.0, 0.0],
            "raise": [-5.0, -50.0, -55.0, -70.0, 110.0, 0.0],
            "overhead": [-20.0, -50.0, -65.0, -75.0, 110.0, 0.0],
            "pregrasp": [-30.0, -50.0, -70.0, -80.0, 110.0, 0.0],
            "side_contact": [-40.0, -50.0, -72.0, -85.0, 110.0, 0.0],
            "lift": [-40.0, -50.0, -55.0, -70.0, 110.0, 0.0],
        },
        "measurements": {
            "object_radius_m": 0.045,
            "object_height_m": 0.09,
            "table_height_m": 0.0,
            "handle_yaw_rad": 0.0,
            "side_approach_clearance_m": 0.02,
        },
        "gripper": {
            "open_command": 0.0,
            "close_command": 1.0,
            "minimum_safe_width_m": 0.04,
        },
    }


class PreviewObjectTeachRecordTests(unittest.TestCase):
    def test_cli_writes_preview_only_plan_for_a_dedicated_teach_record(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            record = root / "cup_record.json"
            output = root / "cup_preview.json"
            record.write_text(json.dumps(_cup_record()), encoding="utf-8")
            arguments = [
                "preview_object_teach_record.py",
                "--record", str(record),
                "--output", str(output),
            ]
            with patch.object(sys, "argv", arguments), contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(main(), 0)
            payload = json.loads(output.read_text(encoding="utf-8"))
        self.assertEqual(payload["execution"], "planning_preview_only")
        self.assertFalse(payload["hardware_motion"])
        self.assertEqual(payload["object"]["object_class"], "cup")
        self.assertEqual(payload["segment_count"], 5)
        self.assertNotIn("transport", payload)


if __name__ == "__main__":
    unittest.main()
