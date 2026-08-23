from __future__ import annotations

import unittest
import ast
from pathlib import Path

from robot_ai.status_dashboard import build_dashboard, build_joint_cards, demo_snapshot


class StatusDashboardTests(unittest.TestCase):
    def test_demo_makes_stale_j5_and_offline_j6_explicit(self) -> None:
        view = build_dashboard(demo_snapshot(now_monotonic_s=100.0), now_monotonic_s=100.0)
        states = {joint["joint_id"]: joint["state"] for joint in view["joints"]}
        self.assertEqual(states[5], "stale")
        self.assertEqual(states[6], "offline_reported")
        self.assertEqual(view["overall_status"], "blocked")
        self.assertFalse(view["hardware_motion"])
        self.assertTrue(view["presentation_only"])

    def test_successful_query_is_not_mistaken_for_fresh_online(self) -> None:
        report = {
            "samples": [
                {
                    "joint_id": 1,
                    "ok": True,
                    "observed_at_monotonic_s": 1.0,
                    "feedback": {"online": True},
                }
            ]
        }
        cards = build_joint_cards(report, now_monotonic_s=2.0, freshness_threshold_s=0.5)
        self.assertEqual(cards[0]["state"], "stale")
        self.assertEqual(cards[1]["state"], "missing")

    def test_simulation_can_be_preview_ready_but_never_motion_enabled(self) -> None:
        snapshot = demo_snapshot(now_monotonic_s=10.0)
        snapshot["mode"] = "simulation"
        view = build_dashboard(snapshot, now_monotonic_s=10.0)
        self.assertEqual(view["overall_status"], "preview_ready")
        self.assertFalse(view["hardware_motion"])

    def test_saved_read_only_motor_report_can_be_loaded_directly(self) -> None:
        report = {
            "read_only": True,
            "samples": [
                {
                    "joint_id": index,
                    "ok": True,
                    "observed_at_monotonic_s": 10.0,
                    "feedback": {"joint_id": index, "online": True},
                }
                for index in range(1, 7)
            ],
        }
        view = build_dashboard(report, now_monotonic_s=10.1)
        self.assertEqual(view["mode"], "read_only_hardware")
        self.assertEqual(view["joints"][4]["state"], "fresh_online")

    def test_unknown_mode_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "unsupported dashboard mode"):
            build_dashboard({"mode": "execute"})

    def test_dashboard_source_has_no_hardware_or_process_imports(self) -> None:
        source_path = Path(__file__).resolve().parents[1] / "robot_ai/xiaou_status_dashboard.py"
        tree = ast.parse(source_path.read_text(encoding="utf-8"))
        imports: set[str] = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imports.update(alias.name.split(".", 1)[0] for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                imports.add(node.module.split(".", 1)[0])
        self.assertFalse({"serial", "can", "rclpy", "subprocess", "socket"} & imports)


if __name__ == "__main__":
    unittest.main()
