from __future__ import annotations

from pathlib import Path
import unittest

from robot_ai.face_presentation import recommend_face
from robot_ai.status_dashboard import build_dashboard, demo_snapshot


class FacePresentationTests(unittest.TestCase):
    def test_stale_j5_is_calmly_visible_as_a_confused_preview(self) -> None:
        view = build_dashboard(demo_snapshot(now_monotonic_s=10.0), now_monotonic_s=10.0)
        result = recommend_face(view)
        self.assertEqual(result.state, "confused")
        self.assertEqual(result.reason, "stale_feedback")

    def test_hardware_safety_gate_takes_priority_over_happy_state(self) -> None:
        snapshot = demo_snapshot(now_monotonic_s=10.0)
        snapshot["mode"] = "hardware"
        for row in snapshot["joint_report"]["samples"]:
            row["feedback"]["online"] = True
            row["observed_at_monotonic_s"] = 10.0
        view = build_dashboard(snapshot, now_monotonic_s=10.1)
        result = recommend_face(view)
        self.assertEqual(result.state, "stop")
        self.assertEqual(result.reason, "safety_gate")

    def test_existing_gif_assets_stay_present(self) -> None:
        assets = Path(__file__).resolve().parents[1] / "robot_ai/emote_assets/gif"
        expected = {"idle.gif", "smile.gif", "investigate.gif", "ponder.gif", "question.gif", "laugh.gif", "sad.gif", "shocked.gif", "angry.gif"}
        self.assertTrue(expected.issubset({path.name for path in assets.glob("*.gif")}))


if __name__ == "__main__":
    unittest.main()
