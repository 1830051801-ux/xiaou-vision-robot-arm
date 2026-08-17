from __future__ import annotations

import importlib.util
from pathlib import Path
import unittest


PROJECT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = PROJECT_DIR / "tools" / "teach_grasp_dry_run.py"
SPEC = importlib.util.spec_from_file_location("teach_grasp_dry_run", SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
teach = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(teach)


class TeachGraspDryRunTests(unittest.TestCase):
    def test_quintic_preview_has_no_motion_output_and_obeys_preview_limits(self) -> None:
        plan = teach.build_quintic_preview(
            [0.0] * 6,
            [8.0, -6.0, 4.0, -3.0, 2.0, -1.0],
            speed_limit_deg_s=[10.0] * 6,
            accel_limit_deg_s2=[20.0] * 6,
            sample_period_s=0.05,
        )
        self.assertEqual(plan["execution"], "offline_preview_only")
        self.assertFalse(plan["motion_commands_emitted"])
        self.assertIn("goal", plan["forward_kinematics_preview"])
        self.assertEqual(plan["points"][0]["positions_deg"], [0.0] * 6)
        self.assertEqual(plan["points"][-1]["positions_deg"], [8.0, -6.0, 4.0, -3.0, 2.0, -1.0])
        self.assertLessEqual(
            max(abs(point["velocities_deg_s"][0]) for point in plan["points"]),
            10.0 + 1e-6,
        )
        self.assertLessEqual(
            max(abs(point["accelerations_deg_s2"][0]) for point in plan["points"]),
            20.0 + 1e-6,
        )

    def test_preview_accepts_user_confirmed_j5_teach_angle_of_120_deg(self) -> None:
        plan = teach.build_quintic_preview(
            [0.0] * 6,
            [0.0, 0.0, 0.0, 0.0, 120.0, 0.0],
            speed_limit_deg_s=[2.0] * 6,
            accel_limit_deg_s2=[1.0] * 6,
        )
        self.assertEqual(plan["goal_deg"][4], 120.0)
        self.assertFalse(plan["motion_commands_emitted"])

    def test_teach_capture_requires_six_online_samples(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "J6 reported offline feedback"):
            original = teach._request_joint
            try:
                teach._request_joint = lambda joint_id, sequence, **_: {
                    "joint_id": joint_id,
                    "seq": sequence,
                    "ok": True,
                    "feedback": {
                        "joint_id": joint_id,
                        "angle_deg": 0.0,
                        "speed_rpm": 0.0,
                        "torque_nm": 0.0,
                        "online": joint_id != 6,
                    },
                    "round_trip_ms": 20.0,
                }
                teach.capture_stationary_pose(
                    port="unused",
                    baud=115200,
                    timeout_s=0.3,
                    samples_per_joint=1,
                    interval_s=0.0,
                    max_angle_spread_deg=0.25,
                    max_speed_rpm=2.0,
                    min_success_ratio=0.85,
                    min_success_samples=1,
                    allow_offline_j6_preview=False,
                    j6_preview_deg=0.0,
                )
            finally:
                teach._request_joint = original

    def test_capture_accepts_one_transient_loss_when_long_sample_budget_passes(self) -> None:
        original = teach._request_joint
        try:
            def fake_request(joint_id: int, sequence: int, **_: object) -> dict:
                if joint_id == 5 and sequence == 5:
                    return {"joint_id": joint_id, "seq": sequence, "ok": False, "error": "timeout", "round_trip_ms": 300.0}
                return {
                    "joint_id": joint_id,
                    "seq": sequence,
                    "ok": True,
                    "feedback": {
                        "joint_id": joint_id,
                        "angle_deg": float(joint_id),
                        "speed_rpm": 0.0,
                        "torque_nm": 0.0,
                        "online": True,
                    },
                    "round_trip_ms": 25.0,
                }
            teach._request_joint = fake_request
            report = teach.capture_stationary_pose(
                port="unused",
                baud=115200,
                timeout_s=0.3,
                samples_per_joint=8,
                interval_s=0.0,
                max_angle_spread_deg=0.25,
                max_speed_rpm=2.0,
                min_success_ratio=0.85,
                min_success_samples=6,
                allow_offline_j6_preview=False,
                j6_preview_deg=0.0,
            )
        finally:
            teach._request_joint = original
        self.assertEqual(report["pose_deg"][4], 5.0)
        self.assertEqual(report["per_joint"][4]["query_failures"], 1)

    def test_j6_assumption_is_explicit_and_preview_only(self) -> None:
        original = teach._request_joint
        try:
            teach._request_joint = lambda joint_id, sequence, **_: {
                "joint_id": joint_id,
                "seq": sequence,
                "ok": True,
                "feedback": {
                    "joint_id": joint_id,
                    "angle_deg": 0.0,
                    "speed_rpm": 0.0,
                    "torque_nm": 0.0,
                    "online": joint_id != 6,
                },
                "round_trip_ms": 25.0,
            }
            report = teach.capture_stationary_pose(
                port="unused",
                baud=115200,
                timeout_s=0.3,
                samples_per_joint=2,
                interval_s=0.0,
                max_angle_spread_deg=0.25,
                max_speed_rpm=2.0,
                min_success_ratio=0.5,
                min_success_samples=1,
                allow_offline_j6_preview=True,
                j6_preview_deg=12.0,
            )
        finally:
            teach._request_joint = original
        self.assertEqual(report["assumed_joint_ids"], [6])
        self.assertFalse(report["all_joints_live_feedback"])
        self.assertEqual(report["pose_deg"][5], 12.0)


if __name__ == "__main__":
    unittest.main()
