from __future__ import annotations

import unittest

from robot_ai.arm_control.feedback_quality import (
    analyse_feedback_samples,
    analyse_transport_events,
    latest_successful_observation_times,
    summarise_feedback_signature,
)


def sample(joint_id: int, timestamp: float, *, online: bool = True, ok: bool = True, latency: float = 20.0) -> dict:
    row = {
        "joint_id": joint_id,
        "ok": ok,
        "observed_at_monotonic_s": timestamp,
        "round_trip_ms": latency,
        "seq": 5,
        "response_seq": 5,
    }
    if ok:
        row["feedback"] = {"joint_id": joint_id, "online": online}
    else:
        row["error"] = "timeout"
    return row


class FeedbackQualityTests(unittest.TestCase):
    def test_j5_stale_is_not_hidden_by_other_fresh_joints(self) -> None:
        rows = [sample(index, 10.0 if index != 5 else 8.0) for index in range(1, 7)]
        report = analyse_feedback_samples(rows, now_monotonic_s=10.25, freshness_threshold_s=0.5)
        self.assertFalse(report["passed"])
        self.assertEqual(report["per_joint"]["5"]["freshness_state"], "stale")
        self.assertEqual(report["per_joint"]["1"]["freshness_state"], "fresh")

    def test_per_joint_sample_completion_avoids_sequential_scan_false_stale(self) -> None:
        rows = [sample(index, (index - 1) * 0.15) for index in range(1, 7)]
        references = latest_successful_observation_times(rows, joint_ids=range(1, 7))
        report = analyse_feedback_samples(
            rows,
            now_monotonic_s=0.90,
            freshness_threshold_s=0.50,
            freshness_reference_s_by_joint=references,
        )
        self.assertTrue(report["passed"])
        self.assertEqual(report["freshness_reference_mode"], "per_joint_sample_completion")
        for joint_id in range(1, 7):
            self.assertEqual(report["per_joint"][str(joint_id)]["freshness_state"], "fresh")
            self.assertEqual(
                report["per_joint"][str(joint_id)]["freshness_reference"],
                "per_joint_sample_completion",
            )

    def test_controller_offline_and_sequence_mismatch_have_distinct_reasons(self) -> None:
        rows = [sample(index, 1.0) for index in range(1, 7)]
        rows[4]["feedback"]["online"] = False
        rows[2]["response_seq"] = 4
        report = analyse_feedback_samples(rows, now_monotonic_s=1.1)
        self.assertEqual(report["per_joint"]["5"]["freshness_state"], "offline_reported")
        self.assertIn("response_sequence", report["per_joint"]["3"]["blockers"])

    def test_future_or_reversed_timestamps_fail_closed(self) -> None:
        rows = [sample(index, 2.0) for index in range(1, 7)]
        rows.extend([sample(1, 1.0), sample(1, 3.0)])
        report = analyse_feedback_samples(rows, now_monotonic_s=2.5)
        self.assertIn("timestamp_order", report["per_joint"]["1"]["blockers"])
        self.assertEqual(report["per_joint"]["1"]["freshness_state"], "future_timestamp")

    def test_latency_budget_is_optional_and_checked_when_requested(self) -> None:
        rows = [sample(index, 1.0, latency=30.0) for index in range(1, 7)]
        report = analyse_feedback_samples(rows, now_monotonic_s=1.1, max_p95_latency_ms=25.0)
        self.assertIn("latency_p95", report["per_joint"]["1"]["blockers"])

    def test_transport_recovery_is_not_reported_as_healthy(self) -> None:
        events = [
            {"seq": 1, "ok": False, "round_trip_ms": 300.0},
            {"seq": 2, "ok": False, "round_trip_ms": 300.0},
            {"seq": 3, "ok": True, "response_seq": 3, "round_trip_ms": 24.0},
        ]
        report = analyse_transport_events(events, fast_response_threshold_ms=100.0)
        self.assertEqual(report["classification"], "controller_recovered_during_run")
        self.assertEqual(report["initial_consecutive_failures"], 2)
        self.assertFalse(report["passed"])

    def test_all_joint_zero_offline_signature_is_shared_not_j5_only(self) -> None:
        rows = [
            {
                "joint_id": joint_id,
                "ok": True,
                "feedback": {
                    "joint_id": joint_id,
                    "online": False,
                    "angle_deg": 0.0,
                    "speed_rpm": 0.0,
                    "torque_nm": 0.0,
                },
            }
            for joint_id in range(1, 7)
        ]
        report = summarise_feedback_signature(rows, joint_ids=range(1, 7))
        self.assertEqual(report["signature"], "all_requested_joints_offline_zero")
        self.assertEqual(report["scope_hint"], "shared_f407_feedback_cache_motor_bus_or_power")
        self.assertTrue(report["physical_fault_not_proven"])

    def test_requested_joint_subset_does_not_fail_unrequested_joints(self) -> None:
        report = analyse_feedback_samples(
            [sample(5, 1.0)],
            now_monotonic_s=1.1,
            joint_ids=[5],
        )
        self.assertTrue(report["passed"])
        self.assertEqual(report["requested_joint_ids"], [5])
        self.assertEqual(set(report["per_joint"]), {"5"})


if __name__ == "__main__":
    unittest.main()
