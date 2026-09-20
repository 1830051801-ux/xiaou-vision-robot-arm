"""Pure quality analysis for saved, read-only joint-feedback samples.

This module never creates a transport.  It exists so J5/J6 instability can be
distinguished into query loss, controller-reported offline, stale feedback,
clock anomaly, sequence mismatch, and slow response without relaxing any
motion gate.
"""

from __future__ import annotations

import math
import statistics
from typing import Any, Iterable, Mapping


JOINT_COUNT = 6


def _finite(value: object) -> float | None:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def _percentile(values: list[float], quantile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * quantile
    lower, upper = math.floor(position), math.ceil(position)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def latest_successful_observation_times(
    samples: Iterable[dict[str, Any]], *, joint_ids: Iterable[int]
) -> dict[int, float]:
    """Return each requested joint's latest successful sample-completion time.

    A serial diagnostic reads joints one at a time.  Its report is produced only
    after the final joint has returned, so using one report-completion timestamp
    makes the first few otherwise healthy joints look artificially old.  These
    per-joint timestamps preserve the meaning of an F407 ``online`` flag at the
    instant that particular response was received.  Query failures, offline
    reports, sequence mismatches, and timestamp-order checks remain the
    responsibility of :func:`analyse_feedback_samples`.
    """

    requested_joint_ids = sorted({int(joint_id) for joint_id in joint_ids})
    if not requested_joint_ids or any(joint_id < 1 or joint_id > JOINT_COUNT for joint_id in requested_joint_ids):
        raise ValueError(f"joint_ids must be non-empty and in 1..{JOINT_COUNT}")
    requested = set(requested_joint_ids)
    references: dict[int, float] = {}
    for row in samples:
        if not isinstance(row, dict) or row.get("ok") is not True:
            continue
        try:
            joint_id = int(row.get("joint_id"))
        except (TypeError, ValueError):
            continue
        if joint_id not in requested:
            continue
        timestamp = _finite(row.get("observed_at_monotonic_s"))
        if timestamp is not None:
            references[joint_id] = timestamp
    return references


def analyse_transport_events(
    events: Iterable[dict[str, Any]], *, fast_response_threshold_ms: float = 100.0
) -> dict[str, Any]:
    """Classify the Pi-to-F407 transport without hiding recovery events.

    ``events`` are ordered PING or GET_JOINT result dictionaries from the
    read-only probe.  A late successful answer is intentionally reported as
    late rather than being folded into a healthy average.  This distinguishes
    a controller that became responsive during the run from a healthy link.
    """
    if not math.isfinite(fast_response_threshold_ms) or fast_response_threshold_ms <= 0.0:
        raise ValueError("fast_response_threshold_ms must be finite and positive")

    rows = [dict(item) for item in events if isinstance(item, dict)]
    successful = [row for row in rows if row.get("ok") is True]
    failed = [row for row in rows if row.get("ok") is not True]
    latencies = [_finite(row.get("round_trip_ms")) for row in successful]
    latencies = [item for item in latencies if item is not None and item >= 0.0]
    delayed = [item for item in latencies if item > fast_response_threshold_ms]
    sequence_mismatches = sum(
        1
        for row in successful
        if row.get("seq") is not None and row.get("response_seq") is not None and row["seq"] != row["response_seq"]
    )

    initial_failures = 0
    for row in rows:
        if row.get("ok") is True:
            break
        initial_failures += 1
    largest_failure_streak = 0
    current_failure_streak = 0
    for row in rows:
        if row.get("ok") is True:
            current_failure_streak = 0
        else:
            current_failure_streak += 1
            largest_failure_streak = max(largest_failure_streak, current_failure_streak)

    if not successful:
        classification = "controller_unresponsive"
        recommended_action = "check_f407_power_uart_and_main_loop"
    elif initial_failures:
        classification = "controller_recovered_during_run"
        recommended_action = "wait_for_f407_ready_then_check_boot_or_blocking_work"
    elif failed:
        classification = "intermittent_transport"
        recommended_action = "check_uart_contention_or_f407_scheduler"
    elif delayed:
        classification = "slow_transport"
        recommended_action = "check_f407_response_deadline"
    else:
        classification = "responsive"
        recommended_action = "transport_ready"

    return {
        "analysis_only": True,
        "fast_response_threshold_ms": fast_response_threshold_ms,
        "events": len(rows),
        "response_pass": len(successful),
        "response_fail": len(failed),
        "initial_consecutive_failures": initial_failures,
        "largest_consecutive_failures": largest_failure_streak,
        "delayed_success_count": len(delayed),
        "latency_p50_ms": _percentile(latencies, 0.50),
        "latency_p95_ms": _percentile(latencies, 0.95),
        "sequence_mismatch_count": sequence_mismatches,
        "classification": classification,
        "recommended_action": recommended_action,
        "passed": not failed and not delayed and not sequence_mismatches,
    }


def summarise_feedback_signature(
    samples: Iterable[dict[str, Any]], *, joint_ids: Iterable[int]
) -> dict[str, Any]:
    """Summarise shared versus joint-specific read-only feedback failures.

    The result deliberately describes the *reported feedback signature*, not a
    physical fault.  In particular, six ``online=false`` zero records can be
    a shared F407 cache/bus/power problem and must not be labelled as six bad
    motors or as a J5-only defect.
    """
    requested_ids = sorted({int(joint_id) for joint_id in joint_ids})
    rows = [dict(item) for item in samples if isinstance(item, dict)]
    records = [
        row["feedback"]
        for row in rows
        if row.get("ok") is True and isinstance(row.get("feedback"), dict)
    ]
    observed_ids = sorted(
        {
            int(record["joint_id"])
            for record in records
            if isinstance(record.get("joint_id"), int)
        }
    )
    online_ids = sorted(
        {
            int(record["joint_id"])
            for record in records
            if record.get("online") is True and isinstance(record.get("joint_id"), int)
        }
    )

    def _zero_offline(record: dict[str, Any]) -> bool:
        values = [_finite(record.get(field)) for field in ("angle_deg", "speed_rpm", "torque_nm")]
        return (
            record.get("online") is False
            and all(value is not None and abs(value) <= 1e-6 for value in values)
        )

    all_offline = bool(records) and not online_ids
    all_zero_offline = bool(records) and all(_zero_offline(record) for record in records)
    complete_coverage = set(requested_ids).issubset(observed_ids)
    if not records:
        signature = "no_joint_feedback"
        scope_hint = "controller_or_uart_transport"
        recommended_action = "check_f407_response_before_assessing_individual_joints"
    elif complete_coverage and all_offline and all_zero_offline:
        signature = "all_requested_joints_offline_zero"
        scope_hint = "shared_f407_feedback_cache_motor_bus_or_power"
        recommended_action = "inspect_shared_feedback_path_before_single_joint_debug"
    elif all_offline:
        signature = "all_reported_joints_offline"
        scope_hint = "shared_feedback_path_or_controller_state"
        recommended_action = "inspect_f407_feedback_state_and_motor_bus"
    elif len(online_ids) < len(requested_ids):
        signature = "mixed_joint_availability"
        scope_hint = "per_joint_feedback_or_wiring"
        recommended_action = "compare_offline_joint_feedback_with_known_good_joint"
    else:
        signature = "online_feedback_present"
        scope_hint = "feedback_available"
        recommended_action = "continue_latency_and_freshness_validation"

    return {
        "analysis_only": True,
        "requested_joint_ids": requested_ids,
        "reported_joint_ids": observed_ids,
        "online_joint_ids": online_ids,
        "all_successful_samples_offline": all_offline,
        "all_successful_feedback_zero_offline": all_zero_offline,
        "signature": signature,
        "scope_hint": scope_hint,
        "recommended_action": recommended_action,
        "physical_fault_not_proven": True,
    }


def analyse_feedback_samples(
    samples: Iterable[dict[str, Any]],
    *,
    now_monotonic_s: float | None,
    freshness_threshold_s: float = 0.5,
    min_online_ratio: float = 1.0,
    max_p95_latency_ms: float | None = None,
    joint_ids: Iterable[int] | None = None,
    freshness_reference_s_by_joint: Mapping[int, float] | None = None,
) -> dict[str, Any]:
    """Return a conservative per-joint availability and freshness report.

    ``now_monotonic_s`` is the normal report-time reference, appropriate for a
    saved log.  A sequential live scan may instead supply
    ``freshness_reference_s_by_joint`` with every axis's own sample-completion
    timestamp.  That prevents scan order from fabricating stale feedback while
    retaining all other transport and feedback failure checks.
    """
    if not math.isfinite(freshness_threshold_s) or freshness_threshold_s <= 0.0:
        raise ValueError("freshness_threshold_s must be finite and positive")
    if not math.isfinite(min_online_ratio) or not 0.0 <= min_online_ratio <= 1.0:
        raise ValueError("min_online_ratio must be between 0 and 1")
    if max_p95_latency_ms is not None and (not math.isfinite(max_p95_latency_ms) or max_p95_latency_ms <= 0.0):
        raise ValueError("max_p95_latency_ms must be finite and positive when supplied")
    rows = [dict(item) for item in samples if isinstance(item, dict)]
    if joint_ids is None:
        requested_joint_ids = list(range(1, JOINT_COUNT + 1))
    else:
        requested_joint_ids = sorted({int(joint_id) for joint_id in joint_ids})
        if not requested_joint_ids or any(joint_id < 1 or joint_id > JOINT_COUNT for joint_id in requested_joint_ids):
            raise ValueError(f"joint_ids must be non-empty and in 1..{JOINT_COUNT}")
    reference_by_joint: dict[int, float] = {}
    if freshness_reference_s_by_joint is not None:
        for raw_joint_id, raw_reference in freshness_reference_s_by_joint.items():
            try:
                joint_id = int(raw_joint_id)
            except (TypeError, ValueError) as exc:
                raise ValueError("freshness reference contains an invalid joint ID") from exc
            reference = _finite(raw_reference)
            if joint_id not in requested_joint_ids or reference is None:
                raise ValueError("freshness reference must be finite and target a requested joint")
            reference_by_joint[joint_id] = reference
    per_joint: dict[str, dict[str, Any]] = {}
    blockers: list[str] = []
    for joint_id in requested_joint_ids:
        joint_rows = [row for row in rows if row.get("joint_id") == joint_id]
        successful = [row for row in joint_rows if row.get("ok") is True]
        online_rows = [row for row in successful if (row.get("feedback") or {}).get("online") is True]
        failed_rows = [row for row in joint_rows if row.get("ok") is not True]
        timestamps = [_finite(row.get("observed_at_monotonic_s")) for row in successful]
        timestamps = [item for item in timestamps if item is not None]
        latencies = [_finite(row.get("round_trip_ms")) for row in successful]
        latencies = [item for item in latencies if item is not None and item >= 0.0]
        sequence_mismatches = sum(
            1
            for row in successful
            if row.get("seq") is not None and row.get("response_seq") is not None and row["seq"] != row["response_seq"]
        )
        latest = successful[-1] if successful else None
        latest_feedback = dict(latest.get("feedback") or {}) if latest else {}
        latest_timestamp = _finite(latest.get("observed_at_monotonic_s")) if latest else None
        freshness_reference_s = reference_by_joint.get(joint_id, now_monotonic_s)
        freshness_reference = (
            "per_joint_sample_completion" if joint_id in reference_by_joint else "report_completion"
        )
        age_s = (
            None
            if latest_timestamp is None or freshness_reference_s is None
            else freshness_reference_s - latest_timestamp
        )
        if latest is None:
            freshness_state = "no_feedback"
        elif latest_feedback.get("online") is not True:
            freshness_state = "offline_reported"
        elif age_s is None:
            freshness_state = "freshness_unknown"
        elif age_s < 0.0:
            freshness_state = "future_timestamp"
        elif age_s > freshness_threshold_s:
            freshness_state = "stale"
        else:
            freshness_state = "fresh"
        online_ratio = len(online_rows) / len(successful) if successful else 0.0
        gaps = [later - earlier for earlier, later in zip(timestamps, timestamps[1:])]
        reversed_timestamps = sum(1 for gap in gaps if gap < 0.0)
        p95_latency = _percentile(latencies, 0.95)
        joint_blockers: list[str] = []
        if freshness_state != "fresh":
            joint_blockers.append(f"freshness:{freshness_state}")
        if failed_rows:
            joint_blockers.append("query_failures")
        if online_ratio < min_online_ratio:
            joint_blockers.append("online_ratio")
        if sequence_mismatches:
            joint_blockers.append("response_sequence")
        if reversed_timestamps:
            joint_blockers.append("timestamp_order")
        if max_p95_latency_ms is not None and (p95_latency is None or p95_latency > max_p95_latency_ms):
            joint_blockers.append("latency_p95")
        per_joint[str(joint_id)] = {
            "attempts": len(joint_rows),
            "query_pass": len(successful),
            "query_fail": len(failed_rows),
            "online_samples": len(online_rows),
            "offline_samples": len(successful) - len(online_rows),
            "online_ratio": round(online_ratio, 6),
            "latest_age_s": age_s,
            "freshness_reference": freshness_reference,
            "freshness_reference_monotonic_s": freshness_reference_s,
            "freshness_state": freshness_state,
            "latest_feedback": latest_feedback or None,
            "latency_p50_ms": _percentile(latencies, 0.50),
            "latency_p95_ms": p95_latency,
            "sequence_mismatch_count": sequence_mismatches,
            "timestamp_reversal_count": reversed_timestamps,
            "blockers": joint_blockers,
            "passed": not joint_blockers,
        }
        blockers.extend(f"J{joint_id}:{item}" for item in joint_blockers)
    return {
        "analysis_only": True,
        "requested_joint_ids": requested_joint_ids,
        "freshness_threshold_s": freshness_threshold_s,
        "freshness_reference_mode": (
            "per_joint_sample_completion" if reference_by_joint else "report_completion"
        ),
        "min_online_ratio": min_online_ratio,
        "max_p95_latency_ms": max_p95_latency_ms,
        "per_joint": per_joint,
        "blockers": blockers,
        "passed": not blockers,
    }
