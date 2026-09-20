"""Explainable risk gates for offline grasp-strategy comparison.

The policy consumes outcomes from a simulator instead of talking to hardware.
It intentionally treats unverified perception or clearance as a reason to
recheck or block the task, never as permission to move a physical arm.
"""

from __future__ import annotations

from collections import Counter
from collections.abc import Iterable
from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class RiskThresholds:
    minimum_success_rate: float
    minimum_perception_coverage: float
    minimum_clearance_m: float

    def __post_init__(self) -> None:
        for label, value in (
            ("minimum_success_rate", self.minimum_success_rate),
            ("minimum_perception_coverage", self.minimum_perception_coverage),
        ):
            if not math.isfinite(value) or not 0.0 <= value <= 1.0:
                raise ValueError(f"{label} must be in 0..1")
        if not math.isfinite(self.minimum_clearance_m) or self.minimum_clearance_m < 0.0:
            raise ValueError("minimum_clearance_m must be finite and non-negative")

    def to_dict(self) -> dict[str, float]:
        return {
            "minimum_success_rate": self.minimum_success_rate,
            "minimum_perception_coverage": self.minimum_perception_coverage,
            "minimum_clearance_m": self.minimum_clearance_m,
        }


@dataclass(frozen=True)
class CandidateOutcome:
    perception_accepted: bool
    trajectory_safe: bool
    minimum_clearance_m: float | None
    duration_s: float | None
    issue: str | None = None

    def __post_init__(self) -> None:
        if self.trajectory_safe and not self.perception_accepted:
            raise ValueError("a safe trajectory requires an accepted perception sample")
        for label, value in (("minimum_clearance_m", self.minimum_clearance_m), ("duration_s", self.duration_s)):
            if value is not None and (not math.isfinite(value) or value < 0.0):
                raise ValueError(f"{label} must be null or a finite non-negative value")
        if self.trajectory_safe and (self.minimum_clearance_m is None or self.duration_s is None):
            raise ValueError("a safe trajectory must report clearance and duration")


@dataclass(frozen=True)
class CandidateSummary:
    name: str
    trial_count: int
    perception_accepted_count: int
    trajectory_success_count: int
    overall_success_rate: float
    execution_success_rate: float
    clearance_p05_m: float | None
    mean_duration_s: float | None
    risk_score: float
    eligible: bool
    failure_counts: dict[str, int]

    def to_dict(self) -> dict[str, object]:
        return {
            "name": self.name,
            "trial_count": self.trial_count,
            "perception_accepted_count": self.perception_accepted_count,
            "trajectory_success_count": self.trajectory_success_count,
            "overall_success_rate": self.overall_success_rate,
            "execution_success_rate": self.execution_success_rate,
            "clearance_p05_m": self.clearance_p05_m,
            "mean_duration_s": self.mean_duration_s,
            "risk_score": self.risk_score,
            "eligible": self.eligible,
            "failure_counts": self.failure_counts,
        }


@dataclass(frozen=True)
class RiskDecision:
    action: str
    selected_candidate: str | None
    rationale: tuple[str, ...]

    def to_dict(self) -> dict[str, object]:
        return {
            "action": self.action,
            "selected_candidate": self.selected_candidate,
            "rationale": list(self.rationale),
        }


def summarize_candidate(
    name: str,
    outcomes: Iterable[CandidateOutcome],
    thresholds: RiskThresholds,
) -> CandidateSummary:
    if not name.strip():
        raise ValueError("candidate name must not be empty")
    samples = list(outcomes)
    if not samples:
        raise ValueError("candidate comparison requires at least one outcome")

    accepted = [sample for sample in samples if sample.perception_accepted]
    successes = [sample for sample in accepted if sample.trajectory_safe]
    # Retain clearance evidence from rejected trajectories too. A sampled
    # scene violation must lower the robust P05 estimate instead of vanishing
    # simply because that rollout was not counted as a completion.
    clearances = [sample.minimum_clearance_m for sample in accepted if sample.minimum_clearance_m is not None]
    durations = [sample.duration_s for sample in successes if sample.duration_s is not None]
    failures = Counter(
        sample.issue or ("perception_recheck" if not sample.perception_accepted else "trajectory_rejected")
        for sample in samples
        if not sample.trajectory_safe
    )
    overall_success = len(successes) / len(samples)
    execution_success = len(successes) / len(accepted) if accepted else 0.0
    clearance_p05 = float(np.quantile(clearances, 0.05)) if clearances else None
    mean_duration = float(np.mean(durations)) if durations else None
    clearance_deficit = (
        thresholds.minimum_clearance_m + 0.10
        if clearance_p05 is None
        else max(0.0, thresholds.minimum_clearance_m - clearance_p05)
    )
    clearance_risk = 1000.0 if clearance_p05 is None else 5.0 / (clearance_p05 + 0.001)
    risk_score = (
        100.0 * (1.0 - overall_success)
        + 1000.0 * clearance_deficit
        + clearance_risk
        + 0.10 * (mean_duration if mean_duration is not None else 30.0)
    )
    perception_coverage = len(accepted) / len(samples)
    eligible = (
        perception_coverage >= thresholds.minimum_perception_coverage
        and overall_success >= thresholds.minimum_success_rate
        and clearance_p05 is not None
        and clearance_p05 >= thresholds.minimum_clearance_m
    )
    return CandidateSummary(
        name=name,
        trial_count=len(samples),
        perception_accepted_count=len(accepted),
        trajectory_success_count=len(successes),
        overall_success_rate=overall_success,
        execution_success_rate=execution_success,
        clearance_p05_m=clearance_p05,
        mean_duration_s=mean_duration,
        risk_score=risk_score,
        eligible=eligible,
        failure_counts=dict(sorted(failures.items())),
    )


def choose_candidate(
    summaries: Iterable[CandidateSummary], thresholds: RiskThresholds
) -> RiskDecision:
    candidates = list(summaries)
    if not candidates:
        raise ValueError("at least one candidate summary is required")
    eligible = sorted(
        (candidate for candidate in candidates if candidate.eligible),
        key=lambda candidate: (candidate.risk_score, -float(candidate.clearance_p05_m or 0.0), candidate.name),
    )
    if eligible:
        selected = eligible[0]
        return RiskDecision(
            action="offline_plan_preview_ready",
            selected_candidate=selected.name,
            rationale=(
                "candidate passes perception coverage, success-rate, and P05 clearance gates",
                f"selected lowest risk score {selected.risk_score:.3f} among {len(eligible)} eligible candidates",
                "result is an offline preview only and does not enable hardware motion",
            ),
        )

    coverage = max(candidate.perception_accepted_count / candidate.trial_count for candidate in candidates)
    if coverage < thresholds.minimum_perception_coverage:
        return RiskDecision(
            action="vision_recheck_required",
            selected_candidate=None,
            rationale=(
                "perception coverage is below the configured threshold",
                "request a new stable detection frame before any planning preview",
            ),
        )
    return RiskDecision(
        action="blocked_by_risk_gate",
        selected_candidate=None,
        rationale=(
            "no candidate satisfies the configured success-rate and P05 clearance gates",
            "retain the hardware motion lock and inspect calibration, TCP, or scene assumptions",
        ),
    )
