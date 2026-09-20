"""Compare grasp candidates under perception and scene uncertainty.

This is an offline diagnostic. It samples the same synthetic observation noise
for every candidate, then gates candidates by perception coverage, completion
rate, and P05 TCP clearance. It never opens CAN, ROS 2 hardware, or a serial
port and cannot enable real motion.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
ROBOT_AI = ROOT / "robot_ai"
if str(ROBOT_AI) not in sys.path:
    sys.path.insert(0, str(ROBOT_AI))

from arm_control import JointLimits, load_default_model
from arm_control.risk_policy import CandidateOutcome, RiskThresholds, choose_candidate, summarize_candidate
from arm_control.scene_review import load_diagnostic_scene
from simulate_six_axis_pick import make_pose, rotation_from_rpy, run_sequence


@dataclass(frozen=True)
class CandidateProfile:
    name: str
    yaw_offset_deg: float
    approach_margin_m: float
    place_x_m: float
    place_y_m: float

    def __post_init__(self) -> None:
        if not self.name.strip():
            raise ValueError("candidate name must not be empty")
        values = {
            "yaw_offset_deg": self.yaw_offset_deg,
            "approach_margin_m": self.approach_margin_m,
            "place_x_m": self.place_x_m,
            "place_y_m": self.place_y_m,
        }
        if not all(math.isfinite(value) for value in values.values()):
            raise ValueError("candidate values must be finite")
        if self.approach_margin_m <= 0.0:
            raise ValueError("approach_margin_m must be positive")

    def to_dict(self) -> dict[str, object]:
        return {
            "name": self.name,
            "yaw_offset_deg": self.yaw_offset_deg,
            "approach_margin_m": self.approach_margin_m,
            "place_xy_m": [self.place_x_m, self.place_y_m],
        }


@dataclass(frozen=True)
class ObservationSample:
    index: int
    confidence: float
    x_m: float
    y_m: float
    yaw_deg: float
    grasp_height_m: float

    def to_dict(self) -> dict[str, object]:
        return {
            "index": self.index,
            "confidence": self.confidence,
            "target_xy_m": [self.x_m, self.y_m],
            "yaw_deg": self.yaw_deg,
            "grasp_height_m": self.grasp_height_m,
        }


def _finite_number(value: object, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{label} must be numeric")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{label} must be finite")
    return number


def load_candidate_profiles(path: Path) -> list[CandidateProfile]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if data.get("schema_version") != 1 or data.get("type") != "offline_risk_aware_grasp_candidates":
        raise ValueError("unsupported risk-aware candidate schema")
    raw_candidates = data.get("candidates")
    if not isinstance(raw_candidates, list) or not raw_candidates:
        raise ValueError("candidate file must contain at least one candidate")
    candidates: list[CandidateProfile] = []
    seen: set[str] = set()
    for entry in raw_candidates:
        if not isinstance(entry, dict):
            raise ValueError("each candidate must be an object")
        name = str(entry.get("name", "")).strip()
        if name in seen:
            raise ValueError(f"duplicate candidate name: {name}")
        place_xy = entry.get("place_xy_m")
        if not isinstance(place_xy, list) or len(place_xy) != 2:
            raise ValueError(f"{name}.place_xy_m must contain exactly two values")
        candidates.append(
            CandidateProfile(
                name=name,
                yaw_offset_deg=_finite_number(entry.get("yaw_offset_deg"), f"{name}.yaw_offset_deg"),
                approach_margin_m=_finite_number(entry.get("approach_margin_m"), f"{name}.approach_margin_m"),
                place_x_m=_finite_number(place_xy[0], f"{name}.place_xy_m[0]"),
                place_y_m=_finite_number(place_xy[1], f"{name}.place_xy_m[1]"),
            )
        )
        seen.add(name)
    return candidates


def sample_observations(args: argparse.Namespace) -> list[ObservationSample]:
    rng = np.random.default_rng(args.seed)
    samples: list[ObservationSample] = []
    for index in range(args.trials):
        samples.append(
            ObservationSample(
                index=index,
                confidence=float(np.clip(rng.normal(args.detection_confidence, args.confidence_sigma), 0.0, 1.0)),
                x_m=float(rng.normal(args.target_x_m, args.xy_sigma_m)),
                y_m=float(rng.normal(args.target_y_m, args.xy_sigma_m)),
                yaw_deg=float(rng.normal(args.yaw_deg, args.yaw_sigma_deg)),
                grasp_height_m=max(0.001, float(rng.normal(args.grasp_height_m, args.height_sigma_m))),
            )
        )
    return samples


def build_poses(model, observation: ObservationSample, profile: CandidateProfile) -> list[tuple[str, np.ndarray]]:
    yaw = math.radians(observation.yaw_deg + profile.yaw_offset_deg)
    rotation = rotation_from_rpy(0.0, 0.0, yaw) @ model.home_grasp_tcp[:3, :3]
    grasp_z = observation.grasp_height_m
    approach_z = grasp_z + profile.approach_margin_m
    return [
        ("approach", make_pose(rotation, (observation.x_m, observation.y_m, approach_z))),
        ("descend", make_pose(rotation, (observation.x_m, observation.y_m, grasp_z))),
        ("lift", make_pose(rotation, (observation.x_m, observation.y_m, approach_z))),
        ("place", make_pose(rotation, (profile.place_x_m, profile.place_y_m, approach_z))),
        ("return_home", model.home_grasp_tcp.copy()),
    ]


def evaluate_candidate(
    profile: CandidateProfile,
    observations: list[ObservationSample],
    *,
    model,
    limits: JointLimits,
    scene,
    minimum_confidence: float,
    max_ik_iterations: int,
    ik_restarts: int,
) -> tuple[list[CandidateOutcome], list[dict[str, object]]]:
    outcomes: list[CandidateOutcome] = []
    details: list[dict[str, object]] = []
    for observation in observations:
        if observation.confidence < minimum_confidence:
            outcome = CandidateOutcome(False, False, None, None, "perception_confidence_below_threshold")
        else:
            segments, _, issues = run_sequence(
                model,
                limits,
                build_poses(model, observation, profile),
                scene=scene,
                max_ik_iterations=max_ik_iterations,
                ik_restarts=ik_restarts,
            )
            clearances: list[float] = []
            for segment in segments:
                if segment.scene_review is None:
                    continue
                for key in ("minimum_table_clearance_m", "minimum_obstacle_clearance_m"):
                    value = segment.scene_review.get(key)
                    if value is not None:
                        clearances.append(float(value))
            duration_s = sum(segment.duration_s for segment in segments)
            safe = not issues and all(segment.ik_converged for segment in segments)
            outcome = CandidateOutcome(
                True,
                safe,
                min(clearances) if clearances else None,
                duration_s if clearances else None,
                None if safe else (issues[0] if issues else "trajectory_rejected"),
            )
        outcomes.append(outcome)
        details.append(
            {
                "observation": observation.to_dict(),
                "perception_accepted": outcome.perception_accepted,
                "trajectory_safe": outcome.trajectory_safe,
                "minimum_clearance_m": outcome.minimum_clearance_m,
                "duration_s": outcome.duration_s,
                "issue": outcome.issue,
            }
        )
    return outcomes, details


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--candidates", type=Path, default=ROOT / "robot_ai" / "arm_control" / "config" / "risk_aware_candidates.json")
    parser.add_argument("--scene", type=Path, default=ROOT / "robot_ai" / "arm_control" / "config" / "offline_diagnostic_scene.json")
    parser.add_argument("--target-x-m", type=float, default=0.2956686)
    parser.add_argument("--target-y-m", type=float, default=-0.1144701)
    parser.add_argument("--grasp-height-m", type=float, default=0.25)
    parser.add_argument("--yaw-deg", type=float, default=0.0)
    parser.add_argument("--trials", type=int, default=12)
    parser.add_argument("--seed", type=int, default=20260814)
    parser.add_argument("--detection-confidence", type=float, default=0.92)
    parser.add_argument("--minimum-confidence", type=float, default=0.78)
    parser.add_argument("--confidence-sigma", type=float, default=0.05)
    parser.add_argument("--xy-sigma-m", type=float, default=0.003)
    parser.add_argument("--yaw-sigma-deg", type=float, default=2.0)
    parser.add_argument("--height-sigma-m", type=float, default=0.005)
    parser.add_argument("--required-success-rate", type=float, default=0.90)
    parser.add_argument("--required-perception-coverage", type=float, default=0.80)
    parser.add_argument("--required-clearance-m", type=float, default=0.015)
    parser.add_argument("--max-ik-iterations", type=int, default=150)
    parser.add_argument("--ik-restarts", type=int, choices=(1, 3, 5), default=3)
    parser.add_argument("--output", type=Path, default=ROOT / "runtime" / "simulations" / "risk_aware_grasp_sweep.json")
    args = parser.parse_args()
    if not 1 <= args.trials <= 100:
        parser.error("--trials must be in 1..100")
    if args.max_ik_iterations < 1:
        parser.error("--max-ik-iterations must be positive")
    finite_inputs = (
        args.target_x_m,
        args.target_y_m,
        args.grasp_height_m,
        args.yaw_deg,
        args.detection_confidence,
        args.minimum_confidence,
        args.confidence_sigma,
        args.xy_sigma_m,
        args.yaw_sigma_deg,
        args.height_sigma_m,
    )
    if not all(math.isfinite(value) for value in finite_inputs):
        parser.error("all numeric simulation inputs must be finite")
    if args.grasp_height_m <= 0.0 or min(args.confidence_sigma, args.xy_sigma_m, args.yaw_sigma_deg, args.height_sigma_m) < 0.0:
        parser.error("grasp height must be positive and standard deviations must be non-negative")
    if not 0.0 <= args.detection_confidence <= 1.0 or not 0.0 <= args.minimum_confidence <= 1.0:
        parser.error("confidence values must be in 0..1")

    candidates = load_candidate_profiles(args.candidates)
    observations = sample_observations(args)
    thresholds = RiskThresholds(
        minimum_success_rate=args.required_success_rate,
        minimum_perception_coverage=args.required_perception_coverage,
        minimum_clearance_m=args.required_clearance_m,
    )
    model = load_default_model()
    scene = load_diagnostic_scene(args.scene)
    limits = JointLimits(
        position_min=np.full(6, -math.pi),
        position_max=np.full(6, math.pi),
        velocity_max=np.full(6, 0.4),
        acceleration_max=np.full(6, 0.8),
    )

    candidate_reports: list[dict[str, object]] = []
    summaries = []
    for candidate in candidates:
        outcomes, details = evaluate_candidate(
            candidate,
            observations,
            model=model,
            limits=limits,
            scene=scene,
            minimum_confidence=args.minimum_confidence,
            max_ik_iterations=args.max_ik_iterations,
            ik_restarts=args.ik_restarts,
        )
        summary = summarize_candidate(candidate.name, outcomes, thresholds)
        summaries.append(summary)
        candidate_reports.append(
            {
                "profile": candidate.to_dict(),
                "summary": summary.to_dict(),
                "trials": details,
            }
        )

    decision = choose_candidate(summaries, thresholds)
    report = {
        "simulation": "offline_risk_aware_grasp_sweep",
        "real_can_opened": False,
        "real_motion_enabled": False,
        "innovation": {
            "name": "uncertainty_aware_risk_constrained_grasp_selection",
            "method": [
                "all candidates use the same sampled perception noise for a fair comparison",
                "each candidate is checked by IK, quintic trajectory limits, and TCP scene clearance",
                "P05 clearance and overall completion rate are hard gates, not only ranking scores",
                "the result is an explainable preview decision: ready, recheck, or blocked",
            ],
        },
        "candidate_source": str(args.candidates),
        "scene_source": str(args.scene),
        "observation_model": {
            "trials": args.trials,
            "seed": args.seed,
            "base_detection_confidence": args.detection_confidence,
            "minimum_confidence": args.minimum_confidence,
            "confidence_sigma": args.confidence_sigma,
            "xy_sigma_m": args.xy_sigma_m,
            "yaw_sigma_deg": args.yaw_sigma_deg,
            "height_sigma_m": args.height_sigma_m,
            "target_source": "diagnostic_cli_override_not_camera_calibration",
            "samples": [sample.to_dict() for sample in observations],
        },
        "thresholds": thresholds.to_dict(),
        "candidates": candidate_reports,
        "decision": decision.to_dict(),
        "limitations": [
            "candidate poses, target coordinates, grasp height, and joint limits are diagnostic assumptions, not measured hardware parameters",
            "TCP clearance review does not replace MoveIt full-link or self-collision checking",
            "the decision never unlocks the real-arm hardware gate",
        ],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    summary = {
        "simulation": report["simulation"],
        "real_can_opened": False,
        "real_motion_enabled": False,
        "thresholds": report["thresholds"],
        "candidate_summaries": [entry["summary"] for entry in candidate_reports],
        "decision": report["decision"],
    }
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
