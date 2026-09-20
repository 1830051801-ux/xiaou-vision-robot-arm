#!/usr/bin/env python3
"""Capture a stationary six-axis teach pose and build an offline smooth replay.

Both subcommands are deliberately non-actuating.  They transmit only
``CMD_GET_JOINT`` while reading a pose, and the planner never opens a UART when
``--start-deg`` is supplied.  The resulting JSON is a preview artifact, not an
F407 trajectory command and not a permission to move the physical arm.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import statistics
import sys
import tempfile
import time
from typing import Any, Iterable


PROJECT_DIR = Path(__file__).resolve().parents[1]
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))

from robot_ai.arm_control.uart_protocol import (  # noqa: E402
    CMD_GET_JOINT,
    DEFAULT_BAUD,
    DEFAULT_PORT,
    JOINT_COUNT,
    RSP_ACK,
    Frame,
    decode_joint_payload,
    exchange,
)


# These are firmware-source envelopes from arm_config.h.  They are only a
# conservative offline-preview boundary until measured limits are recorded in
# hardware_calibration.json; they cannot unlock real motion.
SOURCE_POSITION_MIN_DEG = (-170.0, -130.0, -140.0, -180.0, -90.0, -180.0)
# J5 +120 deg is a user-confirmed, manually reached teach point.  This expands
# only the offline preview envelope; it is not an F407 limit update and cannot
# enable real motion.
SOURCE_POSITION_MAX_DEG = (170.0, 130.0, 140.0, 180.0, 120.0, 180.0)
DEFAULT_PREVIEW_SPEED_DEG_S = (10.0,) * JOINT_COUNT
DEFAULT_PREVIEW_ACCEL_DEG_S2 = (20.0,) * JOINT_COUNT


def _six_floats(value: str) -> list[float]:
    try:
        values = [float(part.strip()) for part in value.split(",")]
    except ValueError as exc:
        raise argparse.ArgumentTypeError("expected six comma-separated numbers") from exc
    if len(values) != JOINT_COUNT or not all(math.isfinite(item) for item in values):
        raise argparse.ArgumentTypeError("expected six finite comma-separated numbers")
    return values


def _positive_int(value: str) -> int:
    parsed = int(value)
    if parsed < 1:
        raise argparse.ArgumentTypeError("value must be at least 1")
    return parsed


def _request_joint(
    joint_id: int, sequence: int, *, port: str, baud: int, timeout_s: float
) -> dict[str, Any]:
    started_s = time.monotonic()
    try:
        response = exchange(
            Frame(CMD_GET_JOINT, sequence, bytes((joint_id,))),
            port=port,
            baud=baud,
            timeout_s=timeout_s,
        )
        if response.cmd != RSP_ACK:
            raise RuntimeError(f"unexpected response command 0x{response.cmd:02X}")
        if response.seq != sequence:
            raise RuntimeError(f"response sequence {response.seq} does not match request {sequence}")
        feedback = decode_joint_payload(response.payload)
        if feedback["joint_id"] != joint_id:
            raise RuntimeError(f"response joint {feedback['joint_id']} does not match request {joint_id}")
    except Exception as exc:
        return {
            "joint_id": joint_id,
            "seq": sequence,
            "ok": False,
            "error": f"{type(exc).__name__}: {exc}",
            "round_trip_ms": round((time.monotonic() - started_s) * 1000.0, 3),
        }
    return {
        "joint_id": joint_id,
        "seq": sequence,
        "ok": True,
        "feedback": feedback,
        "round_trip_ms": round((time.monotonic() - started_s) * 1000.0, 3),
    }


def capture_stationary_pose(
    *,
    port: str,
    baud: int,
    timeout_s: float,
    samples_per_joint: int,
    interval_s: float,
    max_angle_spread_deg: float,
    max_speed_rpm: float,
    min_success_ratio: float,
    min_success_samples: int,
    allow_offline_j6_preview: bool,
    j6_preview_deg: float,
) -> dict[str, Any]:
    """Read a six-axis stationary pose or fail closed without creating a pose."""
    if timeout_s <= 0.0 or interval_s < 0.0:
        raise ValueError("timeout_s must be positive and interval_s must be non-negative")
    if (
        not all(math.isfinite(value) for value in (max_angle_spread_deg, max_speed_rpm, min_success_ratio, j6_preview_deg))
        or max_angle_spread_deg <= 0.0
        or max_speed_rpm < 0.0
    ):
        raise ValueError("stationary thresholds are invalid")
    if not 0.0 < min_success_ratio <= 1.0 or min_success_samples < 1:
        raise ValueError("success thresholds are invalid")
    required_successes = max(min_success_samples, math.ceil(samples_per_joint * min_success_ratio))
    if required_successes > samples_per_joint:
        raise ValueError("success thresholds require more samples than requested")

    sequence = 1
    records: list[dict[str, Any]] = []
    for sample_index in range(1, samples_per_joint + 1):
        for joint_id in range(1, JOINT_COUNT + 1):
            record = _request_joint(joint_id, sequence, port=port, baud=baud, timeout_s=timeout_s)
            record["sample"] = sample_index
            records.append(record)
            sequence = (sequence + 1) & 0xFF
            time.sleep(interval_s)

    per_joint: list[dict[str, Any]] = []
    pose_deg: list[float] = []
    assumed_joint_ids: list[int] = []
    for joint_id in range(1, JOINT_COUNT + 1):
        joint_records = [item for item in records if item["joint_id"] == joint_id]
        failed = [item for item in joint_records if not item["ok"]]
        online = [item for item in joint_records if item["ok"] and item["feedback"]["online"]]
        successful = [item for item in joint_records if item["ok"]]
        if joint_id == JOINT_COUNT and allow_offline_j6_preview and len(online) != samples_per_joint:
            pose_deg.append(j6_preview_deg)
            assumed_joint_ids.append(joint_id)
            per_joint.append(
                {
                    "joint_id": joint_id,
                    "source": "preview_assumption_not_live_feedback",
                    "angle_deg_median": j6_preview_deg,
                    "query_successes": len(successful),
                    "query_failures": len(failed),
                    "online_samples": len(online),
                    "required_successes": required_successes,
                }
            )
            continue
        if len(successful) < required_successes:
            raise RuntimeError(
                f"J{joint_id} has only {len(successful)}/{samples_per_joint} successful queries; "
                f"need at least {required_successes}"
            )
        if len(online) != len(successful):
            raise RuntimeError(f"J{joint_id} reported offline feedback; refusing to teach a live six-axis pose")
        angles = [float(item["feedback"]["angle_deg"]) for item in online]
        speeds = [abs(float(item["feedback"]["speed_rpm"])) for item in online]
        spread = max(angles) - min(angles)
        peak_speed = max(speeds)
        if spread > max_angle_spread_deg:
            raise RuntimeError(f"J{joint_id} moved {spread:.3f} deg during capture; hold the arm stationary")
        if peak_speed > max_speed_rpm:
            raise RuntimeError(f"J{joint_id} reported {peak_speed:.3f} rpm during capture; wait until stationary")
        median = statistics.median(angles)
        pose_deg.append(median)
        per_joint.append(
            {
                "joint_id": joint_id,
                "angle_deg_median": median,
                "angle_spread_deg": spread,
                "peak_abs_speed_rpm": peak_speed,
                "samples": samples_per_joint,
                "query_successes": len(successful),
                "query_failures": len(failed),
                "required_successes": required_successes,
            }
        )
    return {
        "schema": "xiaou_teach_pose_v1",
        "read_only": True,
        "allowed_commands": ["CMD_GET_JOINT"],
        "port": port,
        "baud": baud,
        "captured_at_unix_s": time.time(),
        "pose_deg": pose_deg,
        "per_joint": per_joint,
        "assumed_joint_ids": assumed_joint_ids,
        "all_joints_live_feedback": not assumed_joint_ids,
        "raw_records": records,
        "physical_motion_allowed": False,
        "note": "Captured from stationary feedback only; this file cannot enable motion.",
    }


def _atomic_json_write(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile("w", encoding="utf-8", dir=path.parent, delete=False) as handle:
        json.dump(payload, handle, ensure_ascii=False, indent=2)
        handle.write("\n")
        temporary = Path(handle.name)
    os.replace(temporary, path)


def _load_teach_pose(path: Path) -> list[float]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if payload.get("schema") != "xiaou_teach_pose_v1" or payload.get("read_only") is not True:
        raise ValueError("teach pose is not a trusted read-only capture")
    pose = payload.get("pose_deg")
    if not isinstance(pose, list) or len(pose) != JOINT_COUNT:
        raise ValueError("teach pose must contain six joint angles")
    values = [float(item) for item in pose]
    if not all(math.isfinite(item) for item in values):
        raise ValueError("teach pose contains a non-finite angle")
    return values


def _forward_kinematics_summary(angles_deg: Iterable[float]) -> dict[str, Any]:
    """Return model TCP coordinates for preview only; never use it as calibration."""
    values = [float(value) for value in angles_deg]
    try:
        import numpy as np

        from robot_ai.arm_control.kinematics import fk_space
        from robot_ai.arm_control.model import load_default_model

        model = load_default_model()
        transform = fk_space(
            model.home_grasp_tcp,
            model.screw_axes,
            np.radians(np.asarray(values, dtype=np.float64)),
        )
        return {
            "available": True,
            "frame": "model_base_link",
            "tcp_position_m": [round(float(value), 6) for value in transform[:3, 3]],
            "tcp_rotation_matrix": [
                [round(float(value), 6) for value in row]
                for row in transform[:3, :3]
            ],
            "warning": "model-derived preview only; physical camera and TCP calibration are not yet verified",
        }
    except Exception as exc:
        return {
            "available": False,
            "error": f"{type(exc).__name__}: {exc}",
            "warning": "trajectory preview remains available; model TCP coordinate was not produced",
        }


def build_quintic_preview(
    start_deg: Iterable[float],
    goal_deg: Iterable[float],
    *,
    position_min_deg: Iterable[float] = SOURCE_POSITION_MIN_DEG,
    position_max_deg: Iterable[float] = SOURCE_POSITION_MAX_DEG,
    speed_limit_deg_s: Iterable[float] = DEFAULT_PREVIEW_SPEED_DEG_S,
    accel_limit_deg_s2: Iterable[float] = DEFAULT_PREVIEW_ACCEL_DEG_S2,
    sample_period_s: float = 0.05,
    minimum_duration_s: float = 1.0,
) -> dict[str, Any]:
    """Build a bounded joint-space quintic preview without transport access."""
    arrays = [list(map(float, values)) for values in (start_deg, goal_deg, position_min_deg, position_max_deg, speed_limit_deg_s, accel_limit_deg_s2)]
    if any(len(values) != JOINT_COUNT for values in arrays) or not all(
        math.isfinite(value) for values in arrays for value in values
    ):
        raise ValueError("all preview arrays must contain six finite values")
    start, goal, lower, upper, speed_limit, accel_limit = arrays
    if sample_period_s <= 0.0 or minimum_duration_s <= 0.0:
        raise ValueError("sample period and minimum duration must be positive")
    if any(low >= high for low, high in zip(lower, upper)):
        raise ValueError("preview position limits are invalid")
    if any(value <= 0.0 for value in speed_limit + accel_limit):
        raise ValueError("preview speed and acceleration limits must be positive")
    for label, values in (("start", start), ("goal", goal)):
        if any(value < low or value > high for value, low, high in zip(values, lower, upper)):
            raise ValueError(f"{label} pose exceeds the source preview envelope")

    deltas = [goal_value - start_value for start_value, goal_value in zip(start, goal)]
    velocity_duration = max(1.875 * abs(delta) / limit for delta, limit in zip(deltas, speed_limit))
    acceleration_duration = max(math.sqrt((10.0 / math.sqrt(3.0)) * abs(delta) / limit) for delta, limit in zip(deltas, accel_limit))
    duration_s = max(velocity_duration, acceleration_duration, minimum_duration_s)
    count = max(2, int(math.ceil(duration_s / sample_period_s)) + 1)
    points: list[dict[str, Any]] = []
    for index in range(count):
        time_s = duration_s * index / (count - 1)
        tau = time_s / duration_s
        scale = 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5
        scale_velocity = (30.0 * tau**2 - 60.0 * tau**3 + 30.0 * tau**4) / duration_s
        scale_acceleration = (60.0 * tau - 180.0 * tau**2 + 120.0 * tau**3) / (duration_s * duration_s)
        points.append(
            {
                "time_from_start_s": round(time_s, 6),
                "positions_deg": [round(start_value + scale * delta, 6) for start_value, delta in zip(start, deltas)],
                "velocities_deg_s": [round(scale_velocity * delta, 6) for delta in deltas],
                "accelerations_deg_s2": [round(scale_acceleration * delta, 6) for delta in deltas],
            }
        )
    return {
        "schema": "xiaou_teach_grasp_plan_v1",
        "execution": "offline_preview_only",
        "motion_commands_emitted": False,
        "collision_status": "unverified_no_calibrated_environment_model",
        "limit_source": "F407 arm_config.h source envelope; unmeasured preview only",
        "start_deg": start,
        "goal_deg": goal,
        "forward_kinematics_preview": {
            "start": _forward_kinematics_summary(start),
            "goal": _forward_kinematics_summary(goal),
        },
        "duration_s": duration_s,
        "sample_period_s": sample_period_s,
        "position_min_deg": lower,
        "position_max_deg": upper,
        "speed_limit_deg_s": speed_limit,
        "accel_limit_deg_s2": accel_limit,
        "points": points,
        "real_motion_blockers": [
            "J1-J6 live feedback and calibration must be verified",
            "physical emergency stop and measured joint limits must be verified",
            "collision-free physical environment must be verified",
            "this tool intentionally has no command-transmit implementation",
        ],
    }


def _capture_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=_positive_int, default=DEFAULT_BAUD)
    parser.add_argument("--timeout-s", type=float, default=0.3)
    parser.add_argument("--samples-per-joint", type=_positive_int, default=20)
    parser.add_argument("--interval-s", type=float, default=0.12)
    parser.add_argument("--max-angle-spread-deg", type=float, default=0.25)
    parser.add_argument("--max-speed-rpm", type=float, default=2.0)
    parser.add_argument("--min-success-ratio", type=float, default=0.85)
    parser.add_argument("--min-success-samples", type=_positive_int, default=6)
    parser.add_argument(
        "--allow-offline-j6-preview",
        action="store_true",
        help="use an explicit fixed J6 angle for offline preview only; never enables real motion",
    )
    parser.add_argument("--j6-preview-deg", type=float, default=0.0)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    capture_parser = subparsers.add_parser("capture", help="read and save a stationary six-axis teach pose")
    _capture_args(capture_parser)
    capture_parser.add_argument("--output", type=Path, default=Path("runtime/teach_poses/grasp_pose.json"))

    plan_parser = subparsers.add_parser("plan", help="build an offline smooth joint-space preview")
    _capture_args(plan_parser)
    plan_parser.add_argument("--teach-pose", type=Path, required=True)
    plan_parser.add_argument("--start-deg", type=_six_floats, help="optional offline start pose; otherwise read current joints")
    plan_parser.add_argument("--plan-output", type=Path, default=Path("runtime/teach_poses/grasp_plan_preview.json"))
    plan_parser.add_argument("--speed-limit-deg-s", type=_six_floats, default=list(DEFAULT_PREVIEW_SPEED_DEG_S))
    plan_parser.add_argument("--accel-limit-deg-s2", type=_six_floats, default=list(DEFAULT_PREVIEW_ACCEL_DEG_S2))
    plan_parser.add_argument("--sample-period-s", type=float, default=0.05)
    plan_parser.add_argument("--minimum-duration-s", type=float, default=1.0)
    args = parser.parse_args()

    try:
        if args.command == "capture":
            capture = capture_stationary_pose(
                port=args.port,
                baud=args.baud,
                timeout_s=args.timeout_s,
                samples_per_joint=args.samples_per_joint,
                interval_s=args.interval_s,
                max_angle_spread_deg=args.max_angle_spread_deg,
                max_speed_rpm=args.max_speed_rpm,
                min_success_ratio=args.min_success_ratio,
                min_success_samples=args.min_success_samples,
                allow_offline_j6_preview=args.allow_offline_j6_preview,
                j6_preview_deg=args.j6_preview_deg,
            )
            _atomic_json_write(args.output, capture)
            print(json.dumps({"captured": True, "output": str(args.output), "pose_deg": capture["pose_deg"], "read_only": True}, ensure_ascii=False, indent=2))
            return 0

        goal_deg = _load_teach_pose(args.teach_pose)
        if args.start_deg is None:
            current = capture_stationary_pose(
                port=args.port,
                baud=args.baud,
                timeout_s=args.timeout_s,
                samples_per_joint=args.samples_per_joint,
                interval_s=args.interval_s,
                max_angle_spread_deg=args.max_angle_spread_deg,
                max_speed_rpm=args.max_speed_rpm,
                min_success_ratio=args.min_success_ratio,
                min_success_samples=args.min_success_samples,
                allow_offline_j6_preview=args.allow_offline_j6_preview,
                j6_preview_deg=args.j6_preview_deg,
            )
            start_deg = current["pose_deg"]
        else:
            start_deg = args.start_deg
        plan = build_quintic_preview(
            start_deg,
            goal_deg,
            speed_limit_deg_s=args.speed_limit_deg_s,
            accel_limit_deg_s2=args.accel_limit_deg_s2,
            sample_period_s=args.sample_period_s,
            minimum_duration_s=args.minimum_duration_s,
        )
        _atomic_json_write(args.plan_output, plan)
        print(json.dumps({"planned": True, "output": str(args.plan_output), "duration_s": plan["duration_s"], "points": len(plan["points"]), "motion_commands_emitted": False}, ensure_ascii=False, indent=2))
        return 0
    except Exception as exc:
        print(f"teach workflow blocked: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
