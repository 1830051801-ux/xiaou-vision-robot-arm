#!/usr/bin/env python3
"""Execute one supervised ready-to-teach joint trajectory through the F407.

The default mode is offline planning and never opens a UART. Real execution
requires ``--execute --confirm MOVE-TO-TEACH`` and a hardware configuration
that passes every existing Pi-side motion gate. The F407 remains the final
authority for E-stop, calibration, online feedback, effective joint limits,
FIFO admission, and trajectory interpolation.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import statistics
import sys
import tempfile
import time
from typing import Any, Iterable, Sequence


PROJECT_DIR = Path(__file__).resolve().parents[1]
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))

from robot_ai.arm_control.safety import (  # noqa: E402
    DEFAULT_HARDWARE_CONFIG,
    MotionLockedError,
    load_hardware_config,
    require_motion_ready,
)
from robot_ai.arm_control import build_grasp_family_preview, load_default_model  # noqa: E402
from robot_ai.arm_control.uart_protocol import (  # noqa: E402
    CMD_GET_JOINT,
    CMD_STOP,
    CMD_TRAJ_POINT,
    CMD_TRAJ_BUFFER_CLEAR,
    DEFAULT_BAUD,
    DEFAULT_PORT,
    JOINT_COUNT,
    RSP_ACK,
    RSP_TRAJ_ACK,
    Frame,
    ProtocolError,
    decode_joint_payload,
    exchange,
    pack_trajectory_payload,
)


CONFIRM_TOKEN = "MOVE-TO-TEACH"
TAUGHT_SIDE_ROUTE_CONFIRM_TOKEN = "MOVE-ALONG-TAUGHT-SIDE-ROUTE"
MOTION_PROFILES = ("taught_side_colalocal", "direct_joint")
MAX_BATCH_POINTS = 9
F407_FIFO_CAPACITY = 16
POSITION_EPS_DEG = 0.02
QUINTIC_PEAK_VELOCITY_FACTOR = 1.875
QUINTIC_PEAK_ACCELERATION_FACTOR = 10.0 / math.sqrt(3.0)
SETTLED_MAX_ABS_SPEED_RPM = 2.0


class TeachExecutionError(RuntimeError):
    """A preflight, transport, or completion condition blocked execution."""

    def __init__(self, message: str, *, next_sequence: int | None = None) -> None:
        super().__init__(message)
        self.next_sequence = next_sequence


def _six_floats(value: str) -> list[float]:
    try:
        values = [float(part.strip()) for part in value.split(",")]
    except ValueError as exc:
        raise argparse.ArgumentTypeError("expected six comma-separated numbers") from exc
    if len(values) != JOINT_COUNT or not all(math.isfinite(item) for item in values):
        raise argparse.ArgumentTypeError("expected six finite comma-separated numbers")
    return values


def _three_floats(value: str) -> list[float]:
    try:
        values = [float(part.strip()) for part in value.split(",")]
    except ValueError as exc:
        raise argparse.ArgumentTypeError("expected three comma-separated numbers") from exc
    if len(values) != 3 or not all(math.isfinite(item) for item in values):
        raise argparse.ArgumentTypeError("expected three finite comma-separated numbers")
    return values


def _positive_int(value: str) -> int:
    parsed = int(value)
    if parsed < 1:
        raise argparse.ArgumentTypeError("value must be at least 1")
    return parsed


def _atomic_json_write(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile("w", encoding="utf-8", dir=path.parent, delete=False) as handle:
        json.dump(payload, handle, ensure_ascii=False, indent=2)
        handle.write("\n")
        temporary = Path(handle.name)
    temporary.replace(path)


def load_teach_pose(path: Path) -> tuple[list[float], list[int]]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if payload.get("schema") != "xiaou_teach_pose_v1" or payload.get("read_only") is not True:
        raise TeachExecutionError("teach pose is not a trusted stationary capture")
    pose = payload.get("pose_deg")
    if not isinstance(pose, list) or len(pose) != JOINT_COUNT:
        raise TeachExecutionError("teach pose must contain six joint angles")
    values = [float(item) for item in pose]
    if not all(math.isfinite(item) for item in values):
        raise TeachExecutionError("teach pose contains a non-finite angle")
    assumed = payload.get("assumed_joint_ids", [])
    if not isinstance(assumed, list) or any(
        isinstance(item, bool) or not isinstance(item, int) or not 1 <= item <= JOINT_COUNT
        for item in assumed
    ):
        raise TeachExecutionError("teach pose has invalid assumed_joint_ids provenance")
    return values, sorted(set(assumed))


def limits_deg_from_config(config: dict[str, Any]) -> tuple[list[float], list[float]]:
    lower = config.get("position_min_rad")
    upper = config.get("position_max_rad")
    if not isinstance(lower, list) or not isinstance(upper, list) or len(lower) != JOINT_COUNT or len(upper) != JOINT_COUNT:
        raise TeachExecutionError("hardware configuration does not contain six position limits")
    if any(value is None for value in lower + upper):
        raise TeachExecutionError("hardware position limits are incomplete")
    lower_deg = [math.degrees(float(value)) for value in lower]
    upper_deg = [math.degrees(float(value)) for value in upper]
    if not all(math.isfinite(value) for value in lower_deg + upper_deg) or any(
        low >= high for low, high in zip(lower_deg, upper_deg)
    ):
        raise TeachExecutionError("hardware position limits are invalid")
    return lower_deg, upper_deg


def validate_pose_limits(
    label: str,
    pose_deg: Sequence[float],
    lower_deg: Sequence[float],
    upper_deg: Sequence[float],
) -> None:
    if len(pose_deg) != JOINT_COUNT:
        raise TeachExecutionError(f"{label} pose must contain six angles")
    for joint_id, (value, lower, upper) in enumerate(zip(pose_deg, lower_deg, upper_deg), start=1):
        if not math.isfinite(float(value)) or not lower <= float(value) <= upper:
            raise TeachExecutionError(
                f"{label} J{joint_id}={float(value):.3f} deg is outside "
                f"[{lower:.3f}, {upper:.3f}] deg"
            )


def build_segmented_quintic_plan(
    start_deg: Sequence[float],
    goal_deg: Sequence[float],
    *,
    speed_limit_deg_s: Sequence[float],
    accel_limit_deg_s2: Sequence[float],
    max_point_duration_ms: int,
    minimum_duration_s: float = 1.0,
) -> dict[str, Any]:
    arrays = [list(map(float, values)) for values in (start_deg, goal_deg, speed_limit_deg_s, accel_limit_deg_s2)]
    if any(len(values) != JOINT_COUNT for values in arrays) or not all(
        math.isfinite(value) for values in arrays for value in values
    ):
        raise TeachExecutionError("start, goal, speed, and acceleration must contain six finite values")
    start, goal, speed_limit, accel_limit = arrays
    if any(value <= 0.0 for value in speed_limit + accel_limit):
        raise TeachExecutionError("speed and acceleration limits must be positive")
    if not 100 <= max_point_duration_ms <= 10000:
        raise TeachExecutionError("max point duration must be within 100..10000 ms")
    if not math.isfinite(minimum_duration_s) or minimum_duration_s <= 0.0:
        raise TeachExecutionError("minimum duration must be finite and positive")

    deltas = [goal_value - start_value for start_value, goal_value in zip(start, goal)]
    velocity_duration_s = max(
        QUINTIC_PEAK_VELOCITY_FACTOR * abs(delta) / limit
        for delta, limit in zip(deltas, speed_limit)
    )
    duration_s = max(velocity_duration_s, minimum_duration_s)
    previous_segment_count = 0
    for _ in range(16):
        segment_count = max(1, math.ceil(duration_s * 1000.0 / max_point_duration_ms))
        acceleration_duration_s = max(
            math.sqrt(
                QUINTIC_PEAK_ACCELERATION_FACTOR * abs(delta) * segment_count / limit
            )
            for delta, limit in zip(deltas, accel_limit)
        )
        revised_duration_s = max(velocity_duration_s, acceleration_duration_s, minimum_duration_s)
        if segment_count == previous_segment_count and abs(revised_duration_s - duration_s) < 1e-9:
            duration_s = revised_duration_s
            break
        previous_segment_count = segment_count
        duration_s = revised_duration_s
    else:
        raise TeachExecutionError("segmented duration calculation did not converge")

    segment_count = max(1, math.ceil(duration_s * 1000.0 / max_point_duration_ms))
    segment_duration_ms = math.ceil(duration_s * 1000.0 / segment_count)
    if segment_duration_ms > max_point_duration_ms:
        segment_count += 1
        segment_duration_ms = math.ceil(duration_s * 1000.0 / segment_count)
    actual_duration_s = segment_count * segment_duration_ms / 1000.0
    points: list[dict[str, Any]] = []
    for index in range(1, segment_count + 1):
        ratio = index / segment_count
        positions = [
            goal_value if index == segment_count else start_value + ratio * delta
            for start_value, goal_value, delta in zip(start, goal, deltas)
        ]
        points.append(
            {
                "index": index,
                "positions_deg": positions,
                "duration_ms": segment_duration_ms,
            }
        )

    peak_speed = [
        QUINTIC_PEAK_VELOCITY_FACTOR * abs(delta) / actual_duration_s
        for delta in deltas
    ]
    peak_accel = [
        QUINTIC_PEAK_ACCELERATION_FACTOR
        * abs(delta / segment_count)
        / ((segment_duration_ms / 1000.0) ** 2)
        for delta in deltas
    ]
    if any(value > limit + 1e-9 for value, limit in zip(peak_speed, speed_limit)):
        raise TeachExecutionError("internal error: segmented plan exceeds a speed limit")
    if any(value > limit + 1e-9 for value, limit in zip(peak_accel, accel_limit)):
        raise TeachExecutionError("internal error: segmented plan exceeds an acceleration limit")

    return {
        "schema": "xiaou_teach_execute_plan_v1",
        "trajectory_semantics": "piecewise_f407_quintic_from_feedback",
        "start_deg": start,
        "goal_deg": goal,
        "speed_limit_deg_s": speed_limit,
        "accel_limit_deg_s2": accel_limit,
        "max_point_duration_ms": max_point_duration_ms,
        "segment_duration_ms": segment_duration_ms,
        "segment_count": segment_count,
        "duration_s": actual_duration_s,
        "peak_speed_deg_s": peak_speed,
        "peak_accel_deg_s2": peak_accel,
        "points": points,
    }


def build_taught_side_route_plan(
    start_deg: Sequence[float],
    taught_deg: Sequence[float],
    *,
    lower_deg: Sequence[float],
    upper_deg: Sequence[float],
    speed_limit_deg_s: Sequence[float],
    accel_limit_deg_s2: Sequence[float],
    max_point_duration_ms: int,
    object_class: str = "cola",
    object_radius_m: float | None = None,
    object_height_m: float | None = None,
    target_offset_base_m: Sequence[float] = (0.0, 0.0, 0.0),
) -> dict[str, Any]:
    """Build a local taught-cylinder route as compact F407 stage plans.

    The grasp-family registry permits reuse only for a measured cylinder
    inside the deliberately small demonstrated envelope.  Cups, pens, tissue,
    unknown objects, and a bottle outside that envelope need their own
    multi-pose teach record.  This function therefore cannot silently turn a
    single cola endpoint into a generic-object trajectory.
    Each Cartesian stage gets its own bounded F407 plan so a long route never
    requires filling the controller FIFO with a speculative tail.
    """

    start = [float(value) for value in start_deg]
    taught = [float(value) for value in taught_deg]
    validate_pose_limits("route start", start, lower_deg, upper_deg)
    validate_pose_limits("route taught contact", taught, lower_deg, upper_deg)
    preview = build_grasp_family_preview(
        object_class,
        start,
        taught,
        model=load_default_model(),
        lower_deg=lower_deg,
        upper_deg=upper_deg,
        object_radius_m=object_radius_m,
        object_height_m=object_height_m,
        target_offset_base_m=target_offset_base_m,
    )
    if not preview.preview_ready or preview.plan is None:
        raise TeachExecutionError(
            "taught side route is unavailable: " + preview.reason
        )

    current = start
    route_stages: list[dict[str, Any]] = []
    total_duration_s = 0.0
    total_point_count = 0
    max_j6_travel_deg = 0.0
    for route_index, stage in enumerate(preview.plan.stages, start=1):
        goal = [math.degrees(float(value)) for value in stage.target_joint_rad]
        validate_pose_limits(stage.name, goal, lower_deg, upper_deg)
        stage_plan = build_segmented_quintic_plan(
            current,
            goal,
            speed_limit_deg_s=speed_limit_deg_s,
            accel_limit_deg_s2=accel_limit_deg_s2,
            max_point_duration_ms=max_point_duration_ms,
        )
        # A stage may contain several compact points, but each stage must fit
        # independently because execution clears and settles before the next
        # Cartesian waypoint is admitted.
        partition_batches(stage_plan["points"])
        route_stages.append({
            "route_index": route_index,
            "name": stage.name,
            "phase": stage.phase,
            "target_joint_deg": goal,
            "target_tcp_xyz_m": [float(value) for value in stage.target_pose[:3, 3]],
            "ik_position_error_m": float(stage.ik_position_error_m),
            "ik_orientation_error_rad": float(stage.ik_orientation_error_rad),
            "execution_plan": stage_plan,
        })
        total_duration_s += float(stage_plan["duration_s"])
        total_point_count += int(stage_plan["segment_count"])
        max_j6_travel_deg = max(max_j6_travel_deg, abs(goal[5] - start[5]))
        current = goal

    return {
        "schema": "xiaou_taught_side_route_execute_plan_v1",
        "motion_profile": "taught_side_colalocal",
        "trajectory_semantics": "raise_transfer_descend_side_approach_then_taught_contact",
        "object_class": preview.object_class,
        "hardware_motion": False,
        "start_deg": start,
        "goal_deg": taught,
        "route_stage_count": len(route_stages),
        "total_compact_point_count": total_point_count,
        "total_duration_s": total_duration_s,
        "max_j6_travel_deg": max_j6_travel_deg,
        "requires_j6_motion": max_j6_travel_deg > 0.25,
        "grasp_family_preview": preview.as_dict(),
        "stages": route_stages,
        "limitations": [
            "This is the current local cola-like demonstration only, not a camera-calibrated arbitrary-object route.",
            "The gripper close/open command and grip force remain outside this arm-only route.",
            "MuJoCo replay records approximate CAD contact; it does not prove physical collision clearance.",
        ],
    }


def partition_batches(points: Sequence[dict[str, Any]]) -> list[list[dict[str, Any]]]:
    if not points:
        raise TeachExecutionError("trajectory contains no points")
    if len(points) > F407_FIFO_CAPACITY - 1:
        raise TeachExecutionError(
            f"trajectory has {len(points)} points; supervised executor permits at most "
            f"{F407_FIFO_CAPACITY - 1} so the complete plan fits the cleared F407 FIFO"
        )
    return [list(points[index : index + MAX_BATCH_POINTS]) for index in range(0, len(points), MAX_BATCH_POINTS)]


def validate_joint_snapshot(
    joints: Sequence[dict[str, Any]],
    *,
    require_idle: bool,
    max_abs_speed_rpm: float,
) -> list[float]:
    """Validate one fresh six-axis snapshot assembled from CMD_GET_JOINT.

    The deployed F407 currently answers the compact 14-byte joint replies
    reliably, while its aggregate state reply is unavailable.  The F407 is
    still responsible for E-stop, zero-valid, limit, and fault interlocks at
    trajectory admission and every interpolation tick.
    """

    if len(joints) != JOINT_COUNT:
        raise TeachExecutionError("joint snapshot does not contain six joints")
    angles: list[float] = []
    for expected_joint_id, joint in enumerate(joints, start=1):
        if joint.get("joint_id") != expected_joint_id or not bool(joint.get("online")):
            raise TeachExecutionError(f"J{expected_joint_id} is missing or offline")
        angle = float(joint.get("angle_deg"))
        speed = abs(float(joint.get("speed_rpm")))
        if not math.isfinite(angle) or not math.isfinite(speed):
            raise TeachExecutionError(f"J{expected_joint_id} feedback is non-finite")
        if require_idle and speed > max_abs_speed_rpm:
            raise TeachExecutionError(
                f"J{expected_joint_id} speed {speed:.3f} rpm exceeds stationary preflight "
                f"limit {max_abs_speed_rpm:.3f} rpm"
            )
        angles.append(angle)
    return angles


def _read_joint_snapshot(
    sequence: int,
    *,
    port: str,
    baud: int,
    timeout_s: float,
    attempts: int,
) -> tuple[list[dict[str, Any]], int]:
    """Read J1..J6 individually, retrying only the joint that missed a reply."""

    current_sequence = sequence
    snapshot: list[dict[str, Any]] = []
    for joint_id in range(1, JOINT_COUNT + 1):
        errors: list[str] = []
        for attempt in range(1, attempts + 1):
            request_sequence, current_sequence = _consume_sequence(current_sequence)
            try:
                response = exchange(
                    Frame(CMD_GET_JOINT, request_sequence, bytes((joint_id,))),
                    port=port,
                    baud=baud,
                    timeout_s=timeout_s,
                )
                if response.cmd != RSP_ACK or response.seq != request_sequence:
                    raise TeachExecutionError(
                        f"GET_JOINT J{joint_id} returned cmd=0x{response.cmd:02X}, seq={response.seq}"
                    )
                feedback = decode_joint_payload(response.payload)
                if feedback["joint_id"] != joint_id:
                    raise TeachExecutionError(
                        f"GET_JOINT J{joint_id} returned mismatched J{feedback['joint_id']} feedback"
                    )
                snapshot.append(feedback)
                break
            except Exception as exc:
                errors.append(f"attempt {attempt}: {type(exc).__name__}: {exc}")
        else:
            raise TeachExecutionError(
                f"GET_JOINT J{joint_id} failed: " + " | ".join(errors),
                next_sequence=current_sequence,
            )
    return snapshot, current_sequence


def build_live_authorized_config(
    base_config: dict[str, Any],
    *,
    explicit_session_authorized: bool,
    live_uart_verified: bool,
    live_feedback_verified: bool,
    operator_firmware_verified: bool,
    operator_estop_verified: bool,
    operator_mcu_calibration_verified: bool,
) -> dict[str, Any]:
    """Create an execution-only gate after a complete no-motion preflight.

    A live CMD_GET_JOINT snapshot can establish link and six-axis feedback for
    this supervised session. Firmware identity, physical E-stop validation,
    and F407 zero calibration remain persistent, operator-owned attestations
    and are deliberately never overridden.
    """

    if explicit_session_authorized is not True:
        raise TeachExecutionError("temporary motion authorization requires explicit operator confirmation")
    if live_uart_verified is not True or live_feedback_verified is not True:
        raise TeachExecutionError("temporary motion authorization requires completed live UART and feedback preflight")
    config = json.loads(json.dumps(base_config))
    session_attestations = {
        "f407_firmware_verified": operator_firmware_verified,
        "estop_verified": operator_estop_verified,
        "mcu_calibration_verified": operator_mcu_calibration_verified,
    }
    for field, operator_confirmed in session_attestations.items():
        if config.get(field) is not True and operator_confirmed is True:
            config[field] = True
    if operator_mcu_calibration_verified:
        config["mcu_calibration"] = {
            "authority": "stm32_f407",
            "verification_method": "operator_confirmed_f407_zeros",
            "verified_unix_s": time.time(),
            "zero_valid": [True] * JOINT_COUNT,
        }
    static_attestations = (
        "protocol_confirmed",
        "f407_firmware_verified",
        "estop_verified",
        "mcu_calibration_verified",
    )
    missing_static = [field for field in static_attestations if config.get(field) is not True]
    if missing_static:
        raise TeachExecutionError(
            "execution requires persistent or explicit per-run confirmation: "
            + ", ".join(missing_static)
        )
    # These fields are established by this session only. The config is
    # written to a temporary file for uart_protocol's existing gate and is
    # deleted after execute_plan returns; the supplied candidate JSON stays
    # locked on disk.
    config["motion_enabled"] = True
    config["uart_link_verified"] = True
    config["feedback_verified"] = True
    config["supervised_session_authorization"] = {
        "authority": "explicit_execute_confirm_plus_live_get_joint_preflight",
        "verified_unix_s": time.time(),
        "derived_fields": [
            "motion_enabled",
            "uart_link_verified",
            "feedback_verified",
        ],
        "operator_confirmed_fields": [
            field for field, confirmed in session_attestations.items() if confirmed
        ],
        "must_already_be_confirmed": ["protocol_confirmed"],
    }
    require_motion_ready(config)
    return config


def write_transient_hardware_config(config: dict[str, Any], directory: Path) -> Path:
    directory.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        "w",
        encoding="utf-8",
        dir=directory,
        prefix="live_motion_gate_",
        suffix=".json",
        delete=False,
    ) as handle:
        json.dump(config, handle, ensure_ascii=False, indent=2)
        handle.write("\n")
        return Path(handle.name)


def refresh_session_calibration_heartbeat(path: Path) -> None:
    """Refresh the short-lived Pi session gate before one admitted point.

    This does not alter the candidate calibration file or F407 calibration.
    The live F407 remains responsible for zero-valid checks at trajectory
    admission and every interpolation tick. The timestamp represents the
    already-completed, explicitly authorized Pi session rather than a new
    calibration operation.
    """

    config = json.loads(path.read_text(encoding="utf-8"))
    calibration = config.get("mcu_calibration")
    if (
        not isinstance(calibration, dict)
        or calibration.get("authority") != "stm32_f407"
        or calibration.get("verification_method") != "operator_confirmed_f407_zeros"
        or calibration.get("zero_valid") != [True] * JOINT_COUNT
    ):
        raise TeachExecutionError("temporary session calibration authorization is invalid")
    now = time.time()
    calibration["verified_unix_s"] = now
    session = config.get("supervised_session_authorization")
    if isinstance(session, dict):
        session["verified_unix_s"] = now
    _atomic_json_write(path, config)
    require_motion_ready(config)


def capture_preflight_start(
    *,
    port: str,
    baud: int,
    timeout_s: float,
    read_attempts: int,
    sample_count: int,
    max_abs_speed_rpm: float,
    max_angle_spread_deg: float,
    sequence: int = 1,
) -> tuple[list[float], int, dict[str, Any]]:
    samples: list[list[float]] = []
    last_snapshot: list[dict[str, Any]] | None = None
    for _ in range(sample_count):
        snapshot, sequence = _read_joint_snapshot(
            sequence,
            port=port,
            baud=baud,
            timeout_s=timeout_s,
            attempts=read_attempts,
        )
        samples.append(
            validate_joint_snapshot(
                snapshot,
                require_idle=True,
                max_abs_speed_rpm=max_abs_speed_rpm,
            )
        )
        last_snapshot = snapshot
        time.sleep(0.1)
    start: list[float] = []
    for joint_index in range(JOINT_COUNT):
        values = [sample[joint_index] for sample in samples]
        spread = max(values) - min(values)
        if spread > max_angle_spread_deg:
            raise TeachExecutionError(
                f"J{joint_index + 1} changed {spread:.3f} deg during preflight"
            )
        start.append(statistics.median(values))
    assert last_snapshot is not None
    return start, sequence, last_snapshot


def _best_effort_stop(sequence: int, *, port: str, baud: int, timeout_s: float) -> tuple[int, str]:
    """Attempt STOP with a sequence never used by the failed request.

    A timed-out motion admission is ambiguous: the F407 may have received the
    command even though the Pi did not receive its response.  Consume the
    outgoing sequence before each request and use the next sequence for STOP
    so an old trajectory ACK cannot be accepted as the STOP response.
    """

    stop_sequence = sequence
    next_sequence = (stop_sequence + 1) & 0xFF
    try:
        response = exchange(
            Frame(CMD_STOP, stop_sequence),
            port=port,
            baud=baud,
            timeout_s=timeout_s,
        )
        if response.cmd == RSP_ACK and response.seq == stop_sequence and not response.payload:
            return next_sequence, "STOP acknowledged"
        return (
            next_sequence,
            "STOP attempted but response was not a matching empty ACK "
            f"(cmd=0x{response.cmd:02X}, seq={response.seq}, payload={response.payload.hex()})",
        )
    except Exception as exc:
        return next_sequence, f"STOP attempted but unconfirmed: {type(exc).__name__}: {exc}"


def _consume_sequence(sequence: int) -> tuple[int, int]:
    """Return (request_sequence, next_unused_sequence)."""

    return sequence, (sequence + 1) & 0xFF


def _stop_after_uncertain_execution(
    reason: str,
    sequence: int,
    *,
    port: str,
    baud: int,
    timeout_s: float,
) -> TeachExecutionError:
    """Build an error that accurately reports whether STOP was acknowledged."""

    _, stop_status = _best_effort_stop(sequence, port=port, baud=baud, timeout_s=timeout_s)
    return TeachExecutionError(f"{reason}; {stop_status}")


def execute_plan(
    plan: dict[str, Any],
    *,
    hardware_config_path: Path,
    port: str,
    baud: int,
    timeout_s: float,
    poll_interval_s: float,
    completion_margin_s: float,
    completion_tolerance_deg: float,
    sequence: int,
    active_feedback_timeout_s: float | None = None,
    active_feedback_attempts: int = 3,
    segment_quiet_settle_s: float = 1.5,
    segment_max_abs_speed_rpm: float = SETTLED_MAX_ABS_SPEED_RPM,
    segment_settled_samples: int = 2,
) -> dict[str, Any]:
    if active_feedback_timeout_s is None:
        active_feedback_timeout_s = min(timeout_s, 0.5)
    if (
        not math.isfinite(active_feedback_timeout_s)
        or active_feedback_timeout_s <= 0.0
        or active_feedback_timeout_s > 1.0
    ):
        raise TeachExecutionError("active feedback timeout must be finite, positive, and no more than 1.0 s")
    if active_feedback_attempts < 1:
        raise TeachExecutionError("active feedback attempts must be at least one")
    if not math.isfinite(segment_quiet_settle_s) or segment_quiet_settle_s < 0.0:
        raise TeachExecutionError("segment quiet-settle time must be finite and non-negative")
    if not math.isfinite(segment_max_abs_speed_rpm) or segment_max_abs_speed_rpm < 0.0:
        raise TeachExecutionError("segment settle speed must be finite and non-negative")
    if segment_settled_samples < 1:
        raise TeachExecutionError("segment settled samples must be at least one")
    # The deployed F407 has shown intermittent trajectory ACK loss.  Admit one
    # compact point only after the preceding point has physically settled. This
    # keeps at most one unexecuted point in the F407 FIFO; a lost ACK therefore
    # cannot leave a tail of later points waiting to execute.
    points = plan["points"]
    partition_batches(points)
    try:
        refresh_session_calibration_heartbeat(hardware_config_path)
        clear_sequence, sequence = _consume_sequence(sequence)
        clear_response = exchange(
            Frame(CMD_TRAJ_BUFFER_CLEAR, clear_sequence),
            port=port,
            baud=baud,
            timeout_s=timeout_s,
            hardware_config=hardware_config_path,
        )
        if (
            clear_response.cmd != RSP_TRAJ_ACK
            or clear_response.seq != clear_sequence
            or len(clear_response.payload) != 1
        ):
            raise TeachExecutionError(
                "F407 did not return a trajectory-clear ACK with free_slots: "
                f"cmd=0x{clear_response.cmd:02X}, seq={clear_response.seq}, "
                f"payload={clear_response.payload.hex()}"
            )
        free_slots = clear_response.payload[0]
        if free_slots != F407_FIFO_CAPACITY:
            raise TeachExecutionError(
                f"trajectory-clear ACK reported {free_slots} free slots, expected "
                f"an empty FIFO with {F407_FIFO_CAPACITY}"
            )

        point_feedback_window_s = max(8.0, float(completion_margin_s) / len(points))
        final_angles: list[float] | None = None
        final_errors: list[float] | None = None
        for point_index, point in enumerate(points, start=1):
            if free_slots < 1:
                raise TeachExecutionError(
                    f"trajectory point {point_index} cannot enter a full F407 FIFO"
                )
            payload = pack_trajectory_payload(
                point["positions_deg"], int(point["duration_ms"])
            )
            refresh_session_calibration_heartbeat(hardware_config_path)
            point_sequence, sequence = _consume_sequence(sequence)
            response = exchange(
                Frame(CMD_TRAJ_POINT, point_sequence, payload),
                port=port,
                baud=baud,
                timeout_s=timeout_s,
                hardware_config=hardware_config_path,
            )
            if response.cmd != RSP_TRAJ_ACK or response.seq != point_sequence or len(response.payload) != 1:
                raise TeachExecutionError(
                    f"trajectory point {point_index} was not acknowledged: "
                    f"cmd=0x{response.cmd:02X}, seq={response.seq}, payload={response.payload.hex()}"
                )
            reported_free_slots = response.payload[0]
            # joint_ctrl can dequeue the first point while later compact
            # frames are being admitted, so free slots are not guaranteed to
            # decrease monotonically. A matching compact ACK is the admission
            # proof; its slot count must only remain within the FIFO range.
            if not 0 <= reported_free_slots <= F407_FIFO_CAPACITY:
                raise TeachExecutionError(
                    f"trajectory point {point_index} ACK reported invalid free_slots={reported_free_slots}"
                )
            free_slots = reported_free_slots

            # The F407 comm thread has intermittently missed UART queries while
            # its 10 ms CAN interpolation is active. Do not poll during that
            # interval: the MCU owns the segment until its declared duration
            # has elapsed, then Pi verifies actual settled feedback before the
            # next point is admitted.
            time.sleep(int(point["duration_ms"]) / 1000.0 + segment_quiet_settle_s)
            point_deadline = time.monotonic() + point_feedback_window_s
            settled_point_samples = 0
            last_feedback_error: Exception | None = None
            while time.monotonic() < point_deadline:
                try:
                    snapshot, sequence = _read_joint_snapshot(
                        sequence,
                        port=port,
                        baud=baud,
                        timeout_s=active_feedback_timeout_s,
                        # Retry only read-only feedback. A trajectory admission
                        # with an unknown ACK is never retried.
                        attempts=active_feedback_attempts,
                    )
                except TeachExecutionError as exc:
                    last_feedback_error = exc
                    settled_point_samples = 0
                    time.sleep(min(poll_interval_s, 0.25))
                    continue
                current_angles = validate_joint_snapshot(
                    snapshot,
                    require_idle=False,
                    max_abs_speed_rpm=math.inf,
                )
                point_errors = [
                    abs(actual - target)
                    for actual, target in zip(current_angles, point["positions_deg"])
                ]
                max_speed_rpm = max(abs(float(joint["speed_rpm"])) for joint in snapshot)
                if max(point_errors) <= completion_tolerance_deg and max_speed_rpm <= segment_max_abs_speed_rpm:
                    settled_point_samples += 1
                    if settled_point_samples >= segment_settled_samples:
                        final_angles = current_angles
                        final_errors = point_errors
                        break
                else:
                    settled_point_samples = 0
                time.sleep(poll_interval_s)
            else:
                detail = (
                    f"; last feedback error: {last_feedback_error}"
                    if last_feedback_error is not None
                    else ""
                )
                raise TeachExecutionError(
                    f"trajectory point {point_index} did not settle before its deadline{detail}"
                )
    except Exception as exc:
        next_sequence = getattr(exc, "next_sequence", None)
        if isinstance(next_sequence, int):
            sequence = next_sequence
        reason = (
            str(exc)
            if isinstance(exc, TeachExecutionError)
            else f"trajectory admission became uncertain: {type(exc).__name__}: {exc}"
        )
        raise _stop_after_uncertain_execution(
            reason, sequence, port=port, baud=baud, timeout_s=timeout_s
        ) from exc

    assert final_angles is not None and final_errors is not None
    return {
        "completed": True,
        "feedback_source": "CMD_GET_JOINT",
        "execution_mode": "one_point_admit_then_settle",
        "segments_completed": len(points),
        "settled_goal_samples": segment_settled_samples,
        "final_deg": final_angles,
        "error_deg": final_errors,
        "max_error_deg": max(final_errors),
        "next_sequence": sequence,
    }


def execute_taught_side_route(
    route_plan: dict[str, Any],
    *,
    hardware_config_path: Path,
    port: str,
    baud: int,
    timeout_s: float,
    poll_interval_s: float,
    completion_margin_s: float,
    completion_tolerance_deg: float,
    sequence: int,
    active_feedback_timeout_s: float | None = None,
    active_feedback_attempts: int = 3,
    segment_quiet_settle_s: float = 1.5,
    segment_max_abs_speed_rpm: float = SETTLED_MAX_ABS_SPEED_RPM,
    segment_settled_samples: int = 2,
) -> dict[str, Any]:
    """Execute one taught side route, clearing and settling at every stage.

    ``execute_plan`` keeps the F407 queue shallow and stops on an uncertain
    admission.  Reusing it stage-by-stage preserves that property for a route
    longer than one controller FIFO while retaining the Cartesian ordering.
    """

    if route_plan.get("schema") != "xiaou_taught_side_route_execute_plan_v1":
        raise TeachExecutionError("unexpected taught side route plan schema")
    stages = route_plan.get("stages")
    if not isinstance(stages, list) or not stages:
        raise TeachExecutionError("taught side route contains no stages")
    current_sequence = sequence
    completed_stages: list[dict[str, Any]] = []
    for expected_index, stage in enumerate(stages, start=1):
        if not isinstance(stage, dict) or int(stage.get("route_index", -1)) != expected_index:
            raise TeachExecutionError("taught side route stage order is invalid")
        stage_plan = stage.get("execution_plan")
        if not isinstance(stage_plan, dict):
            raise TeachExecutionError(f"route stage {expected_index} has no execution plan")
        try:
            result = execute_plan(
                stage_plan,
                hardware_config_path=hardware_config_path,
                port=port,
                baud=baud,
                timeout_s=timeout_s,
                poll_interval_s=poll_interval_s,
                completion_margin_s=completion_margin_s,
                completion_tolerance_deg=completion_tolerance_deg,
                sequence=current_sequence,
                active_feedback_timeout_s=active_feedback_timeout_s,
                active_feedback_attempts=active_feedback_attempts,
                segment_quiet_settle_s=segment_quiet_settle_s,
                segment_max_abs_speed_rpm=segment_max_abs_speed_rpm,
                segment_settled_samples=segment_settled_samples,
            )
        except TeachExecutionError as exc:
            raise TeachExecutionError(
                f"taught side route stage {expected_index} ({stage.get('name')}) failed: {exc}",
                next_sequence=exc.next_sequence,
            ) from exc
        next_sequence = result.get("next_sequence")
        if not isinstance(next_sequence, int) or not 0 <= next_sequence <= 0xFF:
            raise TeachExecutionError(f"route stage {expected_index} did not return a valid next sequence")
        current_sequence = next_sequence
        completed_stages.append({
            "route_index": expected_index,
            "name": stage.get("name"),
            "phase": stage.get("phase"),
            "segments_completed": result["segments_completed"],
            "final_deg": result["final_deg"],
            "max_error_deg": result["max_error_deg"],
        })
    final = completed_stages[-1]
    return {
        "completed": True,
        "execution_mode": "taught_side_route_stage_by_stage",
        "feedback_source": "CMD_GET_JOINT",
        "route_stages_completed": len(completed_stages),
        "compact_points_completed": sum(int(item["segments_completed"]) for item in completed_stages),
        "final_deg": final["final_deg"],
        "max_error_deg": max(float(item["max_error_deg"]) for item in completed_stages),
        "stages": completed_stages,
        "next_sequence": current_sequence,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--teach-pose", type=Path, required=True)
    parser.add_argument("--hardware-config", type=Path, default=DEFAULT_HARDWARE_CONFIG)
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=_positive_int, default=DEFAULT_BAUD)
    parser.add_argument(
        "--timeout-s",
        type=float,
        default=5.0,
        help="read-only preflight reply timeout; does not set motion duration or active-motion watchdog time",
    )
    parser.add_argument(
        "--motion-command-timeout-s",
        type=float,
        default=0.75,
        help="maximum wait for CLEAR/POINT/STOP acknowledgement during motion (0..3 s)",
    )
    parser.add_argument(
        "--active-feedback-timeout-s",
        type=float,
        default=0.5,
        help="maximum wait for one CMD_GET_JOINT reply during active motion (0..1 s)",
    )
    parser.add_argument(
        "--active-feedback-attempts",
        type=_positive_int,
        default=3,
        help="read-only retries for an active-motion joint-feedback sample",
    )
    parser.add_argument(
        "--segment-quiet-settle-s",
        type=float,
        default=1.5,
        help="after each admitted point, wait this additional time before reading feedback",
    )
    parser.add_argument(
        "--segment-max-abs-speed-rpm",
        type=float,
        default=SETTLED_MAX_ABS_SPEED_RPM,
        help="maximum absolute feedback speed allowed before admitting the next point",
    )
    parser.add_argument(
        "--segment-settled-samples",
        type=_positive_int,
        default=2,
        help="consecutive in-tolerance feedback snapshots required per segment",
    )
    parser.add_argument("--read-attempts", type=_positive_int, default=3)
    parser.add_argument("--preflight-samples", type=_positive_int, default=3)
    parser.add_argument("--preflight-max-speed-rpm", type=float, default=2.0)
    parser.add_argument("--preflight-max-angle-spread-deg", type=float, default=0.25)
    parser.add_argument("--speed-limit-deg-s", type=_six_floats, default=[2.0] * JOINT_COUNT)
    parser.add_argument("--accel-limit-deg-s2", type=_six_floats, default=[1.0] * JOINT_COUNT)
    parser.add_argument("--max-point-duration-ms", type=_positive_int, default=9000)
    parser.add_argument(
        "--object-class",
        default="cola",
        help="object class for grasp-family authorization; only a local measured cylinder can reuse this route",
    )
    parser.add_argument(
        "--object-radius-m",
        type=float,
        help="measured radius for a non-reference local cylinder; required for bottle reuse",
    )
    parser.add_argument(
        "--object-height-m",
        type=float,
        help="measured height for a non-reference local cylinder; required for bottle reuse",
    )
    parser.add_argument(
        "--target-offset-base-m",
        type=_three_floats,
        default=[0.0, 0.0, 0.0],
        help="measured object-center offset from the taught contact in base frame; only the local registry envelope is allowed",
    )
    parser.add_argument("--poll-interval-s", type=float, default=0.5)
    parser.add_argument("--completion-margin-s", type=float, default=30.0)
    parser.add_argument("--completion-tolerance-deg", type=float, default=0.75)
    parser.add_argument("--allow-assumed-j6-hold", action="store_true")
    parser.add_argument(
        "--motion-profile",
        choices=MOTION_PROFILES,
        default="taught_side_colalocal",
        help="taught_side_colalocal uses raise/transfer/descend/side approach; direct_joint is retained only for explicit regression",
    )
    parser.add_argument(
        "--allow-route-j6-motion",
        action="store_true",
        help="required for real taught-side execution when the planned route changes J6",
    )
    parser.add_argument("--start-deg", type=_six_floats, help="offline planning start; forbidden for execution")
    parser.add_argument("--plan-output", type=Path, default=Path("runtime/teach_poses/grasp_execute_plan.json"))
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm", default="")
    parser.add_argument(
        "--firmware-confirmed",
        action="store_true",
        help="operator attests this run uses the motion-enabled F407 firmware",
    )
    parser.add_argument(
        "--estop-confirmed",
        action="store_true",
        help="operator attests physical E-stop was tested and is released",
    )
    parser.add_argument(
        "--mcu-calibration-confirmed",
        action="store_true",
        help="operator attests all six F407 zero calibrations are valid",
    )
    args = parser.parse_args()

    try:
        if args.timeout_s <= 0.0 or not math.isfinite(args.timeout_s):
            raise TeachExecutionError("timeout must be finite and positive")
        if (
            not math.isfinite(args.motion_command_timeout_s)
            or args.motion_command_timeout_s <= 0.0
            or args.motion_command_timeout_s > 3.0
            or not math.isfinite(args.active_feedback_timeout_s)
            or args.active_feedback_timeout_s <= 0.0
            or args.active_feedback_timeout_s > 1.0
        ):
            raise TeachExecutionError(
                "motion-command timeout must be within 0..3 s and active-feedback timeout within 0..1 s"
            )
        if (
            not math.isfinite(args.poll_interval_s)
            or not math.isfinite(args.completion_margin_s)
            or args.poll_interval_s <= 0.0
            or args.completion_margin_s <= 0.0
        ):
            raise TeachExecutionError("poll interval and completion margin must be finite and positive")
        if not math.isfinite(args.completion_tolerance_deg) or args.completion_tolerance_deg <= 0.0:
            raise TeachExecutionError("completion tolerance must be finite and positive")
        if args.object_radius_m is not None and (
            not math.isfinite(args.object_radius_m) or args.object_radius_m <= 0.0
        ):
            raise TeachExecutionError("object radius must be finite and positive when supplied")
        if args.object_height_m is not None and (
            not math.isfinite(args.object_height_m) or args.object_height_m <= 0.0
        ):
            raise TeachExecutionError("object height must be finite and positive when supplied")
        if not math.isfinite(args.segment_quiet_settle_s) or args.segment_quiet_settle_s < 0.0:
            raise TeachExecutionError("segment quiet-settle time must be finite and non-negative")
        if not math.isfinite(args.segment_max_abs_speed_rpm) or args.segment_max_abs_speed_rpm < 0.0:
            raise TeachExecutionError("segment settle speed must be finite and non-negative")
        if (
            not math.isfinite(args.preflight_max_speed_rpm)
            or args.preflight_max_speed_rpm < 0.0
            or not math.isfinite(args.preflight_max_angle_spread_deg)
            or args.preflight_max_angle_spread_deg < 0.0
        ):
            raise TeachExecutionError("preflight speed and angle-spread limits must be finite and non-negative")

        goal_deg, assumed_joint_ids = load_teach_pose(args.teach_pose)
        config = load_hardware_config(args.hardware_config)
        lower_deg, upper_deg = limits_deg_from_config(config)
        validate_pose_limits("goal", goal_deg, lower_deg, upper_deg)

        if assumed_joint_ids:
            if assumed_joint_ids != [6] or not args.allow_assumed_j6_hold:
                raise TeachExecutionError(
                    "teach pose contains assumed feedback; only an explicitly confirmed J6 hold is permitted"
                )
            if abs(goal_deg[5]) > 0.05:
                raise TeachExecutionError("assumed J6 hold is only permitted for a near-zero teach target")
        if args.motion_profile == "taught_side_colalocal" and assumed_joint_ids:
            raise TeachExecutionError(
                "taught side route requires a fully measured six-axis teach pose; recapture J6 instead of using an assumed hold"
            )

        if not args.execute:
            if args.start_deg is None:
                raise TeachExecutionError("offline planning requires --start-deg and never opens UART")
            start_deg = list(args.start_deg)
            validate_pose_limits("start", start_deg, lower_deg, upper_deg)
            if args.motion_profile == "direct_joint":
                plan = build_segmented_quintic_plan(
                    start_deg,
                    goal_deg,
                    speed_limit_deg_s=args.speed_limit_deg_s,
                    accel_limit_deg_s2=args.accel_limit_deg_s2,
                    max_point_duration_ms=args.max_point_duration_ms,
                )
                partition_batches(plan["points"])
            else:
                plan = build_taught_side_route_plan(
                    start_deg,
                    goal_deg,
                    lower_deg=lower_deg,
                    upper_deg=upper_deg,
                    speed_limit_deg_s=args.speed_limit_deg_s,
                    accel_limit_deg_s2=args.accel_limit_deg_s2,
                    max_point_duration_ms=args.max_point_duration_ms,
                    object_class=args.object_class,
                    object_radius_m=args.object_radius_m,
                    object_height_m=args.object_height_m,
                    target_offset_base_m=args.target_offset_base_m,
                )
            plan["mode"] = "offline_no_uart"
            plan["assumed_joint_ids"] = assumed_joint_ids
            _atomic_json_write(args.plan_output, plan)
            print(json.dumps({
                "planned": True,
                "execution": False,
                "motion_profile": args.motion_profile,
                "output": str(args.plan_output),
                "duration_s": plan.get("duration_s", plan.get("total_duration_s")),
                "segments": plan.get("segment_count", plan.get("route_stage_count")),
                "requires_j6_motion": plan.get("requires_j6_motion", False),
            }, ensure_ascii=False, indent=2))
            return 0

        if args.start_deg is not None:
            raise TeachExecutionError("--start-deg is forbidden for real execution; live feedback is mandatory")
        confirm_token = (
            TAUGHT_SIDE_ROUTE_CONFIRM_TOKEN
            if args.motion_profile == "taught_side_colalocal"
            else CONFIRM_TOKEN
        )
        if args.confirm != confirm_token:
            raise TeachExecutionError(f"--execute requires --confirm {confirm_token}")
        if args.preflight_samples < 3:
            raise TeachExecutionError("real execution requires at least three stationary preflight samples")
        start_deg, sequence, _ = capture_preflight_start(
            port=args.port,
            baud=args.baud,
            timeout_s=args.timeout_s,
            read_attempts=args.read_attempts,
            sample_count=args.preflight_samples,
            max_abs_speed_rpm=args.preflight_max_speed_rpm,
            max_angle_spread_deg=args.preflight_max_angle_spread_deg,
        )
        validate_pose_limits("live start", start_deg, lower_deg, upper_deg)
        live_config = build_live_authorized_config(
            config,
            explicit_session_authorized=True,
            live_uart_verified=True,
            live_feedback_verified=True,
            operator_firmware_verified=args.firmware_confirmed,
            operator_estop_verified=args.estop_confirmed,
            operator_mcu_calibration_verified=args.mcu_calibration_confirmed,
        )
        if assumed_joint_ids == [6] and abs(start_deg[5] - goal_deg[5]) > 0.10:
            raise TeachExecutionError(
                f"J6 hold mismatch: live={start_deg[5]:.3f} deg, goal={goal_deg[5]:.3f} deg"
            )
        if args.motion_profile == "direct_joint":
            plan = build_segmented_quintic_plan(
                start_deg,
                goal_deg,
                speed_limit_deg_s=args.speed_limit_deg_s,
                accel_limit_deg_s2=args.accel_limit_deg_s2,
                max_point_duration_ms=args.max_point_duration_ms,
            )
            partition_batches(plan["points"])
            plan["mode"] = "supervised_real_execution"
        else:
            plan = build_taught_side_route_plan(
                start_deg,
                goal_deg,
                lower_deg=lower_deg,
                upper_deg=upper_deg,
                speed_limit_deg_s=args.speed_limit_deg_s,
                accel_limit_deg_s2=args.accel_limit_deg_s2,
                max_point_duration_ms=args.max_point_duration_ms,
                object_class=args.object_class,
                object_radius_m=args.object_radius_m,
                object_height_m=args.object_height_m,
                target_offset_base_m=args.target_offset_base_m,
            )
            if plan["requires_j6_motion"] and not args.allow_route_j6_motion:
                raise TeachExecutionError(
                    f"taught side route changes J6 by up to {plan['max_j6_travel_deg']:.2f} deg; "
                    "visually verify J6 and add --allow-route-j6-motion"
                )
            plan["mode"] = "supervised_real_taught_side_route"
        plan["assumed_joint_ids"] = assumed_joint_ids
        _atomic_json_write(args.plan_output, plan)
        print(json.dumps({
            "preflight_passed": True,
            "start_deg": start_deg,
            "goal_deg": goal_deg,
            "motion_profile": args.motion_profile,
            "duration_s": plan.get("duration_s", plan.get("total_duration_s")),
            "segments": plan.get("segment_count", plan.get("route_stage_count")),
            "requires_j6_motion": plan.get("requires_j6_motion", False),
            "plan_output": str(args.plan_output),
        }, ensure_ascii=False, indent=2), flush=True)
        transient_config_path = write_transient_hardware_config(
            live_config, args.plan_output.parent
        )
        try:
            executor = execute_taught_side_route if args.motion_profile == "taught_side_colalocal" else execute_plan
            result = executor(
                plan,
                hardware_config_path=transient_config_path,
                port=args.port,
                baud=args.baud,
                timeout_s=args.motion_command_timeout_s,
                poll_interval_s=args.poll_interval_s,
                completion_margin_s=args.completion_margin_s,
                completion_tolerance_deg=args.completion_tolerance_deg,
                sequence=sequence,
                active_feedback_timeout_s=args.active_feedback_timeout_s,
                active_feedback_attempts=args.active_feedback_attempts,
                segment_quiet_settle_s=args.segment_quiet_settle_s,
                segment_max_abs_speed_rpm=args.segment_max_abs_speed_rpm,
                segment_settled_samples=args.segment_settled_samples,
            )
        finally:
            transient_config_path.unlink(missing_ok=True)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0
    except (MotionLockedError, OSError, ProtocolError, TeachExecutionError, ValueError) as exc:
        print(f"teach execution blocked: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
