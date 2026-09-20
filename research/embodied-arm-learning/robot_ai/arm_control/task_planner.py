"""Deterministic YOLO-center task planning for the F407 UART arm path.

The VLA is intentionally outside this module. It may emit only an intent such
as "pick cup" or "tidy_all". This module selects known 2D detections, applies
the fixed-camera calibration, and produces a non-joint preview. It never sends
UART bytes and never invents a height, PWM angle, or placement pose.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any, Iterable

import yaml

from .safety import load_hardware_config, validate_motion_readiness


PROJECT_DIR = Path(__file__).resolve().parents[2]
DEFAULT_PROFILE_PATH = Path(__file__).resolve().parent / "config" / "object_grasp_profiles.json"
DEFAULT_HOMOGRAPHY_PATH = PROJECT_DIR / "codex_pickup_package" / "workspace_homography.yaml"
REQUIRED_WIDTH_PX = 1920
REQUIRED_HEIGHT_PX = 1080


class PlanningBlocked(RuntimeError):
    """The requested task lacks measured data or violates a safety boundary."""


@dataclass(frozen=True)
class DetectionCenter:
    """A YOLO result reduced to the only vision data accepted by this planner."""

    object_class: str
    u_px: float
    v_px: float
    confidence: float = 1.0

    def __post_init__(self) -> None:
        if not isinstance(self.object_class, str) or not self.object_class.strip():
            raise ValueError("object_class must be a non-empty string")
        for label, value in (("u_px", self.u_px), ("v_px", self.v_px), ("confidence", self.confidence)):
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
                raise ValueError(f"{label} must be finite")
        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError("confidence must be in 0..1")

    @property
    def normalized_class(self) -> str:
        return self.object_class.strip().lower()


@dataclass(frozen=True)
class ObjectStep:
    """One class-specific, calibrated object preview; no joint command is stored."""

    object_class: str
    u_px: float
    v_px: float
    x_table_m: float
    y_table_m: float
    grasp_height_m: float
    approach_height_m: float
    gripper_open_pwm_deg: float
    gripper_close_pwm_deg: float
    placement_pose_id: str
    failure_policy: str


@dataclass(frozen=True)
class TaskPlan:
    """Result suitable for display/logging and a later supervised trajectory layer."""

    action: str
    transmittable: bool
    reason: str
    steps: tuple[ObjectStep, ...] = ()


def _as_finite_number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        raise PlanningBlocked(f"{label} is not a measured finite number")
    return float(value)


def load_object_profiles(path: str | Path = DEFAULT_PROFILE_PATH) -> dict[str, Any]:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    if data.get("schema_version") != 2:
        raise PlanningBlocked("object grasp profile schema_version must be 2")
    contract = data.get("camera_contract")
    if not isinstance(contract, dict):
        raise PlanningBlocked("camera_contract is missing")
    if contract.get("width_px") != REQUIRED_WIDTH_PX or contract.get("height_px") != REQUIRED_HEIGHT_PX:
        raise PlanningBlocked("object profile camera contract is not the fixed 1920x1080 setup")
    if contract.get("mount") != "fixed_external_overhead" or contract.get("camera_motion") != "forbidden_during_pick":
        raise PlanningBlocked("camera contract does not describe the fixed overhead camera")
    classes = data.get("classes")
    if not isinstance(classes, dict):
        raise PlanningBlocked("object profile classes are missing")
    return data


def _homography_resolution(data: dict[str, Any]) -> tuple[int, int] | None:
    direct_width = data.get("image_width_px")
    direct_height = data.get("image_height_px")
    if isinstance(direct_width, int) and isinstance(direct_height, int):
        return direct_width, direct_height

    camera = data.get("camera")
    if isinstance(camera, dict):
        width = camera.get("width_px")
        height = camera.get("height_px")
        if isinstance(width, int) and isinstance(height, int):
            return width, height
    return None


def load_homography(path: str | Path = DEFAULT_HOMOGRAPHY_PATH) -> tuple[tuple[float, ...], ...]:
    """Load a 1920x1080-only workspace homography or reject stale calibration."""

    data = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    if not isinstance(data, dict):
        raise PlanningBlocked("workspace calibration must be a mapping")
    resolution = _homography_resolution(data)
    if resolution != (REQUIRED_WIDTH_PX, REQUIRED_HEIGHT_PX):
        raise PlanningBlocked(
            "workspace homography is not verified for the fixed 1920x1080 camera; recalibration is required"
        )
    if data.get("output_frame") != "robot_base_table" or data.get("output_unit") != "m":
        raise PlanningBlocked(
            "workspace homography must declare output_frame=robot_base_table and output_unit=m"
        )
    matrix = data.get("homography")
    if not isinstance(matrix, list) or len(matrix) != 3 or any(not isinstance(row, list) or len(row) != 3 for row in matrix):
        raise PlanningBlocked("workspace homography must be a 3x3 matrix")
    try:
        result = tuple(tuple(_as_finite_number(value, "homography") for value in row) for row in matrix)
    except PlanningBlocked as exc:
        raise PlanningBlocked("workspace homography contains an invalid value") from exc
    return result


def project_center(homography: tuple[tuple[float, ...], ...], u_px: float, v_px: float) -> tuple[float, float]:
    denominator = homography[2][0] * u_px + homography[2][1] * v_px + homography[2][2]
    if not math.isfinite(denominator) or abs(denominator) < 1e-12:
        raise PlanningBlocked("homography maps the target to infinity")
    x = (homography[0][0] * u_px + homography[0][1] * v_px + homography[0][2]) / denominator
    y = (homography[1][0] * u_px + homography[1][1] * v_px + homography[1][2]) / denominator
    if not math.isfinite(x) or not math.isfinite(y):
        raise PlanningBlocked("homography result is not finite")
    return x, y


def _strategy_step(
    detection: DetectionCenter,
    class_profile: Any,
    homography: tuple[tuple[float, ...], ...],
) -> ObjectStep:
    if not isinstance(class_profile, dict):
        raise PlanningBlocked(f"class '{detection.normalized_class}' has no strategy profile")

    missing: list[str] = []
    values: dict[str, float] = {}
    for key in ("grasp_height_m", "approach_height_m", "gripper_open_pwm_deg", "gripper_close_pwm_deg"):
        try:
            values[key] = _as_finite_number(class_profile.get(key), f"{detection.normalized_class}.{key}")
        except PlanningBlocked:
            missing.append(key)
    placement_pose_id = class_profile.get("placement_pose_id")
    if not isinstance(placement_pose_id, str) or not placement_pose_id.strip():
        missing.append("placement_pose_id")
    failure_policy = class_profile.get("failure_policy")
    if failure_policy != "vision_recheck_then_safe_return_then_report":
        missing.append("failure_policy")
    if missing:
        raise PlanningBlocked(
            f"class '{detection.normalized_class}' is uncalibrated: " + ", ".join(missing)
        )

    if values["grasp_height_m"] < 0.0 or values["approach_height_m"] < values["grasp_height_m"]:
        raise PlanningBlocked(f"class '{detection.normalized_class}' has invalid measured grasp heights")
    if not 0.0 <= values["gripper_open_pwm_deg"] <= 180.0:
        raise PlanningBlocked(f"class '{detection.normalized_class}' has invalid gripper_open_pwm_deg")
    if not 0.0 <= values["gripper_close_pwm_deg"] <= 180.0:
        raise PlanningBlocked(f"class '{detection.normalized_class}' has invalid gripper_close_pwm_deg")

    if not (0.0 <= detection.u_px < REQUIRED_WIDTH_PX and 0.0 <= detection.v_px < REQUIRED_HEIGHT_PX):
        raise PlanningBlocked(f"class '{detection.normalized_class}' center is outside the 1920x1080 image")
    x_table_m, y_table_m = project_center(homography, detection.u_px, detection.v_px)
    return ObjectStep(
        object_class=detection.normalized_class,
        u_px=float(detection.u_px),
        v_px=float(detection.v_px),
        x_table_m=x_table_m,
        y_table_m=y_table_m,
        grasp_height_m=values["grasp_height_m"],
        approach_height_m=values["approach_height_m"],
        gripper_open_pwm_deg=values["gripper_open_pwm_deg"],
        gripper_close_pwm_deg=values["gripper_close_pwm_deg"],
        placement_pose_id=placement_pose_id.strip(),
        failure_policy=failure_policy,
    )


def _normalize_action(action: str) -> str:
    normalized = action.strip().lower().replace("-", "_")
    aliases = {"tidy": "tidy_all", "clean_all": "tidy_all"}
    return aliases.get(normalized, normalized)


def plan_task(
    action: str,
    detections: Iterable[DetectionCenter],
    *,
    image_width_px: int,
    image_height_px: int,
    requested_class: str | None = None,
    profile_path: str | Path = DEFAULT_PROFILE_PATH,
    homography_path: str | Path = DEFAULT_HOMOGRAPHY_PATH,
    hardware_config_path: str | Path | None = None,
) -> TaskPlan:
    """Build a deterministic preview with no UART/CAN side effect.

    A STOP is the sole transmittable result before motion calibration because it
    is a non-motion command. HOME stays blocked: no measured six-axis home
    trajectory has been supplied.
    """

    normalized_action = _normalize_action(action)
    if normalized_action == "stop":
        return TaskPlan("stop", True, "non-motion STOP may be passed to the UART safety path")
    if normalized_action == "home":
        return TaskPlan("home", False, "home is locked: no measured six-axis home trajectory is configured")
    if normalized_action not in {"pick", "tidy_all"}:
        return TaskPlan(normalized_action, False, "VLA action is not in the allowed set: pick, tidy_all, stop, home")
    if (image_width_px, image_height_px) != (REQUIRED_WIDTH_PX, REQUIRED_HEIGHT_PX):
        return TaskPlan(normalized_action, False, "YOLO frame must be exactly 1920x1080")

    try:
        profiles = load_object_profiles(profile_path)
        homography = load_homography(homography_path)
    except PlanningBlocked as exc:
        return TaskPlan(normalized_action, False, str(exc))

    normalized_detections = sorted(
        (item for item in detections if item.confidence > 0.0),
        key=lambda item: (item.v_px, item.u_px, item.normalized_class, -item.confidence),
    )
    if normalized_action == "pick":
        if not requested_class or not requested_class.strip():
            return TaskPlan("pick", False, "pick requires a VLA-selected object class")
        selected_class = requested_class.strip().lower()
        matches = [item for item in normalized_detections if item.normalized_class == selected_class]
        if not matches:
            return TaskPlan("pick", False, f"no current YOLO center for requested class '{selected_class}'")
        chosen = max(matches, key=lambda item: (item.confidence, -item.v_px, -item.u_px))
        selected = [chosen]
    else:
        if not normalized_detections:
            return TaskPlan("tidy_all", False, "no current YOLO centers to tidy")
        selected = normalized_detections

    class_profiles = profiles["classes"]
    try:
        steps = tuple(_strategy_step(item, class_profiles.get(item.normalized_class), homography) for item in selected)
    except PlanningBlocked as exc:
        return TaskPlan(normalized_action, False, str(exc))

    config = load_hardware_config(hardware_config_path) if hardware_config_path is not None else load_hardware_config()
    readiness = validate_motion_readiness(config)
    if not readiness.ready:
        return TaskPlan(
            normalized_action,
            False,
            "motion locked by hardware gate: " + ", ".join(readiness.missing_or_invalid),
            steps,
        )
    return TaskPlan(
        normalized_action,
        False,
        "preview complete; supervised IK/trajectory execution must be implemented separately",
        steps,
    )
