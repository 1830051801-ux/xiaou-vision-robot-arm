"""Constraint-diffusion digital twin for the XiaoU six-axis arm.

The pipeline creates counterfactual visual observations from the checked-in
POE arm model, learns a conditional denoising trajectory prior, then projects
sampled trajectories through IK, joint limits, and the diagnostic scene. It is
strictly offline: no ROS hardware, CAN interface, serial port, or physical arm
command is opened by this tool.
"""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys
from typing import Any

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
ROBOT_AI = ROOT / "robot_ai"
if str(ROBOT_AI) not in sys.path:
    sys.path.insert(0, str(ROBOT_AI))

from arm_control import JointLimits, fk_space, ik_space_multistart, load_default_model
from arm_control.scene_review import DiagnosticScene, load_diagnostic_scene, review_tcp_positions


TRAJECTORY_STEPS = 32
JOINT_COUNT = 6
GRASP_INDEX = 10
PLACE_INDEX = 23
ASSUMED_JOINT_LIMIT_RAD = 1.20
TASK_NAMES = (
    "transfer",
    "sort_zone_a",
    "sort_zone_b",
    "inspection_scan",
    "precision_insert",
)
TASK_TO_ID = {name: index for index, name in enumerate(TASK_NAMES)}
TASK_COUNT = len(TASK_NAMES)
PHASE_NAMES = ("home", "approach", "contact", "transfer", "retreat", "dwell")
PHASE_TO_ID = {name: index for index, name in enumerate(PHASE_NAMES)}
PHASE_COUNT = len(PHASE_NAMES)
CONTEXT_DIM = 21 + TASK_COUNT
ARCHITECTURE_NAME = "process_graph_multitask_action_chunk_transformer_diffusion"
CONTEXT_TOKEN_NAMES = (
    "visual_grasp_pose",
    "visual_place_goal",
    "observation_uncertainty",
    "task_profile",
    "learned_task_token",
)


def require_torch():
    try:
        import torch
        from torch import nn
        from torch.utils.data import DataLoader, TensorDataset
    except Exception as exc:  # pragma: no cover - depends on the local CUDA runtime.
        raise RuntimeError(
            "PyTorch is required. Run this tool with D:\\EmbodiedAI\\mujoco-venv\\Scripts\\python.exe."
        ) from exc
    return torch, nn, DataLoader, TensorDataset


def rotation_z(angle_rad: float) -> np.ndarray:
    cosine, sine = math.cos(angle_rad), math.sin(angle_rad)
    return np.array(
        [[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )


def task_id_from_name(task_name: str) -> int:
    try:
        return TASK_TO_ID[task_name]
    except KeyError as exc:
        raise ValueError(f"unknown task profile {task_name!r}; expected one of {', '.join(TASK_NAMES)}") from exc


def task_name_from_id(task_id: int) -> str:
    if not 0 <= int(task_id) < TASK_COUNT:
        raise ValueError(f"invalid task id {task_id}")
    return TASK_NAMES[int(task_id)]


def task_one_hot(task_name: str) -> np.ndarray:
    one_hot = np.zeros(TASK_COUNT, dtype=np.float64)
    one_hot[task_id_from_name(task_name)] = 1.0
    return one_hot


def pose_to_context(
    grasp_pose: np.ndarray,
    place_pose: np.ndarray,
    confidence: float,
    xy_sigma_m: float,
    yaw_sigma_rad: float,
    task_name: str = "transfer",
) -> np.ndarray:
    grasp_rotation6d = grasp_pose[:3, :2].reshape(-1)
    place_rotation6d = place_pose[:3, :2].reshape(-1)
    context = np.concatenate(
        [
            grasp_pose[:3, 3] / 0.5,
            grasp_rotation6d,
            place_pose[:3, 3] / 0.5,
            place_rotation6d,
            np.array([confidence, xy_sigma_m / 0.01, yaw_sigma_rad / math.radians(10.0)], dtype=np.float64),
            task_one_hot(task_name),
        ]
    )
    if context.shape != (CONTEXT_DIM,) or not np.isfinite(context).all():
        raise ValueError("invalid diffusion-policy context")
    return context.astype(np.float32)


def quintic_blend(tau: np.ndarray) -> np.ndarray:
    return 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5


def build_expert_trajectory(home: np.ndarray, grasp: np.ndarray, place: np.ndarray) -> np.ndarray:
    """Build a smooth home -> grasp -> place -> home trajectory."""
    if home.shape != (JOINT_COUNT,) or grasp.shape != (JOINT_COUNT,) or place.shape != (JOINT_COUNT,):
        raise ValueError("every trajectory anchor must contain six joint angles")
    anchors = [(0, home), (GRASP_INDEX, grasp), (PLACE_INDEX, place), (TRAJECTORY_STEPS - 1, home)]
    trajectory = np.zeros((TRAJECTORY_STEPS, JOINT_COUNT), dtype=np.float64)
    for (start_index, start), (end_index, end) in zip(anchors, anchors[1:]):
        tau = np.linspace(0.0, 1.0, end_index - start_index + 1, dtype=np.float64)
        blend = quintic_blend(tau)[:, None]
        trajectory[start_index : end_index + 1] = start + blend * (end - start)
    return trajectory


def build_piecewise_trajectory(anchors: list[tuple[int, np.ndarray]]) -> np.ndarray:
    """Interpolate an ordered joint-space process trajectory through fixed stages."""
    if len(anchors) < 2 or anchors[0][0] != 0 or anchors[-1][0] != TRAJECTORY_STEPS - 1:
        raise ValueError("trajectory anchors must start at zero and end at the final action step")
    previous_index = -1
    trajectory = np.zeros((TRAJECTORY_STEPS, JOINT_COUNT), dtype=np.float64)
    for index, angles in anchors:
        if not previous_index < index < TRAJECTORY_STEPS:
            raise ValueError("trajectory anchor indices must be strictly increasing and in range")
        if np.asarray(angles).shape != (JOINT_COUNT,) or not np.isfinite(angles).all():
            raise ValueError("every trajectory anchor must contain six finite joint angles")
        previous_index = index
    for (start_index, start), (end_index, end) in zip(anchors, anchors[1:]):
        tau = np.linspace(0.0, 1.0, end_index - start_index + 1, dtype=np.float64)
        trajectory[start_index : end_index + 1] = start + quintic_blend(tau)[:, None] * (end - start)
    return trajectory


def trajectory_tcp_positions(model, trajectory: np.ndarray) -> np.ndarray:
    return np.stack(
        [fk_space(model.home_grasp_tcp, model.screw_axes, angles)[:3, 3] for angles in trajectory],
        axis=0,
    )


def pose_is_in_diagnostic_workspace(pose: np.ndarray) -> bool:
    x_m, y_m, z_m = pose[:3, 3]
    return 0.14 <= x_m <= 0.38 and -0.25 <= y_m <= 0.18 and 0.08 <= z_m <= 0.48


def perturb_pose(pose: np.ndarray, rng: np.random.Generator, xy_sigma_m: float, z_sigma_m: float, yaw_sigma_rad: float) -> np.ndarray:
    noisy = pose.copy()
    noisy[:3, 3] += rng.normal([0.0, 0.0, 0.0], [xy_sigma_m, xy_sigma_m, z_sigma_m])
    noisy[:3, :3] = rotation_z(float(rng.normal(0.0, yaw_sigma_rad))) @ noisy[:3, :3]
    return noisy


def rigidify_pose(pose: np.ndarray) -> np.ndarray:
    """Restore a valid SE(3) pose after float32 dataset serialization."""
    result = np.asarray(pose, dtype=np.float64).copy()
    if result.shape != (4, 4) or not np.isfinite(result).all():
        raise ValueError("pose must be a finite 4x4 matrix")
    left, _, right_t = np.linalg.svd(result[:3, :3])
    rotation = left @ right_t
    if np.linalg.det(rotation) < 0.0:
        left[:, -1] *= -1.0
        rotation = left @ right_t
    result[:3, :3] = rotation
    result[3] = [0.0, 0.0, 0.0, 1.0]
    return result


def wrap_angle(angle_rad: float) -> float:
    """Wrap an angle to [-pi, pi) without changing its physical orientation."""
    return float((angle_rad + math.pi) % (2.0 * math.pi) - math.pi)


@dataclass(frozen=True)
class PoseBelief:
    """A robust SE(3) pose estimate with an auditable uncertainty summary.

    The current simulator perturbs target poses in XYZ plus camera-frame yaw.
    This belief deliberately preserves the non-yaw part of the reference pose
    instead of claiming to solve a general raw-RGB 6D pose-estimation problem.
    """

    pose: np.ndarray
    xy_sigma_m: float
    z_sigma_m: float
    yaw_sigma_rad: float
    inlier_fraction: float
    effective_view_count: float
    maximum_normalized_residual: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "pose", rigidify_pose(self.pose))
        for label, value in (
            ("xy_sigma_m", self.xy_sigma_m),
            ("z_sigma_m", self.z_sigma_m),
            ("yaw_sigma_rad", self.yaw_sigma_rad),
            ("effective_view_count", self.effective_view_count),
            ("maximum_normalized_residual", self.maximum_normalized_residual),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{label} must be positive and finite")
        if not math.isfinite(self.inlier_fraction) or not 0.0 <= self.inlier_fraction <= 1.0:
            raise ValueError("inlier_fraction must be in [0, 1]")


def fuse_pose_belief(
    observations: tuple[np.ndarray, ...] | list[np.ndarray],
    xy_sigma_m: float,
    z_sigma_m: float,
    yaw_sigma_rad: float,
    huber_delta: float = 2.5,
) -> PoseBelief:
    """Fuse repeated calibrated pose observations with Huber down-weighting.

    The fusion is intentionally small and inspectable: translation is estimated
    in the base frame and the only rotational perturbation handled is relative
    yaw, matching the synthetic sensor model used throughout this benchmark.
    """
    if not observations:
        raise ValueError("at least one pose observation is required")
    values = (xy_sigma_m, z_sigma_m, yaw_sigma_rad, huber_delta)
    if not all(math.isfinite(value) and value > 0.0 for value in values):
        raise ValueError("fusion uncertainty scales and huber_delta must be positive and finite")

    poses = tuple(rigidify_pose(pose) for pose in observations)
    reference = poses[0]
    translations = np.stack([pose[:3, 3] for pose in poses], axis=0)
    reference_rotation = reference[:3, :3]
    yaw_offsets = np.asarray(
        [
            math.atan2(
                float((pose[:3, :3] @ reference_rotation.T)[1, 0]),
                float((pose[:3, :3] @ reference_rotation.T)[0, 0]),
            )
            for pose in poses
        ],
        dtype=np.float64,
    )
    location = np.median(translations, axis=0)
    yaw_location = float(math.atan2(np.sin(yaw_offsets).mean(), np.cos(yaw_offsets).mean()))
    scales = np.array([xy_sigma_m, xy_sigma_m, z_sigma_m], dtype=np.float64)
    weights = np.ones(len(poses), dtype=np.float64)
    normalized = np.zeros(len(poses), dtype=np.float64)
    for _ in range(8):
        translation_residual = np.linalg.norm((translations - location) / scales, axis=1)
        yaw_residual = np.asarray([wrap_angle(value - yaw_location) for value in yaw_offsets], dtype=np.float64)
        normalized = np.sqrt(translation_residual**2 + (yaw_residual / yaw_sigma_rad) ** 2)
        weights = np.minimum(1.0, huber_delta / np.maximum(normalized, 1e-9))
        updated_location = np.average(translations, axis=0, weights=weights)
        updated_yaw = float(math.atan2(np.sum(weights * np.sin(yaw_offsets)), np.sum(weights * np.cos(yaw_offsets))))
        if np.linalg.norm(updated_location - location) < 1e-10 and abs(wrap_angle(updated_yaw - yaw_location)) < 1e-10:
            location, yaw_location = updated_location, updated_yaw
            break
        location, yaw_location = updated_location, updated_yaw

    effective_view_count = float(weights.sum() ** 2 / np.maximum(np.square(weights).sum(), 1e-9))
    translation_variance = np.average((translations - location) ** 2, axis=0, weights=weights)
    yaw_variance = float(
        np.average(
            np.asarray([wrap_angle(value - yaw_location) ** 2 for value in yaw_offsets], dtype=np.float64),
            weights=weights,
        )
    )
    fused_pose = reference.copy()
    fused_pose[:3, 3] = location
    fused_pose[:3, :3] = rotation_z(yaw_location) @ reference_rotation
    return PoseBelief(
        pose=fused_pose,
        xy_sigma_m=max(0.00025, math.sqrt(max(float(translation_variance[0]), float(translation_variance[1]), 0.0) + xy_sigma_m**2) / math.sqrt(effective_view_count)),
        z_sigma_m=max(0.00025, math.sqrt(max(float(translation_variance[2]), 0.0) + z_sigma_m**2) / math.sqrt(effective_view_count)),
        yaw_sigma_rad=max(math.radians(0.08), math.sqrt(max(yaw_variance, 0.0) + yaw_sigma_rad**2) / math.sqrt(effective_view_count)),
        inlier_fraction=float(np.mean(normalized <= huber_delta)),
        effective_view_count=effective_view_count,
        maximum_normalized_residual=max(1e-9, float(normalized.max())),
    )


def synthesize_multiview_observations(
    clean_pose: np.ndarray,
    primary_observation: np.ndarray,
    view_count: int,
    xy_sigma_m: float,
    z_sigma_m: float,
    yaw_sigma_rad: float,
    outlier_probability: float,
    rng: np.random.Generator,
) -> tuple[np.ndarray, ...]:
    """Create benchmark-only independent re-observations around a clean pose.

    In a deployed system, callers would pass calibrated camera estimates to
    ``fuse_pose_belief`` directly. This helper exists only to make the offline
    benchmark's multi-observation assumption explicit and reproducible.
    """
    if view_count < 1:
        raise ValueError("view_count must be positive")
    if not math.isfinite(outlier_probability) or not 0.0 <= outlier_probability < 1.0:
        raise ValueError("outlier_probability must be in [0, 1)")
    observations = [rigidify_pose(primary_observation)]
    for _ in range(1, view_count):
        scale = 4.0 if rng.random() < outlier_probability else 1.0
        observations.append(
            perturb_pose(clean_pose, rng, xy_sigma_m * scale, z_sigma_m * scale, yaw_sigma_rad * scale)
        )
    return tuple(observations)


@dataclass(frozen=True)
class DatasetSettings:
    count: int
    seed: int
    xy_sigma_m: float
    z_sigma_m: float
    yaw_sigma_deg: float
    tasks: tuple[str, ...] = TASK_NAMES
    domain_randomization: bool = False

    def __post_init__(self) -> None:
        if self.count < 1:
            raise ValueError("dataset count must be positive")
        values = (self.xy_sigma_m, self.z_sigma_m, self.yaw_sigma_deg)
        if not all(math.isfinite(value) and value >= 0.0 for value in values):
            raise ValueError("noise settings must be finite and non-negative")
        if not self.tasks or len(set(self.tasks)) != len(self.tasks):
            raise ValueError("tasks must contain at least one unique task profile")
        for task_name in self.tasks:
            task_id_from_name(task_name)


@dataclass(frozen=True)
class TaskProfile:
    name: str
    description: str
    approach_height_m: float
    place_y_min_m: float | None = None
    place_y_max_m: float | None = None
    place_z_max_m: float | None = None


TASK_PROFILES = {
    "transfer": TaskProfile("transfer", "general target-to-target transfer", 0.04),
    "sort_zone_a": TaskProfile("sort_zone_a", "transfer into simulated placement zone A", 0.04, place_y_min_m=-0.08),
    "sort_zone_b": TaskProfile("sort_zone_b", "transfer into simulated placement zone B", 0.04, place_y_max_m=-0.16),
    "inspection_scan": TaskProfile("inspection_scan", "two-view visual inspection scan", 0.05),
    "precision_insert": TaskProfile("precision_insert", "slow final insertion and dwell", 0.025, place_y_min_m=-0.17, place_y_max_m=-0.04, place_z_max_m=0.25),
}


def task_profile(task_name: str) -> TaskProfile:
    task_id_from_name(task_name)
    return TASK_PROFILES[task_name]


def pose_matches_task_profile(place_pose: np.ndarray, profile: TaskProfile) -> bool:
    _, y_m, z_m = place_pose[:3, 3]
    if profile.place_y_min_m is not None and y_m < profile.place_y_min_m:
        return False
    if profile.place_y_max_m is not None and y_m > profile.place_y_max_m:
        return False
    if profile.place_z_max_m is not None and z_m > profile.place_z_max_m:
        return False
    return True


def pose_with_vertical_clearance(pose: np.ndarray, clearance_m: float) -> np.ndarray:
    result = rigidify_pose(pose)
    result[:3, 3][2] += clearance_m
    return result


def diagnostic_joint_limits() -> JointLimits:
    return JointLimits(
        position_min=np.full(JOINT_COUNT, -ASSUMED_JOINT_LIMIT_RAD),
        position_max=np.full(JOINT_COUNT, ASSUMED_JOINT_LIMIT_RAD),
        velocity_max=np.full(JOINT_COUNT, 0.4),
        acceleration_max=np.full(JOINT_COUNT, 0.8),
    )


def solve_pose_ik(model, limits: JointLimits, target_pose: np.ndarray, seed: np.ndarray):
    bounded_seed = np.clip(np.asarray(seed, dtype=np.float64), limits.position_min, limits.position_max)
    offsets = (
        np.zeros(JOINT_COUNT, dtype=np.float64),
        np.array([0.0, -0.35, 0.35, 0.0, 0.0, 0.0]),
        np.array([0.0, 0.35, -0.35, 0.0, 0.0, 0.0]),
        np.array([0.0, -0.70, 0.70, 0.0, 0.0, 0.0]),
        np.array([0.0, 0.70, -0.70, 0.0, 0.0, 0.0]),
    )
    candidate_seeds = [np.clip(bounded_seed + offset, limits.position_min, limits.position_max) for offset in offsets]
    candidate_seeds.append(np.zeros(JOINT_COUNT, dtype=np.float64))
    return ik_space_multistart(
        model.home_grasp_tcp,
        model.screw_axes,
        rigidify_pose(target_pose),
        candidate_seeds,
        preferred_angles=bounded_seed,
        joint_lower=limits.position_min,
        joint_upper=limits.position_max,
        orientation_tolerance_rad=2e-4,
        position_tolerance_m=2e-4,
        max_iterations=250,
        max_step_rad=0.15,
    )


def build_task_trajectory(
    home: np.ndarray,
    grasp: np.ndarray,
    place: np.ndarray,
    approach_grasp: np.ndarray,
    approach_place: np.ndarray,
    task_name: str,
) -> np.ndarray:
    """Build an approach, contact, transfer, and retreat trajectory for one task profile."""
    task_id_from_name(task_name)
    if task_name == "inspection_scan":
        anchors = [
            (0, home), (6, approach_grasp), (GRASP_INDEX, grasp), (15, approach_grasp),
            (20, approach_place), (PLACE_INDEX, place), (27, approach_place), (TRAJECTORY_STEPS - 1, home),
        ]
    elif task_name == "precision_insert":
        anchors = [
            (0, home), (6, approach_grasp), (GRASP_INDEX, grasp), (14, approach_grasp),
            (20, approach_place), (PLACE_INDEX, place), (26, place), (TRAJECTORY_STEPS - 1, home),
        ]
    elif task_name == "sort_zone_a":
        anchors = [
            (0, home), (4, approach_grasp), (GRASP_INDEX, grasp), (13, approach_grasp),
            (18, approach_place), (PLACE_INDEX, place), (27, approach_place), (TRAJECTORY_STEPS - 1, home),
        ]
    elif task_name == "sort_zone_b":
        anchors = [
            (0, home), (7, approach_grasp), (GRASP_INDEX, grasp), (15, approach_grasp),
            (20, approach_place), (PLACE_INDEX, place), (28, approach_place), (TRAJECTORY_STEPS - 1, home),
        ]
    else:
        anchors = [
            (0, home), (5, approach_grasp), (GRASP_INDEX, grasp), (14, approach_grasp),
            (19, approach_place), (PLACE_INDEX, place), (27, approach_place), (TRAJECTORY_STEPS - 1, home),
        ]
    return build_piecewise_trajectory(anchors)


def build_task_phase_ids(task_name: str) -> np.ndarray:
    """Encode the task process graph over the fixed 32-step action chunk."""
    task_id_from_name(task_name)
    phases = np.full(TRAJECTORY_STEPS, PHASE_TO_ID["transfer"], dtype=np.int64)
    phases[0] = PHASE_TO_ID["home"]
    phases[1:GRASP_INDEX] = PHASE_TO_ID["approach"]
    phases[GRASP_INDEX] = PHASE_TO_ID["contact"]
    phases[PLACE_INDEX] = PHASE_TO_ID["contact"]
    phases[PLACE_INDEX + 1 : TRAJECTORY_STEPS - 1] = PHASE_TO_ID["retreat"]
    phases[-1] = PHASE_TO_ID["home"]
    if task_name == "precision_insert":
        phases[PLACE_INDEX + 1 : 27] = PHASE_TO_ID["dwell"]
    return phases


TASK_PHASE_IDS = np.stack([build_task_phase_ids(task_name) for task_name in TASK_NAMES], axis=0)


def balanced_task_schedule(count: int, tasks: tuple[str, ...], rng: np.random.Generator) -> list[str]:
    schedule = [tasks[index % len(tasks)] for index in range(count)]
    rng.shuffle(schedule)
    return schedule


def generate_dataset(output_path: Path, settings: DatasetSettings, scene: DiagnosticScene) -> dict[str, Any]:
    rng = np.random.default_rng(settings.seed)
    model = load_default_model()
    home = np.zeros(JOINT_COUNT, dtype=np.float64)
    limits = diagnostic_joint_limits()
    trajectories: list[np.ndarray] = []
    contexts: list[np.ndarray] = []
    task_ids: list[int] = []
    clean_grasp_poses: list[np.ndarray] = []
    clean_place_poses: list[np.ndarray] = []
    observed_grasp_poses: list[np.ndarray] = []
    observed_place_poses: list[np.ndarray] = []
    attempts = 0
    max_attempts = settings.count * 200
    task_schedule = balanced_task_schedule(settings.count, settings.tasks, rng)
    attempts_by_task = {task_name: 0 for task_name in settings.tasks}

    while len(trajectories) < settings.count:
        attempts += 1
        if attempts > max_attempts:
            raise RuntimeError(f"only generated {len(trajectories)}/{settings.count} safe episodes after {attempts} attempts")
        task_name = task_schedule[len(trajectories)]
        profile = task_profile(task_name)
        attempts_by_task[task_name] += 1
        # Keep the synthetic expert data inside a conservative portion of the
        # unmeasured joint envelope. Larger excursions are evaluated later as
        # distribution shift rather than silently treated as calibrated motion.
        grasp = rng.uniform(-0.42, 0.42, size=JOINT_COUNT)
        place = rng.uniform(-0.42, 0.42, size=JOINT_COUNT)
        clean_grasp = fk_space(model.home_grasp_tcp, model.screw_axes, grasp)
        clean_place = fk_space(model.home_grasp_tcp, model.screw_axes, place)
        if (
            not pose_is_in_diagnostic_workspace(clean_grasp)
            or not pose_is_in_diagnostic_workspace(clean_place)
            or not pose_matches_task_profile(clean_place, profile)
        ):
            continue
        approach_grasp_pose = pose_with_vertical_clearance(clean_grasp, profile.approach_height_m)
        approach_place_pose = pose_with_vertical_clearance(clean_place, profile.approach_height_m)
        if not pose_is_in_diagnostic_workspace(approach_grasp_pose) or not pose_is_in_diagnostic_workspace(approach_place_pose):
            continue
        approach_grasp_result = solve_pose_ik(model, limits, approach_grasp_pose, grasp)
        approach_place_result = solve_pose_ik(model, limits, approach_place_pose, place)
        if not approach_grasp_result.converged or not approach_place_result.converged:
            continue
        trajectory = build_task_trajectory(
            home,
            grasp,
            place,
            approach_grasp_result.joint_angles,
            approach_place_result.joint_angles,
            task_name,
        )
        if np.max(np.abs(trajectory)) > ASSUMED_JOINT_LIMIT_RAD:
            continue
        review = review_tcp_positions(trajectory_tcp_positions(model, trajectory), scene)
        if not review.safe:
            continue
        if settings.domain_randomization:
            noise_scale = float(rng.uniform(0.55, 1.45))
            xy_sigma_m = settings.xy_sigma_m * noise_scale
            z_sigma_m = settings.z_sigma_m * noise_scale
            yaw_sigma_rad = math.radians(settings.yaw_sigma_deg * noise_scale)
            confidence = float(rng.uniform(0.68, 0.99))
        else:
            xy_sigma_m = settings.xy_sigma_m
            z_sigma_m = settings.z_sigma_m
            yaw_sigma_rad = math.radians(settings.yaw_sigma_deg)
            confidence = float(rng.uniform(0.80, 0.99))
        observed_grasp = perturb_pose(clean_grasp, rng, xy_sigma_m, z_sigma_m, yaw_sigma_rad)
        observed_place = perturb_pose(clean_place, rng, xy_sigma_m, z_sigma_m, yaw_sigma_rad)
        trajectories.append((trajectory / ASSUMED_JOINT_LIMIT_RAD).astype(np.float32))
        contexts.append(pose_to_context(observed_grasp, observed_place, confidence, xy_sigma_m, yaw_sigma_rad, task_name))
        task_ids.append(task_id_from_name(task_name))
        clean_grasp_poses.append(clean_grasp.astype(np.float32))
        clean_place_poses.append(clean_place.astype(np.float32))
        observed_grasp_poses.append(observed_grasp.astype(np.float32))
        observed_place_poses.append(observed_place.astype(np.float32))

    output_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        output_path,
        trajectories=np.stack(trajectories),
        contexts=np.stack(contexts),
        task_ids=np.asarray(task_ids, dtype=np.int8),
        clean_grasp_poses=np.stack(clean_grasp_poses),
        clean_place_poses=np.stack(clean_place_poses),
        observed_grasp_poses=np.stack(observed_grasp_poses),
        observed_place_poses=np.stack(observed_place_poses),
    )
    metadata = {
        "episodes": len(trajectories),
        "attempts": attempts,
        "acceptance_rate": len(trajectories) / attempts,
        "trajectory_steps": TRAJECTORY_STEPS,
        "joint_count": JOINT_COUNT,
        "assumed_joint_limit_rad": ASSUMED_JOINT_LIMIT_RAD,
        "noise": {
            "xy_sigma_m": settings.xy_sigma_m,
            "z_sigma_m": settings.z_sigma_m,
            "yaw_sigma_deg": settings.yaw_sigma_deg,
        },
        "task_suite": list(settings.tasks),
        "task_counts": {task_name: task_ids.count(task_id_from_name(task_name)) for task_name in settings.tasks},
        "attempts_by_task": attempts_by_task,
        "domain_randomization": {
            "enabled": settings.domain_randomization,
            "noise_scale_range": [0.55, 1.45] if settings.domain_randomization else [1.0, 1.0],
            "confidence_range": [0.68, 0.99] if settings.domain_randomization else [0.80, 0.99],
        },
        "scene": "offline_diagnostic_scene.json",
        "real_motion_enabled": False,
    }
    output_path.with_suffix(".metadata.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return metadata


def build_denoiser(hidden_dim: int, diffusion_steps: int):
    torch, nn, _, _ = require_torch()

    if hidden_dim < 16 or hidden_dim % 4 != 0:
        raise ValueError("hidden_dim must be at least 16 and divisible by four")

    class EmbodiedActionChunkTransformer(nn.Module):
        """Process-graph ACT-style decoder conditioned on visual and task tokens.

        The model represents a complete 32-step joint action chunk rather than
        choosing a single grasp point.  A context encoder fuses observed grasp
        pose, place goal, perception uncertainty, and a task-profile token.
        A Transformer decoder then cross-attends from phase-aware six-axis action
        tokens to that context while estimating diffusion noise or a trajectory proposal.
        """

        def __init__(self) -> None:
            super().__init__()
            self.visual_grasp_projection = nn.Sequential(
                nn.Linear(9, hidden_dim),
                nn.SiLU(),
                nn.Linear(hidden_dim, hidden_dim),
            )
            self.visual_goal_projection = nn.Sequential(
                nn.Linear(9, hidden_dim),
                nn.SiLU(),
                nn.Linear(hidden_dim, hidden_dim),
            )
            self.uncertainty_projection = nn.Sequential(
                nn.Linear(3, hidden_dim),
                nn.SiLU(),
                nn.Linear(hidden_dim, hidden_dim),
            )
            self.task_profile_projection = nn.Sequential(
                nn.Linear(TASK_COUNT, hidden_dim),
                nn.SiLU(),
                nn.Linear(hidden_dim, hidden_dim),
            )
            self.task_token = nn.Parameter(torch.zeros(1, 1, hidden_dim))
            self.context_type_embedding = nn.Parameter(torch.zeros(1, len(CONTEXT_TOKEN_NAMES), hidden_dim))

            context_layer = nn.TransformerEncoderLayer(
                d_model=hidden_dim,
                nhead=4,
                dim_feedforward=hidden_dim * 4,
                dropout=0.0,
                activation="gelu",
                batch_first=True,
                norm_first=True,
            )
            self.context_encoder = nn.TransformerEncoder(context_layer, num_layers=2, enable_nested_tensor=False)

            self.state_projection = nn.Linear(JOINT_COUNT, hidden_dim)
            self.time_projection = nn.Sequential(
                nn.Linear(hidden_dim, hidden_dim),
                nn.SiLU(),
                nn.Linear(hidden_dim, hidden_dim),
            )
            self.action_position_embedding = nn.Parameter(torch.zeros(1, TRAJECTORY_STEPS, hidden_dim))
            self.action_type_embedding = nn.Parameter(torch.zeros(1, 1, hidden_dim))
            self.process_phase_embedding = nn.Embedding(PHASE_COUNT, hidden_dim)
            self.prior_query = nn.Parameter(torch.zeros(1, TRAJECTORY_STEPS, hidden_dim))
            self.register_buffer("task_phase_ids", torch.tensor(TASK_PHASE_IDS, dtype=torch.long), persistent=False)
            action_layer = nn.TransformerDecoderLayer(
                d_model=hidden_dim,
                nhead=4,
                dim_feedforward=hidden_dim * 4,
                dropout=0.0,
                activation="gelu",
                batch_first=True,
                norm_first=True,
            )
            self.action_decoder = nn.TransformerDecoder(action_layer, num_layers=3)
            self.output_projection = nn.Sequential(
                nn.LayerNorm(hidden_dim),
                nn.Linear(hidden_dim, hidden_dim),
                nn.SiLU(),
                nn.Linear(hidden_dim, JOINT_COUNT),
            )
            self._reset_parameters()

        def _reset_parameters(self) -> None:
            nn.init.normal_(self.task_token, mean=0.0, std=0.02)
            nn.init.normal_(self.context_type_embedding, mean=0.0, std=0.02)
            nn.init.normal_(self.action_position_embedding, mean=0.0, std=0.02)
            nn.init.normal_(self.action_type_embedding, mean=0.0, std=0.02)
            nn.init.normal_(self.prior_query, mean=0.0, std=0.02)

        def encode_context(self, context):
            if context.ndim != 2 or context.shape[1] != CONTEXT_DIM:
                raise ValueError(f"context must have shape (batch, {CONTEXT_DIM})")
            grasp_token = self.visual_grasp_projection(context[:, 0:9])
            goal_token = self.visual_goal_projection(context[:, 9:18])
            uncertainty_token = self.uncertainty_projection(context[:, 18:21])
            task_profile_token = self.task_profile_projection(context[:, 21 : 21 + TASK_COUNT])
            task_token = self.task_token.expand(context.shape[0], -1, -1).squeeze(1)
            tokens = torch.stack((grasp_token, goal_token, uncertainty_token, task_profile_token, task_token), dim=1)
            return self.context_encoder(tokens + self.context_type_embedding)

        def decode_action_chunk(self, action_tokens, context_tokens):
            return self.output_projection(self.action_decoder(action_tokens, context_tokens))

        def action_phase_embedding(self, context):
            task_ids = torch.argmax(context[:, 21 : 21 + TASK_COUNT], dim=1)
            phase_ids = self.task_phase_ids[task_ids]
            return self.process_phase_embedding(phase_ids)

        def forward(self, noisy_trajectory, time_index, context):
            if noisy_trajectory.ndim != 3 or noisy_trajectory.shape[1:] != (TRAJECTORY_STEPS, JOINT_COUNT):
                raise ValueError(f"noisy_trajectory must have shape (batch, {TRAJECTORY_STEPS}, {JOINT_COUNT})")
            half = hidden_dim // 2
            frequencies = torch.exp(
                -math.log(10000.0) * torch.arange(half, device=noisy_trajectory.device, dtype=torch.float32) / max(half - 1, 1)
            )
            phase = time_index.float().unsqueeze(1) * frequencies.unsqueeze(0)
            time_embedding = torch.cat([phase.sin(), phase.cos()], dim=1)
            if time_embedding.shape[1] < hidden_dim:
                time_embedding = torch.nn.functional.pad(time_embedding, (0, hidden_dim - time_embedding.shape[1]))
            action_tokens = self.state_projection(noisy_trajectory)
            action_tokens = action_tokens + self.time_projection(time_embedding).unsqueeze(1)
            action_tokens = action_tokens + self.action_position_embedding + self.action_type_embedding + self.action_phase_embedding(context)
            return self.decode_action_chunk(action_tokens, self.encode_context(context))

        def predict_prior(self, context):
            action_queries = self.prior_query.expand(context.shape[0], -1, -1)
            action_queries = action_queries + self.action_position_embedding + self.action_type_embedding + self.action_phase_embedding(context)
            return self.decode_action_chunk(action_queries, self.encode_context(context))

    return EmbodiedActionChunkTransformer()


def diffusion_schedule(torch, steps: int, device):
    betas = torch.linspace(1e-4, 0.02, steps, dtype=torch.float32, device=device)
    alphas = 1.0 - betas
    alpha_bars = torch.cumprod(alphas, dim=0)
    return {"betas": betas, "alphas": alphas, "alpha_bars": alpha_bars}


def train_model(dataset_path: Path, checkpoint_path: Path, epochs: int, batch_size: int, hidden_dim: int, diffusion_steps: int, seed: int) -> dict[str, Any]:
    torch, _, DataLoader, TensorDataset = require_torch()
    if epochs < 1 or batch_size < 1:
        raise ValueError("epochs and batch_size must be positive")
    torch.manual_seed(seed)
    np.random.seed(seed)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    data = np.load(dataset_path)
    trajectories = torch.tensor(data["trajectories"], dtype=torch.float32)
    contexts = torch.tensor(data["contexts"], dtype=torch.float32)
    if contexts.ndim != 2 or contexts.shape[1] != CONTEXT_DIM:
        raise ValueError(f"dataset contexts must have shape (episodes, {CONTEXT_DIM})")
    context_np = data["contexts"].astype(np.float64)
    context_mean = context_np.mean(axis=0)
    # Some context fields describe declared sensor noise and are intentionally
    # constant inside one training split. A scale floor prevents a harmless
    # metadata change from creating a numerical divide-by-near-zero OOD score.
    context_std = np.maximum(context_np.std(axis=0), 0.05)
    support_distances = np.linalg.norm((context_np - context_mean) / context_std, axis=1)
    support_threshold = float(np.quantile(support_distances, 0.995))
    loader = DataLoader(TensorDataset(trajectories, contexts), batch_size=batch_size, shuffle=True, drop_last=False)
    model = build_denoiser(hidden_dim, diffusion_steps).to(device)
    model_parameters = sum(parameter.numel() for parameter in model.parameters())
    optimizer = torch.optim.AdamW(model.parameters(), lr=2e-4, weight_decay=1e-4)
    schedule = diffusion_schedule(torch, diffusion_steps, device)
    losses: list[float] = []
    model.train()
    for epoch in range(1, epochs + 1):
        epoch_losses: list[float] = []
        for x0, context in loader:
            x0 = x0.to(device)
            context = context.to(device)
            time_index = torch.randint(0, diffusion_steps, (x0.shape[0],), device=device)
            alpha_bar = schedule["alpha_bars"][time_index].view(-1, 1, 1)
            noise = torch.randn_like(x0)
            noisy = torch.sqrt(alpha_bar) * x0 + torch.sqrt(1.0 - alpha_bar) * noise
            predicted_noise = model(noisy, time_index, context)
            prior = model.predict_prior(context)
            noise_loss = torch.nn.functional.mse_loss(predicted_noise, noise)
            prior_loss = torch.nn.functional.mse_loss(prior, x0)
            loss = noise_loss + 0.5 * prior_loss
            optimizer.zero_grad(set_to_none=True)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
            optimizer.step()
            epoch_losses.append(float(loss.detach().cpu()))
        losses.append(float(np.mean(epoch_losses)))
        if epoch == 1 or epoch % max(1, epochs // 8) == 0 or epoch == epochs:
            print(f"epoch={epoch:03d} loss={losses[-1]:.6f}")
    checkpoint_path.parent.mkdir(parents=True, exist_ok=True)
    torch.save(
        {
            "state_dict": model.state_dict(),
            "hidden_dim": hidden_dim,
            "diffusion_steps": diffusion_steps,
            "trajectory_steps": TRAJECTORY_STEPS,
            "context_dim": CONTEXT_DIM,
            "architecture": ARCHITECTURE_NAME,
            "context_tokens": list(CONTEXT_TOKEN_NAMES),
            "task_names": list(TASK_NAMES),
            "task_count": TASK_COUNT,
            "phase_names": list(PHASE_NAMES),
            "task_phase_ids": TASK_PHASE_IDS.tolist(),
            "action_chunk_length": TRAJECTORY_STEPS,
            "model_parameters": model_parameters,
            "joint_limit_rad": ASSUMED_JOINT_LIMIT_RAD,
            "context_mean": context_mean.astype(np.float32),
            "context_std": context_std.astype(np.float32),
            "context_support_threshold": support_threshold,
            "losses": losses,
            "device": str(device),
        },
        checkpoint_path,
    )
    return {
        "device": str(device),
        "epochs": epochs,
        "final_loss": losses[-1],
        "initial_loss": losses[0],
        "dataset_episodes": int(trajectories.shape[0]),
        "model_parameters": model_parameters,
        "context_support_threshold": support_threshold,
        "checkpoint": str(checkpoint_path),
    }


def sample_trajectories(model, context, diffusion_steps: int, sample_count: int, device):
    torch, _, _, _ = require_torch()
    model.eval()
    samples = []
    with torch.no_grad():
        for _ in range(sample_count):
            repeated_context = context.unsqueeze(0).to(device)
            trajectory = model.predict_prior(repeated_context) + 0.35 * torch.randn(
                (1, TRAJECTORY_STEPS, JOINT_COUNT), dtype=torch.float32, device=device
            )
            for step in range(diffusion_steps - 1, -1, -1):
                time_index = torch.full((1,), step, dtype=torch.long, device=device)
                beta = 1e-4 + (0.02 - 1e-4) * step / max(diffusion_steps - 1, 1)
                alpha = 1.0 - beta
                alpha_bar = float(np.prod([1.0 - (1e-4 + (0.02 - 1e-4) * index / max(diffusion_steps - 1, 1)) for index in range(step + 1)]))
                predicted_noise = model(trajectory, time_index, repeated_context)
                predicted_x0 = (trajectory - math.sqrt(1.0 - alpha_bar) * predicted_noise) / math.sqrt(alpha_bar)
                if step == 0:
                    trajectory = predicted_x0
                else:
                    previous_alpha_bar = float(np.prod([1.0 - (1e-4 + (0.02 - 1e-4) * index / max(diffusion_steps - 1, 1)) for index in range(step)]))
                    trajectory = math.sqrt(previous_alpha_bar) * predicted_x0 + math.sqrt(1.0 - previous_alpha_bar) * predicted_noise
            samples.append(trajectory.squeeze(0).cpu().numpy())
    return np.stack(samples, axis=0)


def counterfactual_sensor_scales(context: np.ndarray, perception_noise_fraction: float) -> tuple[float, float, float]:
    """Recover conservative perturbation scales from the policy context metadata."""
    if not 0.0 < perception_noise_fraction <= 1.0:
        raise ValueError("perception_noise_fraction must be in (0, 1]")
    xy_sigma_m = max(float(context[19]) * 0.01 * perception_noise_fraction, 0.0005)
    z_sigma_m = max(xy_sigma_m * 0.67, 0.0005)
    yaw_sigma_rad = max(float(context[20]) * math.radians(10.0) * perception_noise_fraction, math.radians(0.25))
    return xy_sigma_m, z_sigma_m, yaw_sigma_rad


def evaluate_counterfactual_consensus(
    model,
    context_np: np.ndarray,
    observed_grasp_pose: np.ndarray,
    observed_place_pose: np.ndarray,
    clean_grasp_pose: np.ndarray,
    task_name: str,
    scene: DiagnosticScene,
    diffusion_steps: int,
    rollouts: int,
    minimum_safe_fraction: float,
    perception_noise_fraction: float,
    context_mean: np.ndarray,
    context_std: np.ndarray,
    support_threshold: float,
    device,
    rng: np.random.Generator,
) -> dict[str, Any]:
    """Evaluate a plan under counterfactual re-observations before accepting it.

    Every rollout regenerates a plausible sensor observation, creates a fresh
    policy action chunk, and projects it through the same kinematic and scene
    shield. This is an offline robust-decision check, not a real-time control
    loop or a substitute for real perception calibration.
    """
    if rollouts < 1:
        raise ValueError("rollouts must be positive")
    if not 0.0 < minimum_safe_fraction <= 1.0:
        raise ValueError("minimum_safe_fraction must be in (0, 1]")
    torch, _, _, _ = require_torch()
    xy_sigma_m, z_sigma_m, yaw_sigma_rad = counterfactual_sensor_scales(context_np, perception_noise_fraction)
    confidence = float(np.clip(context_np[18], 0.0, 1.0))
    safe_rollouts = 0
    endpoint_errors: list[float] = []
    reasons: Counter[str] = Counter()
    arm = load_default_model()
    for _ in range(rollouts):
        counterfactual_grasp = perturb_pose(rigidify_pose(observed_grasp_pose), rng, xy_sigma_m, z_sigma_m, yaw_sigma_rad)
        counterfactual_place = perturb_pose(rigidify_pose(observed_place_pose), rng, xy_sigma_m, z_sigma_m, yaw_sigma_rad)
        counterfactual_context = pose_to_context(
            counterfactual_grasp,
            counterfactual_place,
            confidence,
            float(context_np[19]) * 0.01,
            float(context_np[20]) * math.radians(10.0),
            task_name,
        )
        support_distance = float(np.linalg.norm((counterfactual_context.astype(np.float64) - context_mean) / context_std))
        if support_distance > support_threshold:
            reasons["counterfactual context outside training support"] += 1
            continue
        context = torch.tensor(counterfactual_context, dtype=torch.float32)
        sample = sample_trajectories(model, context, diffusion_steps, 1, device)[0]
        prediction = np.clip(sample * ASSUMED_JOINT_LIMIT_RAD, -ASSUMED_JOINT_LIMIT_RAD, ASSUMED_JOINT_LIMIT_RAD)
        projected, failure = project_with_constraints(prediction, counterfactual_grasp, counterfactual_place, scene, task_name)
        if projected is None:
            reasons[failure or "counterfactual projection failed"] += 1
            continue
        safe_rollouts += 1
        endpoint_errors.append(pose_position_error(arm, projected[GRASP_INDEX], clean_grasp_pose))
    safe_fraction = safe_rollouts / rollouts
    return {
        "rollouts": rollouts,
        "safe_rollouts": safe_rollouts,
        "safe_fraction": safe_fraction,
        "accepted": safe_fraction >= minimum_safe_fraction,
        "mean_projected_grasp_error_m": float(np.mean(endpoint_errors)) if endpoint_errors else None,
        "rejection_reasons": dict(sorted(reasons.items())),
    }


def project_with_constraints(
    prediction: np.ndarray,
    observed_grasp_pose: np.ndarray,
    observed_place_pose: np.ndarray,
    scene: DiagnosticScene,
    task_name: str,
):
    profile = task_profile(task_name)
    model = load_default_model()
    limits = diagnostic_joint_limits()
    grasp_pose = rigidify_pose(observed_grasp_pose)
    place_pose = rigidify_pose(observed_place_pose)
    if not pose_is_in_diagnostic_workspace(grasp_pose) or not pose_is_in_diagnostic_workspace(place_pose):
        return None, "observed pose outside diagnostic workspace"
    if not pose_matches_task_profile(place_pose, profile):
        return None, "observed place pose outside selected task profile"
    grasp_result = solve_pose_ik(model, limits, grasp_pose, prediction[GRASP_INDEX])
    if not grasp_result.converged:
        return None, "IK projection failed at grasp"
    place_result = solve_pose_ik(model, limits, place_pose, prediction[PLACE_INDEX])
    if not place_result.converged:
        return None, "IK projection failed at place"
    approach_grasp_pose = pose_with_vertical_clearance(grasp_pose, profile.approach_height_m)
    approach_place_pose = pose_with_vertical_clearance(place_pose, profile.approach_height_m)
    if not pose_is_in_diagnostic_workspace(approach_grasp_pose) or not pose_is_in_diagnostic_workspace(approach_place_pose):
        return None, "approach pose outside diagnostic workspace"
    approach_grasp_result = solve_pose_ik(model, limits, approach_grasp_pose, grasp_result.joint_angles)
    if not approach_grasp_result.converged:
        return None, "IK projection failed at grasp approach"
    approach_place_result = solve_pose_ik(model, limits, approach_place_pose, place_result.joint_angles)
    if not approach_place_result.converged:
        return None, "IK projection failed at place approach"
    trajectory = build_task_trajectory(
        np.zeros(JOINT_COUNT),
        grasp_result.joint_angles,
        place_result.joint_angles,
        approach_grasp_result.joint_angles,
        approach_place_result.joint_angles,
        task_name,
    )
    review = review_tcp_positions(trajectory_tcp_positions(model, trajectory), scene)
    if not review.safe:
        return None, "projected trajectory violates diagnostic scene clearance"
    return trajectory, None


def trajectory_risk_features(trajectory: np.ndarray, scene: DiagnosticScene) -> dict[str, float]:
    """Compute transparent path-quality terms for offline candidate ranking."""
    values = np.asarray(trajectory, dtype=np.float64)
    if values.shape != (TRAJECTORY_STEPS, JOINT_COUNT) or not np.isfinite(values).all():
        raise ValueError(f"trajectory must have shape ({TRAJECTORY_STEPS}, {JOINT_COUNT})")
    model = load_default_model()
    review = review_tcp_positions(trajectory_tcp_positions(model, values), scene)
    obstacle_margin = math.inf
    if review.minimum_obstacle_clearance_m is not None:
        obstacle_margin = review.minimum_obstacle_clearance_m - scene.minimum_obstacle_clearance_m
    table_margin = review.minimum_table_clearance_m - scene.minimum_table_clearance_m
    step_norms = np.linalg.norm(np.diff(values, axis=0), axis=1)
    accelerations = np.linalg.norm(np.diff(values, n=2, axis=0), axis=1)
    return {
        "minimum_clearance_margin_m": float(min(table_margin, obstacle_margin)),
        "joint_travel_rad": float(step_norms.sum()),
        "maximum_joint_step_rad": float(step_norms.max(initial=0.0)),
        "mean_joint_acceleration_rad": float(accelerations.mean()) if len(accelerations) else 0.0,
    }


def select_cvar_belief_candidate(
    candidates: np.ndarray,
    scenario_grasp_poses: tuple[np.ndarray, ...],
    scenario_place_poses: tuple[np.ndarray, ...],
    fused_grasp_pose: np.ndarray,
    fused_place_pose: np.ndarray,
    scene: DiagnosticScene,
    task_name: str,
    minimum_safe_fraction: float,
) -> dict[str, Any]:
    """Select a policy proposal by lower-tail clearance across pose scenarios.

    This is a risk-sensitive *offline* selector. It ranks existing diffusion
    action chunks by their scenario safety fraction and 25th-percentile
    clearance, then uses joint travel and acceleration only as tie breakers.
    It is not a replacement for a calibrated physical safety controller.
    """
    predictions = np.asarray(candidates, dtype=np.float64)
    if predictions.ndim != 3 or predictions.shape[1:] != (TRAJECTORY_STEPS, JOINT_COUNT):
        raise ValueError(f"candidates must have shape (count, {TRAJECTORY_STEPS}, {JOINT_COUNT})")
    if not scenario_grasp_poses or len(scenario_grasp_poses) != len(scenario_place_poses):
        raise ValueError("grasp and place scenario observations must be non-empty and paired")
    if not 0.0 < minimum_safe_fraction <= 1.0:
        raise ValueError("minimum_safe_fraction must be in (0, 1]")

    reports: list[dict[str, Any]] = []
    rejection_counts: Counter[str] = Counter()
    for candidate_index, prediction in enumerate(predictions):
        margins: list[float] = []
        travels: list[float] = []
        accelerations: list[float] = []
        failures: Counter[str] = Counter()
        for grasp_pose, place_pose in zip(scenario_grasp_poses, scenario_place_poses):
            projected, failure = project_with_constraints(prediction, grasp_pose, place_pose, scene, task_name)
            if projected is None:
                failures[failure or "scenario projection failed"] += 1
                continue
            features = trajectory_risk_features(projected, scene)
            margins.append(float(features["minimum_clearance_margin_m"]))
            travels.append(float(features["joint_travel_rad"]))
            accelerations.append(float(features["mean_joint_acceleration_rad"]))
        safe_fraction = len(margins) / len(scenario_grasp_poses)
        if margins:
            # CVaR is the mean outcome in the lower tail, not merely the
            # 25th-percentile VaR threshold. With five scenarios this averages
            # the two least-clear projected paths.
            tail_count = max(1, int(math.ceil(len(margins) * 0.25)))
            clearance_cvar = float(np.mean(np.partition(np.asarray(margins), tail_count - 1)[:tail_count]))
        else:
            clearance_cvar = -math.inf
        mean_travel = float(np.mean(travels)) if travels else math.inf
        mean_acceleration = float(np.mean(accelerations)) if accelerations else math.inf
        # Safe-scenario coverage is the primary term. Lower-tail clearance
        # avoids selecting a path that is only safe under its average view.
        risk_score = (
            100.0 * safe_fraction
            + 1000.0 * clearance_cvar
            - 0.25 * mean_travel
            - 0.50 * mean_acceleration
        )
        reports.append(
            {
                "candidate_index": candidate_index,
                "safe_fraction": safe_fraction,
                "clearance_cvar_m": clearance_cvar,
                "mean_joint_travel_rad": mean_travel,
                "mean_joint_acceleration_rad": mean_acceleration,
                "risk_score": risk_score,
                "rejection_reasons": dict(sorted(failures.items())),
            }
        )
        rejection_counts.update(failures)

    feasible = [report for report in reports if report["safe_fraction"] >= minimum_safe_fraction]
    if not feasible:
        return {
            "accepted": False,
            "reason": "no diffusion candidate met the belief-scenario safety threshold",
            "candidate_reports": reports,
            "rejection_reasons": dict(sorted(rejection_counts.items())),
        }
    selected = max(feasible, key=lambda report: float(report["risk_score"]))
    selected_prediction = predictions[int(selected["candidate_index"])]
    final_trajectory, failure = project_with_constraints(
        selected_prediction,
        fused_grasp_pose,
        fused_place_pose,
        scene,
        task_name,
    )
    if final_trajectory is None:
        rejection_counts[failure or "fused-pose projection failed"] += 1
        return {
            "accepted": False,
            "reason": failure or "fused-pose projection failed",
            "candidate_reports": reports,
            "rejection_reasons": dict(sorted(rejection_counts.items())),
        }
    return {
        "accepted": True,
        "trajectory": final_trajectory,
        "selected_candidate_index": int(selected["candidate_index"]),
        "safe_fraction": float(selected["safe_fraction"]),
        "clearance_cvar_m": float(selected["clearance_cvar_m"]),
        "risk_score": float(selected["risk_score"]),
        "final_features": trajectory_risk_features(final_trajectory, scene),
        "candidate_reports": reports,
        "rejection_reasons": dict(sorted(rejection_counts.items())),
    }


def balanced_episode_subset(data, episodes_per_task: int):
    """Return a deterministic balanced in-memory subset for development runs."""
    if episodes_per_task <= 0:
        return data
    if "task_ids" not in data:
        raise ValueError("balanced evaluation requires task_ids in the dataset")
    task_ids = np.asarray(data["task_ids"])
    selected = []
    for task_id, task_name in enumerate(TASK_NAMES):
        matches = np.flatnonzero(task_ids == task_id)
        if len(matches) < episodes_per_task:
            raise ValueError(f"dataset has only {len(matches)} {task_name!r} episodes; need {episodes_per_task}")
        selected.extend(matches[:episodes_per_task])
    indices = np.asarray(sorted(selected), dtype=np.int64)
    keys = data.files if hasattr(data, "files") else data.keys()
    return {key: np.asarray(data[key])[indices] for key in keys}


def evaluate_belief_space_planning(
    model,
    checkpoint: dict[str, Any],
    data,
    scene: DiagnosticScene,
    arm,
    device,
    seed: int,
    view_count: int,
    candidate_count: int,
    outlier_probability: float,
    minimum_safe_fraction: float,
) -> dict[str, Any]:
    """Benchmark robust pose fusion and CVaR planning on synthetic re-observations.

    This evaluation deliberately keeps the primary benchmark untouched. It is
    an ablation-style side path whose clean poses are used only to synthesize
    repeatable observations and compute metrics, never as planner inputs.
    """
    if view_count < 2:
        raise ValueError("belief-space evaluation requires at least two observations")
    if candidate_count < 1:
        raise ValueError("candidate_count must be positive")
    torch, _, _, _ = require_torch()
    contexts = data["contexts"]
    task_ids = data["task_ids"] if "task_ids" in data else np.zeros(len(contexts), dtype=np.int8)
    clean_grasp = data["clean_grasp_poses"]
    clean_place = data["clean_place_poses"]
    observed_grasp = data["observed_grasp_poses"]
    observed_place = data["observed_place_poses"]
    context_mean = np.asarray(checkpoint["context_mean"], dtype=np.float64)
    context_std = np.asarray(checkpoint["context_std"], dtype=np.float64)
    support_threshold = float(checkpoint["context_support_threshold"])

    primary_pose_errors: list[float] = []
    fused_pose_errors: list[float] = []
    selected_errors: list[float] = []
    inlier_fractions: list[float] = []
    effective_view_counts: list[float] = []
    selected_safe_fractions: list[float] = []
    selected_clearance_cvars: list[float] = []
    selected_joint_travels: list[float] = []
    selected_accelerations: list[float] = []
    support_abstentions = 0
    accepted = 0
    rejection_reasons: Counter[str] = Counter()
    task_records: dict[str, dict[str, Any]] = {
        task_name: {
            "episodes": 0,
            "primary_errors": [],
            "fused_errors": [],
            "selected_errors": [],
            "inliers": [],
            "effective_views": [],
            "safe_fractions": [],
            "clearance_cvars": [],
            "accepted": 0,
            "support_abstentions": 0,
        }
        for task_name in TASK_NAMES
    }
    for index, source_context in enumerate(contexts):
        task_name = task_name_from_id(int(task_ids[index]))
        record = task_records[task_name]
        record["episodes"] += 1
        rng = np.random.default_rng(seed * 1_000_003 + index)
        xy_sigma_m, z_sigma_m, yaw_sigma_rad = counterfactual_sensor_scales(source_context, 1.0)
        grasp_views = synthesize_multiview_observations(
            clean_grasp[index],
            observed_grasp[index],
            view_count,
            xy_sigma_m,
            z_sigma_m,
            yaw_sigma_rad,
            outlier_probability,
            rng,
        )
        place_views = synthesize_multiview_observations(
            clean_place[index],
            observed_place[index],
            view_count,
            xy_sigma_m,
            z_sigma_m,
            yaw_sigma_rad,
            outlier_probability,
            rng,
        )
        grasp_belief = fuse_pose_belief(grasp_views, xy_sigma_m, z_sigma_m, yaw_sigma_rad)
        place_belief = fuse_pose_belief(place_views, xy_sigma_m, z_sigma_m, yaw_sigma_rad)
        primary_error = float(
            0.5
            * (
                np.linalg.norm(rigidify_pose(observed_grasp[index])[:3, 3] - rigidify_pose(clean_grasp[index])[:3, 3])
                + np.linalg.norm(rigidify_pose(observed_place[index])[:3, 3] - rigidify_pose(clean_place[index])[:3, 3])
            )
        )
        fused_error = float(
            0.5
            * (
                np.linalg.norm(grasp_belief.pose[:3, 3] - rigidify_pose(clean_grasp[index])[:3, 3])
                + np.linalg.norm(place_belief.pose[:3, 3] - rigidify_pose(clean_place[index])[:3, 3])
            )
        )
        primary_pose_errors.append(primary_error)
        fused_pose_errors.append(fused_error)
        record["primary_errors"].append(primary_error)
        record["fused_errors"].append(fused_error)
        inlier_fraction = min(grasp_belief.inlier_fraction, place_belief.inlier_fraction)
        effective_views = min(grasp_belief.effective_view_count, place_belief.effective_view_count)
        inlier_fractions.append(inlier_fraction)
        effective_view_counts.append(effective_views)
        record["inliers"].append(inlier_fraction)
        record["effective_views"].append(effective_views)

        # The policy can use the reduced belief uncertainty, while the support
        # gate deliberately retains the raw declared sensor scale. Multi-view
        # fusion must not turn an OOD camera condition into an in-distribution
        # one merely by averaging several observations.
        raw_confidence = float(source_context[18])
        fused_confidence = min(0.995, max(0.05, raw_confidence * inlier_fraction))
        fused_xy_sigma_m = max(grasp_belief.xy_sigma_m, place_belief.xy_sigma_m)
        fused_yaw_sigma_rad = max(grasp_belief.yaw_sigma_rad, place_belief.yaw_sigma_rad)
        policy_context = pose_to_context(
            grasp_belief.pose,
            place_belief.pose,
            fused_confidence,
            fused_xy_sigma_m,
            fused_yaw_sigma_rad,
            task_name,
        )
        support_context = pose_to_context(
            grasp_belief.pose,
            place_belief.pose,
            raw_confidence,
            xy_sigma_m,
            yaw_sigma_rad,
            task_name,
        )
        support_distance = float(np.linalg.norm((support_context.astype(np.float64) - context_mean) / context_std))
        if support_distance > support_threshold:
            support_abstentions += 1
            record["support_abstentions"] += 1
            rejection_reasons["belief observation outside calibrated training support"] += 1
            continue
        context = torch.tensor(policy_context, dtype=torch.float32)
        candidates = sample_trajectories(
            model,
            context,
            int(checkpoint["diffusion_steps"]),
            candidate_count,
            device,
        ) * ASSUMED_JOINT_LIMIT_RAD
        candidates = np.clip(candidates, -ASSUMED_JOINT_LIMIT_RAD, ASSUMED_JOINT_LIMIT_RAD)
        selection = select_cvar_belief_candidate(
            candidates,
            grasp_views,
            place_views,
            grasp_belief.pose,
            place_belief.pose,
            scene,
            task_name,
            minimum_safe_fraction,
        )
        rejection_reasons.update(selection["rejection_reasons"])
        if not selection["accepted"]:
            rejection_reasons[str(selection["reason"])] += 1
            continue
        trajectory = selection["trajectory"]
        endpoint_error = pose_position_error(arm, trajectory[GRASP_INDEX], clean_grasp[index])
        selected_errors.append(endpoint_error)
        record["selected_errors"].append(endpoint_error)
        selected_safe_fractions.append(float(selection["safe_fraction"]))
        selected_clearance_cvars.append(float(selection["clearance_cvar_m"]))
        record["safe_fractions"].append(float(selection["safe_fraction"]))
        record["clearance_cvars"].append(float(selection["clearance_cvar_m"]))
        features = selection["final_features"]
        selected_joint_travels.append(float(features["joint_travel_rad"]))
        selected_accelerations.append(float(features["mean_joint_acceleration_rad"]))
        accepted += 1
        record["accepted"] += 1

    task_metrics = {}
    for task_name, record in task_records.items():
        episodes = int(record["episodes"])
        task_metrics[task_name] = {
            "episodes": episodes,
            "primary_mean_pose_error_m": float(np.mean(record["primary_errors"])),
            "fused_mean_pose_error_m": float(np.mean(record["fused_errors"])),
            "selected_safe_rate": record["accepted"] / episodes,
            "support_abstention_rate": record["support_abstentions"] / episodes,
            "mean_inlier_fraction": float(np.mean(record["inliers"])),
            "mean_effective_view_count": float(np.mean(record["effective_views"])),
            "mean_scenario_safe_fraction": float(np.mean(record["safe_fractions"])) if record["safe_fractions"] else None,
            "mean_clearance_cvar_m": float(np.mean(record["clearance_cvars"])) if record["clearance_cvars"] else None,
            "selected_mean_grasp_position_error_m": float(np.mean(record["selected_errors"])) if record["selected_errors"] else None,
        }
    primary_mean = float(np.mean(primary_pose_errors))
    fused_mean = float(np.mean(fused_pose_errors))
    return {
        "enabled": True,
        "benchmark_only_synthetic_multiview_observations": True,
        "view_count": view_count,
        "candidate_count": candidate_count,
        "synthetic_outlier_probability": outlier_probability,
        "minimum_scenario_safe_fraction": minimum_safe_fraction,
        "task_metrics": task_metrics,
        "metrics": {
            "primary_mean_pose_error_m": primary_mean,
            "fused_mean_pose_error_m": fused_mean,
            "pose_error_reduction_fraction": (primary_mean - fused_mean) / primary_mean if primary_mean else 0.0,
            "mean_inlier_fraction": float(np.mean(inlier_fractions)),
            "mean_effective_view_count": float(np.mean(effective_view_counts)),
            "selected_safe_rate": accepted / len(contexts),
            "support_abstention_rate": support_abstentions / len(contexts),
            "mean_selected_scenario_safe_fraction": float(np.mean(selected_safe_fractions)) if selected_safe_fractions else None,
            "mean_selected_clearance_cvar_m": float(np.mean(selected_clearance_cvars)) if selected_clearance_cvars else None,
            "selected_mean_grasp_position_error_m": float(np.mean(selected_errors)) if selected_errors else None,
            "mean_selected_joint_travel_rad": float(np.mean(selected_joint_travels)) if selected_joint_travels else None,
            "mean_selected_joint_acceleration_rad": float(np.mean(selected_accelerations)) if selected_accelerations else None,
        },
        "rejection_reason_counts": dict(sorted(rejection_reasons.items())),
    }


def pose_position_error(model, joint_angles: np.ndarray, target_pose: np.ndarray) -> float:
    actual = fk_space(model.home_grasp_tcp, model.screw_axes, joint_angles)
    return float(np.linalg.norm(actual[:3, 3] - target_pose[:3, 3]))


def evaluate_model(
    checkpoint_path: Path,
    dataset_path: Path,
    output_path: Path,
    samples_per_context: int,
    abstain_dispersion_rad: float,
    seed: int,
    counterfactual_rollouts: int = 0,
    minimum_counterfactual_safe_fraction: float = 0.75,
    counterfactual_noise_fraction: float = 0.50,
    belief_views: int = 0,
    belief_candidate_count: int = 3,
    belief_outlier_probability: float = 0.15,
    belief_minimum_safe_fraction: float = 0.75,
    episodes_per_task: int = 0,
) -> dict[str, Any]:
    torch, _, _, _ = require_torch()
    if samples_per_context < 1 or not math.isfinite(abstain_dispersion_rad) or abstain_dispersion_rad <= 0.0:
        raise ValueError("invalid evaluation settings")
    if counterfactual_rollouts < 0:
        raise ValueError("counterfactual_rollouts must be non-negative")
    if not 0.0 < minimum_counterfactual_safe_fraction <= 1.0:
        raise ValueError("minimum_counterfactual_safe_fraction must be in (0, 1]")
    if not 0.0 < counterfactual_noise_fraction <= 1.0:
        raise ValueError("counterfactual_noise_fraction must be in (0, 1]")
    if belief_views not in (0, 1) and belief_views < 2:
        raise ValueError("belief_views must be zero, one, or at least two")
    if belief_views and belief_candidate_count < 1:
        raise ValueError("belief_candidate_count must be positive when belief planning is enabled")
    if not math.isfinite(belief_outlier_probability) or not 0.0 <= belief_outlier_probability < 1.0:
        raise ValueError("belief_outlier_probability must be in [0, 1)")
    if not 0.0 < belief_minimum_safe_fraction <= 1.0:
        raise ValueError("belief_minimum_safe_fraction must be in (0, 1]")
    if episodes_per_task < 0:
        raise ValueError("episodes_per_task must be non-negative")
    torch.manual_seed(seed)
    np.random.seed(seed)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    checkpoint = torch.load(checkpoint_path, map_location=device, weights_only=False)
    if checkpoint.get("architecture") != ARCHITECTURE_NAME:
        found = checkpoint.get("architecture", "legacy_checkpoint_without_architecture")
        raise ValueError(
            f"checkpoint architecture is {found!r}; retrain with {ARCHITECTURE_NAME!r} before evaluation"
        )
    if int(checkpoint.get("context_dim", -1)) != CONTEXT_DIM:
        raise ValueError("checkpoint context dimension does not match the active multi-task model")
    if tuple(checkpoint.get("task_names", ())) != TASK_NAMES:
        raise ValueError("checkpoint task suite does not match the active multi-task benchmark")
    if tuple(checkpoint.get("phase_names", ())) != PHASE_NAMES:
        raise ValueError("checkpoint process-phase graph does not match the active multi-task benchmark")
    model = build_denoiser(int(checkpoint["hidden_dim"]), int(checkpoint["diffusion_steps"])).to(device)
    model.load_state_dict(checkpoint["state_dict"])
    data = balanced_episode_subset(np.load(dataset_path), episodes_per_task)
    contexts = data["contexts"]
    task_ids = data["task_ids"] if "task_ids" in data else np.zeros(len(contexts), dtype=np.int8)
    if len(task_ids) != len(contexts):
        raise ValueError("dataset task_ids length does not match contexts")
    clean_grasp = data["clean_grasp_poses"]
    clean_place = data["clean_place_poses"]
    observed_grasp = data["observed_grasp_poses"]
    observed_place = data["observed_place_poses"]
    scene = load_diagnostic_scene()
    arm = load_default_model()
    context_mean = np.asarray(checkpoint["context_mean"], dtype=np.float64)
    context_std = np.asarray(checkpoint["context_std"], dtype=np.float64)
    support_threshold = float(checkpoint["context_support_threshold"])

    raw_errors: list[float] = []
    shielded_errors: list[float] = []
    raw_scene_safe = 0
    shielded_scene_safe = 0
    abstentions = 0
    ood_abstentions = 0
    dispersion_abstentions = 0
    projection_failures: list[str] = []
    examples: list[dict[str, object]] = []
    counterfactual_reason_counts: Counter[str] = Counter()
    counterfactual_safe_fractions: list[float] = []
    counterfactual_accepted = 0
    counterfactual_errors: list[float] = []
    task_records: dict[str, dict[str, Any]] = {
        task_name: {
            "episodes": 0,
            "raw_errors": [],
            "raw_safe": 0,
            "shielded_errors": [],
            "shielded_safe": 0,
            "abstentions": 0,
            "ood_abstentions": 0,
            "dispersion_abstentions": 0,
            "projection_failures": 0,
            "counterfactual_safe_fractions": [],
            "counterfactual_accepted": 0,
            "counterfactual_errors": [],
        }
        for task_name in TASK_NAMES
    }
    for index, context_np in enumerate(contexts):
        task_name = task_name_from_id(int(task_ids[index]))
        record = task_records[task_name]
        record["episodes"] += 1
        context = torch.tensor(context_np, dtype=torch.float32)
        samples = sample_trajectories(model, context, int(checkpoint["diffusion_steps"]), samples_per_context, device)
        dispersion = float(np.mean(np.linalg.norm(samples[:, GRASP_INDEX] - samples[:, GRASP_INDEX].mean(axis=0), axis=1)))
        prediction = np.mean(samples, axis=0) * ASSUMED_JOINT_LIMIT_RAD
        prediction = np.clip(prediction, -ASSUMED_JOINT_LIMIT_RAD, ASSUMED_JOINT_LIMIT_RAD)
        raw_errors.append(pose_position_error(arm, prediction[GRASP_INDEX], clean_grasp[index]))
        record["raw_errors"].append(raw_errors[-1])
        raw_review = review_tcp_positions(trajectory_tcp_positions(arm, prediction), scene)
        raw_scene_safe += int(raw_review.safe)
        record["raw_safe"] += int(raw_review.safe)
        if counterfactual_rollouts:
            consensus = evaluate_counterfactual_consensus(
                model,
                context_np,
                observed_grasp[index],
                observed_place[index],
                clean_grasp[index],
                task_name,
                scene,
                int(checkpoint["diffusion_steps"]),
                counterfactual_rollouts,
                minimum_counterfactual_safe_fraction,
                counterfactual_noise_fraction,
                context_mean,
                context_std,
                support_threshold,
                device,
                np.random.default_rng(seed * 100_003 + index),
            )
            counterfactual_safe_fractions.append(float(consensus["safe_fraction"]))
            record["counterfactual_safe_fractions"].append(float(consensus["safe_fraction"]))
            if consensus["accepted"]:
                counterfactual_accepted += 1
                record["counterfactual_accepted"] += 1
            if consensus["mean_projected_grasp_error_m"] is not None:
                error = float(consensus["mean_projected_grasp_error_m"])
                counterfactual_errors.append(error)
                record["counterfactual_errors"].append(error)
            counterfactual_reason_counts.update(consensus["rejection_reasons"])
        support_distance = float(np.linalg.norm((context_np.astype(np.float64) - context_mean) / context_std))
        if support_distance > support_threshold:
            abstentions += 1
            ood_abstentions += 1
            record["abstentions"] += 1
            record["ood_abstentions"] += 1
            if len(examples) < 8:
                examples.append(
                    {
                        "index": index,
                        "task": task_name,
                        "support_distance": support_distance,
                        "dispersion_rad": dispersion,
                        "decision": "abstain",
                        "reason": "context outside calibrated training support",
                    }
                )
            continue
        if dispersion > abstain_dispersion_rad:
            abstentions += 1
            dispersion_abstentions += 1
            record["abstentions"] += 1
            record["dispersion_abstentions"] += 1
            if len(examples) < 8:
                examples.append({"index": index, "task": task_name, "dispersion_rad": dispersion, "decision": "abstain", "reason": "sample dispersion above threshold"})
            continue
        projected, failure = project_with_constraints(prediction, observed_grasp[index], observed_place[index], scene, task_name)
        if projected is None:
            projection_failures.append(failure or "unknown")
            record["projection_failures"] += 1
            if len(examples) < 8:
                examples.append({"index": index, "task": task_name, "dispersion_rad": dispersion, "decision": "projection_failed", "reason": failure})
            continue
        shielded_errors.append(pose_position_error(arm, projected[GRASP_INDEX], clean_grasp[index]))
        shielded_scene_safe += 1
        record["shielded_errors"].append(shielded_errors[-1])
        record["shielded_safe"] += 1
        if len(examples) < 8:
            examples.append(
                {
                    "index": index,
                    "task": task_name,
                    "dispersion_rad": dispersion,
                    "decision": "projected_safe",
                    "raw_grasp_error_m": raw_errors[-1],
                    "shielded_grasp_error_m": shielded_errors[-1],
                }
            )
    task_metrics = {}
    for task_name, record in task_records.items():
        episodes = int(record["episodes"])
        if not episodes:
            continue
        task_metrics[task_name] = {
            "episodes": episodes,
            "raw_mean_grasp_position_error_m": float(np.mean(record["raw_errors"])),
            "raw_scene_safe_rate": record["raw_safe"] / episodes,
            "shielded_mean_grasp_position_error_m": float(np.mean(record["shielded_errors"])) if record["shielded_errors"] else None,
            "projected_safe_rate": record["shielded_safe"] / episodes,
            "projection_success_rate": record["shielded_safe"] / max(1, episodes - record["abstentions"]),
            "abstention_rate": record["abstentions"] / episodes,
            "ood_abstention_rate": record["ood_abstentions"] / episodes,
            "dispersion_abstention_rate": record["dispersion_abstentions"] / episodes,
            "projection_failure_rate": record["projection_failures"] / episodes,
        }
        if counterfactual_rollouts:
            task_metrics[task_name].update(
                {
                    "counterfactual_consensus_rate": record["counterfactual_accepted"] / episodes,
                    "counterfactual_mean_safe_rollout_rate": float(np.mean(record["counterfactual_safe_fractions"])),
                    "counterfactual_mean_projected_grasp_error_m": float(np.mean(record["counterfactual_errors"])) if record["counterfactual_errors"] else None,
                }
            )
    belief_space_planning = None
    if belief_views >= 2:
        belief_space_planning = evaluate_belief_space_planning(
            model,
            checkpoint,
            data,
            scene,
            arm,
            device,
            seed,
            belief_views,
            belief_candidate_count,
            belief_outlier_probability,
            belief_minimum_safe_fraction,
        )
    report = {
        "evaluation": "constraint_diffusion_digital_twin",
        "architecture": ARCHITECTURE_NAME,
        "context_tokens": list(CONTEXT_TOKEN_NAMES),
        "task_suite": list(TASK_NAMES),
        "task_metrics": task_metrics,
        "action_chunk_length": TRAJECTORY_STEPS,
        "model_parameters": int(checkpoint.get("model_parameters", 0)),
        "real_motion_enabled": False,
        "episodes": int(len(contexts)),
        "episodes_per_task": episodes_per_task or None,
        "seed": seed,
        "samples_per_context": samples_per_context,
        "abstain_dispersion_rad": abstain_dispersion_rad,
        "context_support_threshold": support_threshold,
        "counterfactual_consensus": {
            "enabled": bool(counterfactual_rollouts),
            "rollouts_per_episode": counterfactual_rollouts,
            "minimum_safe_fraction": minimum_counterfactual_safe_fraction,
            "perception_noise_fraction": counterfactual_noise_fraction,
            "accepted_rate": counterfactual_accepted / len(contexts) if counterfactual_rollouts else None,
            "mean_safe_rollout_rate": float(np.mean(counterfactual_safe_fractions)) if counterfactual_safe_fractions else None,
            "mean_projected_grasp_error_m": float(np.mean(counterfactual_errors)) if counterfactual_errors else None,
            "rejection_reason_counts": dict(sorted(counterfactual_reason_counts.items())),
        },
        "belief_space_planning": belief_space_planning,
        "metrics": {
            "raw_mean_grasp_position_error_m": float(np.mean(raw_errors)),
            "raw_scene_safe_rate": raw_scene_safe / len(contexts),
            "shielded_mean_grasp_position_error_m": float(np.mean(shielded_errors)) if shielded_errors else None,
            "shielded_scene_safe_rate": shielded_scene_safe / len(contexts),
            "projection_success_rate": shielded_scene_safe / max(1, len(contexts) - abstentions),
            "abstention_rate": abstentions / len(contexts),
            "ood_abstention_rate": ood_abstentions / len(contexts),
            "dispersion_abstention_rate": dispersion_abstentions / len(contexts),
        },
        "projection_failure_counts": {key: projection_failures.count(key) for key in sorted(set(projection_failures))},
        "examples": examples,
        "limitations": [
            "training data is generated from the checked-in POE kinematic model and a synthetic perception noise model",
            "constraint projection is a planning safety shield, not a measured real-arm control law",
            "the diagnostic scene checks TCP clearance only and does not replace full-link collision checking",
        ],
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return report


def parse_task_suite(value: str) -> tuple[str, ...]:
    tasks = tuple(item.strip() for item in value.split(",") if item.strip())
    if not tasks:
        raise argparse.ArgumentTypeError("task suite must include at least one task")
    if len(set(tasks)) != len(tasks):
        raise argparse.ArgumentTypeError("task suite must not contain duplicate task names")
    try:
        for task_name in tasks:
            task_id_from_name(task_name)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(str(exc)) from exc
    return tasks


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    generate = subparsers.add_parser("generate", help="generate counterfactual expert trajectories")
    generate.add_argument("--output", type=Path, required=True)
    generate.add_argument("--count", type=int, default=512)
    generate.add_argument("--seed", type=int, default=20260815)
    generate.add_argument("--xy-sigma-m", type=float, default=0.003)
    generate.add_argument("--z-sigma-m", type=float, default=0.002)
    generate.add_argument("--yaw-sigma-deg", type=float, default=2.0)
    generate.add_argument("--tasks", type=parse_task_suite, default=TASK_NAMES, help=f"comma-separated task profiles; defaults to {','.join(TASK_NAMES)}")
    generate.add_argument("--domain-randomization", action="store_true", help="randomize declared visual noise and confidence per episode")

    train = subparsers.add_parser("train", help="train the conditional diffusion trajectory prior")
    train.add_argument("--dataset", type=Path, required=True)
    train.add_argument("--checkpoint", type=Path, required=True)
    train.add_argument("--epochs", type=int, default=120)
    train.add_argument("--batch-size", type=int, default=64)
    train.add_argument("--hidden-dim", type=int, default=96)
    train.add_argument("--diffusion-steps", type=int, default=32)
    train.add_argument("--seed", type=int, default=20260815)

    evaluate = subparsers.add_parser("evaluate", help="evaluate raw samples against constraint-projected trajectories")
    evaluate.add_argument("--checkpoint", type=Path, required=True)
    evaluate.add_argument("--dataset", type=Path, required=True)
    evaluate.add_argument("--output", type=Path, required=True)
    evaluate.add_argument("--samples-per-context", type=int, default=3)
    evaluate.add_argument("--abstain-dispersion-rad", type=float, default=0.45)
    evaluate.add_argument("--seed", type=int, default=20260815)
    evaluate.add_argument("--counterfactual-rollouts", type=int, default=0, help="additional perception rollouts used for robust plan consensus")
    evaluate.add_argument("--minimum-counterfactual-safe-fraction", type=float, default=0.75)
    evaluate.add_argument("--counterfactual-noise-fraction", type=float, default=0.50)
    evaluate.add_argument("--belief-views", type=int, default=0, help="enable benchmark-only robust fusion with this many pose observations")
    evaluate.add_argument("--belief-candidate-count", type=int, default=3, help="diffusion candidates ranked by scenario-CVaR clearance")
    evaluate.add_argument("--belief-outlier-probability", type=float, default=0.15, help="synthetic re-observation outlier rate used only by the offline benchmark")
    evaluate.add_argument("--belief-minimum-safe-fraction", type=float, default=0.75)
    evaluate.add_argument("--episodes-per-task", type=int, default=0, help="deterministic balanced development subset; zero uses the full dataset")

    args = parser.parse_args()
    if args.command == "generate":
        result = generate_dataset(
            args.output,
            DatasetSettings(args.count, args.seed, args.xy_sigma_m, args.z_sigma_m, args.yaw_sigma_deg, args.tasks, args.domain_randomization),
            load_diagnostic_scene(),
        )
    elif args.command == "train":
        result = train_model(args.dataset, args.checkpoint, args.epochs, args.batch_size, args.hidden_dim, args.diffusion_steps, args.seed)
    else:
        result = evaluate_model(
            args.checkpoint,
            args.dataset,
            args.output,
            args.samples_per_context,
            args.abstain_dispersion_rad,
            args.seed,
            args.counterfactual_rollouts,
            args.minimum_counterfactual_safe_fraction,
            args.counterfactual_noise_fraction,
            args.belief_views,
            args.belief_candidate_count,
            args.belief_outlier_probability,
            args.belief_minimum_safe_fraction,
            args.episodes_per_task,
        )
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
