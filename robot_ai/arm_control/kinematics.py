from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass

import numpy as np

from .lie import adjoint, matrix_exp6, matrix_log6, se3_to_vec, transform_inverse, vec_to_se3


def _validate_transform(transform: np.ndarray, label: str) -> np.ndarray:
    transform = np.asarray(transform, dtype=np.float64)
    if transform.shape != (4, 4):
        raise ValueError(f"{label} transform must be 4x4")
    if not np.isfinite(transform).all():
        raise ValueError(f"{label} transform must be finite")
    if not np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0], atol=1e-10):
        raise ValueError(f"{label} transform has an invalid homogeneous bottom row")
    rotation = transform[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-8):
        raise ValueError(f"{label} rotation must be orthonormal")
    if not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-8):
        raise ValueError(f"{label} rotation determinant must be +1")
    return transform


def _validate_inputs(home: np.ndarray, screw_axes: np.ndarray, joint_angles: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    home = _validate_transform(home, "home")
    screw_axes = np.asarray(screw_axes, dtype=np.float64)
    joint_angles = np.asarray(joint_angles, dtype=np.float64).reshape(-1)
    if screw_axes.shape != (6, joint_angles.size):
        raise ValueError(f"screw_axes must be 6xN, got {screw_axes.shape}")
    if not np.isfinite(screw_axes).all():
        raise ValueError("screw axes must be finite")
    if not np.isfinite(joint_angles).all():
        raise ValueError("joint angles must be finite")
    return home, screw_axes, joint_angles


def fk_space(home: np.ndarray, screw_axes: np.ndarray, joint_angles: np.ndarray) -> np.ndarray:
    home, screw_axes, joint_angles = _validate_inputs(home, screw_axes, joint_angles)
    transform = home.copy()
    for index in range(joint_angles.size - 1, -1, -1):
        transform = matrix_exp6(vec_to_se3(screw_axes[:, index] * joint_angles[index])) @ transform
    return transform


def jacobian_space(screw_axes: np.ndarray, joint_angles: np.ndarray) -> np.ndarray:
    screw_axes = np.asarray(screw_axes, dtype=np.float64)
    joint_angles = np.asarray(joint_angles, dtype=np.float64).reshape(-1)
    if screw_axes.shape != (6, joint_angles.size):
        raise ValueError(f"screw_axes must be 6xN, got {screw_axes.shape}")
    jacobian = screw_axes.copy()
    transform = np.eye(4, dtype=np.float64)
    for index in range(1, joint_angles.size):
        transform = transform @ matrix_exp6(vec_to_se3(screw_axes[:, index - 1] * joint_angles[index - 1]))
        jacobian[:, index] = adjoint(transform) @ screw_axes[:, index]
    return jacobian


@dataclass(frozen=True)
class IKResult:
    joint_angles: np.ndarray
    converged: bool
    iterations: int
    orientation_error_rad: float
    position_error_m: float


def ik_space_multistart(
    home: np.ndarray,
    screw_axes: np.ndarray,
    target: np.ndarray,
    initial_seeds: Iterable[np.ndarray],
    preferred_angles: np.ndarray | None = None,
    **kwargs: object,
) -> IKResult:
    """Solve IK from several bounded seeds and return the best result.

    This is a deterministic local-search fallback for targets near a solver
    singularity or joint-limit boundary. It does not relax limits or change
    tolerances. Converged candidates are preferred; otherwise the candidate
    with the smallest normalized residual is returned for diagnostics.
    """
    seeds = [np.asarray(seed, dtype=np.float64).reshape(-1) for seed in initial_seeds]
    if not seeds:
        raise ValueError("initial_seeds must contain at least one seed")
    preferred = None if preferred_angles is None else np.asarray(preferred_angles, dtype=np.float64).reshape(-1)
    if preferred is not None and preferred.size != seeds[0].size:
        raise ValueError("preferred_angles must match the seed joint count")
    results = [ik_space(home, screw_axes, target, seed, **kwargs) for seed in seeds]

    def score(result: IKResult) -> tuple[bool, float, float, int]:
        orientation_tolerance = float(kwargs.get("orientation_tolerance_rad", 1e-4))
        position_tolerance = float(kwargs.get("position_tolerance_m", 1e-4))
        normalized = max(
            result.orientation_error_rad / orientation_tolerance,
            result.position_error_m / position_tolerance,
        )
        distance = 0.0 if preferred is None else float(np.linalg.norm(result.joint_angles - preferred))
        # Once a candidate satisfies tolerances, continuity is more useful
        # than chasing a slightly smaller residual on a different branch.
        return (not result.converged, distance if result.converged else normalized, normalized, result.iterations)

    return min(results, key=score)


def ik_space(
    home: np.ndarray,
    screw_axes: np.ndarray,
    target: np.ndarray,
    initial_angles: np.ndarray,
    *,
    orientation_tolerance_rad: float = 1e-4,
    position_tolerance_m: float = 1e-4,
    damping: float = 1e-4,
    max_iterations: int = 100,
    joint_lower: np.ndarray | None = None,
    joint_upper: np.ndarray | None = None,
    max_step_rad: float = 0.25,
    line_search_steps: int = 8,
) -> IKResult:
    home, screw_axes, angles = _validate_inputs(home, screw_axes, initial_angles)
    target = _validate_transform(target, "target")
    if orientation_tolerance_rad <= 0.0 or position_tolerance_m <= 0.0:
        raise ValueError("IK tolerances must be positive")
    if damping <= 0.0 or max_step_rad <= 0.0:
        raise ValueError("IK damping and max_step_rad must be positive")
    if not isinstance(max_iterations, int) or max_iterations < 1:
        raise ValueError("max_iterations must be a positive integer")
    if not isinstance(line_search_steps, int) or line_search_steps < 1:
        raise ValueError("line_search_steps must be a positive integer")

    lower: np.ndarray | None = None
    upper: np.ndarray | None = None
    if (joint_lower is None) != (joint_upper is None):
        raise ValueError("joint_lower and joint_upper must be supplied together")
    if joint_lower is not None and joint_upper is not None:
        lower = np.asarray(joint_lower, dtype=np.float64).reshape(-1)
        upper = np.asarray(joint_upper, dtype=np.float64).reshape(-1)
        if lower.size != angles.size or upper.size != angles.size:
            raise ValueError("joint limits must match the joint count")
        if not np.isfinite(lower).all() or not np.isfinite(upper).all():
            raise ValueError("joint limits must be finite measured values")
        if np.any(lower >= upper):
            raise ValueError("every joint lower limit must be below its upper limit")
        if np.any(angles < lower) or np.any(angles > upper):
            raise ValueError("initial angles exceed the supplied joint limits")
    angles = angles.copy()

    for iteration in range(max_iterations + 1):
        current = fk_space(home, screw_axes, angles)
        body_error = se3_to_vec(matrix_log6(transform_inverse(current) @ target))
        space_error = adjoint(current) @ body_error
        orientation_error = float(np.linalg.norm(space_error[:3]))
        position_error = float(np.linalg.norm(space_error[3:]))
        if orientation_error <= orientation_tolerance_rad and position_error <= position_tolerance_m:
            return IKResult(angles, True, iteration, orientation_error, position_error)
        if iteration == max_iterations:
            break
        jacobian = jacobian_space(screw_axes, angles)
        regularizer = (damping * damping) * np.eye(6, dtype=np.float64)
        delta = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + regularizer, space_error)
        largest_step = float(np.max(np.abs(delta)))
        if largest_step > max_step_rad:
            delta *= max_step_rad / largest_step

        current_error_norm = float(np.linalg.norm(space_error))
        accepted = False
        scale = 1.0
        for _ in range(line_search_steps):
            candidate = angles + scale * delta
            if lower is not None and upper is not None:
                candidate = np.clip(candidate, lower, upper)
            candidate_pose = fk_space(home, screw_axes, candidate)
            candidate_body_error = se3_to_vec(matrix_log6(transform_inverse(candidate_pose) @ target))
            candidate_error = adjoint(candidate_pose) @ candidate_body_error
            if np.linalg.norm(candidate_error) < current_error_norm:
                angles = candidate
                accepted = True
                break
            scale *= 0.5
        if not accepted:
            break

    current = fk_space(home, screw_axes, angles)
    body_error = se3_to_vec(matrix_log6(transform_inverse(current) @ target))
    space_error = adjoint(current) @ body_error
    return IKResult(
        angles,
        False,
        min(iteration, max_iterations),
        float(np.linalg.norm(space_error[:3])),
        float(np.linalg.norm(space_error[3:])),
    )
