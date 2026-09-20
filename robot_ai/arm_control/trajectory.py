from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class JointLimits:
    position_min: np.ndarray
    position_max: np.ndarray
    velocity_max: np.ndarray
    acceleration_max: np.ndarray

    def __post_init__(self) -> None:
        arrays = [
            np.asarray(self.position_min, dtype=np.float64).reshape(-1),
            np.asarray(self.position_max, dtype=np.float64).reshape(-1),
            np.asarray(self.velocity_max, dtype=np.float64).reshape(-1),
            np.asarray(self.acceleration_max, dtype=np.float64).reshape(-1),
        ]
        size = arrays[0].size
        if size == 0 or any(array.size != size for array in arrays):
            raise ValueError("all joint-limit arrays must have the same non-zero length")
        if not all(np.isfinite(array).all() for array in arrays):
            raise ValueError("joint limits must be measured finite values")
        if np.any(arrays[0] >= arrays[1]):
            raise ValueError("every position_min must be less than position_max")
        if np.any(arrays[2] <= 0.0) or np.any(arrays[3] <= 0.0):
            raise ValueError("velocity and acceleration limits must be positive")
        object.__setattr__(self, "position_min", arrays[0])
        object.__setattr__(self, "position_max", arrays[1])
        object.__setattr__(self, "velocity_max", arrays[2])
        object.__setattr__(self, "acceleration_max", arrays[3])


@dataclass(frozen=True)
class TrajectoryPoint:
    time_from_start_s: float
    positions: np.ndarray
    velocities: np.ndarray
    accelerations: np.ndarray


def _quintic_scaling(tau: float) -> tuple[float, float, float]:
    s = 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5
    ds_dtau = 30.0 * tau**2 - 60.0 * tau**3 + 30.0 * tau**4
    d2s_dtau2 = 60.0 * tau - 180.0 * tau**2 + 120.0 * tau**3
    return s, ds_dtau, d2s_dtau2


def plan_quintic_joint_trajectory(
    start: np.ndarray,
    goal: np.ndarray,
    limits: JointLimits,
    *,
    sample_period_s: float = 0.01,
    minimum_duration_s: float = 0.25,
) -> list[TrajectoryPoint]:
    start = np.asarray(start, dtype=np.float64).reshape(-1)
    goal = np.asarray(goal, dtype=np.float64).reshape(-1)
    if start.size != goal.size or start.size != limits.position_min.size:
        raise ValueError("start, goal, and limits must use the same joint count")
    if (
        not math.isfinite(sample_period_s)
        or not math.isfinite(minimum_duration_s)
        or sample_period_s <= 0.0
        or minimum_duration_s <= 0.0
    ):
        raise ValueError("sample period and minimum duration must be finite and positive")
    for label, values in (("start", start), ("goal", goal)):
        if not np.isfinite(values).all():
            raise ValueError(f"{label} joint values must be finite")
        if np.any(values < limits.position_min) or np.any(values > limits.position_max):
            raise ValueError(f"{label} joint values exceed measured position limits")

    delta = goal - start
    velocity_time = np.max(1.875 * np.abs(delta) / limits.velocity_max)
    acceleration_time = np.max(np.sqrt((10.0 / math.sqrt(3.0)) * np.abs(delta) / limits.acceleration_max))
    duration = max(float(velocity_time), float(acceleration_time), minimum_duration_s)
    sample_count = max(2, int(math.ceil(duration / sample_period_s)) + 1)
    times = np.linspace(0.0, duration, sample_count)
    points: list[TrajectoryPoint] = []
    for time_s in times:
        tau = float(time_s / duration)
        s, ds_dtau, d2s_dtau2 = _quintic_scaling(tau)
        points.append(
            TrajectoryPoint(
                time_from_start_s=float(time_s),
                positions=start + s * delta,
                velocities=(ds_dtau / duration) * delta,
                accelerations=(d2s_dtau2 / (duration * duration)) * delta,
            )
        )
    return points
