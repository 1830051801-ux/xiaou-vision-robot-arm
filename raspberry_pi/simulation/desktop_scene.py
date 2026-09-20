"""Deterministic, no-motion desktop task simulator.

This is deliberately a task-level simulator rather than a claim of rigid-body
physics. It reuses the calibrated coordinate conventions and produces the same
kind of structured observation that the camera/YOLO ROS path will publish. The
simulator is useful for policy wiring, failure classification, and repeatable
training data before a Gazebo or MuJoCo runtime is installed.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any

import numpy as np


OBJECT_CLASSES = ("bottle", "cup", "pen", "desktop_item", "tissue_pull", "cola")
PHASES = ("observe", "approach", "grasp", "lift", "pull", "abort")
STRATEGIES = ("top_down_pinch", "side_wrap", "pen_pinch", "flat_pick", "tissue_pull", "reject")
OBSERVATION_DIM = 46


OBJECT_SPECS: dict[str, dict[str, float | str]] = {
    "bottle": {
        "shape": "cylinder",
        "radius_m": 0.033,
        "height_m": 0.210,
        "mass_kg": 0.35,
        "friction": 0.55,
        "strategy": "side_wrap",
        "grasp_mode": "side",
    },
    "cup": {
        "shape": "cylinder",
        "radius_m": 0.045,
        "height_m": 0.090,
        "mass_kg": 0.20,
        "friction": 0.45,
        "strategy": "side_wrap",
        "grasp_mode": "side",
    },
    "pen": {
        "shape": "capsule",
        "radius_m": 0.006,
        "height_m": 0.140,
        "mass_kg": 0.015,
        "friction": 0.35,
        "strategy": "pen_pinch",
        "grasp_mode": "top",
    },
    "desktop_item": {
        "shape": "box",
        "radius_m": 0.035,
        "height_m": 0.030,
        "mass_kg": 0.10,
        "friction": 0.50,
        "strategy": "flat_pick",
        "grasp_mode": "top",
    },
    "tissue_pull": {
        "shape": "sheet_stack",
        "radius_m": 0.060,
        "height_m": 0.040,
        "mass_kg": 0.08,
        "friction": 0.25,
        "strategy": "tissue_pull",
        "grasp_mode": "top",
    },
    "cola": {
        "shape": "cylinder",
        "radius_m": 0.032,
        "height_m": 0.190,
        "mass_kg": 0.40,
        "friction": 0.55,
        "strategy": "side_wrap",
        "grasp_mode": "side",
    },
}


def _finite_vector(values: Any, size: int, label: str) -> np.ndarray:
    array = np.asarray(values, dtype=np.float32).reshape(-1)
    if array.size != size or not np.isfinite(array).all():
        raise ValueError(f"{label} must be a finite vector of length {size}")
    return array


@dataclass
class SceneObject:
    object_class: str
    position_m: np.ndarray
    yaw_rad: float = 0.0
    phase: str = "observe"
    held: bool = False

    def __post_init__(self) -> None:
        self.object_class = str(self.object_class).strip().lower()
        if self.object_class not in OBJECT_SPECS:
            raise ValueError(f"unknown object class: {self.object_class}")
        self.position_m = _finite_vector(self.position_m, 3, "position_m")
        if not math.isfinite(float(self.yaw_rad)):
            raise ValueError("yaw_rad must be finite")
        if self.phase not in PHASES:
            raise ValueError(f"unknown phase: {self.phase}")


@dataclass(frozen=True)
class ActionCandidate:
    strategy: str
    phase: str
    score: float
    gripper_open_m: float
    gripper_close_m: float
    target_xyz_m: tuple[float, float, float]
    pull_distance_m: float = 0.0
    reason: str = ""

    def as_dict(self) -> dict[str, Any]:
        return {
            "strategy": self.strategy,
            "phase": self.phase,
            "score": round(float(self.score), 5),
            "gripper_open_m": round(float(self.gripper_open_m), 5),
            "gripper_close_m": round(float(self.gripper_close_m), 5),
            "target_xyz_m": [round(float(value), 5) for value in self.target_xyz_m],
            "pull_distance_m": round(float(self.pull_distance_m), 5),
            "reason": self.reason,
        }


@dataclass
class DesktopScene:
    """One repeatable table scene with a single target object."""

    scenario: str = "bottle"
    seed: int = 20260809
    table_z_m: float = 0.0
    object_position_m: tuple[float, float, float] = (0.27, -0.12, 0.0)
    object_yaw_rad: float = 0.0
    step_index: int = 0
    rng: np.random.Generator = field(init=False, repr=False)
    target: SceneObject = field(init=False)

    def __post_init__(self) -> None:
        self.scenario = str(self.scenario).strip().lower()
        aliases = {"water_bottle": "bottle", "paper": "tissue_pull", "tissue": "tissue_pull", "desktop": "desktop_item"}
        self.scenario = aliases.get(self.scenario, self.scenario)
        if self.scenario not in OBJECT_SPECS:
            raise ValueError(f"unsupported scenario: {self.scenario}")
        if not math.isfinite(float(self.table_z_m)):
            raise ValueError("table_z_m must be finite")
        self.rng = np.random.default_rng(int(self.seed))
        position = _finite_vector(self.object_position_m, 3, "object_position_m")
        position[2] = float(self.table_z_m)
        self.target = SceneObject(self.scenario, position, float(self.object_yaw_rad))

    @property
    def spec(self) -> dict[str, float | str]:
        return OBJECT_SPECS[self.target.object_class]

    def _pseudo_yolo(self) -> dict[str, Any]:
        # This maps a table position into a stable synthetic camera observation.
        # Real runs replace this section with OpenCV/YOLO detections.
        x, y, _ = self.target.position_m
        u = 320.0 + (float(x) - 0.27) * 580.0
        v = 300.0 - (float(y) + 0.12) * 580.0
        size = max(8.0, float(self.spec["height_m"]) * 420.0)
        return {
            "class": self.target.object_class,
            "confidence": 0.92,
            "center_px": [round(u, 3), round(v, 3)],
            "bbox_px": [round(u - size / 2, 3), round(v - size / 2, 3), round(size, 3), round(size, 3)],
            "camera_resolution": [1920, 1080],
            "source": "synthetic_yolo_contract",
        }

    def candidates(self) -> list[ActionCandidate]:
        strategy = str(self.spec["strategy"])
        radius = float(self.spec["radius_m"])
        height = float(self.spec["height_m"])
        x, y, z = map(float, self.target.position_m)
        if self.target.object_class == "tissue_pull":
            return [
                ActionCandidate("tissue_pull", "pull", 0.91, 0.065, 0.035, (x, y, z + height), 0.08, "grip sheet edge then pull along table plane"),
                ActionCandidate("reject", "abort", 0.09, 0.0, 0.0, (x, y, z), 0.0, "do not lift tissue stack as a rigid object"),
            ]
        open_width = max(0.035, min(0.095, 2.0 * radius + 0.018))
        close_width = max(0.008, 2.0 * radius - 0.004)
        phase = "grasp" if self.target.phase in {"observe", "approach", "grasp"} else "lift"
        return [
            ActionCandidate(strategy, phase, 0.88, open_width, close_width, (x, y, z + height), 0.0, f"profile={self.target.object_class}"),
            ActionCandidate("reject", "abort", 0.12, 0.0, 0.0, (x, y, z), 0.0, "fallback until geometry and collision checks pass"),
        ]

    def observation(self) -> dict[str, Any]:
        spec = self.spec
        phase_index = PHASES.index(self.target.phase)
        return {
            "schema_version": 1,
            "timestamp_step": self.step_index,
            "source": "offline_desktop_scene",
            "scenario": self.scenario,
            "table": {"frame": "base_link", "z_m": float(self.table_z_m), "normal": [0.0, 0.0, 1.0]},
            "yolo": self._pseudo_yolo(),
            "target": {
                "class": self.target.object_class,
                "position_base_m": self.target.position_m.tolist(),
                "yaw_rad": float(self.target.yaw_rad),
                "shape": spec["shape"],
                "radius_m": float(spec["radius_m"]),
                "height_m": float(spec["height_m"]),
                "mass_kg": float(spec["mass_kg"]),
                "friction": float(spec["friction"]),
                "held": self.target.held,
            },
            "arm": {
                "joint_position_rad": [0.0] * 6,
                "joint_velocity_rad_s": [0.0] * 6,
                "joint_online": [True] * 6,
                "gripper_open_m": 0.095,
                "gripper_force_pct": 0.0,
            },
            "phase": {"name": self.target.phase, "index": phase_index},
            "perception": {
                "detection_fresh": True,
                "calibration_valid": True,
                "source_status": "offline_simulation",
            },
            "safety": {
                "motion_enabled": False,
                "hardware_ready": False,
                "collision_free": True,
                "feedback_verified": False,
            },
            "action_candidates": [candidate.as_dict() for candidate in self.candidates()],
        }

    def observation_vector(self) -> np.ndarray:
        data = self.observation()
        target = data["target"]
        yolo = data["yolo"]
        vector: list[float] = []
        vector.extend(1.0 if target["class"] == name else 0.0 for name in OBJECT_CLASSES)
        vector.extend((yolo["center_px"][0] / 1920.0, yolo["center_px"][1] / 1080.0, yolo["confidence"]))
        vector.extend((*target["position_base_m"], target["yaw_rad"]))
        vector.extend((target["radius_m"], target["height_m"], target["mass_kg"], target["friction"]))
        vector.extend(data["arm"]["joint_position_rad"])
        vector.extend(data["arm"]["joint_velocity_rad_s"])
        vector.extend(1.0 if online else 0.0 for online in data["arm"]["joint_online"])
        vector.extend(1.0 if self.target.phase == phase else 0.0 for phase in PHASES)
        vector.extend(
            1.0 if data["safety"][name] else 0.0
            for name in ("motion_enabled", "hardware_ready", "collision_free")
        )
        vector.extend((data["arm"]["gripper_open_m"], data["arm"]["gripper_force_pct"]))
        result = np.asarray(vector, dtype=np.float32)
        if result.size != OBSERVATION_DIM:
            raise RuntimeError(f"observation dimension drift: {result.size} != {OBSERVATION_DIM}")
        return result

    def apply(self, strategy: str) -> dict[str, Any]:
        """Advance only the simulated task state; never calls ROS hardware."""
        strategy = str(strategy).strip().lower()
        expected = str(self.spec["strategy"])
        if strategy == "reject":
            self.target.phase = "abort"
            return {"success": False, "stage": "abort", "reason": "policy_rejected"}
        if self.target.object_class == "tissue_pull":
            valid = strategy == "tissue_pull"
            self.target.phase = "pull" if valid else "abort"
            return {"success": valid, "stage": self.target.phase, "reason": "pull_candidate" if valid else "wrong_strategy"}
        valid = strategy == expected
        if not valid:
            self.target.phase = "abort"
            return {"success": False, "stage": "abort", "reason": "wrong_grasp_strategy"}
        self.target.phase = "lift"
        self.target.held = True
        return {"success": True, "stage": "lift", "reason": "simulated_grasp_accepted"}

    def reset(self) -> None:
        self.step_index = 0
        self.target.phase = "observe"
        self.target.held = False

    def step(self) -> dict[str, Any]:
        self.step_index += 1
        return self.observation()


def run_scenario(scenario: str, seed: int = 20260809) -> dict[str, Any]:
    scene = DesktopScene(scenario=scenario, seed=seed)
    observation = scene.observation()
    chosen = scene.candidates()[0]
    result = scene.apply(chosen.strategy)
    return {
        "scenario": scenario,
        "seed": seed,
        "observation": observation,
        "chosen_candidate": chosen.as_dict(),
        "result": result,
        "observation_vector_dim": int(scene.observation_vector().size),
    }
