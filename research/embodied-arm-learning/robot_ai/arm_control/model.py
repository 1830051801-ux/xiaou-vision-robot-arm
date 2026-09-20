from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path

import numpy as np


DEFAULT_MODEL_PATH = Path(__file__).resolve().parent / "config" / "arm_model.json"


@dataclass(frozen=True)
class ArmModel:
    name: str
    joint_names: tuple[str, ...]
    screw_axes: np.ndarray
    home_grasp_tcp: np.ndarray
    home_tip_tcp: np.ndarray

    def __post_init__(self) -> None:
        if self.screw_axes.shape != (6, 6):
            raise ValueError(f"screw_axes must be 6x6, got {self.screw_axes.shape}")
        if not np.isfinite(self.screw_axes).all():
            raise ValueError("screw_axes must be finite")
        if len(self.joint_names) != 6:
            raise ValueError("exactly six joint names are required")
        if len(set(self.joint_names)) != 6:
            raise ValueError("joint names must be unique")
        angular_norms = np.linalg.norm(self.screw_axes[:3], axis=0)
        if not np.allclose(angular_norms, np.ones(6), atol=1e-8):
            raise ValueError("this six-axis model requires normalized revolute screw axes")
        for label, transform in (
            ("home_grasp_tcp", self.home_grasp_tcp),
            ("home_tip_tcp", self.home_tip_tcp),
        ):
            if transform.shape != (4, 4):
                raise ValueError(f"{label} must be 4x4")
            if not np.isfinite(transform).all():
                raise ValueError(f"{label} must be finite")
            if not np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0], atol=1e-12):
                raise ValueError(f"{label} has an invalid homogeneous bottom row")
            rotation = transform[:3, :3]
            if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-8):
                raise ValueError(f"{label} rotation must be orthonormal")
            if not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-8):
                raise ValueError(f"{label} rotation determinant must be +1")


def load_model(path: str | Path) -> ArmModel:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    axes_by_joint = np.asarray(data["space_screw_axes_columns"], dtype=np.float64)
    return ArmModel(
        name=str(data["name"]),
        joint_names=tuple(str(name) for name in data["joint_names"]),
        screw_axes=axes_by_joint.T.copy(),
        home_grasp_tcp=np.asarray(data["home_grasp_tcp"], dtype=np.float64),
        home_tip_tcp=np.asarray(data["home_tip_tcp"], dtype=np.float64),
    )


def load_default_model() -> ArmModel:
    return load_model(DEFAULT_MODEL_PATH)
