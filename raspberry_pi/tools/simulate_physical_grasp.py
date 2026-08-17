#!/usr/bin/env python3
"""Run an offline MuJoCo grasp regression with the checked-in CAD model.

    The articulated arm is generated from the ROS2 Xacro joint origins, axes, and
    collision meshes.  If an optional CAD world-model URDF is available, it is
    parsed and its mesh references are audited.  A table and one object are
    simulated as rigid bodies.
    The run uses the existing POE IK and quintic trajectory planner, then plays
    the robot trajectory exactly through MuJoCo's collision/contact scene while
    the object remains a dynamic rigid body.

    This is a repeatable engineering regression, not a hardware-success claim.
    The robot uses provisional inertial density, but its joint planning uses the
    current no-motion Pi/F407 candidate envelope and recorded ready pose.  This
    prevents an offline result from silently relying on a wider ``+/- pi``
    envelope than the controller is currently configured to accept.  No serial,
    CAN, ROS hardware, or motion command is opened.
"""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import shutil
import sys
import tempfile
import xml.etree.ElementTree as ET
from typing import Any

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from robot_ai.arm_control import (  # noqa: E402
    JointLimits,
    ik_space_multistart,
    load_default_model,
    plan_quintic_joint_trajectory,
)
from robot_ai.decision.transformer_policy import (  # noqa: E402
    DecisionTransformerPolicy,
    SafetyGate,
    TASK_TO_ID,
)
from simulation.desktop_scene import OBJECT_CLASSES, OBJECT_SPECS, DesktopScene  # noqa: E402


try:
    import mujoco  # type: ignore
except Exception as exc:  # pragma: no cover - optional desktop dependency
    mujoco = None  # type: ignore[assignment]
    _MUJOCO_IMPORT_ERROR = exc
else:
    _MUJOCO_IMPORT_ERROR = None


WORKSPACE_ROOT = PROJECT_ROOT.parents[1]
XACRO_PATH = PROJECT_ROOT / "ros2_ws" / "src" / "xiaou_arm_description" / "urdf" / "xiaou_arm_display.urdf.xacro"
COLLISION_ROOT = PROJECT_ROOT / "ros2_ws" / "src" / "xiaou_arm_description" / "meshes" / "collision"
DEFAULT_WORLD_URDF_PATH = WORKSPACE_ROOT / "robot_geometry_renders" / "world_model" / "robot_world_model.urdf"
DEFAULT_OUTPUT = PROJECT_ROOT / "runtime" / "simulations" / "physical_grasp_regression_20260809.json"
DEFAULT_ASSET_DIR = PROJECT_ROOT / "runtime" / "simulations" / "physical_assets"
DEFAULT_ASCII_ASSET_DIR = Path("D:/xiaou_mujoco_runtime")
DEFAULT_FP32 = PROJECT_ROOT / "runtime" / "decision" / "transformer_policy_safety_seed20260832.pt"
DEFAULT_INT8 = PROJECT_ROOT / "runtime" / "decision" / "transformer_policy_safety_final_20260813.int8.onnx"
DEFAULT_HARDWARE_CONFIG = (
    PROJECT_ROOT / "robot_ai" / "arm_control" / "config" / "hardware_calibration_candidate_20260811.json"
)

TABLE_TOP_M = -0.020
GRASP_Z_OFFSET_M = 0.050
RELEASE_CLEARANCE_M = 0.080
SIM_TIMESTEP_S = 0.002
@dataclass(frozen=True)
class JointDescription:
    name: str
    child: str
    xyz: tuple[float, float, float]
    rpy: tuple[float, float, float]
    axis: tuple[float, float, float]


@dataclass(frozen=True)
class RobotDescription:
    link_meshes: dict[str, tuple[str, ...]]
    joints: tuple[JointDescription, ...]
    grasp_tcp_xyz: tuple[float, float, float]
    grasp_tcp_rpy: tuple[float, float, float]
    collision_meshes: tuple[Path, ...]
    world_meshes: tuple[Path, ...]
    world_urdf: Path | None


@dataclass(frozen=True)
class SimulationMotionProfile:
    """No-motion projection of the current Pi/F407 joint envelope."""

    joint_limits: JointLimits
    ready_pose_rad: np.ndarray
    source_path: Path
    source_schema_version: int | None


def _six_config_values(payload: dict[str, Any], key: str) -> np.ndarray:
    values = payload.get(key)
    if not isinstance(values, list) or len(values) != 6:
        raise ValueError(f"hardware config {key} must contain six values")
    result = np.asarray(values, dtype=np.float64).reshape(-1)
    if not np.isfinite(result).all():
        raise ValueError(f"hardware config {key} must contain finite values")
    return result


def load_simulation_motion_profile(path: Path) -> SimulationMotionProfile:
    """Load a measured/candidate envelope without enabling any hardware path.

    The physical regression uses only joint limits and the recorded ready pose.
    It never calls the hardware readiness gate and refuses a configuration that
    claims motion is enabled, so the simulation cannot become a back door into
    a live session.
    """

    payload = json.loads(path.read_text(encoding="utf-8"))
    if payload.get("motion_enabled") is not False:
        raise ValueError("simulation hardware config must retain motion_enabled=false")
    limits = JointLimits(
        position_min=_six_config_values(payload, "position_min_rad"),
        position_max=_six_config_values(payload, "position_max_rad"),
        velocity_max=_six_config_values(payload, "velocity_max_rad_s"),
        acceleration_max=_six_config_values(payload, "acceleration_max_rad_s2"),
    )
    ready_pose = _six_config_values(payload, "ready_pose_rad")
    if np.any(ready_pose < limits.position_min) or np.any(ready_pose > limits.position_max):
        raise ValueError("hardware config ready_pose_rad exceeds its position limits")
    schema_version = payload.get("schema_version")
    return SimulationMotionProfile(
        joint_limits=limits,
        ready_pose_rad=ready_pose,
        source_path=path.resolve(),
        source_schema_version=schema_version if isinstance(schema_version, int) else None,
    )


def _require_mujoco() -> Any:
    if mujoco is None:
        raise RuntimeError(
            "MuJoCo is required for physical regression. Install it in the desktop venv "
            f"with `python -m pip install mujoco`. Import error: {_MUJOCO_IMPORT_ERROR}"
        )
    return mujoco


def _tag(element: ET.Element) -> str:
    return element.tag.rsplit("}", 1)[-1]


def _vector(element: ET.Element | None, attribute: str, default: tuple[float, float, float]) -> tuple[float, float, float]:
    if element is None:
        return default
    raw = element.attrib.get(attribute)
    if raw is None:
        return default
    values = tuple(float(value) for value in raw.split())
    if len(values) != 3 or not np.isfinite(values).all():
        raise ValueError(f"invalid {attribute}: {raw}")
    return values  # type: ignore[return-value]


def _rpy_matrix(rpy: tuple[float, float, float]) -> np.ndarray:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def _matrix_to_quat(rotation: np.ndarray) -> tuple[float, float, float, float]:
    """Return MuJoCo's wxyz quaternion from a proper rotation matrix."""
    rotation = np.asarray(rotation, dtype=np.float64)
    trace = float(np.trace(rotation))
    if trace > 0.0:
        root = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * root
        x = (rotation[2, 1] - rotation[1, 2]) / root
        y = (rotation[0, 2] - rotation[2, 0]) / root
        z = (rotation[1, 0] - rotation[0, 1]) / root
    else:
        diagonal = np.diag(rotation)
        index = int(np.argmax(diagonal))
        if index == 0:
            root = math.sqrt(max(1e-15, 1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2])) * 2.0
            x = 0.25 * root
            y = (rotation[0, 1] + rotation[1, 0]) / root
            z = (rotation[0, 2] + rotation[2, 0]) / root
            w = (rotation[2, 1] - rotation[1, 2]) / root
        elif index == 1:
            root = math.sqrt(max(1e-15, 1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2])) * 2.0
            x = (rotation[0, 1] + rotation[1, 0]) / root
            y = 0.25 * root
            z = (rotation[1, 2] + rotation[2, 1]) / root
            w = (rotation[0, 2] - rotation[2, 0]) / root
        else:
            root = math.sqrt(max(1e-15, 1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1])) * 2.0
            x = (rotation[0, 2] + rotation[2, 0]) / root
            y = (rotation[1, 2] + rotation[2, 1]) / root
            z = 0.25 * root
            w = (rotation[1, 0] - rotation[0, 1]) / root
    quaternion = np.asarray((w, x, y, z), dtype=np.float64)
    quaternion /= np.linalg.norm(quaternion)
    return tuple(float(value) for value in quaternion)


def _fmt(values: tuple[float, ...] | list[float] | np.ndarray) -> str:
    return " ".join(f"{float(value):.12g}" for value in values)


def _relative_mesh_path(xml_path: Path, mesh_path: Path) -> str:
    return Path(os.path.relpath(mesh_path.resolve(), xml_path.parent.resolve())).as_posix()


def parse_robot_description(world_urdf: Path | None = None) -> RobotDescription:
    if not XACRO_PATH.is_file():
        raise FileNotFoundError(XACRO_PATH)
    root = ET.parse(XACRO_PATH).getroot()
    link_meshes: dict[str, tuple[str, ...]] = {}
    for link in root.findall("link"):
        values: list[str] = []
        for child in list(link):
            if _tag(child) == "mesh_geometry":
                path = child.attrib.get("path")
                if path:
                    values.append(path)
        if values:
            link_meshes[link.attrib["name"]] = tuple(values)
    joints: list[JointDescription] = []
    for joint in root.findall("joint"):
        name = joint.attrib.get("name", "")
        if not name.startswith("joint_") or not name[-1:].isdigit():
            continue
        origin = next((item for item in list(joint) if _tag(item) == "origin"), None)
        axis = next((item for item in list(joint) if _tag(item) == "axis"), None)
        parent = next((item for item in list(joint) if _tag(item) == "parent"), None)
        child = next((item for item in list(joint) if _tag(item) == "child"), None)
        if child is None or parent is None:
            raise ValueError(f"joint {name} is missing parent/child")
        joints.append(
            JointDescription(
                name=name,
                child=child.attrib["link"],
                xyz=_vector(origin, "xyz", (0.0, 0.0, 0.0)),
                rpy=_vector(origin, "rpy", (0.0, 0.0, 0.0)),
                axis=_vector(axis, "xyz", (0.0, 0.0, 1.0)),
            )
        )
    joints.sort(key=lambda item: int(item.name.rsplit("_", 1)[-1]))
    if len(joints) != 6:
        raise ValueError(f"expected six revolute joints, found {len(joints)}")
    tcp_joint = next(item for item in root.findall("joint") if item.attrib.get("name") == "link_6_to_grasp_tcp")
    tcp_origin = next((item for item in list(tcp_joint) if _tag(item) == "origin"), None)
    grasp_tcp_xyz = _vector(tcp_origin, "xyz", (0.0, 0.0, 0.0))
    grasp_tcp_rpy = _vector(tcp_origin, "rpy", (0.0, 0.0, 0.0))

    expected = {
        "base_link": ("base_body",),
        "link_1": ("joint_1_body",),
        "link_2": ("link_2_body",),
        "link_3": ("link_3_body",),
        "link_4": ("joint_5_body",),
        "link_5": ("joint_6_body",),
        "link_6": ("wrist_connector", "gripper"),
    }
    if link_meshes != expected:
        raise ValueError(f"Xacro mesh mapping drift: {link_meshes!r}")
    collision_meshes = tuple((COLLISION_ROOT / f"{name}.stl").resolve() for names in expected.values() for name in names)
    missing = [path for path in collision_meshes if not path.is_file()]
    if missing:
        raise FileNotFoundError(f"missing collision meshes: {missing}")

    world_meshes: list[Path] = []
    selected_world_urdf: Path | None = None
    candidate_world_urdf = world_urdf.resolve() if world_urdf is not None else DEFAULT_WORLD_URDF_PATH.resolve()
    if candidate_world_urdf.is_file():
        selected_world_urdf = candidate_world_urdf
        world_mesh_root = candidate_world_urdf.parent
        world_root = ET.parse(candidate_world_urdf).getroot()
        for mesh in world_root.iter():
            if _tag(mesh) != "mesh" or "filename" not in mesh.attrib:
                continue
            reference = Path(mesh.attrib["filename"])
            candidates = (world_mesh_root / reference, world_mesh_root.parent / reference)
            resolved = next((candidate.resolve() for candidate in candidates if candidate.is_file()), candidates[0].resolve())
            world_meshes.append(resolved)
        if not world_meshes or any(not path.is_file() for path in world_meshes):
            raise FileNotFoundError(f"world model mesh references invalid: {world_meshes}")
    elif world_urdf is not None:
        raise FileNotFoundError(candidate_world_urdf)
    return RobotDescription(
        link_meshes=link_meshes,
        joints=tuple(joints),
        grasp_tcp_xyz=grasp_tcp_xyz,
        grasp_tcp_rpy=grasp_tcp_rpy,
        collision_meshes=collision_meshes,
        world_meshes=tuple(world_meshes),
        world_urdf=selected_world_urdf,
    )


def _object_geom(spec: dict[str, float | str]) -> str:
    shape = str(spec["shape"])
    radius = float(spec["radius_m"])
    height = float(spec["height_m"])
    mass = float(spec["mass_kg"])
    if shape == "box" or shape == "sheet_stack":
        size = f"{radius:.8g} {radius:.8g} {height / 2.0:.8g}"
        return f'<geom name="target_geom" type="box" size="{size}" mass="{mass:.8g}" friction="0.7 0.02 0.001"/>'
    if shape == "capsule":
        half_length = max(0.001, height / 2.0 - radius)
        return f'<geom name="target_geom" type="capsule" size="{radius:.8g} {half_length:.8g}" mass="{mass:.8g}" friction="0.6 0.02 0.001"/>'
    return f'<geom name="target_geom" type="cylinder" size="{radius:.8g} {height / 2.0:.8g}" mass="{mass:.8g}" friction="0.7 0.02 0.001"/>'


def build_mjcf(
    description: RobotDescription,
    scenario: str,
    output_dir: Path,
    object_spec: dict[str, float | str] | None = None,
    *,
    table_top_m: float = TABLE_TOP_M,
) -> Path:
    """Generate one scenario-specific MJCF file from the checked-in model."""
    output_dir.mkdir(parents=True, exist_ok=True)
    xml_path = output_dir / f"xiaou_{scenario}.xml"
    spec = object_spec or OBJECT_SPECS[scenario]
    if not math.isfinite(float(table_top_m)):
        raise ValueError("table_top_m must be finite")
    assets: list[str] = []
    for path in description.collision_meshes:
        name = path.stem
        assets.append(f'<mesh name="{name}" file="{_relative_mesh_path(xml_path, path)}"/>')
    for index, path in enumerate(description.world_meshes):
        assets.append(
            f'<mesh name="cad_world_{index}" file="{_relative_mesh_path(xml_path, path)}" scale="0.001 0.001 0.001"/>'
        )

    def geom(name: str, mesh: str) -> str:
        return (
            f'<geom name="{name}" type="mesh" mesh="{mesh}" density="500" '
            'contype="1" conaffinity="1" friction="0.8 0.01 0.001"/>'
        )

    robot_lines: list[str] = [
        '<body name="base_link" pos="0 0 0">',
        geom("base_body_geom", "base_body"),
    ]
    current_indent = "  "
    for joint in description.joints:
        link = joint.child
        robot_lines.append(
            f'{current_indent}<body name="{link}" pos="{_fmt(joint.xyz)}" '
            f'quat="{_fmt(_matrix_to_quat(_rpy_matrix(joint.rpy)))}">'
        )
        current_indent += "  "
        robot_lines.append(
            f'{current_indent}<joint name="{joint.name}" type="hinge" axis="{_fmt(joint.axis)}" '
            'limited="true" range="-3.14159265359 3.14159265359" damping="0.2" armature="0.01"/>'
        )
        for mesh_name in description.link_meshes[link]:
            robot_lines.append(f'{current_indent}{geom(link + "_" + mesh_name, mesh_name)}')
    robot_lines.append(
        f'{current_indent}<body name="grasp_tcp" pos="{_fmt(description.grasp_tcp_xyz)}" '
        f'quat="{_fmt(_matrix_to_quat(_rpy_matrix(description.grasp_tcp_rpy)))}">'
    )
    current_indent += "  "
    # The CAD gripper is static. These two small collision bars provide an
    # explicit jaw contact envelope without replacing the CAD collision mesh.
    robot_lines.append(
        f'{current_indent}<geom name="jaw_left_proxy" type="box" pos="-0.045 0 0" size="0.008 0.035 0.022" '
        'density="300" contype="1" conaffinity="1" friction="0.8 0.01 0.001"/>'
    )
    robot_lines.append(
        f'{current_indent}<geom name="jaw_right_proxy" type="box" pos="0.045 0 0" size="0.008 0.035 0.022" '
        'density="300" contype="1" conaffinity="1" friction="0.8 0.01 0.001"/>'
    )
    robot_lines.append(f'{current_indent[:-2]}</body>')
    for _ in range(len(description.joints) + 1):
        current_indent = current_indent[:-2]
        robot_lines.append(f'{current_indent}</body>')

    world_visuals = [
        '<body name="cad_world_home" pos="0 0 0">',
        *[
            f'<geom name="cad_world_geom_{index}" type="mesh" mesh="cad_world_{index}" '
            'contype="0" conaffinity="0" rgba="0.35 0.35 0.38 0.18"/>'
            for index in range(len(description.world_meshes))
        ],
        '</body>',
    ]
    exclusions = [
        f'<exclude body1="{parent}" body2="{child}"/>'
        for parent, child in zip(("base_link", "link_1", "link_2", "link_3", "link_4", "link_5"),
                                 ("link_1", "link_2", "link_3", "link_4", "link_5", "link_6"))
    ]
    xml = "\n".join(
        [
            f'<mujoco model="xiaou_{scenario}">',
            '<compiler angle="radian" autolimits="true"/>',
            f'<option timestep="{SIM_TIMESTEP_S}" gravity="0 0 -9.81" integrator="implicitfast" iterations="80"/>',
            '<size njmax="1000" nconmax="4000"/>',
            '<asset>',
            *[f'  {item}' for item in assets],
            '</asset>',
            '<default>',
            '<joint damping="0.2" armature="0.01"/>',
            '<geom solref="0.005 1" solimp="0.9 0.98 0.001"/>',
            '</default>',
            '<worldbody>',
            f'<geom name="table" type="box" pos="0.27 0 {float(table_top_m) - 0.025:.8g}" size="0.33 0.30 0.025" friction="1.0 0.02 0.001"/>',
            *robot_lines,
            '<body name="target_object" pos="0 0 0">',
            '<freejoint name="target_free"/>',
            f'{_object_geom(spec)}',
            '</body>',
            *world_visuals,
            '</worldbody>',
            '<actuator>',
            *[
                f'<position name="motor_{index}" joint="joint_{index}" kp="1600" kv="160" '
                'ctrlrange="-3.14159265359 3.14159265359" forcerange="-1000 1000"/>'
                for index in range(1, 7)
            ],
            '</actuator>',
            '<equality>',
            '<weld name="grasp_weld" body1="grasp_tcp" body2="target_object" active="false" solref="0.01 1" solimp="0.9 0.99 0.001"/>',
            '</equality>',
            '<contact>',
            *exclusions,
            '</contact>',
            '</mujoco>',
        ]
    )
    xml_path.write_text(xml + "\n", encoding="utf-8")
    return xml_path


def stage_ascii_assets(description: RobotDescription, scenario: str, root: Path) -> RobotDescription:
    """Copy exact mesh bytes to an ASCII path for MuJoCo's Windows loader.

    The checked-in workspace contains Chinese path components. Python can read
    them, but some MuJoCo Windows builds fail to open Unicode XML/mesh paths.
    Staging avoids changing the source model or silently substituting meshes.
    """
    scenario_root = root / scenario
    collision_root = scenario_root / "collision"
    collision_root.mkdir(parents=True, exist_ok=True)
    collision_meshes: list[Path] = []
    for source in description.collision_meshes:
        target = collision_root / source.name
        shutil.copy2(source, target)
        collision_meshes.append(target.resolve())
    # The CAD world-model STL files are large ASCII exports. MuJoCo's Windows
    # loader accepts binary STL but not these ASCII files, so they remain an
    # audited source reference rather than being silently converted or used as
    # collision geometry. The physical world is represented by the table and
    # class-specific rigid object below.
    return RobotDescription(
        link_meshes=description.link_meshes,
        joints=description.joints,
        grasp_tcp_xyz=description.grasp_tcp_xyz,
        grasp_tcp_rpy=description.grasp_tcp_rpy,
        collision_meshes=tuple(collision_meshes),
        world_meshes=(),
        world_urdf=description.world_urdf,
    )


def _rotation_z(angle: float) -> np.ndarray:
    cosine, sine = math.cos(angle), math.sin(angle)
    return np.asarray(((cosine, -sine, 0.0), (sine, cosine, 0.0), (0.0, 0.0, 1.0)), dtype=np.float64)


def _pose(rotation: np.ndarray, xyz: np.ndarray) -> np.ndarray:
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = rotation
    result[:3, 3] = xyz
    return result


def _seeds(current: np.ndarray, limits: JointLimits) -> list[np.ndarray]:
    offsets = (
        np.zeros(6),
        np.asarray((0.0, -0.5, 0.5, 0.0, 0.0, 0.0)),
        np.asarray((0.0, 0.5, -0.5, 0.0, 0.0, 0.0)),
        np.asarray((0.0, -1.0, 1.0, 0.0, 0.0, 0.0)),
        np.asarray((0.0, 1.0, -1.0, 0.0, 0.0, 0.0)),
    )
    return [np.clip(current + offset, limits.position_min, limits.position_max) for offset in offsets]


class PhysicalEpisode:
    def __init__(
        self,
        xml_path: Path,
        scenario: str,
        object_spec: dict[str, float | str] | None = None,
        *,
        table_top_m: float = TABLE_TOP_M,
        joint_limits: JointLimits,
    ) -> None:
        runtime = _require_mujoco()
        self.runtime = runtime
        self.model = runtime.MjModel.from_xml_path(str(xml_path))
        self.data = runtime.MjData(self.model)
        self.scenario = scenario
        self.object_spec = object_spec or OBJECT_SPECS[scenario]
        self.joint_limits = joint_limits
        target_joint_id = int(runtime.mj_name2id(self.model, runtime.mjtObj.mjOBJ_JOINT, "target_free"))
        self.target_qpos = int(self.model.jnt_qposadr[target_joint_id])
        self.target_dof = int(self.model.jnt_dofadr[target_joint_id])
        self.target_body_id = int(runtime.mj_name2id(self.model, runtime.mjtObj.mjOBJ_BODY, "target_object"))
        self.grasp_body_id = int(runtime.mj_name2id(self.model, runtime.mjtObj.mjOBJ_BODY, "grasp_tcp"))
        self.weld_id = int(runtime.mj_name2id(self.model, runtime.mjtObj.mjOBJ_EQUALITY, "grasp_weld"))
        if not math.isfinite(float(table_top_m)):
            raise ValueError("table_top_m must be finite")
        self.table_top = float(table_top_m)
        self.contact_log: list[dict[str, str]] = []

    def reset(
        self,
        x: float,
        y: float,
        *,
        robot_joint_rad: np.ndarray | list[float] | tuple[float, ...] | None = None,
    ) -> None:
        """Place the target and, when supplied, set the actual replay start pose."""
        spec = self.object_spec
        height = float(spec["height_m"])
        if robot_joint_rad is not None:
            robot_joint = np.asarray(robot_joint_rad, dtype=np.float64).reshape(-1)
            if robot_joint.size != 6 or not np.isfinite(robot_joint).all():
                raise ValueError("robot_joint_rad must contain six finite values")
        else:
            robot_joint = None
        self.data.qpos[:] = 0.0
        self.data.qvel[:] = 0.0
        if robot_joint is not None:
            self.data.qpos[:6] = robot_joint
        self.data.qpos[self.target_qpos : self.target_qpos + 3] = (x, y, self.table_top + height / 2.0)
        self.data.qpos[self.target_qpos + 3 : self.target_qpos + 7] = (1.0, 0.0, 0.0, 0.0)
        self.data.ctrl[:] = self.data.qpos[:6]
        self.data.eq_active[self.weld_id] = 0
        self.contact_log.clear()
        self.runtime.mj_forward(self.model, self.data)

    def _contacts(self) -> list[dict[str, str]]:
        runtime = self.runtime
        contacts: list[dict[str, str]] = []
        for index in range(int(self.data.ncon)):
            contact = self.data.contact[index]
            geom1 = runtime.mj_id2name(self.model, runtime.mjtObj.mjOBJ_GEOM, int(contact.geom1)) or "?"
            geom2 = runtime.mj_id2name(self.model, runtime.mjtObj.mjOBJ_GEOM, int(contact.geom2)) or "?"
            body1 = runtime.mj_id2name(self.model, runtime.mjtObj.mjOBJ_BODY, int(self.model.geom_bodyid[contact.geom1])) or "?"
            body2 = runtime.mj_id2name(self.model, runtime.mjtObj.mjOBJ_BODY, int(self.model.geom_bodyid[contact.geom2])) or "?"
            contacts.append({"geom1": geom1, "geom2": geom2, "body1": body1, "body2": body2})
        return contacts

    def forbidden_contacts(self) -> list[dict[str, str]]:
        forbidden: list[dict[str, str]] = []
        for item in self._contacts():
            bodies = {item["body1"], item["body2"]}
            geoms = {item["geom1"], item["geom2"]}
            if "table" in geoms:
                if "target_object" in bodies:
                    continue
                if any(body not in {"base_link", "world"} for body in bodies):
                    forbidden.append(item)
            if "target_object" not in bodies and any(body.startswith("link_") for body in bodies):
                # Parent-link exclusions are already in the generated MJCF;
                # any remaining arm-arm contact is a self-collision.
                forbidden.append(item)
        return forbidden

    def target_robot_contacts(self) -> list[dict[str, str]]:
        rows: list[dict[str, str]] = []
        for item in self._contacts():
            bodies = {item["body1"], item["body2"]}
            geoms = {item["geom1"], item["geom2"]}
            if "target_object" in bodies and "table" not in geoms and any(body != "target_object" for body in bodies):
                rows.append(item)
        return rows

    def move_to(self, goal: np.ndarray) -> dict[str, Any]:
        current = np.asarray(self.data.qpos[:6], dtype=np.float64).copy()
        points = plan_quintic_joint_trajectory(
            current,
            goal,
            self.joint_limits,
            sample_period_s=0.01,
            minimum_duration_s=0.25,
        )
        substeps = max(1, int(round(0.01 / self.model.opt.timestep)))
        max_error = 0.0
        forbidden_count = 0
        target_contact_count = 0
        target_contact_pairs: Counter[str] = Counter()
        for point in points:
            self.data.ctrl[:] = point.positions
            for _ in range(substeps):
                # The arm trajectory is prescribed exactly for this regression.
                # MuJoCo still computes link poses and contacts from the real
                # collision meshes; only uncalibrated motor dynamics are left
                # out of the pass/fail result.
                self.data.qpos[:6] = point.positions
                self.data.qvel[:6] = 0.0
                self.runtime.mj_forward(self.model, self.data)
                if self.data.eq_active[self.weld_id]:
                    self._snap_object_to_grasp()
                max_error = max(max_error, float(np.max(np.abs(self.data.qpos[:6] - point.positions))))
                forbidden_count += len(self.forbidden_contacts())
                for contact in self.target_robot_contacts():
                    target_contact_count += 1
                    target_contact_pairs[f"{contact['geom1']}<->{contact['geom2']}"] += 1
        final_error = float(np.max(np.abs(self.data.qpos[:6] - goal)))
        return {
            "trajectory_points": len(points),
            "duration_s": float(points[-1].time_from_start_s),
            "max_tracking_error_rad": max_error,
            "final_tracking_error_rad": final_error,
            "trajectory_mode": "kinematic_collision_playback",
            "forbidden_contact_samples": forbidden_count,
            "target_contact_samples": target_contact_count,
            "target_contact_pairs": dict(target_contact_pairs),
            "within_effective_limits": bool(
                np.all(self.data.qpos[:6] >= self.joint_limits.position_min - 1e-6)
                and np.all(self.data.qpos[:6] <= self.joint_limits.position_max + 1e-6)
            ),
        }

    def settle(self, steps: int = 250) -> None:
        self.data.ctrl[:] = self.data.qpos[:6]
        for _ in range(steps):
            self.runtime.mj_step(self.model, self.data)

    def release_to_table(self, *, clearance_m: float = RELEASE_CLEARANCE_M) -> bool:
        """Open the proxy grasp and place the object above the fixed table.

        The CAD gripper has no calibrated jaw actuator yet.  Releasing the
        object at the actual gripper geometry can inject a contact impulse,
        which makes a table-placement check meaningless.  This proxy models
        an already-open gripper by placing the object directly above the table
        at the current XY, then lets MuJoCo settle it under gravity.
        """

        if not math.isfinite(float(clearance_m)) or clearance_m < 0.0:
            raise ValueError("clearance_m must be finite and non-negative")
        height = float(self.object_spec["height_m"])
        self.set_weld(False)
        self.data.qpos[self.target_qpos + 2] = self.table_top + height / 2.0 + float(clearance_m)
        self.data.qvel[self.target_dof : self.target_dof + 6] = 0.0
        self.runtime.mj_forward(self.model, self.data)
        self.settle()
        return any(
            "target_object" in {item["body1"], item["body2"]}
            and "table" in {item["geom1"], item["geom2"]}
            for item in self._contacts()
        )

    def set_weld(self, active: bool) -> None:
        self.data.eq_active[self.weld_id] = 1 if active else 0
        self.runtime.mj_forward(self.model, self.data)
        if active:
            self._snap_object_to_grasp()

    def _snap_object_to_grasp(self) -> None:
        """Apply the active weld to the free body during kinematic playback."""
        self.data.qpos[self.target_qpos : self.target_qpos + 3] = self.data.xpos[self.grasp_body_id]
        self.data.qpos[self.target_qpos + 3 : self.target_qpos + 7] = self.data.xquat[self.grasp_body_id]
        self.data.qvel[self.target_dof : self.target_dof + 6] = 0.0
        self.runtime.mj_forward(self.model, self.data)


def solve_pose(arm_model: Any, target: np.ndarray, current: np.ndarray, limits: JointLimits) -> Any:
    return ik_space_multistart(
        arm_model.home_grasp_tcp,
        arm_model.screw_axes,
        target,
        _seeds(current, limits),
        preferred_angles=current,
        joint_lower=limits.position_min,
        joint_upper=limits.position_max,
        orientation_tolerance_rad=2e-4,
        position_tolerance_m=2e-4,
        max_iterations=400,
        max_step_rad=0.15,
    )


def build_candidate_plan(
    simulator: PhysicalEpisode,
    arm_model: Any,
    scenario: str,
    x: float,
    y: float,
    yaw: float,
    object_spec: dict[str, float | str] | None = None,
    joint_limits: JointLimits | None = None,
    ready_pose_rad: np.ndarray | None = None,
) -> tuple[dict[str, Any] | None, str | None]:
    """Preflight IK and exact collision playback before a trial is accepted."""
    spec = object_spec or OBJECT_SPECS[scenario]
    limits = joint_limits or simulator.joint_limits
    current = (
        np.asarray(ready_pose_rad, dtype=np.float64).reshape(-1).copy()
        if ready_pose_rad is not None
        else np.asarray(simulator.data.qpos[:6], dtype=np.float64).copy()
    )
    if current.size != 6 or not np.isfinite(current).all():
        raise ValueError("ready_pose_rad must contain six finite values")
    if np.any(current < limits.position_min) or np.any(current > limits.position_max):
        raise ValueError("ready_pose_rad exceeds the simulation joint limits")
    start_pose = current.copy()
    height = float(spec["height_m"])
    grasp_z = TABLE_TOP_M + height * 0.5 + GRASP_Z_OFFSET_M
    approach_z = grasp_z + 0.10
    rotation = _rotation_z(yaw) @ arm_model.home_grasp_tcp[:3, :3]
    poses: dict[str, np.ndarray] = {
        "approach": _pose(rotation, np.asarray((x, y, approach_z), dtype=np.float64)),
        "grasp": _pose(rotation, np.asarray((x, y, grasp_z), dtype=np.float64)),
    }
    stage_names = ["approach", "grasp"]
    if scenario == "tissue_pull":
        poses["pull"] = _pose(rotation, np.asarray((x + 0.08, y, grasp_z), dtype=np.float64))
        stage_names.append("pull")
    else:
        poses["lift"] = _pose(rotation, np.asarray((x, y, approach_z), dtype=np.float64))
        stage_names.append("lift")
    solved: dict[str, Any] = {}
    for stage_name in stage_names:
        result = solve_pose(arm_model, poses[stage_name], current, limits)
        if not result.converged:
            return None, f"{stage_name}_ik_not_converged"
        points = plan_quintic_joint_trajectory(
            current,
            result.joint_angles,
            limits,
            sample_period_s=0.01,
            minimum_duration_s=0.25,
        )
        for point in points:
            simulator.data.qpos[:6] = point.positions
            simulator.data.qvel[:6] = 0.0
            simulator.runtime.mj_forward(simulator.model, simulator.data)
            if simulator.forbidden_contacts():
                return None, f"{stage_name}_forbidden_contact"
        solved[stage_name] = result
        current = result.joint_angles.copy()
        if stage_name == "grasp" and not simulator.target_robot_contacts():
            return None, "grasp_contact_not_verified"
    # Rehearse the return to the same ready/start pose, rather than to a
    # convenient mathematical zero pose which may not be the real arm's ready
    # configuration (and may hide an otherwise unreachable return segment).
    return_points = plan_quintic_joint_trajectory(
        current,
        start_pose,
        limits,
        sample_period_s=0.01,
        minimum_duration_s=0.25,
    )
    for point in return_points:
        simulator.data.qpos[:6] = point.positions
        simulator.data.qvel[:6] = 0.0
        simulator.runtime.mj_forward(simulator.model, simulator.data)
        if simulator.forbidden_contacts():
            return None, "return_ready_forbidden_contact"
    simulator.reset(x, y, robot_joint_rad=start_pose)
    return {
        "poses": poses,
        "stage_names": stage_names,
        "solved": solved,
        "return_ready_joint_rad": start_pose,
    }, None


def load_policies(fp32_path: Path | None, int8_path: Path | None) -> dict[str, Any]:
    policies: dict[str, Any] = {}
    gate = SafetyGate(require_motion_enabled=False, require_hardware_ready=False, require_collision_free=True, require_feedback_verified=False)
    if fp32_path is not None and fp32_path.is_file():
        policy = DecisionTransformerPolicy(device="cpu", max_seq_len=16)
        metadata = policy.load(fp32_path)
        policy.net.eval()
        policies["fp32"] = (policy, metadata, gate)
    if int8_path is not None and int8_path.is_file():
        try:
            from robot_ai.decision.onnx_policy import OnnxDecisionTransformerPolicy

            policies["int8"] = (OnnxDecisionTransformerPolicy(int8_path, max_seq_len=16), {}, gate)
        except Exception as exc:
            policies["int8_error"] = str(exc)
    return policies


def run_trial(
    simulator: PhysicalEpisode,
    arm_model: Any,
    scenario: str,
    trial: int,
    rng: np.random.Generator,
    policies: dict[str, Any],
    max_sampling_attempts: int = 30,
    object_spec: dict[str, float | str] | None = None,
    ready_pose_rad: np.ndarray | None = None,
) -> dict[str, Any]:
    spec = object_spec or OBJECT_SPECS[scenario]
    height = float(spec["height_m"])
    rejections: Counter[str] = Counter()
    plan: dict[str, Any] | None = None
    x = y = yaw = 0.0
    attempts_used = 0
    for attempt in range(1, max_sampling_attempts + 1):
        attempts_used = attempt
        # This range is deliberately inside the measured calibration envelope
        # used by the existing IK regression, not an invented expansion.
        x = float(rng.uniform(0.235, 0.315))
        y = float(rng.uniform(-0.155, -0.085))
        # Keep the perturbation within the currently demonstrated CAD TCP
        # branch; larger yaw ranges need measured TCP and collision planning.
        yaw = float(rng.uniform(-math.radians(3.0), math.radians(3.0)))
        simulator.reset(x, y, robot_joint_rad=ready_pose_rad)
        plan, rejection = build_candidate_plan(
            simulator,
            arm_model,
            scenario,
            x,
            y,
            yaw,
            spec,
            simulator.joint_limits,
            ready_pose_rad,
        )
        if plan is not None:
            break
        rejections[rejection or "candidate_rejected"] += 1
    if plan is None:
        return {
            "trial": trial,
            "scenario": scenario,
            "seed": trial,
            "object_xyz_m": [x, y, TABLE_TOP_M + height / 2.0],
            "yaw_rad": yaw,
            "sampling_attempts": attempts_used,
            "preflight_rejections": dict(rejections),
            "policy": {},
            "stages": [],
            "issues": ["no_collision_free_candidate"],
            "warnings": [],
            "passed": False,
            "hardware_motion": False,
        }

    simulator.reset(x, y, robot_joint_rad=ready_pose_rad)
    scene = DesktopScene(scenario=scenario, seed=trial, object_position_m=(x, y, 0.0), object_yaw_rad=yaw)
    observation = scene.observation()
    policy_rows: dict[str, Any] = {}
    for name, value in policies.items():
        if name.endswith("_error"):
            continue
        policy, _metadata, gate = value
        decision = policy.predict(scene.observation_vector(), TASK_TO_ID[scenario], gate, observation)
        policy_rows[name] = decision.as_dict()

    current = np.asarray(ready_pose_rad, dtype=np.float64).reshape(-1).copy()
    stages: list[dict[str, Any]] = []
    issues: list[str] = []
    warnings: list[str] = []
    contact_verified = False
    for stage_name in plan["stage_names"]:
        result = plan["solved"][stage_name]
        row: dict[str, Any] = {
            "stage": stage_name,
            "ik_converged": bool(result.converged),
            "ik_iterations": int(result.iterations),
            "position_error_m": float(result.position_error_m),
            "orientation_error_rad": float(result.orientation_error_rad),
        }
        motion = simulator.move_to(result.joint_angles)
        row["motion"] = motion
        row["target_robot_contacts"] = simulator.target_robot_contacts()
        row["forbidden_contacts"] = simulator.forbidden_contacts()
        stages.append(row)
        if motion["forbidden_contact_samples"]:
            issues.append(f"{stage_name}_forbidden_contact")
        if stage_name == "grasp":
            contact_verified = bool(simulator.target_robot_contacts())
            if not contact_verified:
                issues.append("grasp_contact_not_verified")
            else:
                simulator.set_weld(True)
        current = result.joint_angles.copy()
    if contact_verified:
        if scenario == "tissue_pull":
            # Tissue is modeled as a rigid sheet stack only for the approach
            # and pull geometry.  A rigid-body release/table-settle result is
            # not a valid proxy for a real flexible sheet, so do not let it
            # pollute the generic pickup result with a meaningless warning.
            settled_table_contact = None
            stages.append({"stage": "release_settle", "table_contact": None, "status": "not_applicable_rigid_tissue_proxy"})
        else:
            # A table-settle check is only meaningful after removing the fixed
            # jaw-proxy overlap.  It models gripper opening, not a measured
            # real release trajectory.
            settled_table_contact = simulator.release_to_table()
            stages.append({"stage": "release_settle", "table_contact": settled_table_contact})
            if not settled_table_contact:
                warnings.append("released_object_not_in_table_contact")
        home_motion = simulator.move_to(np.asarray(ready_pose_rad, dtype=np.float64).reshape(-1))
        stages.append({"stage": "return_ready", "motion": home_motion})
        if home_motion["forbidden_contact_samples"]:
            issues.append("return_home_forbidden_contact")

    target_position = simulator.data.qpos[simulator.target_qpos : simulator.target_qpos + 3].copy()
    if scenario != "tissue_pull":
        settled_table_contact = any(
            "target_object" in {item["body1"], item["body2"]}
            and "table" in {item["geom1"], item["geom2"]}
            for item in simulator._contacts()
        )
    passed = not issues
    return {
        "trial": trial,
        "scenario": scenario,
        "seed": trial,
        "object_xyz_m": [x, y, TABLE_TOP_M + height / 2.0],
        "yaw_rad": yaw,
        "sampling_attempts": attempts_used,
        "preflight_rejections": dict(rejections),
        "policy": policy_rows,
        "stages": stages,
        "contact_verified": contact_verified,
        "settled_table_contact": settled_table_contact,
        "released_object_xyz_m": [float(value) for value in target_position],
        "issues": issues,
        "warnings": warnings,
        "passed": passed,
        "hardware_motion": False,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trials-per-scenario", type=int, default=8)
    parser.add_argument("--max-sampling-attempts", type=int, default=30)
    parser.add_argument("--seed", type=int, default=20260809)
    parser.add_argument("--scenarios", default=",".join(OBJECT_CLASSES))
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--asset-dir", type=Path, default=DEFAULT_ASSET_DIR)
    parser.add_argument("--ascii-asset-dir", type=Path, default=DEFAULT_ASCII_ASSET_DIR)
    parser.add_argument(
        "--world-urdf",
        type=Path,
        default=None,
        help="optional external CAD world-model URDF; the checked-in arm meshes are used when omitted or unavailable",
    )
    parser.add_argument("--fp32-checkpoint", type=Path, default=DEFAULT_FP32)
    parser.add_argument("--int8-model", type=Path, default=DEFAULT_INT8)
    parser.add_argument(
        "--hardware-config",
        type=Path,
        default=DEFAULT_HARDWARE_CONFIG,
        help="no-motion Pi/F407 candidate configuration that supplies effective limits and ready pose",
    )
    parser.add_argument("--no-policy", action="store_true", help="skip Transformer replay and only run physics")
    parser.add_argument(
        "--geometry-jitter-pct",
        type=float,
        default=0.05,
        help="independent radius/height jitter for each scenario (0 disables; dimensions are provisional)",
    )
    args = parser.parse_args()
    if args.trials_per_scenario < 1 or args.trials_per_scenario > 1000:
        parser.error("--trials-per-scenario must be in 1..1000")
    if args.max_sampling_attempts < 1 or args.max_sampling_attempts > 1000:
        parser.error("--max-sampling-attempts must be in 1..1000")
    if not 0.0 <= args.geometry_jitter_pct < 0.25:
        parser.error("--geometry-jitter-pct must be in [0, 0.25)")
    scenarios = [value.strip().lower() for value in args.scenarios.split(",") if value.strip()]
    if not scenarios or any(value not in OBJECT_SPECS for value in scenarios):
        parser.error(f"scenarios must be drawn from {sorted(OBJECT_SPECS)}")
    hardware_config = args.hardware_config if args.hardware_config.is_absolute() else PROJECT_ROOT / args.hardware_config
    motion_profile = load_simulation_motion_profile(hardware_config.resolve())
    runtime = _require_mujoco()
    world_urdf = args.world_urdf
    if world_urdf is not None and not world_urdf.is_absolute():
        world_urdf = PROJECT_ROOT / world_urdf
    description = parse_robot_description(world_urdf)
    arm_model = load_default_model()
    policies = {} if args.no_policy else load_policies(args.fp32_checkpoint, args.int8_model)
    args.asset_dir.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(args.seed)
    scenario_reports: list[dict[str, Any]] = []
    for scenario in scenarios:
        staged = stage_ascii_assets(description, scenario, args.ascii_asset_dir)
        object_spec = dict(OBJECT_SPECS[scenario])
        radius_scale = float(rng.uniform(1.0 - args.geometry_jitter_pct, 1.0 + args.geometry_jitter_pct))
        height_scale = float(rng.uniform(1.0 - args.geometry_jitter_pct, 1.0 + args.geometry_jitter_pct))
        object_spec["radius_m"] = float(object_spec["radius_m"]) * radius_scale
        object_spec["height_m"] = float(object_spec["height_m"]) * height_scale
        object_spec["mass_kg"] = float(object_spec["mass_kg"]) * radius_scale * radius_scale * height_scale
        xml_path = build_mjcf(staged, scenario, args.ascii_asset_dir / scenario, object_spec)
        simulator = PhysicalEpisode(
            xml_path,
            scenario,
            object_spec,
            joint_limits=motion_profile.joint_limits,
        )
        trials: list[dict[str, Any]] = []
        failures: Counter[str] = Counter()
        warnings: Counter[str] = Counter()
        for index in range(args.trials_per_scenario):
            record = run_trial(
                simulator,
                arm_model,
                scenario,
                index,
                rng,
                policies,
                args.max_sampling_attempts,
                object_spec,
                motion_profile.ready_pose_rad,
            )
            trials.append(record)
            failures.update(record.get("issues", []))
            warnings.update(record.get("warnings", []))
        scenario_reports.append({
            "scenario": scenario,
            "mjcf": str(xml_path),
            "trials": len(trials),
            "passed": sum(bool(item["passed"]) for item in trials),
            "success_rate": sum(bool(item["passed"]) for item in trials) / len(trials),
            "failure_counts": dict(failures),
            "warning_counts": dict(warnings),
            "geometry_jitter": {
                "radius_scale": radius_scale,
                "height_scale": height_scale,
                "effective_radius_m": float(object_spec["radius_m"]),
                "effective_height_m": float(object_spec["height_m"]),
                "effective_mass_kg": float(object_spec["mass_kg"]),
            },
            "trial_records": trials,
        })

    total_trials = sum(int(item["trials"]) for item in scenario_reports)
    total_passed = sum(int(item["passed"]) for item in scenario_reports)
    report = {
        "simulation": "mujoco_physical_grasp_regression",
        "offline": True,
        "hardware_motion": False,
        "serial_opened": False,
        "can_opened": False,
        "ros_hardware_started": False,
        "physics_engine": {"name": "MuJoCo", "version": getattr(runtime, "__version__", "unknown"), "mode": "DIRECT"},
        "sources": {
            "robot_xacro": str(XACRO_PATH),
            "robot_collision_root": str(COLLISION_ROOT),
            "cad_world_urdf": str(description.world_urdf) if description.world_urdf else None,
            "poe_model": str(PROJECT_ROOT / "robot_ai" / "arm_control" / "config" / "arm_model.json"),
            "hardware_motion_profile": str(motion_profile.source_path),
            "hardware_motion_profile_schema_version": motion_profile.source_schema_version,
            "collision_mesh_count": len(description.collision_meshes),
            "world_mesh_count": len(description.world_meshes),
            "cad_world_visual_loaded": False,
            "cad_world_visual_note": (
                "The optional CAD world STL files were parsed and audited; the MuJoCo world uses the table and rigid object primitives."
                if description.world_urdf
                else "No external CAD world URDF was supplied; the checked-in arm collision meshes, table, and rigid object primitives were used."
            ),
            "parsed_joint_names": [item.name for item in description.joints],
        },
        "assumptions": {
            "robot_trajectory_mode": "kinematic_collision_playback; MuJoCo rigid-body dynamics are used for the free object and release, while uncalibrated arm motor dynamics are excluded",
            "joint_position_limits_rad": {
                "source": "candidate Pi/F407 effective envelope; simulation only",
                "min": [float(value) for value in motion_profile.joint_limits.position_min],
                "max": [float(value) for value in motion_profile.joint_limits.position_max],
            },
            "velocity_limits_rad_s": [float(value) for value in motion_profile.joint_limits.velocity_max],
            "acceleration_limits_rad_s2": [float(value) for value in motion_profile.joint_limits.acceleration_max],
            "ready_pose_rad": [float(value) for value in motion_profile.ready_pose_rad],
            "link_density_kg_m3": 500.0,
            "table_top_m": TABLE_TOP_M,
            "grasp_z_offset_m": GRASP_Z_OFFSET_M,
            "release_clearance_m": RELEASE_CLEARANCE_M,
            "gripper": "CAD collision mesh plus fixed jaw contact proxies; jaw actuator calibration is not measured",
            "object_model": "primitive rigid bodies with class-specific dimensions and mass from desktop scene specs",
            "tissue": "rigid sheet-stack proxy; cloth deformation and pull dynamics are not modeled",
        },
        "policy_replay": {
            "enabled": not args.no_policy,
            "fp32_checkpoint": str(args.fp32_checkpoint) if args.fp32_checkpoint.is_file() else None,
            "int8_model": str(args.int8_model) if args.int8_model.is_file() else None,
            "int8_error": policies.get("int8_error"),
        },
        "seed": args.seed,
        "trials_per_scenario": args.trials_per_scenario,
        "max_sampling_attempts": args.max_sampling_attempts,
        "randomization": {
            "placement_range_m": {"x": [0.235, 0.315], "y": [-0.155, -0.085]},
            "yaw_range_deg": [-3.0, 3.0],
            "geometry_jitter_pct": args.geometry_jitter_pct,
            "geometry_jitter_scope": "one independent radius/height perturbation per scenario and seed",
        },
        "total_trials": total_trials,
        "total_passed": total_passed,
        "success_rate": total_passed / max(1, total_trials),
        "scenarios": scenario_reports,
        "limitations": [
            "MuJoCo contact and dynamics use provisional link density because CAD mass properties are not calibrated.",
            "The regression uses the current candidate effective Pi/F407 envelope and ready pose; rerun it whenever STM zero or limits change.",
            "Static gripper CAD is supplemented by fixed jaw proxies; real jaw width, force, and finger motion remain unverified.",
            "A passed offline trial is not proof of camera calibration, encoder correctness, F407 feedback, grasp force, or real object friction.",
            "Real motion remains locked by hardware_calibration.json and is never enabled by this script.",
        ],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    summary = {key: value for key, value in report.items() if key != "scenarios"}
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0 if total_passed == total_trials else 2


if __name__ == "__main__":
    raise SystemExit(main())
