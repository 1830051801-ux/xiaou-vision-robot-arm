from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[1]
ROBOT_AI_DIR = ROOT / "robot_ai"
ROS2_SRC = ROOT / "ros2_ws" / "src"
URDF_PATH = (
    ROS2_SRC
    / "xiaou_arm_description"
    / "urdf"
    / "xiaou_arm_display.urdf.xacro"
)
if str(ROBOT_AI_DIR) not in sys.path:
    sys.path.insert(0, str(ROBOT_AI_DIR))

from arm_control.model import load_default_model
from arm_control.safety import load_hardware_config, validate_motion_readiness


def _vector(value: str | None, default: str) -> np.ndarray:
    return np.asarray([float(item) for item in (value or default).split()], dtype=np.float64)


def _rotation_from_rpy(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def _origin_transform(joint: ET.Element) -> np.ndarray:
    origin = joint.find("origin")
    xyz = _vector(origin.get("xyz") if origin is not None else None, "0 0 0")
    rpy = _vector(origin.get("rpy") if origin is not None else None, "0 0 0")
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = _rotation_from_rpy(rpy)
    transform[:3, 3] = xyz
    return transform


def derive_urdf_model() -> tuple[np.ndarray, np.ndarray]:
    root = ET.parse(URDF_PATH).getroot()
    joints = {joint.get("name"): joint for joint in root.findall("joint")}
    transform = np.eye(4, dtype=np.float64)
    columns: list[np.ndarray] = []
    for index in range(1, 7):
        joint = joints[f"joint_{index}"]
        transform = transform @ _origin_transform(joint)
        local_axis = _vector(joint.find("axis").get("xyz"), "0 0 0")
        space_axis = transform[:3, :3] @ local_axis
        space_axis /= np.linalg.norm(space_axis)
        point = transform[:3, 3]
        columns.append(np.concatenate((space_axis, -np.cross(space_axis, point))))
    transform = transform @ _origin_transform(joints["link_6_to_grasp_tcp"])
    return np.column_stack(columns), transform


def validate_meshes() -> dict[str, object]:
    mesh_root = ROS2_SRC / "xiaou_arm_description" / "meshes"
    manifest = json.loads((mesh_root / "manifest.json").read_text(encoding="utf-8"))
    results: dict[str, object] = {}
    for layer in ("visual", "collision"):
        files = sorted((mesh_root / layer).glob("*.stl"))
        if len(files) != 8:
            raise RuntimeError(f"expected 8 {layer} meshes, found {len(files)}")
        empty = [path.name for path in files if path.stat().st_size <= 84]
        if empty:
            raise RuntimeError(f"empty or invalid {layer} meshes: {empty}")
        results[layer] = {
            "count": len(files),
            "bytes": sum(path.stat().st_size for path in files),
        }
    components = manifest.get("components")
    if not isinstance(components, list) or len(components) != 8:
        raise RuntimeError("mesh manifest must describe exactly 8 components")
    results["manifest_components"] = len(components)
    return results


def validate_structured_files() -> dict[str, int]:
    xml_files = list(ROS2_SRC.rglob("package.xml")) + [
        ROS2_SRC / "xiaou_arm_moveit_config" / "config" / "xiaou_arm.srdf",
        URDF_PATH,
    ]
    yaml_files = list(ROS2_SRC.rglob("*.yaml"))
    for path in xml_files:
        ET.parse(path)
    for path in yaml_files:
        yaml.safe_load(path.read_text(encoding="utf-8"))
    return {"xml_files": len(xml_files), "yaml_files": len(yaml_files)}


def validate_execution_locks() -> dict[str, bool]:
    pipeline = (
        ROS2_SRC / "xiaou_arm_planning" / "launch" / "pipeline.launch.py"
    ).read_text(encoding="utf-8")
    planner = (
        ROS2_SRC / "xiaou_arm_planning" / "src" / "target_planner_node.cpp"
    ).read_text(encoding="utf-8")
    checks = {
        "move_group_started": 'executable="move_group"' in pipeline,
        "move_group_execution_disabled": '"allow_trajectory_execution": False' in pipeline,
        "planner_execution_disabled": 'parameters=[moveit_parameters, {"allow_execution": False}]'
        in pipeline,
        "hardware_ready_required": "!allow_execution_ || !hardware_ready_" in planner,
    }
    if not all(checks.values()):
        raise RuntimeError(f"execution-lock verification failed: {checks}")
    return checks


def validate_grasp_profiles() -> dict[str, object]:
    path = ROBOT_AI_DIR / "arm_control" / "config" / "object_grasp_profiles.json"
    data = json.loads(path.read_text(encoding="utf-8"))
    if data.get("schema_version") != 2:
        raise RuntimeError("object grasp profiles must use schema_version 2")
    camera_contract = data.get("camera_contract")
    if not isinstance(camera_contract, dict):
        raise RuntimeError("object grasp profiles require a camera_contract")
    expected_camera = {
        "width_px": 1920,
        "height_px": 1080,
        "mount": "fixed_external_overhead",
        "camera_motion": "forbidden_during_pick",
    }
    if any(camera_contract.get(key) != value for key, value in expected_camera.items()):
        raise RuntimeError("object grasp profiles use an unexpected camera contract")
    profiles = data.get("classes")
    if not isinstance(profiles, dict) or not profiles:
        raise RuntimeError("object grasp profiles must contain at least one class")

    calibrated: list[str] = []
    unmeasured: list[str] = []
    for name, profile in profiles.items():
        if not isinstance(name, str) or not name or not isinstance(profile, dict):
            raise RuntimeError("object grasp profiles must use named object entries")
        if profile.get("grasp_mode") != "top_down":
            raise RuntimeError(f"{name} has an unexpected grasp_mode")
        if profile.get("failure_policy") != "vision_recheck_then_safe_return_then_report":
            raise RuntimeError(f"{name} has an unexpected failure_policy")

        values: dict[str, float | None] = {}
        for key in (
            "grasp_height_m",
            "approach_height_m",
            "gripper_open_pwm_deg",
            "gripper_close_pwm_deg",
        ):
            value = profile.get(key)
            if value is None:
                values[key] = None
                continue
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise RuntimeError(f"{name}.{key} must be null or a finite number")
            values[key] = float(value)

        if values["grasp_height_m"] is not None and values["grasp_height_m"] < 0.0:
            raise RuntimeError(f"{name}.grasp_height_m must be non-negative")
        if values["approach_height_m"] is not None and values["approach_height_m"] < 0.0:
            raise RuntimeError(f"{name}.approach_height_m must be non-negative")
        if (
            values["grasp_height_m"] is not None
            and values["approach_height_m"] is not None
            and values["approach_height_m"] < values["grasp_height_m"]
        ):
            raise RuntimeError(f"{name}.approach_height_m must not be below grasp_height_m")
        for key in ("gripper_open_pwm_deg", "gripper_close_pwm_deg"):
            if values[key] is not None and not 0.0 <= values[key] <= 180.0:
                raise RuntimeError(f"{name}.{key} must be in 0..180 degrees")

        placement = profile.get("placement_pose_id")
        if placement is not None and (not isinstance(placement, str) or not placement.strip()):
            raise RuntimeError(f"{name}.placement_pose_id must be null or a non-empty string")
        complete = all(value is not None for value in values.values()) and placement is not None
        (calibrated if complete else unmeasured).append(name)

    return {
        "schema_version": 2,
        "classes": sorted(profiles),
        "calibrated": sorted(calibrated),
        "unmeasured": sorted(unmeasured),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Offline verification for the XiaoU six-axis stack")
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "runtime" / "arm_model_checks" / "six_axis_verification.json",
    )
    args = parser.parse_args()

    model = load_default_model()
    urdf_screw_axes, urdf_home = derive_urdf_model()
    screw_axis_error = float(np.max(np.abs(urdf_screw_axes - model.screw_axes)))
    home_error = float(np.max(np.abs(urdf_home - model.home_grasp_tcp)))
    if screw_axis_error > 1e-8 or home_error > 1e-8:
        raise RuntimeError(
            f"POE/URDF mismatch: screw={screw_axis_error:.3e}, home={home_error:.3e}"
        )

    readiness = validate_motion_readiness(load_hardware_config())
    if readiness.ready:
        raise RuntimeError("default hardware configuration unexpectedly enables motion")

    calibration = load_hardware_config()
    node_ids = calibration.get("joint_node_ids")
    if node_ids != [1, 2, 3, 4, 5, 6]:
        raise RuntimeError(f"expected provisional J1..J6 node IDs [1..6], got {node_ids!r}")

    report = {
        "offline_algorithm_verified": True,
        "real_motion_ready": False,
        "poe_urdf_max_screw_axis_error": screw_axis_error,
        "poe_urdf_max_home_transform_error": home_error,
        "meshes": validate_meshes(),
        "structured_files": validate_structured_files(),
        "execution_locks": validate_execution_locks(),
        "object_grasp_profiles": validate_grasp_profiles(),
        "provisional_node_ids": {f"J{i}": i for i in range(1, 7)},
        "hardware_gate_missing_or_invalid": list(readiness.missing_or_invalid),
        "limitations": [
            "ROS2 and MoveIt runtime build must be verified on the Raspberry Pi",
            "J1..J6 IDs are provisionally assigned 1..6; verify each ID on the live CAN bus before enabling motion",
            "zero offsets, directions, limits, feedback, and the actual STM32 frame implementation remain unmeasured",
            "expanded camera calibration, measured table Z, and per-class grasp heights are required",
            "real motion remains disabled",
        ],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
