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
        raise RuntimeError("object grasp profiles must use schema_version=2")
    if data.get("height_reference") != "relative_to_table_m":
        raise RuntimeError("object grasp profiles use an unexpected height reference")
    contract = data.get("camera_contract")
    if not isinstance(contract, dict) or (
        contract.get("width_px"), contract.get("height_px")
    ) != (1920, 1080):
        raise RuntimeError("object grasp profiles must declare the fixed 1920x1080 camera")
    profiles = data.get("classes")
    if not isinstance(profiles, dict) or not profiles:
        raise RuntimeError("object grasp profiles must contain a classes mapping")

    required_classes = {"pen", "cup", "cola", "bottle", "earphone"}
    missing_classes = sorted(required_classes - set(profiles))
    if missing_classes:
        raise RuntimeError(f"object grasp profiles are missing classes: {missing_classes}")

    invalid: dict[str, list[str]] = {}
    measured_heights: list[str] = []
    unmeasured_heights: list[str] = []
    for name, profile in profiles.items():
        if not isinstance(profile, dict):
            invalid[name] = ["profile must be an object"]
            continue
        errors: list[str] = []
        grasp_height = profile.get("grasp_height_m")
        approach_height = profile.get("approach_height_m")
        if grasp_height is None:
            unmeasured_heights.append(name)
        elif (
            isinstance(grasp_height, bool)
            or not isinstance(grasp_height, (int, float))
            or not math.isfinite(grasp_height)
            or grasp_height < 0.0
        ):
            errors.append("grasp_height_m")
        else:
            measured_heights.append(name)
        if approach_height is not None and (
            isinstance(approach_height, bool)
            or not isinstance(approach_height, (int, float))
            or not math.isfinite(approach_height)
            or approach_height < 0.0
        ):
            errors.append("approach_height_m")
        if grasp_height is not None and approach_height is not None:
            if isinstance(grasp_height, (int, float)) and isinstance(approach_height, (int, float)):
                if approach_height < grasp_height:
                    errors.append("approach_height_m_before_grasp_height_m")
        for field in ("gripper_open_pwm_deg", "gripper_close_pwm_deg"):
            value = profile.get(field)
            if value is not None and (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or not 0.0 <= value <= 180.0
            ):
                errors.append(field)
        placement = profile.get("placement_pose_id")
        if placement is not None and (not isinstance(placement, str) or not placement.strip()):
            errors.append("placement_pose_id")
        if profile.get("grasp_mode") != "top_down":
            errors.append("grasp_mode")
        if profile.get("failure_policy") != "vision_recheck_then_safe_return_then_report":
            errors.append("failure_policy")
        if errors:
            invalid[name] = errors
    if invalid:
        raise RuntimeError(f"invalid object grasp profiles: {invalid}")
    return {
        "classes": sorted(profiles),
        "height_reference": data["height_reference"],
        "measured_grasp_heights": sorted(measured_heights),
        "unmeasured_grasp_heights": sorted(unmeasured_heights),
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
        raise RuntimeError(f"expected J1..J6 node IDs [1..6], got {node_ids!r}")
    if calibration.get("joint_node_ids_confirmed") is not True:
        raise RuntimeError("J1..J6 node ID confirmation is missing")

    report = {
        "offline_algorithm_verified": True,
        "real_motion_ready": False,
        "poe_urdf_max_screw_axis_error": screw_axis_error,
        "poe_urdf_max_home_transform_error": home_error,
        "meshes": validate_meshes(),
        "structured_files": validate_structured_files(),
        "execution_locks": validate_execution_locks(),
        "object_grasp_profiles": validate_grasp_profiles(),
        "confirmed_node_ids": {f"J{i}": i for i in range(1, 7)},
        "hardware_gate_missing_or_invalid": list(readiness.missing_or_invalid),
        "limitations": [
            "ROS2 and MoveIt runtime build must be verified on the Raspberry Pi",
            "J1..J6 CAN IDs are user-confirmed as 1..6; the F407 hardware target must still prove each physical response before enabling motion",
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
