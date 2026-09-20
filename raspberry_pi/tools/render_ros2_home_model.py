from __future__ import annotations

from pathlib import Path
import sys

import numpy as np


LOCAL_MESH_PACKAGES = Path(r"D:\机械臂\.codex_tmp_py")
if LOCAL_MESH_PACKAGES.is_dir():
    sys.path.insert(0, str(LOCAL_MESH_PACKAGES))

import matplotlib.pyplot as plt
import trimesh


ROOT = Path(__file__).resolve().parents[1]
MESH_ROOT = ROOT / "ros2_ws" / "src" / "xiaou_arm_description" / "meshes" / "visual"
OUTPUT = ROOT / "runtime" / "arm_model_checks" / "ros2_home_model.png"

COMPONENTS = (
    ("base_body.stl", "base_link", "#3b4147"),
    ("joint_1_body.stl", "link_1", "#8b949e"),
    ("link_2_body.stl", "link_2", "#3b4147"),
    ("link_3_body.stl", "link_3", "#3b4147"),
    ("joint_5_body.stl", "link_4", "#8b949e"),
    ("joint_6_body.stl", "link_5", "#8b949e"),
    ("wrist_connector.stl", "link_6", "#8b949e"),
    ("gripper.stl", "link_6", "#2474a5"),
)

ORIGINS = {
    "base_link": np.array([0.0, 0.0, 0.0]),
    "link_1": np.array([0.0, 0.0, 0.0]),
    "link_2": np.array([-0.000000035529, 0.0, 0.155999957265]),
    "link_3": np.array([0.179999573587, -0.000000000534, 0.156367431963]),
    "link_4": np.array([0.189786868388, -0.000000000534, 0.336101183735]),
    "link_5": np.array([0.189786868332, 0.092999999466, 0.336101186004]),
    "link_6": np.array([0.295755047669, 0.092999999399, 0.338701799816]),
}

AXES = {
    "joint_1": np.array([0.0, 0.0, 1.0]),
    "joint_2": np.array([0.0, 1.0, 0.0]),
    "joint_3": np.array([0.0, 1.0, 0.0]),
    "joint_4": np.array([0.0, 1.0, 0.0]),
    "joint_5": np.array([-0.999698993944, 0.0, -0.024534088676]),
    "joint_6": np.array([0.000416435776, 0.999855935329, -0.016968652540]),
}


def equal_axes(ax, points: np.ndarray) -> None:
    lower = points.min(axis=0)
    upper = points.max(axis=0)
    center = (lower + upper) / 2.0
    radius = max(upper - lower) / 2.0
    ax.set_xlim(center[0] - radius, center[0] + radius)
    ax.set_ylim(center[1] - radius, center[1] + radius)
    ax.set_zlim(center[2] - radius, center[2] + radius)


def main() -> int:
    rng = np.random.default_rng(20260805)
    figure = plt.figure(figsize=(10, 8), dpi=160)
    ax = figure.add_subplot(111, projection="3d")
    all_points = []
    for filename, link_name, color in COMPONENTS:
        mesh = trimesh.load_mesh(MESH_ROOT / filename, process=False)
        vertices = np.asarray(mesh.vertices) + ORIGINS[link_name]
        count = min(6000, len(vertices))
        sample = vertices[rng.choice(len(vertices), size=count, replace=False)]
        all_points.append(sample)
        ax.scatter(sample[:, 0], sample[:, 1], sample[:, 2], s=0.25, c=color, alpha=0.35)

    for index, (joint_name, axis) in enumerate(AXES.items(), 1):
        origin = ORIGINS[f"link_{index}"]
        segment = np.vstack((origin - axis * 0.035, origin + axis * 0.035))
        ax.plot(segment[:, 0], segment[:, 1], segment[:, 2], c="#d62728", linewidth=2.0)
        ax.text(*origin, joint_name, fontsize=7)

    points = np.vstack(all_points)
    equal_axes(ax, points)
    ax.set_xlabel("base X (m)")
    ax.set_ylabel("base Y (m)")
    ax.set_zlabel("base Z (m)")
    ax.set_title("XiaoU six-axis ROS2 home model - CAD meshes and joint axes")
    ax.view_init(elev=24, azim=-58)
    figure.tight_layout()
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(OUTPUT)
    print(OUTPUT)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
