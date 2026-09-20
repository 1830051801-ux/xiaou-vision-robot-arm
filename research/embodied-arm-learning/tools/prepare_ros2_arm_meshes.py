from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import numpy as np


LOCAL_MESH_PACKAGES = Path(r"D:\机械臂\.codex_tmp_py")
if LOCAL_MESH_PACKAGES.is_dir():
    sys.path.insert(0, str(LOCAL_MESH_PACKAGES))

import trimesh


R_STEP_FROM_BASE = np.array(
    [
        [-0.194580976621, 0.980886458025, 0.0],
        [0.0, 0.0, 1.0],
        [0.980886458025, 0.194580976621, 0.0],
    ],
    dtype=np.float64,
)
R_BASE_FROM_STEP = R_STEP_FROM_BASE.T
BASE_ORIGIN_STEP_MM = np.array([-0.000007933, 0.0, 0.000042735], dtype=np.float64)

LINK_ORIGINS_BASE_M = {
    "base_link": np.array([0.0, 0.0, 0.0]),
    "link_1": np.array([0.0, 0.0, 0.0]),
    "link_2": np.array([-0.000000017765, 0.0, 0.155999957265]),
    "link_3": np.array([0.179999573587, -0.000000000534, 0.156367431963]),
    "link_4": np.array([0.189786868388, -0.000000000534, 0.336101183735]),
    "link_5": np.array([0.189786868332, 0.092999999466, 0.336101186004]),
    "link_6": np.array([0.295755047669, 0.092999999399, 0.338701799816]),
}

# Each top-level CAD product contains the structure immediately before the
# next joint. The mapping is based on the measured CAD bounding boxes and must
# be visually checked in RViz before collision planning is accepted.
COMPONENTS = (
    ("link_1_world.stl", "base_link", "base_body"),
    ("link_2_world.stl", "link_1", "joint_1_body"),
    ("link_3_world.stl", "link_2", "link_2_body"),
    ("link_4_world.stl", "link_3", "link_3_body"),
    ("link_5_world.stl", "link_4", "joint_5_body"),
    ("link_6_world.stl", "link_5", "joint_6_body"),
    ("wrist_connector_world.stl", "link_6", "wrist_connector"),
    ("gripper_world.stl", "link_6", "gripper"),
)


def load_single_mesh(path: Path) -> trimesh.Trimesh:
    loaded = trimesh.load(path, force="scene", process=False)
    if isinstance(loaded, trimesh.Scene):
        mesh = loaded.to_mesh()
    else:
        mesh = loaded
    if not isinstance(mesh, trimesh.Trimesh) or mesh.faces.size == 0:
        raise RuntimeError(f"no triangles loaded from {path}")
    return mesh


def transform_to_link_frame(mesh: trimesh.Trimesh, link_name: str) -> None:
    vertices_step_mm = np.asarray(mesh.vertices, dtype=np.float64)
    vertices_base_m = ((vertices_step_mm - BASE_ORIGIN_STEP_MM) @ R_BASE_FROM_STEP.T) * 0.001
    mesh.vertices = vertices_base_m - LINK_ORIGINS_BASE_M[link_name]


def simplify(mesh: trimesh.Trimesh, face_count: int) -> trimesh.Trimesh:
    if len(mesh.faces) <= face_count:
        return mesh.copy()
    result = mesh.simplify_quadric_decimation(face_count=face_count, aggression=7)
    result.remove_unreferenced_vertices()
    return result


def export_mesh(mesh: trimesh.Trimesh, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    mesh.export(path, file_type="stl")
    if not path.is_file() or path.stat().st_size < 84:
        raise RuntimeError(f"invalid STL output: {path}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Build link-local ROS 2 meshes from STEP-world STL files")
    parser.add_argument(
        "--source-dir",
        type=Path,
        default=Path(r"D:\机械臂\robot_geometry_renders\world_model"),
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(__file__).resolve().parents[1]
        / "ros2_ws"
        / "src"
        / "xiaou_arm_description"
        / "meshes",
    )
    parser.add_argument("--visual-faces", type=int, default=80000)
    parser.add_argument("--collision-faces", type=int, default=12000)
    args = parser.parse_args()
    if args.visual_faces <= 0 or args.collision_faces <= 0:
        raise SystemExit("face targets must be positive")

    manifest: dict[str, object] = {
        "source_frame": "STEP world in millimetres",
        "output_frame": "URDF link-local in metres",
        "rigid_body_mapping_status": "CAD-derived provisional; RViz articulation review required",
        "visual_target_faces": args.visual_faces,
        "collision_target_faces": args.collision_faces,
        "components": [],
    }
    for source_name, link_name, output_stem in COMPONENTS:
        source = args.source_dir / source_name
        if not source.is_file():
            raise FileNotFoundError(source)
        mesh = load_single_mesh(source)
        source_faces = len(mesh.faces)
        transform_to_link_frame(mesh, link_name)
        visual = simplify(mesh, args.visual_faces)
        collision = simplify(mesh, args.collision_faces)
        visual_path = args.output_dir / "visual" / f"{output_stem}.stl"
        collision_path = args.output_dir / "collision" / f"{output_stem}.stl"
        export_mesh(visual, visual_path)
        export_mesh(collision, collision_path)
        row = {
            "source": str(source),
            "link": link_name,
            "visual": visual_path.name,
            "collision": collision_path.name,
            "source_faces": source_faces,
            "visual_faces": len(visual.faces),
            "collision_faces": len(collision.faces),
            "bounds_link_m": np.asarray(mesh.bounds).round(9).tolist(),
        }
        manifest["components"].append(row)
        print(json.dumps(row, ensure_ascii=False))

    manifest_path = args.output_dir / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"manifest: {manifest_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
