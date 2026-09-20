#!/usr/bin/env python3
"""Render a six-axis mesh replay from a simulation report, without hardware IO.

Frames prescribe joint positions with quintic interpolation. They illustrate the
planned approach path, not calibrated actuator dynamics or physical grasping.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import imageio.v2 as imageio
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np
from PIL import Image, ImageDraw


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--mjcf", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = json.loads(args.report.read_bytes())
    if not report.get("offline") or report.get("hardware_motion"):
        raise ValueError("an explicitly offline simulation report is required")
    args.output.mkdir(parents=True, exist_ok=True)
    model = mujoco.MjModel.from_xml_path(str(args.mjcf.resolve()))
    model.vis.global_.offwidth = 720
    model.vis.global_.offheight = 540
    model.vis.headlight.active = 1
    model.vis.headlight.ambient[:] = [0.55, 0.55, 0.55]
    model.vis.headlight.diffuse[:] = [0.75, 0.75, 0.75]
    model.geom_rgba[:] = [0.25, 0.52, 0.76, 1.0]
    for name, color in (("table", [0.75, 0.78, 0.82, 1]), ("target_geom", [0.84, 0.22, 0.2, 1]), ("jaw_left_proxy", [0.95, 0.72, 0.25, 1]), ("jaw_right_proxy", [0.95, 0.72, 0.25, 1])):
        geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        model.geom_rgba[geom] = color
    data = mujoco.MjData(model)
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [0.03, 0.02, 0.12]
    camera.distance = 1.15
    camera.azimuth = 135
    camera.elevation = -22
    target_joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "target_free")
    target_offset = model.jnt_qposadr[target_joint]
    center = report["world_alignment"]["object_center_m"]
    data.qpos[target_offset:target_offset + 3] = center
    data.qpos[target_offset + 3:target_offset + 7] = [1, 0, 0, 0]
    data.eq_active[:] = 0
    start = np.deg2rad(report["initial_robot_state"]["joint_deg"])
    renderer = mujoco.Renderer(model, height=540, width=720)
    option = mujoco.MjvOption()
    option.flags[mujoco.mjtVisFlag.mjVIS_CONVEXHULL] = 1
    images = []
    samples = []
    tcp_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "grasp_tcp")
    for index, stage in enumerate(report["playback"]):
        goal = np.deg2rad(stage["target_joint_deg"])
        for fraction in np.linspace(0, 1, 12):
            weight = 10 * fraction**3 - 15 * fraction**4 + 6 * fraction**5
            data.qpos[:6] = start + weight * (goal - start)
            mujoco.mj_forward(model, data)
            renderer.update_scene(data, camera=camera, scene_option=option)
            renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0
            frame = Image.fromarray(renderer.render().copy())
            draw = ImageDraw.Draw(frame)
            draw.rectangle((0, 0, 720, 42), fill=(245, 247, 250))
            draw.text((14, 8), f"XiaoU / MuJoCo kinematic replay / {index + 1}. {stage['name']}", fill=(25, 35, 45))
            draw.text((14, 25), "Collision hulls; gripper inactive; animation timing compressed", fill=(65, 75, 85))
            images.append(np.asarray(frame))
            samples.append({"stage": stage["name"], "frame": len(images) - 1, "joint_deg": np.rad2deg(data.qpos[:6]).tolist(), "tcp_m": data.xpos[tcp_id].tolist()})
        start = goal
    renderer.close()
    imageio.mimsave(args.output / "taught-approach.gif", images, duration=60, loop=0)
    indices = np.linspace(0, len(images) - 1, 4).round().astype(int)
    fig, axes = plt.subplots(2, 2, figsize=(12, 9), constrained_layout=True)
    for ax, index in zip(axes.flat, indices):
        ax.imshow(images[index]); ax.axis("off")
    fig.savefig(args.output / "taught-approach-overview.png", dpi=130)
    plt.close(fig)
    points = np.asarray([item["tcp_m"] for item in samples])
    fig, axes = plt.subplots(1, 2, figsize=(10, 4), constrained_layout=True)
    axes[0].plot(points[:, 0] * 1000, points[:, 1] * 1000, color="#1d4ed8")
    axes[0].set(xlabel="X [mm]", ylabel="Y [mm]", title="TCP trajectory / top view")
    axes[0].axis("equal")
    axes[1].plot(np.arange(len(points)), points[:, 2] * 1000, color="#1d4ed8")
    axes[1].set(xlabel="Replay frame", ylabel="Z [mm]", title="Vertical approach profile")
    for ax in axes: ax.grid(alpha=0.2)
    fig.savefig(args.output / "tcp-path.png", dpi=160)
    plt.close(fig)
    (args.output / "replay-frames.json").write_text(json.dumps({"kind": "kinematic_visualization", "hardware_motion": False, "gripper_active": False, "frames": samples}, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"frames": len(images), "stages": len(report["playback"]), "hardware_motion": False}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
