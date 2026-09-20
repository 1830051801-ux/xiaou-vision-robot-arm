"""Render a publishable dashboard for the constraint-diffusion digital twin."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
ROBOT_AI = ROOT / "robot_ai"
if str(ROBOT_AI) not in sys.path:
    sys.path.insert(0, str(ROBOT_AI))

from arm_control import fk_space, load_default_model
from arm_control.scene_review import load_diagnostic_scene


def load_json(path: Path) -> dict[str, object]:
    return json.loads(path.read_text(encoding="utf-8"))


def draw_box(ax, minimum: np.ndarray, maximum: np.ndarray, color: str) -> None:
    corners = np.array(
        [
            [minimum[0], minimum[1], minimum[2]],
            [maximum[0], minimum[1], minimum[2]],
            [maximum[0], maximum[1], minimum[2]],
            [minimum[0], maximum[1], minimum[2]],
            [minimum[0], minimum[1], maximum[2]],
            [maximum[0], minimum[1], maximum[2]],
            [maximum[0], maximum[1], maximum[2]],
            [minimum[0], maximum[1], maximum[2]],
        ]
    )
    for start, end in ((0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4), (0, 4), (1, 5), (2, 6), (3, 7)):
        ax.plot(*zip(corners[start], corners[end]), color=color, linewidth=1.2, alpha=0.85)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--nominal", type=Path, required=True)
    parser.add_argument("--shift", type=Path, required=True)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=ROOT / "assets" / "constraint_diffusion_dashboard.png")
    args = parser.parse_args()

    import torch

    nominal = load_json(args.nominal)
    shifted = load_json(args.shift)
    checkpoint = torch.load(args.checkpoint, map_location="cpu", weights_only=False)
    data = np.load(args.dataset)
    trajectory = data["trajectories"][0] * float(checkpoint["joint_limit_rad"])
    model = load_default_model()
    tcp = np.stack([fk_space(model.home_grasp_tcp, model.screw_axes, q)[:3, 3] for q in trajectory])
    scene = load_diagnostic_scene()

    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10, "axes.titleweight": "bold"})
    fig = plt.figure(figsize=(16, 9), facecolor="#f7f8fa")
    grid = fig.add_gridspec(2, 3, width_ratios=(1.45, 1.0, 1.0), height_ratios=(1.0, 0.9), wspace=0.35, hspace=0.38)

    ax3d = fig.add_subplot(grid[:, 0], projection="3d")
    x_mesh, y_mesh = np.meshgrid(np.linspace(0.10, 0.42, 2), np.linspace(-0.28, 0.22, 2))
    ax3d.plot_surface(x_mesh, y_mesh, np.zeros_like(x_mesh), color="#d9e6ee", alpha=0.5, shade=False)
    for box in scene.keep_out_boxes:
        draw_box(ax3d, box.minimum_m, box.maximum_m, "#d55656")
    ax3d.plot(tcp[:, 0], tcp[:, 1], tcp[:, 2], color="#1677a8", linewidth=2.8, label="expert TCP trajectory")
    ax3d.scatter(*tcp[0], color="#2b7a3d", s=44, label="home")
    ax3d.scatter(*tcp[10], color="#f0a202", s=48, label="grasp")
    ax3d.scatter(*tcp[23], color="#a33a8a", s=48, label="place")
    ax3d.set_title("Six-Axis Kinematic Digital Twin")
    ax3d.set_xlabel("X (m)")
    ax3d.set_ylabel("Y (m)")
    ax3d.set_zlabel("Z (m)")
    ax3d.set_xlim(0.10, 0.42)
    ax3d.set_ylim(-0.28, 0.22)
    ax3d.set_zlim(0.0, 0.50)
    ax3d.view_init(elev=25, azim=-58)
    ax3d.legend(loc="upper left", fontsize=8)

    ax_loss = fig.add_subplot(grid[0, 1])
    losses = checkpoint["losses"]
    ax_loss.plot(np.arange(1, len(losses) + 1), losses, color="#1677a8", linewidth=2)
    ax_loss.fill_between(np.arange(1, len(losses) + 1), losses, color="#1677a8", alpha=0.15)
    ax_loss.set_title("Action-Chunk Transformer Training")
    ax_loss.set_xlabel("Epoch")
    ax_loss.set_ylabel("Combined loss")
    ax_loss.grid(alpha=0.25)

    nominal_metrics = nominal["metrics"]
    ax_error = fig.add_subplot(grid[0, 2])
    error_labels = ["Raw\npolicy", "IK + scene\nprojection"]
    errors_mm = [
        float(nominal_metrics["raw_mean_grasp_position_error_m"]) * 1000.0,
        float(nominal_metrics["shielded_mean_grasp_position_error_m"]) * 1000.0,
    ]
    bars = ax_error.bar(error_labels, errors_mm, color=["#b34a4a", "#2b7a3d"], width=0.62)
    ax_error.set_title("Nominal Test Endpoint Error")
    ax_error.set_ylabel("Mean position error (mm)")
    ax_error.grid(axis="y", alpha=0.25)
    for bar, value in zip(bars, errors_mm):
        ax_error.text(bar.get_x() + bar.get_width() / 2, value + 0.5, f"{value:.1f}", ha="center", va="bottom", fontsize=10)

    ax_rate = fig.add_subplot(grid[1, 1:])
    labels = ["Nominal\nraw safe", "Nominal\nprojected safe", "Shift\nOOD abstain"]
    rates = [
        float(nominal_metrics["raw_scene_safe_rate"]) * 100.0,
        float(nominal_metrics["shielded_scene_safe_rate"]) * 100.0,
        float(shifted["metrics"]["ood_abstention_rate"]) * 100.0,
    ]
    bars = ax_rate.bar(labels, rates, color=["#487aa1", "#2b7a3d", "#d58a22"], width=0.58)
    ax_rate.set_title("Safety Gate Behavior")
    ax_rate.set_ylabel("Rate (%)")
    ax_rate.set_ylim(0, 108)
    ax_rate.grid(axis="y", alpha=0.25)
    for bar, value in zip(bars, rates):
        ax_rate.text(bar.get_x() + bar.get_width() / 2, value + 2, f"{value:.1f}%", ha="center", va="bottom", fontsize=10)

    fig.suptitle("XiaoU Embodied Action-Chunk Transformer: Learn, Project, Abstain", fontsize=16, fontweight="bold", x=0.49, y=0.98)
    fig.text(
        0.51,
        0.02,
        "Visual pose + goal + uncertainty tokens -> 32 x 6 action chunk -> diffusion refinement -> constraint projection. Offline only; no real arm motion.",
        ha="center",
        color="#5e6670",
        fontsize=9,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=180, bbox_inches="tight", facecolor=fig.get_facecolor())
    plt.close(fig)
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
