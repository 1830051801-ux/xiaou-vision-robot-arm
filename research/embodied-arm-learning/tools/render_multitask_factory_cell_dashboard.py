"""Render a task-conditioned factory-cell digital-twin benchmark dashboard."""

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
TOOLS = ROOT / "tools"
for directory in (ROBOT_AI, TOOLS):
    if str(directory) not in sys.path:
        sys.path.insert(0, str(directory))

from arm_control import fk_space, load_default_model
from arm_control.scene_review import load_diagnostic_scene
from constraint_diffusion_twin import ASSUMED_JOINT_LIMIT_RAD, TASK_NAMES


TASK_LABELS = {
    "transfer": "Transfer",
    "sort_zone_a": "Zone A",
    "sort_zone_b": "Zone B",
    "inspection_scan": "Inspect",
    "precision_insert": "Insert",
}
TASK_COLORS = {
    "transfer": "#1677a8",
    "sort_zone_a": "#2b7a3d",
    "sort_zone_b": "#a54b85",
    "inspection_scan": "#d58a22",
    "precision_insert": "#8a5ca8",
}


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
        ax.plot(*zip(corners[start], corners[end]), color=color, linewidth=1.0, alpha=0.72)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--nominal", type=Path, required=True)
    parser.add_argument("--shift", type=Path, required=True)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=ROOT / "assets" / "multitask_factory_cell_dashboard.png")
    args = parser.parse_args()

    import torch

    checkpoint = torch.load(args.checkpoint, map_location="cpu", weights_only=False)
    nominal = load_json(args.nominal)
    shifted = load_json(args.shift)
    data = np.load(args.dataset)
    task_ids = data["task_ids"]
    task_metrics = nominal["task_metrics"]
    shift_metrics = shifted["task_metrics"]

    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 9, "axes.titleweight": "bold"})
    fig = plt.figure(figsize=(17, 13), facecolor="#f7f8fa")
    grid = fig.add_gridspec(3, 3, width_ratios=(1.45, 1.0, 1.0), hspace=0.44, wspace=0.34)

    ax3d = fig.add_subplot(grid[:, 0], projection="3d")
    x_mesh, y_mesh = np.meshgrid(np.linspace(0.10, 0.42, 2), np.linspace(-0.28, 0.22, 2))
    ax3d.plot_surface(x_mesh, y_mesh, np.zeros_like(x_mesh), color="#d9e6ee", alpha=0.5, shade=False)
    scene = load_diagnostic_scene()
    for box in scene.keep_out_boxes:
        draw_box(ax3d, box.minimum_m, box.maximum_m, "#d55656")
    arm = load_default_model()
    for task_id, task_name in enumerate(TASK_NAMES):
        matches = np.flatnonzero(task_ids == task_id)
        if not len(matches):
            continue
        trajectory = data["trajectories"][int(matches[0])] * ASSUMED_JOINT_LIMIT_RAD
        tcp = np.stack([fk_space(arm.home_grasp_tcp, arm.screw_axes, angles)[:3, 3] for angles in trajectory])
        ax3d.plot(tcp[:, 0], tcp[:, 1], tcp[:, 2], color=TASK_COLORS[task_name], linewidth=2.1, label=TASK_LABELS[task_name])
        ax3d.scatter(*tcp[10], color=TASK_COLORS[task_name], s=20)
    ax3d.set_title("Task-Conditioned Six-Axis TCP Paths")
    ax3d.set_xlabel("X (m)")
    ax3d.set_ylabel("Y (m)")
    ax3d.set_zlabel("Z (m)")
    ax3d.set_xlim(0.10, 0.42)
    ax3d.set_ylim(-0.28, 0.22)
    ax3d.set_zlim(0.0, 0.50)
    ax3d.view_init(elev=25, azim=-58)
    ax3d.legend(loc="upper left", fontsize=8)

    labels = [TASK_LABELS[name] for name in TASK_NAMES]
    positions = np.arange(len(TASK_NAMES))
    raw_errors = [float(task_metrics[name]["raw_mean_grasp_position_error_m"]) * 1000.0 for name in TASK_NAMES]
    projected_errors = [
        float(task_metrics[name]["shielded_mean_grasp_position_error_m"] or 0.0) * 1000.0 for name in TASK_NAMES
    ]
    ax_error = fig.add_subplot(grid[0, 1])
    width = 0.36
    ax_error.bar(positions - width / 2, raw_errors, width, color="#b34a4a", label="raw policy")
    ax_error.bar(positions + width / 2, projected_errors, width, color="#2b7a3d", label="IK + scene")
    ax_error.set_title("Per-Task Grasp Endpoint Error")
    ax_error.set_ylabel("Mean error (mm)")
    ax_error.set_xticks(positions, labels, rotation=20)
    ax_error.grid(axis="y", alpha=0.25)
    ax_error.legend(fontsize=8)

    ax_safe = fig.add_subplot(grid[0, 2])
    projected_safe = [float(task_metrics[name]["projected_safe_rate"]) * 100.0 for name in TASK_NAMES]
    consensus_safe = [float(task_metrics[name].get("counterfactual_consensus_rate", 0.0)) * 100.0 for name in TASK_NAMES]
    width = 0.36
    bars = ax_safe.bar(positions - width / 2, projected_safe, width, color=[TASK_COLORS[name] for name in TASK_NAMES], label="single-view projection")
    consensus_bars = ax_safe.bar(positions + width / 2, consensus_safe, width, color="#34495e", label="4-rollout consensus")
    ax_safe.set_title("Projection vs Counterfactual Consensus")
    ax_safe.set_ylabel("Rate (%)")
    ax_safe.set_ylim(0, 108)
    ax_safe.set_xticks(positions, labels, rotation=20)
    ax_safe.grid(axis="y", alpha=0.25)
    ax_safe.legend(fontsize=7, loc="lower left")
    for bar, value in zip(consensus_bars, consensus_safe):
        ax_safe.text(bar.get_x() + bar.get_width() / 2, value + 2.0, f"{value:.1f}%", ha="center", va="bottom", fontsize=8)

    ax_loss = fig.add_subplot(grid[1, 1])
    losses = checkpoint["losses"]
    ax_loss.plot(np.arange(1, len(losses) + 1), losses, color="#1677a8", linewidth=1.8)
    ax_loss.fill_between(np.arange(1, len(losses) + 1), losses, color="#1677a8", alpha=0.14)
    ax_loss.set_title("Process-Graph Training Loss")
    ax_loss.set_xlabel("Epoch")
    ax_loss.set_ylabel("Combined loss")
    ax_loss.grid(alpha=0.25)

    ax_ood = fig.add_subplot(grid[1, 2])
    ood_rates = [float(shift_metrics[name]["ood_abstention_rate"]) * 100.0 for name in TASK_NAMES]
    bars = ax_ood.bar(labels, ood_rates, color="#d58a22", width=0.62)
    ax_ood.set_title("High-Noise OOD Rejection")
    ax_ood.set_ylabel("Abstention rate (%)")
    ax_ood.set_ylim(0, 108)
    ax_ood.tick_params(axis="x", rotation=20)
    ax_ood.grid(axis="y", alpha=0.25)
    for bar, value in zip(bars, ood_rates):
        ax_ood.text(bar.get_x() + bar.get_width() / 2, value + 2.0, f"{value:.1f}%", ha="center", va="bottom", fontsize=8)

    belief = nominal.get("belief_space_planning")
    ax_fusion = fig.add_subplot(grid[2, 1])
    ax_risk = fig.add_subplot(grid[2, 2])
    if isinstance(belief, dict) and belief.get("enabled"):
        belief_metrics = belief["metrics"]
        pose_errors = [
            float(belief_metrics["primary_mean_pose_error_m"]) * 1000.0,
            float(belief_metrics["fused_mean_pose_error_m"]) * 1000.0,
        ]
        fusion_bars = ax_fusion.bar(
            ["primary\nobservation", "robust\nbelief"],
            pose_errors,
            color=["#b34a4a", "#2b7a3d"],
            width=0.58,
        )
        reduction = float(belief_metrics["pose_error_reduction_fraction"]) * 100.0
        ax_fusion.set_title("Robust Multi-Observation Pose Belief")
        ax_fusion.set_ylabel("Mean pose error (mm)")
        ax_fusion.set_ylim(0, max(pose_errors) * 1.28)
        ax_fusion.grid(axis="y", alpha=0.25)
        for bar, value in zip(fusion_bars, pose_errors):
            ax_fusion.text(bar.get_x() + bar.get_width() / 2, value + max(pose_errors) * 0.04, f"{value:.2f}", ha="center", va="bottom", fontsize=8)
        ax_fusion.text(0.5, 0.93, f"error reduction {reduction:.1f}%", transform=ax_fusion.transAxes, ha="center", va="top", color="#2b7a3d", fontsize=9, fontweight="bold")

        selection_values = [
            float(belief_metrics["selected_safe_rate"]) * 100.0,
            float(belief_metrics["mean_selected_scenario_safe_fraction"] or 0.0) * 100.0,
            float(belief_metrics["mean_inlier_fraction"]) * 100.0,
        ]
        selection_bars = ax_risk.bar(
            ["accepted\nplans", "selected\nscenarios", "view\ninliers"],
            selection_values,
            color=["#345d8c", "#7957a6", "#d58a22"],
            width=0.60,
        )
        ax_risk.set_title("Scenario-CVaR Candidate Selection")
        ax_risk.set_ylabel("Rate (%)")
        ax_risk.set_ylim(0, 120)
        ax_risk.grid(axis="y", alpha=0.25)
        for bar, value in zip(selection_bars, selection_values):
            ax_risk.text(bar.get_x() + bar.get_width() / 2, value + 2.0, f"{value:.1f}%", ha="center", va="bottom", fontsize=8)
        clearance = float(belief_metrics["mean_selected_clearance_cvar_m"] or 0.0) * 1000.0
        ax_risk.text(0.5, 0.93, f"lower-tail CVaR clearance: {clearance:.1f} mm", transform=ax_risk.transAxes, ha="center", va="top", color="#34495e", fontsize=8)
    else:
        for axis, title in ((ax_fusion, "Robust Multi-Observation Pose Belief"), (ax_risk, "Scenario-CVaR Candidate Selection")):
            axis.set_title(title)
            axis.text(0.5, 0.5, "Not enabled in this report", ha="center", va="center", color="#5e6670")
            axis.set_xticks([])
            axis.set_yticks([])

    episodes_per_task = nominal.get("episodes_per_task")
    evaluation_label = (
        f"Balanced offline stress run: {nominal['episodes']} episodes / {episodes_per_task} per task"
        if episodes_per_task
        else f"Offline evaluation: {nominal['episodes']} episodes"
    )
    fig.subplots_adjust(top=0.90, bottom=0.07)
    fig.suptitle("XiaoU Process-Graph Multi-Task Digital Twin Benchmark", fontsize=16, fontweight="bold", x=0.51, y=0.965)
    fig.text(0.51, 0.943, evaluation_label, ha="center", color="#5e6670", fontsize=9)
    fig.text(
        0.51,
        0.025,
        "POE kinematics | URDF/ROS 2 review | process-graph Action-Chunk Transformer | robust pose belief | scenario-CVaR selection | offline only",
        ha="center",
        color="#5e6670",
        fontsize=9,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=180, facecolor=fig.get_facecolor())
    plt.close(fig)
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
