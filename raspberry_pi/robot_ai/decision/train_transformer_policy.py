#!/usr/bin/env python3
"""Train the task-level Transformer on explicit synthetic demonstrations.

The labels are deterministic strategy demonstrations, not real grasp data. A
checkpoint trained here proves the ROS/GPU data path only; real grasp episodes
must replace these labels before hardware use.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from simulation.desktop_scene import DesktopScene, OBJECT_CLASSES, OBSERVATION_DIM, STRATEGIES
from robot_ai.decision.transformer_policy import DecisionTransformerPolicy, TASK_TO_ID


ACTION_LOSS_WEIGHT = 1.0


def _build_dataset(
    episodes: int,
    sequence_length: int,
    seed: int,
    *,
    safety_negative_ratio: float = 0.0,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    rng = np.random.default_rng(seed)
    features: list[np.ndarray] = []
    tasks: list[int] = []
    strategy_targets: list[int] = []
    phase_targets: list[int] = []
    action_targets: list[list[float]] = []
    risk_targets: list[float] = []
    for episode in range(episodes):
        scenario = OBJECT_CLASSES[episode % len(OBJECT_CLASSES)]
        episode_rng = np.random.default_rng(seed + episode)
        position = (
            float(episode_rng.uniform(0.18, 0.38)),
            float(episode_rng.uniform(-0.18, 0.18)),
            0.0,
        )
        scene = DesktopScene(
            scenario=scenario,
            seed=seed + episode,
            object_position_m=position,
            object_yaw_rad=float(episode_rng.uniform(-np.pi, np.pi)),
        )
        if scenario != "tissue_pull":
            scene.target.phase = ("observe", "approach", "grasp", "lift")[episode_rng.integers(0, 4)]
        candidate = scene.candidates()[0]
        base = scene.observation_vector()
        is_safety_negative = safety_negative_ratio > 0.0 and float(episode_rng.random()) < safety_negative_ratio
        if is_safety_negative:
            negative_type = ("low_confidence", "unknown_class", "collision_risk", "joint_offline", "outside_workspace")[
                episode % 5
            ]
            if negative_type == "low_confidence":
                base[8] = float(episode_rng.uniform(0.01, 0.14))
            elif negative_type == "unknown_class":
                base[: len(OBJECT_CLASSES)] = 0.0
            elif negative_type == "collision_risk":
                base[43] = 0.0
            elif negative_type == "joint_offline":
                base[29 + int(episode_rng.integers(0, 6))] = 0.0
            elif negative_type == "outside_workspace":
                base[9] = float(episode_rng.choice((-0.05, 0.65)))
            strategy_target = STRATEGIES.index("reject")
            phase_target = ("observe", "approach", "grasp", "lift", "pull", "abort").index("abort")
            action_target = [0.0] * 6
            risk_target = 0.95
        else:
            strategy_target = STRATEGIES.index(candidate.strategy)
            phase_target = ("observe", "approach", "grasp", "lift", "pull", "abort").index(candidate.phase)
            action_target = [
                candidate.target_xyz_m[0], candidate.target_xyz_m[1], candidate.target_xyz_m[2],
                candidate.gripper_open_m, candidate.gripper_close_m, candidate.pull_distance_m,
            ]
            risk_target = 0.2 if scenario == "tissue_pull" else 0.1
        sequence = np.stack([base + rng.normal(0.0, 0.002, OBSERVATION_DIM).astype(np.float32) for _ in range(sequence_length)])
        features.append(sequence)
        tasks.append(TASK_TO_ID[scenario])
        strategy_targets.append(strategy_target)
        phase_targets.append(phase_target)
        action_targets.append(action_target)
        risk_targets.append(risk_target)
    return (
        np.asarray(features, dtype=np.float32),
        np.asarray(tasks, dtype=np.int64),
        np.asarray(strategy_targets, dtype=np.int64),
        np.asarray(phase_targets, dtype=np.int64),
        np.asarray(action_targets, dtype=np.float32),
        np.asarray(risk_targets, dtype=np.float32),
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episodes", type=int, default=256)
    parser.add_argument("--sequence-length", type=int, default=8)
    parser.add_argument("--epochs", type=int, default=8)
    parser.add_argument("--batch-size", type=int, default=32)
    parser.add_argument("--val-ratio", type=float, default=0.2)
    parser.add_argument("--seed", type=int, default=20260809)
    parser.add_argument("--safety-negative-ratio", type=float, default=0.0)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--output", type=Path, default=PROJECT_ROOT / "runtime" / "decision" / "transformer_policy.pt")
    parser.add_argument("--metrics", type=Path, default=PROJECT_ROOT / "runtime" / "decision" / "transformer_training.json")
    args = parser.parse_args()
    if args.episodes < 2 or args.sequence_length < 1 or args.epochs < 1 or args.batch_size < 1:
        parser.error("episodes must be at least 2; sequence-length, epochs, and batch-size must be positive")
    if not 0.05 <= args.val_ratio < 0.5:
        parser.error("val-ratio must be in [0.05, 0.5)")
    if not 0.0 <= args.safety_negative_ratio <= 0.8:
        parser.error("safety-negative-ratio must be in [0.0, 0.8]")

    import torch
    import torch.nn.functional as F

    torch.manual_seed(args.seed)
    features, tasks, strategy_targets, phase_targets, action_targets, risk_targets = _build_dataset(
        args.episodes,
        args.sequence_length,
        args.seed,
        safety_negative_ratio=args.safety_negative_ratio,
    )
    policy = DecisionTransformerPolicy(device=args.device, max_seq_len=max(16, args.sequence_length))
    optimizer = torch.optim.AdamW(policy.parameters(), lr=3e-4, weight_decay=1e-4)
    x = torch.from_numpy(features)
    task = torch.from_numpy(tasks)
    strategy = torch.from_numpy(strategy_targets)
    phase = torch.from_numpy(phase_targets)
    actions = torch.from_numpy(action_targets)
    risk = torch.from_numpy(risk_targets)

    split_generator = torch.Generator().manual_seed(args.seed)
    split = torch.randperm(args.episodes, generator=split_generator)
    val_count = max(1, int(round(args.episodes * args.val_ratio)))
    val_indices = split[:val_count]
    train_indices = split[val_count:]

    def evaluate(indices: torch.Tensor) -> dict[str, float]:
        policy.net.eval()
        with torch.no_grad():
            output = policy.forward(x[indices], task[indices])
            target_strategy = strategy[indices].to(policy.device)
            target_phase = phase[indices].to(policy.device)
            target_actions = actions[indices].to(policy.device)
            target_risk = risk[indices].to(policy.device)
            action_mask = target_strategy != STRATEGIES.index("reject")
            loss_strategy = F.cross_entropy(output["strategy_logits"], target_strategy)
            loss_phase = F.cross_entropy(output["phase_logits"], target_phase)
            loss_action = (
                F.smooth_l1_loss(output["action_params"][action_mask], target_actions[action_mask])
                if bool(action_mask.any())
                else output["action_params"].sum() * 0.0
            )
            loss_risk = F.binary_cross_entropy_with_logits(output["risk_logit"], target_risk)
            loss = loss_strategy + 0.5 * loss_phase + ACTION_LOSS_WEIGHT * loss_action + 0.1 * loss_risk
            count = float(indices.numel())
            return {
                "loss": float(loss.item()),
                "strategy_accuracy": float((output["strategy_logits"].argmax(-1).cpu() == strategy[indices]).sum().item()) / count,
                "phase_accuracy": float((output["phase_logits"].argmax(-1).cpu() == phase[indices]).sum().item()) / count,
                "action_mae": float(
                    torch.abs(output["action_params"][action_mask] - target_actions[action_mask]).mean().item()
                ) if bool(action_mask.any()) else 0.0,
            }

    history: list[dict[str, float]] = []
    for epoch in range(1, args.epochs + 1):
        permutation = train_indices[torch.randperm(train_indices.numel())]
        totals = {"loss": 0.0, "strategy_acc": 0.0, "phase_acc": 0.0, "count": 0.0}
        policy.net.train()
        for start in range(0, train_indices.numel(), args.batch_size):
            indices = permutation[start : start + args.batch_size]
            output = policy.forward(x[indices], task[indices])
            loss_strategy = F.cross_entropy(output["strategy_logits"], strategy[indices].to(policy.device))
            loss_phase = F.cross_entropy(output["phase_logits"], phase[indices].to(policy.device))
            loss_risk = F.binary_cross_entropy_with_logits(output["risk_logit"], risk[indices].to(policy.device))
            action_mask = strategy[indices].to(policy.device) != STRATEGIES.index("reject")
            loss_action = (
                F.smooth_l1_loss(output["action_params"][action_mask], actions[indices].to(policy.device)[action_mask])
                if bool(action_mask.any())
                else output["action_params"].sum() * 0.0
            )
            loss = loss_strategy + 0.5 * loss_phase + ACTION_LOSS_WEIGHT * loss_action + 0.1 * loss_risk
            optimizer.zero_grad(set_to_none=True)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(policy.parameters(), 1.0)
            optimizer.step()
            count = float(indices.numel())
            totals["loss"] += float(loss.item()) * count
            totals["strategy_acc"] += float((output["strategy_logits"].argmax(-1).cpu() == strategy[indices]).sum().item())
            totals["phase_acc"] += float((output["phase_logits"].argmax(-1).cpu() == phase[indices]).sum().item())
            totals["count"] += count
        train_metrics = {
            "loss": totals["loss"] / totals["count"],
            "strategy_accuracy": totals["strategy_acc"] / totals["count"],
            "phase_accuracy": totals["phase_acc"] / totals["count"],
        }
        val_metrics = evaluate(val_indices)
        history.append({
            "epoch": float(epoch),
            "loss": train_metrics["loss"],
            "strategy_accuracy": train_metrics["strategy_accuracy"],
            "phase_accuracy": train_metrics["phase_accuracy"],
            "val_loss": val_metrics["loss"],
            "val_strategy_accuracy": val_metrics["strategy_accuracy"],
            "val_phase_accuracy": val_metrics["phase_accuracy"],
            "val_action_mae": val_metrics["action_mae"],
        })

    policy.save(args.output, {
        "synthetic_demonstrations": True,
        "episodes": args.episodes,
        "train_episodes": int(train_indices.numel()),
        "validation_episodes": int(val_indices.numel()),
        "sequence_length": args.sequence_length,
        "safety_negative_ratio": args.safety_negative_ratio,
        "safety_negative_episodes": int((strategy_targets == STRATEGIES.index("reject")).sum()),
        "val_ratio": args.val_ratio,
        "action_loss_weight": ACTION_LOSS_WEIGHT,
        "device": str(policy.device),
    })
    report = {
        "offline": True,
        "hardware_motion": False,
        "training_data": "deterministic synthetic scene demonstrations",
        "gpu_device": str(policy.device),
        "cuda_available": bool(torch.cuda.is_available()),
        "episodes": args.episodes,
        "train_episodes": int(train_indices.numel()),
        "validation_episodes": int(val_indices.numel()),
        "val_ratio": args.val_ratio,
        "sequence_length": args.sequence_length,
        "safety_negative_ratio": args.safety_negative_ratio,
        "safety_negative_episodes": int((strategy_targets == STRATEGIES.index("reject")).sum()),
        "observation_dim": OBSERVATION_DIM,
        "history": history,
        "final_validation": history[-1],
        "action_loss_weight": ACTION_LOSS_WEIGHT,
        "checkpoint": str(args.output),
        "warning": "Synthetic labels validate architecture only; collect real camera/feedback grasp episodes before hardware decisions.",
    }
    args.metrics.parent.mkdir(parents=True, exist_ok=True)
    args.metrics.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
