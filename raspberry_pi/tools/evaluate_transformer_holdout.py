#!/usr/bin/env python3
"""Compare Transformer checkpoints on an independently seeded offline holdout.

The evaluator reuses the synthetic scene generator used by training, but it
builds a fresh dataset from a caller-provided seed and never opens ROS, serial,
CAN, a camera, or a motion path.  It is intended for relative model selection;
it is not evidence of real-world grasp success.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from robot_ai.decision.train_transformer_policy import _build_dataset
from robot_ai.decision.transformer_policy import DecisionTransformerPolicy
from simulation.desktop_scene import OBJECT_CLASSES


def evaluate_checkpoint(
    checkpoint: Path,
    features: np.ndarray,
    tasks: np.ndarray,
    strategy_targets: np.ndarray,
    phase_targets: np.ndarray,
    action_targets: np.ndarray,
    risk_targets: np.ndarray,
    *,
    device: str,
    batch_size: int,
) -> dict:
    import torch

    policy = DecisionTransformerPolicy(device=device, max_seq_len=max(16, features.shape[1]))
    metadata = policy.load(checkpoint)
    strategy_correct = 0
    phase_correct = 0
    action_abs_sum = 0.0
    action_abs_max = 0.0
    risk_abs_sum = 0.0
    count = int(features.shape[0])
    reject_index = 5
    reject_count = 0
    reject_correct = 0
    positive_count = 0
    positive_strategy_correct = 0
    action_count = 0

    per_scenario = {
        name: {
            "count": 0,
            "strategy_correct": 0,
            "phase_correct": 0,
            "action_abs_sum": 0.0,
            "action_count": 0,
        }
        for name in OBJECT_CLASSES
    }

    policy.net.eval()
    with torch.no_grad():
        for start in range(0, count, batch_size):
            stop = min(count, start + batch_size)
            output = policy.forward(
                torch.from_numpy(features[start:stop]),
                torch.from_numpy(tasks[start:stop]),
            )
            strategy_pred = output["strategy_logits"].argmax(-1).cpu().numpy()
            phase_pred = output["phase_logits"].argmax(-1).cpu().numpy()
            action_pred = output["action_params"].cpu().numpy()
            risk_pred = torch.sigmoid(output["risk_logit"]).cpu().numpy()
            target_strategy = strategy_targets[start:stop]
            positive_mask = target_strategy != reject_index
            reject_mask = ~positive_mask
            action_delta = np.abs(action_pred - action_targets[start:stop])
            risk_delta = np.abs(risk_pred - risk_targets[start:stop])
            strategy_correct += int((strategy_pred == target_strategy).sum())
            phase_correct += int((phase_pred == phase_targets[start:stop]).sum())
            reject_count += int(reject_mask.sum())
            reject_correct += int((strategy_pred[reject_mask] == reject_index).sum())
            positive_count += int(positive_mask.sum())
            positive_strategy_correct += int((strategy_pred[positive_mask] == target_strategy[positive_mask]).sum())
            if bool(positive_mask.any()):
                positive_delta = action_delta[positive_mask]
                action_abs_sum += float(positive_delta.sum())
                action_count += int(positive_delta.size)
                action_abs_max = max(action_abs_max, float(positive_delta.max()))
            risk_abs_sum += float(risk_delta.sum())

            for local_index, global_index in enumerate(range(start, stop)):
                scenario = OBJECT_CLASSES[global_index % len(OBJECT_CLASSES)]
                row = per_scenario[scenario]
                row["count"] += 1
                row["strategy_correct"] += int(strategy_pred[local_index] == strategy_targets[global_index])
                row["phase_correct"] += int(phase_pred[local_index] == phase_targets[global_index])
                if strategy_targets[global_index] != reject_index:
                    row["action_abs_sum"] += float(action_delta[local_index].sum())
                    row["action_count"] += int(action_delta[local_index].size)

    scenario_metrics = {}
    for name, row in per_scenario.items():
        scenario_metrics[name] = {
            "count": row["count"],
            "strategy_accuracy": row["strategy_correct"] / row["count"],
            "phase_accuracy": row["phase_correct"] / row["count"],
            "action_mae": row["action_abs_sum"] / row["action_count"] if row["action_count"] else 0.0,
        }

    return {
        "checkpoint": str(checkpoint.resolve()),
        "checkpoint_metadata": metadata,
        "strategy_accuracy": strategy_correct / count,
        "phase_accuracy": phase_correct / count,
        "positive_strategy_accuracy": positive_strategy_correct / positive_count if positive_count else 1.0,
        "reject_recall": reject_correct / reject_count if reject_count else 1.0,
        "unsafe_accept_count": reject_count - reject_correct,
        "unsafe_accept_rate": (reject_count - reject_correct) / reject_count if reject_count else 0.0,
        "positive_count": positive_count,
        "reject_count": reject_count,
        "action_mae": action_abs_sum / action_count if action_count else 0.0,
        "action_max_abs_error": action_abs_max,
        "risk_mae": risk_abs_sum / count,
        "per_scenario": scenario_metrics,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoints", required=True, help="comma-separated PyTorch checkpoints")
    parser.add_argument("--episodes", type=int, default=4096)
    parser.add_argument("--sequence-length", type=int, default=8)
    parser.add_argument("--seed", type=int, default=20260901)
    parser.add_argument("--safety-negative-ratio", type=float, default=0.0)
    parser.add_argument("--batch-size", type=int, default=256)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.episodes < len(OBJECT_CLASSES) or args.sequence_length < 1 or args.batch_size < 1:
        parser.error("episodes must cover all scenarios; sequence-length and batch-size must be positive")

    checkpoints = [Path(value.strip()) for value in args.checkpoints.split(",") if value.strip()]
    if not checkpoints:
        parser.error("at least one checkpoint is required")
    checkpoints = [path if path.is_absolute() else PROJECT_ROOT / path for path in checkpoints]
    missing = [str(path) for path in checkpoints if not path.is_file()]
    if missing:
        raise FileNotFoundError("missing checkpoints: " + ", ".join(missing))

    if not 0.0 <= args.safety_negative_ratio <= 0.8:
        parser.error("safety-negative-ratio must be in [0.0, 0.8]")
    dataset = _build_dataset(
        args.episodes,
        args.sequence_length,
        args.seed,
        safety_negative_ratio=args.safety_negative_ratio,
    )
    rows = [
        evaluate_checkpoint(
            checkpoint,
            *dataset,
            device=args.device,
            batch_size=args.batch_size,
        )
        for checkpoint in checkpoints
    ]
    ranked = sorted(
        rows,
        key=lambda row: (
            -row["reject_recall"],
            -row["positive_strategy_accuracy"],
            -row["strategy_accuracy"],
            -row["phase_accuracy"],
            row["action_mae"],
            row["risk_mae"],
        ),
    )
    report = {
        "offline": True,
        "hardware_motion": False,
        "serial_opened": False,
        "can_opened": False,
        "holdout": {
            "generator": "fresh deterministic synthetic scenes",
            "episodes": args.episodes,
            "sequence_length": args.sequence_length,
            "seed": args.seed,
            "safety_negative_ratio": args.safety_negative_ratio,
        },
        "models": rows,
        "best_checkpoint": ranked[0]["checkpoint"],
        "selection_rule": "reject recall, positive strategy accuracy, overall strategy accuracy, phase accuracy, action MAE, then risk MAE",
        "warning": "Synthetic holdout ranks architecture candidates only; it does not measure real grasp success.",
    }
    output = args.output if args.output.is_absolute() else PROJECT_ROOT / args.output
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
