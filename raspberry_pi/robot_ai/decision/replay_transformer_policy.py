#!/usr/bin/env python3
"""Replay the trained Transformer on deterministic desktop scenes."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from simulation.desktop_scene import DesktopScene, OBJECT_SPECS
from robot_ai.decision.onnx_policy import OnnxDecisionTransformerPolicy
from robot_ai.decision.transformer_policy import DecisionTransformerPolicy, SafetyGate, TASK_TO_ID


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=Path, default=PROJECT_ROOT / "runtime" / "decision" / "transformer_policy.pt")
    parser.add_argument("--scenarios", default=",".join(OBJECT_SPECS))
    parser.add_argument("--device", default="auto")
    parser.add_argument("--output", type=Path, help="optional JSON report path")
    args = parser.parse_args()
    checkpoint = args.checkpoint if args.checkpoint.is_absolute() else PROJECT_ROOT / args.checkpoint
    if checkpoint.suffix.lower() == ".onnx":
        policy = OnnxDecisionTransformerPolicy(checkpoint, max_seq_len=16)
        metadata = policy.metadata
        backend = "onnxruntime-cpu"
    else:
        policy = DecisionTransformerPolicy(device=args.device, max_seq_len=16)
        metadata = policy.load(checkpoint)
        policy.net.eval()
        backend = f"pytorch-{policy.device}"
    gate = SafetyGate(require_motion_enabled=False, require_hardware_ready=False, require_collision_free=True, require_feedback_verified=False)
    reports = []
    for index, scenario in enumerate(value.strip().lower() for value in args.scenarios.split(",") if value.strip()):
        scene = DesktopScene(scenario, seed=20260809 + index)
        observation = scene.observation()
        decision = policy.predict(scene.observation_vector(), TASK_TO_ID[scenario], gate, observation)
        reports.append({"scenario": scenario, "decision": decision.as_dict(), "rule_candidate": scene.candidates()[0].as_dict()})
    report = {
        "offline": True,
        "hardware_motion": False,
        "backend": backend,
        "checkpoint": str(checkpoint),
        "checkpoint_metadata": metadata,
        "reports": reports,
    }
    if args.output is not None:
        output = args.output if args.output.is_absolute() else PROJECT_ROOT / args.output
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
