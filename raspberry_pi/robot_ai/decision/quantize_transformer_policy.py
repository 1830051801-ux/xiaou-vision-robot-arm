#!/usr/bin/env python3
"""Export the trained Transformer and make a function-preserving INT8 copy.

The PyTorch checkpoint is never modified.  Dynamic quantization changes the
weights of Linear layers to INT8 while preserving the strategy, phase, action,
and risk outputs.  The script compares the original CPU model with the
quantized ONNX model on deterministic scenes before reporting it as usable.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import tempfile
from typing import Any

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from robot_ai.decision.onnx_policy import OnnxDecisionTransformerPolicy
from robot_ai.decision.transformer_policy import (
    DecisionTransformerPolicy,
    SafetyGate,
    TASK_TO_ID,
)
from simulation.desktop_scene import DesktopScene, OBJECT_SPECS


def _configure_console_encoding() -> None:
    """Keep PyTorch ONNX exporter diagnostics printable in Windows shells."""
    for stream in (sys.stdout, sys.stderr):
        reconfigure = getattr(stream, "reconfigure", None)
        if reconfigure is None:
            continue
        try:
            reconfigure(encoding="utf-8", errors="replace")
        except (OSError, ValueError):
            pass


def _require_quantization() -> Any:
    try:
        import onnxruntime  # type: ignore
        from onnxruntime.quantization import QuantType, quantize_dynamic  # type: ignore
    except Exception as exc:  # pragma: no cover - optional desktop/Pi dependency
        raise RuntimeError("install onnxruntime to export and quantize the Transformer") from exc
    return onnxruntime, QuantType, quantize_dynamic


def _quantize_without_stale_shape_annotations(
    source: Path,
    target: Path,
    quantize_dynamic: Any,
    quant_type: Any,
) -> None:
    """Feed ORT a copy without exporter-only intermediate value_info.

    PyTorch's new exporter records intermediate attention shapes that are
    valid for the trace but too strict for ORT's shape-inference pass.  The
    graph and initializers are unchanged; only those optional annotations are
    removed from a temporary copy used by the quantizer.
    """
    import onnx

    with tempfile.TemporaryDirectory(prefix="xiaou_transformer_quant_") as directory:
        sanitized = Path(directory) / "model.onnx"
        model = onnx.load(str(source))
        del model.graph.value_info[:]
        onnx.save(model, str(sanitized))
        quantize_dynamic(
            str(sanitized),
            str(target),
            weight_type=quant_type.QInt8,
            per_channel=True,
        )


def _export_fp32(policy: DecisionTransformerPolicy, output: Path, sequence_length: int) -> None:
    import torch
    from torch import nn
    from torch.export import Dim

    class ExportWrapper(nn.Module):
        def __init__(self, network: nn.Module) -> None:
            super().__init__()
            self.network = network

        def forward(self, observations: Any, task_ids: Any) -> tuple[Any, Any, Any, Any]:
            values = self.network(observations, task_ids)
            return (
                values["strategy_logits"],
                values["phase_logits"],
                values["action_params"],
                values["risk_logit"],
            )

    output.parent.mkdir(parents=True, exist_ok=True)
    wrapper = ExportWrapper(policy.net).eval().cpu()
    observations = torch.zeros((1, sequence_length, 46), dtype=torch.float32)
    task_ids = torch.zeros((1,), dtype=torch.long)
    with torch.no_grad():
        # The torch.export-based exporter keeps the attention sequence axis
        # dynamic.  The legacy exporter traces it as a fixed reshape and
        # fails when the Pi receives a shorter history.
        torch.onnx.export(
            wrapper,
            (observations, task_ids),
            str(output),
            input_names=["observations", "task_ids"],
            output_names=["strategy_logits", "phase_logits", "action_params", "risk_logit"],
            dynamic_shapes=(
                {0: Dim("batch"), 1: Dim("sequence", min=1, max=sequence_length)},
                {0: Dim("batch")},
            ),
            opset_version=18,
            dynamo=True,
        )


def _write_metadata(path: Path, *, checkpoint: Path, quantized: bool, max_seq_len: int) -> None:
    payload = {
        "schema_version": 1,
        "source_checkpoint": str(checkpoint),
        "quantization": "dynamic_int8_linear_weights" if quantized else "fp32",
        "max_seq_len": max_seq_len,
        "observation_dim": 46,
        "inputs": {"observations": "float32[batch,sequence,46]", "task_ids": "int64[batch]"},
        "outputs": {
            "strategy_logits": "float32[batch,6]",
            "phase_logits": "float32[batch,6]",
            "action_params": "float32[batch,6]",
            "risk_logit": "float32[batch]",
        },
        "strategy_names": ["top_down_pinch", "side_wrap", "pen_pinch", "flat_pick", "tissue_pull", "reject"],
        "phase_names": ["observe", "approach", "grasp", "lift", "pull", "abort"],
        "hardware_motion": False,
    }
    path.with_suffix(".json").write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def _artifact_bytes(path: Path) -> tuple[int, int]:
    """Return main-file bytes and referenced external-data bytes.

    The current PyTorch ONNX exporter stores FP32 initializers in a sibling
    ``.onnx.data`` file.  Counting only the graph file would make a quantized
    model appear larger than its FP32 source even when the deployable bundle
    is smaller.
    """
    main_bytes = path.stat().st_size
    external_files: set[Path] = set()
    try:
        import onnx

        model = onnx.load(str(path), load_external_data=False)
        for initializer in model.graph.initializer:
            for item in initializer.external_data:
                if item.key != "location":
                    continue
                location = Path(item.value)
                if not location.is_absolute():
                    location = path.parent / location
                if location.is_file():
                    external_files.add(location.resolve())
    except Exception:
        # Keep reporting useful with minimal ONNX installations.  The
        # exporter convention is a sibling ``<model>.onnx.data`` file.
        fallback = path.with_name(path.name + ".data")
        if fallback.is_file():
            external_files.add(fallback.resolve())
    return main_bytes, sum(item.stat().st_size for item in external_files)


def _compare(checkpoint: Path, model_path: Path, max_seq_len: int) -> dict[str, Any]:
    fp32 = DecisionTransformerPolicy(device="cpu", max_seq_len=max_seq_len)
    fp32.load(checkpoint)
    fp32.net.eval()
    int8 = OnnxDecisionTransformerPolicy(model_path, max_seq_len=max_seq_len)
    gate = SafetyGate(require_motion_enabled=False, require_hardware_ready=False, require_collision_free=True, require_feedback_verified=False)
    rows: list[dict[str, Any]] = []
    for index, scenario in enumerate(OBJECT_SPECS):
        scene = DesktopScene(scenario, seed=20260809 + index)
        observation = scene.observation()
        vector = scene.observation_vector()
        fp = fp32.predict(vector, TASK_TO_ID[scenario], gate, observation)
        q = int8.predict(vector, TASK_TO_ID[scenario], gate, observation)
        action_delta = max(abs(a - b) for a, b in zip(fp.action_params, q.action_params))
        rows.append({
            "scenario": scenario,
            "fp32": fp.as_dict(),
            "int8": q.as_dict(),
            "strategy_match": fp.strategy == q.strategy,
            "phase_match": fp.phase == q.phase,
            "status_match": fp.status == q.status,
            "max_action_abs_delta": round(float(action_delta), 8),
            "confidence_abs_delta": round(abs(fp.confidence - q.confidence), 8),
            "risk_abs_delta": round(abs(fp.risk - q.risk), 8),
        })
    return {
        "offline": True,
        "hardware_motion": False,
        "checkpoint": str(checkpoint),
        "model": str(model_path),
        "scenarios": rows,
        "strategy_matches": sum(bool(row["strategy_match"]) for row in rows),
        "phase_matches": sum(bool(row["phase_match"]) for row in rows),
        "status_matches": sum(bool(row["status_match"]) for row in rows),
        "scenario_count": len(rows),
    }


def main() -> int:
    _configure_console_encoding()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=Path, default=PROJECT_ROOT / "runtime" / "decision" / "transformer_policy.pt")
    parser.add_argument("--output", type=Path, default=PROJECT_ROOT / "runtime" / "decision" / "transformer_policy.onnx")
    parser.add_argument("--output-int8", type=Path, default=PROJECT_ROOT / "runtime" / "decision" / "transformer_policy.int8.onnx")
    parser.add_argument("--report", type=Path, default=PROJECT_ROOT / "runtime" / "decision" / "transformer_quantization_report.json")
    parser.add_argument("--max-seq-len", type=int, default=16)
    args = parser.parse_args()
    if args.max_seq_len < 1 or args.max_seq_len > 128:
        parser.error("max-seq-len must be between 1 and 128")
    if not args.checkpoint.is_file():
        raise FileNotFoundError(args.checkpoint)
    _, quant_type, quantize_dynamic = _require_quantization()

    policy = DecisionTransformerPolicy(device="cpu", max_seq_len=args.max_seq_len)
    policy.load(args.checkpoint)
    policy.net.eval()
    _export_fp32(policy, args.output, args.max_seq_len)
    _write_metadata(args.output, checkpoint=args.checkpoint, quantized=False, max_seq_len=args.max_seq_len)
    args.output_int8.parent.mkdir(parents=True, exist_ok=True)
    _quantize_without_stale_shape_annotations(args.output, args.output_int8, quantize_dynamic, quant_type)
    _write_metadata(args.output_int8, checkpoint=args.checkpoint, quantized=True, max_seq_len=args.max_seq_len)
    report = _compare(args.checkpoint, args.output_int8, args.max_seq_len)
    fp32_main_bytes, fp32_external_bytes = _artifact_bytes(args.output)
    int8_main_bytes, int8_external_bytes = _artifact_bytes(args.output_int8)
    fp32_total_bytes = fp32_main_bytes + fp32_external_bytes
    int8_total_bytes = int8_main_bytes + int8_external_bytes
    report.update({
        "fp32_model": str(args.output),
        "int8_model": str(args.output_int8),
        "fp32_bytes": fp32_main_bytes,
        "int8_bytes": int8_main_bytes,
        "fp32_external_data_bytes": fp32_external_bytes,
        "int8_external_data_bytes": int8_external_bytes,
        "fp32_total_bytes": fp32_total_bytes,
        "int8_total_bytes": int8_total_bytes,
        "size_ratio": round(int8_total_bytes / max(1, fp32_total_bytes), 6),
        "size_measurement": "main_file_plus_referenced_external_data",
        "quantization": "dynamic_int8_linear_weights",
        "warning": "This validates policy equivalence on deterministic synthetic scenes; collect real episodes before hardware use.",
    })
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["strategy_matches"] == report["scenario_count"] and report["phase_matches"] == report["scenario_count"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
