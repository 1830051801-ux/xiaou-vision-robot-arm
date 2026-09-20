#!/usr/bin/env python3
"""CPU ONNX Runtime inference for the trained task-level Transformer.

The exported model keeps all four policy outputs.  This module intentionally
does not contain any motor transport code; the decision and hardware safety
gates remain in Python and on the STM32 side.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

import numpy as np

try:
    import onnxruntime as ort  # type: ignore
except Exception as exc:  # Keep the rest of the project importable on Pi images without ORT.
    ort = None  # type: ignore[assignment]
    _ORT_IMPORT_ERROR = exc
else:
    _ORT_IMPORT_ERROR = None

try:
    from .transformer_policy import (
        ACTION_STRATEGIES,
        DECISION_OBSERVATION_DIM,
        PHASES,
        TASK_TO_ID,
        DecisionOutput,
        SafetyGate,
        validate_action_params,
    )
except ImportError:  # Direct invocation from robot_ai/decision.
    from transformer_policy import (  # type: ignore
        ACTION_STRATEGIES,
        DECISION_OBSERVATION_DIM,
        PHASES,
        TASK_TO_ID,
        DecisionOutput,
        SafetyGate,
        validate_action_params,
    )


def _require_ort() -> Any:
    if ort is None:
        raise RuntimeError(f"onnxruntime is unavailable: {_ORT_IMPORT_ERROR}")
    return ort


def _softmax(values: np.ndarray) -> np.ndarray:
    shifted = values - np.max(values, axis=-1, keepdims=True)
    exp_values = np.exp(shifted)
    return exp_values / np.sum(exp_values, axis=-1, keepdims=True)


class OnnxDecisionTransformerPolicy:
    """Drop-in CPU inference wrapper for ``DecisionTransformerPolicy``."""

    def __init__(
        self,
        model_path: str | Path,
        *,
        providers: list[str] | None = None,
        max_seq_len: int = 16,
    ) -> None:
        runtime = _require_ort()
        self.model_path = Path(model_path)
        if not self.model_path.is_file():
            raise FileNotFoundError(f"Transformer ONNX model not found: {self.model_path}")
        self.metadata = self._load_metadata()
        self.max_seq_len = int(self.metadata.get("max_seq_len", max_seq_len))
        if self.max_seq_len < 1:
            raise ValueError("max_seq_len must be positive")
        requested = providers or ["CPUExecutionProvider"]
        available = set(runtime.get_available_providers())
        selected = [name for name in requested if name in available]
        if not selected:
            selected = ["CPUExecutionProvider"]
        self.session = runtime.InferenceSession(str(self.model_path), providers=selected)
        self.input_names = {item.name for item in self.session.get_inputs()}
        required_inputs = {"observations", "task_ids"}
        if not required_inputs.issubset(self.input_names):
            raise RuntimeError(f"ONNX inputs {sorted(self.input_names)} do not contain {sorted(required_inputs)}")
        self.output_names = [item.name for item in self.session.get_outputs()]
        required_outputs = {"strategy_logits", "phase_logits", "action_params", "risk_logit"}
        if not required_outputs.issubset(set(self.output_names)):
            raise RuntimeError(f"ONNX outputs {self.output_names} do not contain {sorted(required_outputs)}")
        observation_shape = self.session.get_inputs()[0].shape
        self.fixed_sequence_length = (
            int(observation_shape[1])
            if len(observation_shape) > 1 and isinstance(observation_shape[1], int)
            else None
        )

    def _load_metadata(self) -> dict[str, Any]:
        path = self.model_path.with_suffix(".json")
        if not path.is_file():
            return {}
        value = json.loads(path.read_text(encoding="utf-8"))
        return value if isinstance(value, dict) else {}

    def _prepare_sequence(self, observations: np.ndarray) -> np.ndarray:
        array = np.asarray(observations, dtype=np.float32)
        if array.ndim == 1:
            array = array[None, :]
        if array.ndim != 2 or array.shape[-1] != DECISION_OBSERVATION_DIM:
            raise ValueError("observations must have shape [sequence, 46]")
        if not np.isfinite(array).all():
            raise ValueError("observations must contain finite values")
        if array.shape[0] > self.max_seq_len:
            array = array[-self.max_seq_len :]
        if self.fixed_sequence_length is not None:
            if self.fixed_sequence_length > self.max_seq_len:
                raise ValueError("fixed ONNX sequence length exceeds configured max_seq_len")
            if array.shape[0] < self.fixed_sequence_length:
                prefix = np.repeat(array[:1], self.fixed_sequence_length - array.shape[0], axis=0)
                array = np.concatenate((prefix, array), axis=0)
        return array

    def predict(
        self,
        observations: np.ndarray,
        task_id: int,
        safety_gate: SafetyGate | None = None,
        context: dict[str, Any] | None = None,
    ) -> DecisionOutput:
        array = self._prepare_sequence(observations)
        task = int(task_id)
        if task < 0 or task >= len(TASK_TO_ID):
            raise ValueError(f"task_id out of range: {task}")
        raw = self.session.run(
            ["strategy_logits", "phase_logits", "action_params", "risk_logit"],
            {
                "observations": array[None, ...].astype(np.float32, copy=False),
                "task_ids": np.asarray([task], dtype=np.int64),
            },
        )
        strategy_logits, phase_logits, action_params, risk_logit = (np.asarray(item) for item in raw)
        strategy_prob = _softmax(strategy_logits)[0]
        phase_prob = _softmax(phase_logits)[0]
        strategy_index = int(np.argmax(strategy_prob))
        phase_index = int(np.argmax(phase_prob))
        strategy = ACTION_STRATEGIES[strategy_index]
        phase = PHASES[phase_index]
        confidence = float(strategy_prob[strategy_index])
        risk = float(1.0 / (1.0 + math.exp(-float(risk_logit.reshape(-1)[0]))))
        params = tuple(float(value) for value in action_params.reshape(-1)[:6])
        if strategy == "reject":
            return DecisionOutput(
                "reject", "abort", confidence, risk, params,
                "rejected_by_policy", ("policy_reject",),
            )
        if strategy != "reject":
            failures = validate_action_params(strategy, params)
            if failures:
                return DecisionOutput(
                    "reject", "abort", confidence, risk, params,
                    "blocked_by_action_limits", tuple(failures),
                )
        if safety_gate is not None and context is not None:
            allowed, failures = safety_gate.evaluate(context)
            if not allowed:
                return DecisionOutput(
                    "reject", "abort", confidence, risk, params,
                    "blocked_by_safety_gate", tuple(failures),
                )
        return DecisionOutput(strategy, phase, confidence, risk, params, "candidate")


__all__ = ["OnnxDecisionTransformerPolicy"]
