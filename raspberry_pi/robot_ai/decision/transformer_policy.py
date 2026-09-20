"""Small Transformer policy for task-level grasp strategy selection.

The network selects a strategy and phase from a short observation history. It
does not emit joint commands. A separate safety gate and MoveIt planner must
accept any candidate before it can become a motion plan.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any

import numpy as np

try:
    import torch
    from torch import nn
except Exception as exc:  # Keep offline tooling importable without torch.
    torch = None  # type: ignore[assignment]
    nn = None  # type: ignore[assignment]
    _TORCH_IMPORT_ERROR = exc
else:
    _TORCH_IMPORT_ERROR = None


OBJECT_CLASSES = ("bottle", "cup", "pen", "desktop_item", "tissue_pull", "cola")
PHASES = ("observe", "approach", "grasp", "lift", "pull", "abort")
ACTION_STRATEGIES = ("top_down_pinch", "side_wrap", "pen_pinch", "flat_pick", "tissue_pull", "reject")
DECISION_OBSERVATION_DIM = 46
TASK_TO_ID = {name: index for index, name in enumerate(OBJECT_CLASSES)}


def validate_action_params(strategy: str, params: tuple[float, ...] | list[float]) -> list[str]:
    """Validate the six continuous outputs before a planner can consume them."""
    values = tuple(float(value) for value in params)
    failures: list[str] = []
    if len(values) != 6 or not all(math.isfinite(value) for value in values):
        return ["action_params_not_finite_or_length_mismatch"]
    ranges = (
        (0.0, 0.60, "target_x_m"),
        (-0.60, 0.60, "target_y_m"),
        (0.0, 0.50, "target_z_m"),
        (0.0, 0.15, "gripper_open_m"),
        (0.0, 0.15, "gripper_close_m"),
        (0.0, 0.50, "pull_distance_m"),
    )
    for value, (lower, upper, name) in zip(values, ranges):
        if not lower <= value <= upper:
            failures.append(f"{name}_out_of_range")
    if values[4] > values[3]:
        failures.append("gripper_close_exceeds_open")
    if strategy == "tissue_pull" and values[5] < 0.005:
        failures.append("tissue_pull_distance_too_small")
    if strategy != "tissue_pull" and values[5] > 0.15:
        failures.append("unexpected_pull_distance")
    return failures


def _require_torch() -> Any:
    if torch is None or nn is None:
        raise RuntimeError(f"PyTorch is unavailable: {_TORCH_IMPORT_ERROR}")
    return torch


if nn is not None:

    class _DecisionTransformerNet(nn.Module):
        def __init__(self, observation_dim: int, d_model: int, nhead: int, layers: int, max_seq_len: int) -> None:
            super().__init__()
            self.observation_projection = nn.Sequential(
                nn.LayerNorm(observation_dim),
                nn.Linear(observation_dim, d_model),
                nn.GELU(),
            )
            self.task_embedding = nn.Embedding(len(OBJECT_CLASSES), d_model)
            self.position_embedding = nn.Parameter(torch.zeros(1, max_seq_len, d_model))
            nn.init.normal_(self.position_embedding, std=0.02)
            encoder_layer = nn.TransformerEncoderLayer(
                d_model=d_model,
                nhead=nhead,
                dim_feedforward=d_model * 4,
                dropout=0.1,
                activation="gelu",
                batch_first=True,
                norm_first=True,
            )
            self.encoder = nn.TransformerEncoder(encoder_layer, num_layers=layers)
            self.output_norm = nn.LayerNorm(d_model)
            self.strategy_head = nn.Linear(d_model, len(ACTION_STRATEGIES))
            self.phase_head = nn.Linear(d_model, len(PHASES))
            self.action_head = nn.Linear(d_model, 6)
            self.risk_head = nn.Linear(d_model, 1)

        def forward(self, observations: Any, task_ids: Any) -> dict[str, Any]:
            if observations.ndim != 3:
                raise ValueError("observations must have shape [batch, sequence, features]")
            if observations.shape[-1] != DECISION_OBSERVATION_DIM:
                raise ValueError("observation feature dimension mismatch")
            if observations.shape[1] > self.position_embedding.shape[1]:
                raise ValueError("sequence length exceeds checkpoint capacity")
            task = self.task_embedding(task_ids).unsqueeze(1)
            encoded = self.observation_projection(observations) + task
            encoded = encoded + self.position_embedding[:, : encoded.shape[1]]
            encoded = self.output_norm(self.encoder(encoded))[:, -1]
            raw_action = self.action_head(encoded)
            open_width = 0.15 * torch.sigmoid(raw_action[:, 3])
            close_width = open_width * torch.sigmoid(raw_action[:, 4])
            action_params = torch.stack(
                (
                    0.60 * torch.sigmoid(raw_action[:, 0]),
                    0.60 * torch.tanh(raw_action[:, 1]),
                    0.50 * torch.sigmoid(raw_action[:, 2]),
                    open_width,
                    close_width,
                    0.15 * torch.sigmoid(raw_action[:, 5]),
                ),
                dim=-1,
            )
            return {
                "strategy_logits": self.strategy_head(encoded),
                "phase_logits": self.phase_head(encoded),
                "action_params": action_params,
                "risk_logit": self.risk_head(encoded).squeeze(-1),
            }

else:

    class _DecisionTransformerNet:  # type: ignore[no-redef]
        def __init__(self, *args: Any, **kwargs: Any) -> None:
            _require_torch()


@dataclass(frozen=True)
class SafetyGate:
    """Decision-level gate; false flags produce a blocked result."""

    require_motion_enabled: bool = False
    require_hardware_ready: bool = False
    require_collision_free: bool = True
    require_feedback_verified: bool = True
    require_detection_fresh: bool = False
    require_calibration_valid: bool = False
    require_all_joints_online: bool = False
    min_detection_confidence: float | None = None

    def evaluate(self, observation: dict[str, Any]) -> tuple[bool, list[str]]:
        safety = observation.get("safety") or {}
        failures: list[str] = []
        checks = (
            (self.require_motion_enabled, "motion_enabled"),
            (self.require_hardware_ready, "hardware_ready"),
            (self.require_collision_free, "collision_free"),
            (self.require_feedback_verified, "feedback_verified"),
        )
        for required, name in checks:
            if required and safety.get(name) is not True:
                failures.append(name)
        perception = observation.get("perception") or {}
        if self.require_detection_fresh and perception.get("detection_fresh") is not True:
            failures.append("detection_fresh")
        if self.require_calibration_valid and perception.get("calibration_valid") is not True:
            failures.append("calibration_valid")
        if self.require_all_joints_online:
            online = (observation.get("arm") or {}).get("joint_online") or []
            if len(online) != 6 or any(value is not True for value in online):
                failures.append("joint_online")
        if self.min_detection_confidence is not None:
            confidence = (observation.get("yolo") or {}).get("confidence")
            try:
                valid_confidence = math.isfinite(float(confidence)) and float(confidence) >= self.min_detection_confidence
            except (TypeError, ValueError):
                valid_confidence = False
            if not valid_confidence:
                failures.append("detection_confidence")
        return not failures, failures


@dataclass(frozen=True)
class DecisionOutput:
    strategy: str
    phase: str
    confidence: float
    risk: float
    action_params: tuple[float, ...]
    status: str
    blocked_reasons: tuple[str, ...] = ()

    def as_dict(self) -> dict[str, Any]:
        return {
            "strategy": self.strategy,
            "phase": self.phase,
            "confidence": round(float(self.confidence), 6),
            "risk": round(float(self.risk), 6),
            "action_params": [round(float(value), 6) for value in self.action_params],
            "status": self.status,
            "blocked_reasons": list(self.blocked_reasons),
        }


def observation_dict_to_vector(observation: dict[str, Any]) -> np.ndarray:
    """Convert the camera/ROS observation contract into 46 float features."""
    target = observation.get("target") or {}
    yolo = observation.get("yolo") or {}
    arm = observation.get("arm") or {}
    phase = observation.get("phase") or {}
    safety = observation.get("safety") or {}
    target_class = str(target.get("class") or yolo.get("class") or "").strip().lower()
    center = yolo.get("center_px") or [0.0, 0.0]
    position = target.get("position_base_m") or observation.get("target_xyz_m") or [0.0, 0.0, 0.0]
    vector: list[float] = []
    vector.extend(1.0 if target_class == name else 0.0 for name in OBJECT_CLASSES)
    vector.extend((float(center[0]) / 1920.0, float(center[1]) / 1080.0, float(yolo.get("confidence", 0.0))))
    vector.extend((float(position[0]), float(position[1]), float(position[2]), float(target.get("yaw_rad", 0.0))))
    vector.extend((float(target.get("radius_m", 0.0)), float(target.get("height_m", 0.0)), float(target.get("mass_kg", 0.0)), float(target.get("friction", 0.0))))
    vector.extend(float(value) for value in (arm.get("joint_position_rad") or [0.0] * 6))
    vector.extend(float(value) for value in (arm.get("joint_velocity_rad_s") or [0.0] * 6))
    vector.extend(1.0 if value else 0.0 for value in (arm.get("joint_online") or [False] * 6))
    phase_name = str(phase.get("name") or "observe")
    vector.extend(1.0 if phase_name == name else 0.0 for name in PHASES)
    vector.extend(1.0 if safety.get(name) is True else 0.0 for name in ("motion_enabled", "hardware_ready", "collision_free"))
    vector.extend((float(arm.get("gripper_open_m", 0.0)), float(arm.get("gripper_force_pct", 0.0))))
    result = np.asarray(vector, dtype=np.float32)
    if result.shape != (DECISION_OBSERVATION_DIM,) or not np.isfinite(result).all():
        raise ValueError(f"invalid decision observation vector: {result.shape}")
    return result


class DecisionTransformerPolicy:
    """Inference/training wrapper around the small task-level Transformer."""

    def __init__(
        self,
        *,
        d_model: int = 128,
        nhead: int = 4,
        layers: int = 2,
        max_seq_len: int = 16,
        device: str = "auto",
    ) -> None:
        _require_torch()
        if d_model % nhead:
            raise ValueError("d_model must be divisible by nhead")
        self.device = self._resolve_device(device)
        self.net = _DecisionTransformerNet(DECISION_OBSERVATION_DIM, d_model, nhead, layers, max_seq_len).to(self.device)
        self.max_seq_len = max_seq_len

    @staticmethod
    def _resolve_device(device: str) -> Any:
        _require_torch()
        if device == "auto":
            return torch.device("cuda" if torch.cuda.is_available() else "cpu")
        resolved = torch.device(device)
        if resolved.type == "cuda" and not torch.cuda.is_available():
            raise RuntimeError("CUDA requested but torch.cuda.is_available() is false")
        return resolved

    def parameters(self) -> Any:
        return self.net.parameters()

    def forward(self, observations: Any, task_ids: Any) -> dict[str, Any]:
        return self.net(observations.to(self.device), task_ids.to(self.device))

    def predict(self, observations: np.ndarray, task_id: int, safety_gate: SafetyGate | None = None, context: dict[str, Any] | None = None) -> DecisionOutput:
        _require_torch()
        array = np.asarray(observations, dtype=np.float32)
        if array.ndim == 1:
            array = array[None, :]
        if array.ndim != 2 or array.shape[-1] != DECISION_OBSERVATION_DIM:
            raise ValueError("observations must have shape [sequence, 46]")
        if array.shape[0] > self.max_seq_len:
            array = array[-self.max_seq_len :]
        with torch.no_grad():
            tensor = torch.from_numpy(array).unsqueeze(0).to(self.device)
            task = torch.tensor([int(task_id)], dtype=torch.long, device=self.device)
            output = self.net(tensor, task)
            strategy_prob = torch.softmax(output["strategy_logits"], dim=-1)[0]
            phase_prob = torch.softmax(output["phase_logits"], dim=-1)[0]
            strategy_index = int(torch.argmax(strategy_prob).item())
            phase_index = int(torch.argmax(phase_prob).item())
            risk = float(torch.sigmoid(output["risk_logit"])[0].item())
            params = tuple(float(value) for value in output["action_params"][0].detach().cpu().tolist())
            confidence = float(strategy_prob[strategy_index].item())
        strategy = ACTION_STRATEGIES[strategy_index]
        if strategy == "reject":
            return DecisionOutput("reject", "abort", confidence, risk, params, "rejected_by_policy", ("policy_reject",))
        if strategy != "reject":
            action_failures = validate_action_params(strategy, params)
            if action_failures:
                return DecisionOutput("reject", "abort", confidence, risk, params, "blocked_by_action_limits", tuple(action_failures))
        if safety_gate is not None and context is not None:
            allowed, failures = safety_gate.evaluate(context)
            if not allowed:
                return DecisionOutput("reject", "abort", confidence, risk, params, "blocked_by_safety_gate", tuple(failures))
        return DecisionOutput(strategy, PHASES[phase_index], confidence, risk, params, "candidate")

    def save(self, path: str | Path, metadata: dict[str, Any] | None = None) -> None:
        _require_torch()
        payload = {"state_dict": self.net.state_dict(), "max_seq_len": self.max_seq_len, "metadata": metadata or {}}
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        torch.save(payload, path)

    def load(self, path: str | Path) -> dict[str, Any]:
        _require_torch()
        checkpoint = torch.load(path, map_location=self.device, weights_only=False)
        self.net.load_state_dict(checkpoint["state_dict"])
        return dict(checkpoint.get("metadata") or {})


def load_json_observation(path: str | Path) -> dict[str, Any]:
    return json.loads(Path(path).read_text(encoding="utf-8"))
