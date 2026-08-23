"""Run the GPU-side task-level Transformer over a ROS observation history."""

from __future__ import annotations

from collections import deque
import json
import os
from pathlib import Path
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class DecisionNode(Node):
    def __init__(self) -> None:
        super().__init__("xiaou_transformer_decision")
        default_root = os.environ.get("XIAOU_PROJECT_DIR", str(Path.home() / "raspi_robot_ai"))
        self.declare_parameter("project_root", default_root)
        # The Pi deployment is CPU/ONNX by default. Desktop CUDA runs remain
        # explicit by passing a .pt checkpoint and device:=cuda.
        self.declare_parameter("checkpoint", "runtime/decision/transformer_policy_safety_final_20260813.int8.onnx")
        self.declare_parameter("device", "cpu")
        self.declare_parameter("backend", "auto")
        self.declare_parameter("sequence_length", 8)
        self.declare_parameter("simulation_mode", True)
        self.declare_parameter("input_topic", "/xiaou/decision_observation")
        self.declare_parameter("output_topic", "/xiaou/decision")
        project_root = Path(str(self.get_parameter("project_root").value)).expanduser().resolve()
        if str(project_root) not in sys.path:
            sys.path.insert(0, str(project_root))
        self.project_root = project_root
        self.history: deque[object] = deque(maxlen=max(1, int(self.get_parameter("sequence_length").value)))
        self.history_task_id: int | None = None
        self.publisher = self.create_publisher(String, str(self.get_parameter("output_topic").value), 10)
        self.policy = None
        self.policy_error = ""
        self._load_policy()
        self.create_subscription(String, str(self.get_parameter("input_topic").value), self.on_observation, 10)

    def _load_policy(self) -> None:
        checkpoint_value = Path(str(self.get_parameter("checkpoint").value))
        checkpoint = checkpoint_value if checkpoint_value.is_absolute() else self.project_root / checkpoint_value
        if not checkpoint.exists():
            self.policy_error = f"checkpoint_missing:{checkpoint}"
            self.get_logger().warn(self.policy_error)
            return
        try:
            backend = str(self.get_parameter("backend").value).strip().lower()
            use_onnx = backend == "onnx" or (backend == "auto" and checkpoint.suffix.lower() == ".onnx")
            if use_onnx:
                from robot_ai.decision.onnx_policy import OnnxDecisionTransformerPolicy

                policy = OnnxDecisionTransformerPolicy(checkpoint, max_seq_len=max(16, len(self.history) or 8))
            else:
                from robot_ai.decision.transformer_policy import DecisionTransformerPolicy

                policy = DecisionTransformerPolicy(device=str(self.get_parameter("device").value), max_seq_len=max(16, len(self.history) or 8))
                policy.load(checkpoint)
                policy.net.eval()
            self.policy = policy
            runtime_name = "onnxruntime-cpu" if use_onnx else str(policy.device)
            self.get_logger().info(f"loaded Transformer checkpoint on {runtime_name}: {checkpoint}")
        except Exception as exc:
            self.policy_error = f"checkpoint_load_failed:{exc}"
            self.get_logger().error(self.policy_error)

    def publish(self, payload: dict[str, object]) -> None:
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False)
        self.publisher.publish(message)

    def on_observation(self, message: String) -> None:
        try:
            observation = json.loads(message.data)
            if not isinstance(observation, dict):
                raise ValueError("observation must be a JSON object")
        except (TypeError, ValueError) as exc:
            self.publish({"status": "invalid_observation", "error": str(exc)})
            return
        if self.policy is None:
            self.publish({"status": "blocked", "reason": self.policy_error, "hardware_motion": False})
            return
        try:
            from robot_ai.decision.ros2_message_contract import validate_observation_contract
            from robot_ai.decision.transformer_policy import SafetyGate, TASK_TO_ID, observation_dict_to_vector

            valid, contract_failures = validate_observation_contract(observation)
            if not valid:
                self.publish({
                    "status": "invalid_observation",
                    "reason": ",".join(contract_failures),
                    "hardware_motion": False,
                })
                return

            target = observation.get("target") or {}
            object_class = str(target.get("class") or (observation.get("yolo") or {}).get("class") or "").lower()
            task_id = TASK_TO_ID.get(object_class)
            if task_id is None:
                self.publish({"status": "blocked", "reason": f"unknown_object_class:{object_class}", "hardware_motion": False})
                return
            vector = observation_dict_to_vector(observation)
            if self.history_task_id is not None and self.history_task_id != task_id:
                self.history.clear()
            self.history_task_id = task_id
            self.history.append(vector)
            import numpy as np

            sequence = np.stack(tuple(self.history), axis=0)
            simulation_mode = bool(self.get_parameter("simulation_mode").value)
            gate = SafetyGate(
                require_motion_enabled=not simulation_mode,
                require_hardware_ready=not simulation_mode,
                require_collision_free=True,
                require_feedback_verified=not simulation_mode,
                require_detection_fresh=not simulation_mode,
                require_calibration_valid=not simulation_mode,
                require_all_joints_online=not simulation_mode,
                min_detection_confidence=0.10 if not simulation_mode else None,
            )
            decision = self.policy.predict(sequence, task_id, gate, observation)
            self.publish({
                "status": decision.status,
                "hardware_motion": False,
                "simulation_mode": simulation_mode,
                "object_class": object_class,
                "decision": decision.as_dict(),
            })
        except Exception as exc:
            self.publish({"status": "blocked", "reason": f"inference_failed:{exc}", "hardware_motion": False})


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DecisionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
