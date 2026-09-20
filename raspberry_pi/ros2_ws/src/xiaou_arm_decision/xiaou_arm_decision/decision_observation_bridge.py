"""Bridge YOLO target topics into a freshness-checked Transformer contract."""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
import sys

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String


class DecisionObservationBridge(Node):
    """Publishes observations only when their local-reception ages are valid."""

    def __init__(self) -> None:
        super().__init__("xiaou_decision_observation_bridge")
        default_root = os.environ.get("XIAOU_PROJECT_DIR", str(Path.home() / "raspi_robot_ai"))
        self.declare_parameter("project_root", default_root)
        project_root = Path(str(self.get_parameter("project_root").value)).expanduser().resolve()
        if str(project_root) not in sys.path:
            sys.path.insert(0, str(project_root))
        from robot_ai.decision.ros2_message_contract import FreshnessBook, validate_observation_contract

        self.contract_validator = validate_observation_contract
        self.freshness = FreshnessBook()
        self.declare_parameter("max_target_age_s", 0.50)
        self.declare_parameter("max_joint_age_s", 0.50)
        self.declare_parameter("output_topic", "/xiaou/decision_observation")
        self.output = self.create_publisher(String, str(self.get_parameter("output_topic").value), 10)
        self.target_status: dict[str, object] = {}
        self.target_pose: PoseStamped | None = None
        self.joints: JointState | None = None
        self.hardware_ready = False
        self.create_subscription(String, "/xiaou/target_status", self.on_status, 10)
        self.create_subscription(PoseStamped, "/xiaou/target_pose", self.on_pose, 10)
        self.create_subscription(JointState, "/joint_states", self.on_joints, 10)
        self.create_subscription(Bool, "/xiaou/hardware_ready", self.on_ready, 10)

    def _now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def on_status(self, message: String) -> None:
        try:
            value = json.loads(message.data)
            self.target_status = value if isinstance(value, dict) else {}
        except (TypeError, ValueError):
            self.target_status = {}
        self.freshness.update("target_status", self._now_ns())
        self.publish()

    def on_pose(self, message: PoseStamped) -> None:
        self.target_pose = message
        self.freshness.update("target_pose", self._now_ns())
        self.publish()

    def on_joints(self, message: JointState) -> None:
        self.joints = message
        self.freshness.update("joint_states", self._now_ns())
        self.publish()

    def on_ready(self, message: Bool) -> None:
        self.hardware_ready = bool(message.data)
        self.freshness.update("hardware_ready", self._now_ns())
        self.publish()

    def publish(self) -> None:
        if not self.target_status or self.target_pose is None:
            return
        now_ns = self._now_ns()
        max_target_age_s = float(self.get_parameter("max_target_age_s").value)
        max_joint_age_s = float(self.get_parameter("max_joint_age_s").value)
        status_fresh = self.freshness.is_fresh("target_status", now_ns, max_target_age_s)
        pose_fresh = self.freshness.is_fresh("target_pose", now_ns, max_target_age_s)
        joint_fresh = self.freshness.is_fresh("joint_states", now_ns, max_joint_age_s)
        object_class = str(self.target_status.get("target_class") or self.target_status.get("class") or "").lower()
        confidence = float(self.target_status.get("confidence", 0.0) or 0.0)
        pose = self.target_pose.pose
        joints = self.joints
        positions = list(joints.position[:6]) if joints is not None else [0.0] * 6
        velocities = list(joints.velocity[:6]) if joints is not None and joints.velocity else [0.0] * 6
        online = joint_fresh and len(positions) == 6 and all(math.isfinite(float(value)) for value in positions)
        published_target = self.target_status.get("status") == "target_pose_published"
        observation = {
            "schema_version": 1,
            "source": "ros_yolo_target_bridge",
            "yolo": {"class": object_class, "confidence": confidence, "center_px": [0.0, 0.0], "source": "target_pose_node"},
            "target": {
                "class": object_class,
                "position_base_m": [float(pose.position.x), float(pose.position.y), float(pose.position.z)],
                "yaw_rad": float(self.target_status.get("yaw_rad", 0.0) or 0.0),
                "radius_m": 0.0,
                "height_m": 0.0,
                "mass_kg": 0.0,
                "friction": 0.0,
            },
            "arm": {
                "joint_position_rad": positions,
                "joint_velocity_rad_s": velocities,
                "joint_online": [online] * 6,
                "gripper_open_m": 0.0,
                "gripper_force_pct": 0.0,
            },
            "phase": {"name": "observe", "index": 0},
            "perception": {
                "detection_fresh": bool(published_target and status_fresh and pose_fresh),
                "calibration_valid": self.target_status.get("calibration_valid") is True,
                "source_status": self.target_status.get("status"),
                "target_status_age_s": self.freshness.age_s("target_status", now_ns),
                "target_pose_age_s": self.freshness.age_s("target_pose", now_ns),
                "joint_state_age_s": self.freshness.age_s("joint_states", now_ns),
            },
            "safety": {
                "motion_enabled": False,
                "hardware_ready": self.hardware_ready,
                "collision_free": False,
                "feedback_verified": False,
            },
        }
        valid, failures = self.contract_validator(observation)
        if not valid:
            self.get_logger().warn("decision observation contract rejected: " + ",".join(failures))
            return
        message = String()
        message.data = json.dumps(observation, ensure_ascii=False)
        self.output.publish(message)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DecisionObservationBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
