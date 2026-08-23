"""Publish an offline table scene through ROS2 without hardware side effects."""

from __future__ import annotations

import json
import os
from pathlib import Path
import sys

from geometry_msgs.msg import PoseStamped, TransformStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tf2_ros


class DesktopSceneNode(Node):
    def __init__(self) -> None:
        super().__init__("xiaou_desktop_scene")
        default_root = os.environ.get("XIAOU_PROJECT_DIR", str(Path.home() / "raspi_robot_ai"))
        self.declare_parameter("project_root", default_root)
        self.declare_parameter("scenario", "bottle")
        self.declare_parameter("seed", 20260809)
        self.declare_parameter("publish_period_s", 0.2)
        project_root = Path(str(self.get_parameter("project_root").value)).expanduser().resolve()
        if str(project_root) not in sys.path:
            sys.path.insert(0, str(project_root))
        from simulation.desktop_scene import DesktopScene

        self.scene = DesktopScene(
            scenario=str(self.get_parameter("scenario").value),
            seed=int(self.get_parameter("seed").value),
        )
        self.observation_publisher = self.create_publisher(String, "/xiaou/sim/observation", 10)
        self.candidate_publisher = self.create_publisher(String, "/xiaou/sim/action_candidates", 10)
        self.status_publisher = self.create_publisher(String, "/xiaou/sim/status", 10)
        self.target_publisher = self.create_publisher(PoseStamped, "/xiaou/sim/target_pose", 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.create_subscription(String, "/xiaou/sim/strategy", self.on_strategy, 10)
        period = float(self.get_parameter("publish_period_s").value)
        if period <= 0.0:
            raise ValueError("publish_period_s must be positive")
        self.timer = self.create_timer(period, self.publish_scene)
        self.publish_scene()

    def on_strategy(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            strategy = str(payload.get("strategy", ""))
        except (TypeError, ValueError):
            self.publish_status("invalid_strategy_message")
            return
        result = self.scene.apply(strategy)
        self.publish_status("strategy_applied", strategy=strategy, result=result)

    def publish_status(self, status: str, **details: object) -> None:
        message = String()
        message.data = json.dumps({"status": status, **details}, ensure_ascii=False)
        self.status_publisher.publish(message)

    def publish_scene(self) -> None:
        observation = self.scene.step()
        serialized = json.dumps(observation, ensure_ascii=False)
        observation_message = String()
        observation_message.data = serialized
        self.observation_publisher.publish(observation_message)
        candidate_message = String()
        candidate_message.data = json.dumps({"candidates": observation["action_candidates"]}, ensure_ascii=False)
        self.candidate_publisher.publish(candidate_message)

        x, y, z = (float(value) for value in observation["target"]["position_base_m"])
        target_pose = PoseStamped()
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.header.frame_id = "base_link"
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z + float(observation["target"]["height_m"])
        target_pose.pose.orientation.w = 1.0
        self.target_publisher.publish(target_pose)

        transform = TransformStamped()
        transform.header = target_pose.header
        transform.child_frame_id = "sim_" + str(observation["target"]["class"])
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = z
        transform.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(transform)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DesktopSceneNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
