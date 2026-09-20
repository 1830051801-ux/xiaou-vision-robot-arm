from __future__ import annotations

import importlib
import json
import math
import os
from pathlib import Path
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class HardwareReadinessNode(Node):
    def __init__(self) -> None:
        super().__init__("xiaou_hardware_readiness")
        default_root = os.environ.get("XIAOU_PROJECT_DIR", str(Path.home() / "raspi_robot_ai"))
        self.declare_parameter("project_root", default_root)
        self.declare_parameter(
            "config_path", "robot_ai/arm_control/config/hardware_calibration.json"
        )
        self.declare_parameter("reload_period_s", 1.0)
        project_root = Path(self.get_parameter("project_root").value).expanduser().resolve()
        robot_ai_dir = project_root / "robot_ai"
        if str(robot_ai_dir) not in sys.path:
            sys.path.insert(0, str(robot_ai_dir))
        safety = importlib.import_module("arm_control.safety")
        config_path = Path(str(self.get_parameter("config_path").value))
        if not config_path.is_absolute():
            config_path = project_root / config_path
        self.safety = safety
        self.config_path = config_path

        qos = rclpy.qos.QoSProfile(depth=1)
        qos.reliability = rclpy.qos.ReliabilityPolicy.RELIABLE
        qos.durability = rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
        self.ready_publisher = self.create_publisher(Bool, "/xiaou/hardware_ready", qos)
        self.detail_publisher = self.create_publisher(String, "/xiaou/hardware_readiness", qos)
        reload_period_s = float(self.get_parameter("reload_period_s").value)
        if not math.isfinite(reload_period_s) or reload_period_s <= 0.0:
            raise RuntimeError("reload_period_s must be finite and positive")
        self.last_detail = ""
        self.evaluate_and_publish()
        self.timer = self.create_timer(reload_period_s, self.evaluate_and_publish)

    def evaluate_and_publish(self) -> None:
        try:
            config = self.safety.load_hardware_config(self.config_path)
            readiness = self.safety.validate_motion_readiness(config)
            details = {
                "ready": readiness.ready,
                "missing_or_invalid": list(readiness.missing_or_invalid),
                "config": str(self.config_path),
                "protocol_family": config.get("protocol_family"),
                "protocol_status": config.get("protocol_status"),
            }
        except Exception as exc:
            readiness = None
            details = {
                "ready": False,
                "missing_or_invalid": ["configuration_load_failed"],
                "config": str(self.config_path),
                "error": str(exc),
            }

        ready_message = Bool()
        ready_message.data = bool(details["ready"])
        detail_message = String()
        detail_message.data = json.dumps(details, ensure_ascii=False)
        self.ready_publisher.publish(ready_message)
        self.detail_publisher.publish(detail_message)
        if detail_message.data != self.last_detail:
            if details["ready"]:
                self.get_logger().info(detail_message.data)
            else:
                self.get_logger().warn(detail_message.data)
            self.last_detail = detail_message.data


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = HardwareReadinessNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
