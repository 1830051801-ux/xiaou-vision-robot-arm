#!/usr/bin/env python3
"""Publish an exported taught-grasp waypoint artifact as a transient PoseArray.

The publisher is intended only for RViz/MoveIt Cartesian preview.  It has no
hardware import and sends no controller or UART command.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time
from typing import Any

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def _finite_vector(values: Any, size: int, label: str) -> list[float]:
    result = [float(value) for value in values]
    if len(result) != size or not all(math.isfinite(value) for value in result):
        raise ValueError(f"{label} must contain {size} finite values")
    return result


def _load_pose_array(path: Path, topic: str) -> PoseArray:
    payload = json.loads(path.read_text(encoding="utf-8"))
    contract = payload.get("planner_contract") or {}
    if payload.get("execution") != "planning_preview_only" or contract.get("hardware_execution") is not False:
        raise ValueError("only planning_preview_only artifacts may be published")
    if contract.get("frame_id") != "base_link":
        raise ValueError("expected base_link waypoint artifact")
    declared_topic = contract.get("output_topic")
    if declared_topic and declared_topic != topic:
        raise ValueError(f"artifact expects topic {declared_topic!r}, not {topic!r}")
    message = PoseArray()
    message.header.frame_id = "base_link"
    waypoints = payload.get("waypoints")
    if not isinstance(waypoints, list) or len(waypoints) < 2:
        raise ValueError("artifact must contain at least two waypoints")
    for index, waypoint in enumerate(waypoints):
        if waypoint.get("frame_id") != "base_link":
            raise ValueError(f"waypoint {index} is not in base_link")
        position = _finite_vector(waypoint.get("position_m") or [], 3, f"waypoint {index} position")
        orientation = _finite_vector(waypoint.get("orientation_xyzw") or [], 4, f"waypoint {index} orientation")
        if not math.isclose(math.sqrt(sum(value * value for value in orientation)), 1.0, abs_tol=1e-6):
            raise ValueError(f"waypoint {index} orientation is not unit length")
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = position
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = orientation
        message.poses.append(pose)
    return message


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--waypoints-file", type=Path, required=True)
    parser.add_argument("--project-root", type=Path, default=Path.cwd())
    parser.add_argument("--topic", default="/xiaou/cartesian_waypoints")
    parser.add_argument("--publish-count", type=int, default=3)
    parser.add_argument("--period-s", type=float, default=0.5)
    args = parser.parse_args()
    if args.publish_count < 1 or args.publish_count > 20 or not math.isfinite(args.period_s) or args.period_s <= 0.0:
        parser.error("publish-count must be 1..20 and period-s must be positive")
    waypoint_file = args.waypoints_file
    if not waypoint_file.is_absolute():
        waypoint_file = args.project_root / waypoint_file
    message = _load_pose_array(waypoint_file, args.topic)
    rclpy.init()
    node = rclpy.create_node("xiaou_taught_waypoint_publisher")
    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    publisher = node.create_publisher(PoseArray, args.topic, qos)
    try:
        for _ in range(args.publish_count):
            message.header.stamp = node.get_clock().now().to_msg()
            publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=min(args.period_s, 0.1))
            time.sleep(max(0.0, args.period_s - 0.1))
        node.get_logger().info(
            f"Published {len(message.poses)} Cartesian preview waypoints to {args.topic}; hardware execution is disabled."
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
