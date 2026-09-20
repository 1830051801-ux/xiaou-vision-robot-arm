from __future__ import annotations

from collections import deque
import importlib
import json
import math
import os
from pathlib import Path
import sys

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import yaml


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class TargetPoseNode(Node):
    def __init__(self) -> None:
        super().__init__("xiaou_target_pose")
        default_root = os.environ.get("XIAOU_PROJECT_DIR", str(Path.home() / "raspi_robot_ai"))
        self.declare_parameter("project_root", default_root)
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("target_class", "")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("table_z_m", float("nan"))
        self.declare_parameter("stable_frames", 5)
        self.declare_parameter("max_center_spread_px", 18.0)
        self.declare_parameter("homography_path", "codex_pickup_package/workspace_homography.yaml")
        self.declare_parameter(
            "grasp_profiles_path",
            "robot_ai/arm_control/config/object_grasp_profiles.json",
        )

        project_root = Path(self.get_parameter("project_root").value).expanduser().resolve()
        robot_ai_dir = project_root / "robot_ai"
        if str(robot_ai_dir) not in sys.path:
            sys.path.insert(0, str(robot_ai_dir))
        self.yolo_module = importlib.import_module("yolo_opencv")
        self.targeting_module = importlib.import_module("vision_targeting")
        self.detector = self.yolo_module.OpenCVDnnYolo()

        homography_value = str(self.get_parameter("homography_path").value)
        homography_path = Path(homography_value)
        if not homography_path.is_absolute():
            homography_path = project_root / homography_path
        calibration = yaml.safe_load(homography_path.read_text(encoding="utf-8")) or {}
        if calibration.get("type") != "pixel_to_robot_base_mm":
            raise RuntimeError(f"unexpected calibration type in {homography_path}")
        self.homography = np.asarray(calibration["homography"], dtype=np.float64)
        if self.homography.shape != (3, 3) or not np.isfinite(self.homography).all():
            raise RuntimeError(f"homography must be a finite 3x3 matrix: {homography_path}")
        if np.linalg.matrix_rank(self.homography) != 3:
            raise RuntimeError(f"homography is singular: {homography_path}")
        pixel_points = np.asarray(calibration["pixel_points"], dtype=np.float32)
        if pixel_points.ndim != 2 or pixel_points.shape[0] < 4 or pixel_points.shape[1] != 2:
            raise RuntimeError(f"at least four 2D pixel points are required: {homography_path}")
        if not np.isfinite(pixel_points).all():
            raise RuntimeError(f"pixel calibration points must be finite: {homography_path}")
        self.calibration_hull = cv2.convexHull(pixel_points)
        self.calibration_path = homography_path

        profiles_value = str(self.get_parameter("grasp_profiles_path").value)
        profiles_path = Path(profiles_value)
        if not profiles_path.is_absolute():
            profiles_path = project_root / profiles_path
        profile_data = json.loads(profiles_path.read_text(encoding="utf-8"))
        if profile_data.get("height_reference") != "relative_to_table_m":
            raise RuntimeError(f"unexpected grasp-height reference in {profiles_path}")
        profiles = profile_data.get("grasp_height_m_by_class")
        if not isinstance(profiles, dict):
            raise RuntimeError(f"grasp_height_m_by_class must be a mapping: {profiles_path}")
        for class_name, height in profiles.items():
            if not isinstance(class_name, str) or not class_name.strip():
                raise RuntimeError(f"grasp profile class names must be non-empty: {profiles_path}")
            if height is not None and (
                isinstance(height, bool)
                or not isinstance(height, (int, float))
                or not math.isfinite(height)
                or height < 0.0
            ):
                raise RuntimeError(f"invalid grasp height for {class_name}: {height}")
        self.grasp_heights = profiles
        self.grasp_profiles_path = profiles_path

        self.bridge = CvBridge()
        stable_frames = int(self.get_parameter("stable_frames").value)
        max_center_spread_px = float(self.get_parameter("max_center_spread_px").value)
        if stable_frames < 2:
            raise RuntimeError("stable_frames must be at least 2")
        if not math.isfinite(max_center_spread_px) or max_center_spread_px <= 0.0:
            raise RuntimeError("max_center_spread_px must be finite and positive")
        if not str(self.get_parameter("base_frame").value).strip():
            raise RuntimeError("base_frame must not be empty")
        self.samples: deque[tuple[float, float]] = deque(
            maxlen=stable_frames
        )
        self.pose_publisher = self.create_publisher(PoseStamped, "/xiaou/target_pose", 10)
        self.status_publisher = self.create_publisher(String, "/xiaou/target_status", 10)
        self.create_subscription(
            Image,
            str(self.get_parameter("image_topic").value),
            self.on_image,
            10,
        )

    def publish_status(self, status: str, **details: object) -> None:
        message = String()
        message.data = json.dumps({"status": status, **details}, ensure_ascii=False)
        self.status_publisher.publish(message)

    def pixel_to_base(self, u: float, v: float) -> tuple[float, float]:
        projected = self.homography @ np.array([u, v, 1.0], dtype=np.float64)
        if abs(float(projected[2])) < 1e-12:
            raise ValueError("homography produced a point at infinity")
        xy_mm = projected[:2] / projected[2]
        return float(xy_mm[0]) * 0.001, float(xy_mm[1]) * 0.001

    def on_image(self, message: Image) -> None:
        target_class = str(self.get_parameter("target_class").value).strip()
        table_z_m = float(self.get_parameter("table_z_m").value)
        grasp_height_m = self.grasp_heights.get(target_class)
        if (
            not target_class
            or not math.isfinite(table_z_m)
            or isinstance(grasp_height_m, bool)
            or not isinstance(grasp_height_m, (int, float))
            or not math.isfinite(grasp_height_m)
        ):
            self.publish_status(
                "configuration_incomplete",
                target_class=target_class,
                table_z_m=table_z_m,
                grasp_height_m=grasp_height_m,
                grasp_profiles=str(self.grasp_profiles_path),
            )
            return
        if message.header.stamp.sec == 0 and message.header.stamp.nanosec == 0:
            self.publish_status("image_timestamp_missing")
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            target_aliases = self.targeting_module.TARGET_ALIASES.get(
                target_class, {target_class}
            )
            candidates = [
                item for item in self.detector.detect(frame) if item.name in target_aliases
            ]
        except Exception as exc:
            self.samples.clear()
            self.get_logger().error(f"image processing failed: {exc}")
            self.publish_status("image_processing_failed", error=str(exc))
            return
        if not candidates:
            self.samples.clear()
            self.publish_status("target_not_found", target_class=target_class)
            return
        detection = max(candidates, key=lambda item: (item.conf, item.area))
        if not all(
            math.isfinite(float(value))
            for value in (detection.cx, detection.cy, detection.conf, detection.area)
        ):
            self.samples.clear()
            self.publish_status("invalid_detection_values")
            return
        if cv2.pointPolygonTest(self.calibration_hull, (float(detection.cx), float(detection.cy)), False) < 0:
            self.samples.clear()
            self.publish_status(
                "outside_calibrated_image_region",
                u=detection.cx,
                v=detection.cy,
                calibration=str(self.calibration_path),
            )
            return

        self.samples.append((float(detection.cx), float(detection.cy)))
        required = self.samples.maxlen or 2
        if len(self.samples) < required:
            self.publish_status("stabilizing", samples=len(self.samples), required=required)
            return
        samples = np.asarray(self.samples, dtype=np.float64)
        center = np.median(samples, axis=0)
        spread = float(np.max(np.linalg.norm(samples - center, axis=1)))
        if spread > float(self.get_parameter("max_center_spread_px").value):
            self.publish_status("target_unstable", spread_px=spread)
            return

        x_m, y_m = self.pixel_to_base(float(center[0]), float(center[1]))
        if not self.targeting_module.is_reachable(x_m * 1000.0, y_m * 1000.0):
            self.samples.clear()
            self.publish_status(
                "outside_robot_workspace",
                x_m=x_m,
                y_m=y_m,
                workspace="vision_targeting.is_reachable",
            )
            return
        target_z_m = table_z_m + grasp_height_m
        yaw = math.radians(float(self.targeting_module.estimate_theta_deg(frame, detection)))
        if not math.isfinite(yaw):
            self.samples.clear()
            self.publish_status("invalid_target_orientation")
            return
        qx, qy, qz, qw = quaternion_from_rpy(math.pi, 0.0, yaw)
        pose = PoseStamped()
        pose.header.stamp = message.header.stamp
        pose.header.frame_id = str(self.get_parameter("base_frame").value)
        pose.pose.position.x = x_m
        pose.pose.position.y = y_m
        pose.pose.position.z = target_z_m
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self.pose_publisher.publish(pose)
        self.publish_status(
            "target_pose_published",
            target_class=target_class,
            confidence=float(detection.conf),
            xyz_m=[x_m, y_m, target_z_m],
            table_z_m=table_z_m,
            grasp_height_m=grasp_height_m,
            yaw_rad=yaw,
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TargetPoseNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
