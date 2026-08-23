"""Voice and YOLO planning demo for the six-axis ROS2 stack.

This module deliberately stops at a validated arm-plan request. Execution is
owned by ROS2/MoveIt and is protected by the hardware safety gate.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2

PROJECT_DIR = Path(__file__).resolve().parents[1]
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))

from common import ask_cloud_intent
from device_runtime import open_cv_camera
from face_state import set_face_state
from vision_targeting import find_stable_target
from yolo_opencv import OpenCVDnnYolo
from xiaou_runtime import get_logger, get_xiaou_config


CFG = get_xiaou_config()
LOGGER = get_logger(__name__)


def build_plan(object_name: str, result=None) -> dict:
    """Convert a stable 2D target into a ROS2 planning request."""
    if result is not None and result.ok:
        payload = result.payload()
        payload["execution"] = "ros2_moveit"
        payload["six_axis"] = True
        return payload
    return {
        "cmd": "target_lost",
        "object": object_name,
        "execution": "ros2_moveit",
        "six_axis": True,
    }


def run_command(text: str, model: OpenCVDnnYolo, camera: cv2.VideoCapture) -> dict:
    intent = ask_cloud_intent(text)
    action = str(intent.get("action") or "chat").lower()
    object_name = intent.get("object")
    if action not in {"pick", "look"} or not object_name:
        plan = {"cmd": action, "object": object_name, "execution": "ros2_moveit"}
        print("arm_plan_request:", json.dumps(plan, ensure_ascii=False))
        return plan

    set_face_state("searching", f"searching {object_name}")
    result = find_stable_target(model, object_name, cap=camera)
    plan = build_plan(object_name, result)
    print("arm_plan_request:", json.dumps(plan, ensure_ascii=False))
    if result.ok:
        print("Planning only: ROS2 MoveIt owns six-axis execution after calibration.")
        set_face_state("happy", "plan ready")
    else:
        set_face_state("error", result.reason)
    return plan


def main() -> None:
    parser = argparse.ArgumentParser(description="XiaoU voice/vision planning preview")
    parser.add_argument("--text", help="single text command")
    args = parser.parse_args()

    model = OpenCVDnnYolo()
    camera = open_cv_camera()
    if camera is None or not camera.isOpened():
        raise RuntimeError("camera open failed")
    try:
        if args.text:
            run_command(args.text, model, camera)
            return
        while True:
            text = input("Command (q=quit): ").strip()
            if text.lower() in {"q", "quit", "exit"}:
                return
            if text:
                run_command(text, model, camera)
    finally:
        camera.release()
        set_face_state("idle", "standby")


if __name__ == "__main__":
    main()
