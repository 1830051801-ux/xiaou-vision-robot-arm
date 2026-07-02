"""Find a pen and send pick command to STM32 via serial.

Usage:
    python robot_ai/pick_pen_demo.py              # find pen and send
    python robot_ai/pick_pen_demo.py --dry-run    # just show what would be sent
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import cv2

from common import PROJECT_DIR, ask_cloud_intent, serial_enabled
from device_runtime import configure_sounddevice, open_cv_camera
from face_state import set_face_state
from robot_protocol import send_robot_payload, frame_to_hex, encode_motion_frame
from vision_targeting import find_stable_target
from yolo_opencv import OpenCVDnnYolo
from xiaou_runtime import get_xiaou_config, get_logger

LOGGER = get_logger(__name__)


def main() -> None:
    parser = argparse.ArgumentParser(description="Pick pen — find and send to STM32")
    parser.add_argument("--dry-run", action="store_true", help="print command without sending")
    parser.add_argument("--timeout", type=float, default=10.0, help="search timeout seconds")
    args = parser.parse_args()

    cfg = get_xiaou_config()
    print(f"Pen picker starting — model: {cfg.yolo_model}")
    print(f"Serial: {cfg.serial_port} @ {cfg.serial_baud} ({cfg.serial_protocol})")

    # Init
    configure_sounddevice()
    print("Loading YOLO...")
    model = OpenCVDnnYolo()
    print("Opening camera...")
    cap = open_cv_camera()
    if cap is None:
        print("ERROR: Camera failed to open")
        sys.exit(1)

    try:
        print(f"Searching for pen (timeout {args.timeout}s)...")
        set_face_state("searching", "正在找笔")
        result = find_stable_target(model, "pen", timeout_s=args.timeout, cap=cap)

        if not result.ok:
            print(f"Pen NOT found: {result.reason}")
            set_face_state("error", f"没找到笔: {result.reason}")
            return

        # Build pick payload
        payload = result.payload()
        print("\n" + "=" * 50)
        print(f"  PEN FOUND — {result.obj}")
        print(f"  Position: X={payload.get('x_base_mm')}mm Y={payload.get('y_base_mm')}mm")
        print(f"  Angle: {payload.get('theta_deg')}deg")
        print(f"  Size: {payload.get('width_mm')}mm")
        print(f"  Grip: {payload.get('grip_type')} open={payload.get('gripper_open_mm')}mm close={payload.get('gripper_close_mm')}mm")
        print(f"  Z: safe={payload.get('z_safe_mm')}mm grab={payload.get('z_grab_mm')}mm")
        print("=" * 50)

        # Show binary frame
        frame = encode_motion_frame(payload)
        print(f"  MCU frame: {frame_to_hex(frame)}")
        print(f"  JSON: {json.dumps(payload, ensure_ascii=False)}")

        set_face_state("happy", "找到笔了")

        if args.dry_run:
            print("\n[Dry run — not sent to STM32]")
            return

        if not serial_enabled():
            print("\nWARNING: ENABLE_SERIAL=false in config.env — forcing send anyway.")
        else:
            print("\nSerial enabled — sending to STM32...")

        try:
            send_robot_payload(payload)
            print("Sent OK")
        except Exception as exc:
            print(f"Send failed: {exc}")

    finally:
        cap.release()


if __name__ == "__main__":
    main()
