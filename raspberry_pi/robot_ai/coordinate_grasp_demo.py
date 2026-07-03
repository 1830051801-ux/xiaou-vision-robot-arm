from __future__ import annotations

import argparse

from robot_protocol import encode_motion_frame, frame_to_hex, send_robot_payload
from vision_targeting import find_stable_target
from yolo_opencv import OpenCVDnnYolo


def build_coordinate_payload(obj: str) -> dict:
    model = OpenCVDnnYolo()
    result = find_stable_target(model, obj)
    payload = result.payload()
    if not result.ok:
        raise RuntimeError(f"target not ready: {payload}")

    payload["cmd"] = "pick"
    payload.setdefault("pick_z_mm", payload.get("z_grab_mm", 225.0))
    payload.setdefault("grasp_z_safe_mm", payload.get("z_safe_mm", 80.0))
    payload.setdefault("drop_x_mm", payload["x_base_mm"])
    payload.setdefault("drop_y_mm", payload["y_base_mm"])
    payload.setdefault("drop_z_mm", payload["pick_z_mm"])
    payload.setdefault("drop_yaw_deg", payload.get("theta_deg", 0.0))
    payload.setdefault("grip_id", 3)
    payload.setdefault("grip_profile", 1)
    return payload


def main() -> None:
    parser = argparse.ArgumentParser(description="Detect an object and send calibrated coordinates to STM32.")
    parser.add_argument("--object", default="pen", help="target name used by the YOLO adapter, for example pen/cup/bottle")
    parser.add_argument("--send-serial", action="store_true", help="write the frame to the configured STM32 serial port")
    parser.add_argument("--seq", type=int, default=1)
    args = parser.parse_args()

    payload = build_coordinate_payload(args.object)
    frame = encode_motion_frame(payload, seq=args.seq)

    print("target:", payload["object"])
    print("pixel:", payload.get("u"), payload.get("v"))
    print("base_mm:", payload["x_base_mm"], payload["y_base_mm"])
    print("yaw_deg:", payload.get("theta_deg", 0.0))
    print("frame:", frame_to_hex(frame))

    if args.send_serial:
        send_robot_payload(payload, seq=args.seq)
    else:
        print("dry-run: serial write skipped")


if __name__ == "__main__":
    main()
