from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import yaml

from common import PROJECT_DIR
from arm_control import fk_space, load_default_model
from arm_control.safety import load_hardware_config, validate_motion_readiness
from arm_control.task_planner import (
    PlanningBlocked,
    REQUIRED_HEIGHT_PX,
    REQUIRED_WIDTH_PX,
    load_homography,
    load_object_profiles,
)
from xiaou_runtime import get_xiaou_config


def check(condition: bool, label: str, detail: str, failures: list[str], *, critical: bool = True) -> None:
    status = "OK" if condition else ("FAIL" if critical else "WARN")
    print(f"[{status}] {label}: {detail}")
    if not condition and critical:
        failures.append(label)


def main() -> int:
    parser = argparse.ArgumentParser(description="XiaoU project preflight check")
    parser.add_argument("--camera", action="store_true", help="also open one camera frame")
    parser.add_argument(
        "--require-motion",
        action="store_true",
        help="treat unmeasured camera, object strategy, and hardware gates as failures",
    )
    args = parser.parse_args()

    cfg = get_xiaou_config()
    failures: list[str] = []

    model_path = Path(cfg.yolo_model)
    if not model_path.is_absolute():
        model_path = PROJECT_DIR / "models" / model_path
    check(model_path.is_file(), "YOLO model", str(model_path), failures)

    names_path = model_path.with_suffix(".names")
    check(names_path.is_file(), "YOLO classes", str(names_path), failures)

    homography_path = PROJECT_DIR / "codex_pickup_package" / "workspace_homography.yaml"
    homography_ok = False
    homography_detail = str(homography_path)
    try:
        load_homography(homography_path)
        homography_ok = True
        homography_detail += f"; verified for {REQUIRED_WIDTH_PX}x{REQUIRED_HEIGHT_PX}"
    except (OSError, PlanningBlocked) as exc:
        homography_detail += f"; {exc}"
    check(
        homography_ok,
        "workspace calibration",
        homography_detail,
        failures,
        critical=args.require_motion,
    )

    profile_ok = False
    profile_detail = ""
    try:
        profiles = load_object_profiles()
        incomplete: list[str] = []
        for name, profile in profiles["classes"].items():
            required = ("grasp_height_m", "approach_height_m", "gripper_open_pwm_deg", "gripper_close_pwm_deg", "placement_pose_id")
            if not isinstance(profile, dict) or any(profile.get(field) is None for field in required):
                incomplete.append(str(name))
        profile_ok = not incomplete
        profile_detail = "all classes measured" if profile_ok else "locked/unmeasured: " + ", ".join(incomplete)
    except (OSError, PlanningBlocked, ValueError) as exc:
        profile_detail = str(exc)
    check(
        profile_ok,
        "object grasp strategies",
        profile_detail,
        failures,
        critical=args.require_motion,
    )

    try:
        arm_model = load_default_model()
        home = fk_space(arm_model.home_grasp_tcp, arm_model.screw_axes, np.zeros(6))
        arm_model_ok = home.shape == (4, 4) and np.isfinite(home).all()
        arm_model_detail = f"{arm_model.name}; joints={len(arm_model.joint_names)}"
    except Exception as exc:
        arm_model_ok = False
        arm_model_detail = str(exc)
    check(arm_model_ok, "six-axis kinematics", arm_model_detail, failures)

    readiness = validate_motion_readiness(load_hardware_config())
    readiness_detail = "ready" if readiness.ready else "locked: " + ", ".join(readiness.missing_or_invalid)
    check(
        readiness.ready,
        "six-axis hardware calibration",
        readiness_detail,
        failures,
        critical=args.require_motion,
    )

    if args.camera:
        import cv2

        cap = cv2.VideoCapture(cfg.camera_index)
        ok, frame = cap.read() if cap.isOpened() else (False, None)
        cap.release()
        resolution_ok = bool(
            ok
            and frame is not None
            and frame.shape[1] == REQUIRED_WIDTH_PX
            and frame.shape[0] == REQUIRED_HEIGHT_PX
        )
        detail = (
            f"index={cfg.camera_index}; frame={frame.shape[1]}x{frame.shape[0]}"
            if ok and frame is not None
            else f"index={cfg.camera_index}; no frame"
        )
        check(resolution_ok, "camera", detail, failures, critical=args.require_motion)

    if failures:
        print("Preflight failed: " + ", ".join(failures))
        return 1
    print("Preflight passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
