from __future__ import annotations

import math
import time
from collections import deque
from dataclasses import dataclass

import cv2
import numpy as np
import yaml

from common import PROJECT_DIR, get_camera_index
from arm_control.task_planner import PlanningBlocked, load_homography, project_center
from xiaou_runtime import get_xiaou_config
from yolo_opencv import Detection, OpenCVDnnYolo


DEBUG_DIR = PROJECT_DIR / "runtime" / "vision_debug"
TARGET_ALIASES = {
    "pen": {"pen", "Pen", "pencil", "Pencil"},
    "cup": {"cup", "coffee cup", "mug", "tea cup"},
    "cola": {"cola", "coke", "coca cola"},
    "bottle": {"bottle", "water bottle", "drink bottle"},
    "earphone": {"earphone", "headphone", "headphones", "headset"},
}

SMALL_OBJECT_TARGETS = {"pen"}
PEN_TARGETS = {"pen"}

PEN_MIN_ASPECT = 1.2
PEN_MIN_CONF = 0.22

TARGET_CONF_OVERRIDES = {
    "pen": PEN_MIN_CONF,
    "cup": 0.45,
    "cola": 0.55,
    "bottle": 0.55,
    "earphone": 0.60,
}


def env_float(name: str, default: float) -> float:
    cfg = get_xiaou_config()
    value = getattr(cfg, name.lower(), None)
    if value is None:
        return default
    return float(value)


def env_int(name: str, default: int) -> int:
    cfg = get_xiaou_config()
    value = getattr(cfg, name.lower(), None)
    if value is None:
        return default
    return int(value)


@dataclass
class TargetResult:
    ok: bool
    reason: str
    obj: str
    u: int | None = None
    v: int | None = None
    x_base_mm: float | None = None
    y_base_mm: float | None = None
    theta_deg: float = 0.0
    z_safe_mm: float = 80.0
    z_grab_mm: float = 25.0
    width_mm: float = 0.0
    grip_type: str = "top_grip"
    gripper_open_mm: float = 60.0
    gripper_close_mm: float = 35.0
    grip_force_pct: int = 60

    def payload(self) -> dict:
        if not self.ok:
            return {
                "cmd": self.reason,
                "object": self.obj,
                "execution": "ros2_moveit",
                "six_axis": True,
            }
        return {
            "cmd": "pick",
            "object": self.obj,
            "execution": "ros2_moveit",
            "six_axis": True,
            "u": self.u,
            "v": self.v,
            "x_base_mm": round(float(self.x_base_mm), 1),
            "y_base_mm": round(float(self.y_base_mm), 1),
            "theta_deg": round(float(self.theta_deg), 1),
            "z_safe_mm": round(float(self.z_safe_mm), 1),
            "z_grab_mm": round(float(self.z_grab_mm), 1),
            "width_mm": round(float(self.width_mm), 1),
            "grip_type": self.grip_type,
            "gripper_open_mm": round(float(self.gripper_open_mm), 1),
            "gripper_close_mm": round(float(self.gripper_close_mm), 1),
            "grip_force_pct": int(self.grip_force_pct),
        }


class SlidingTargetFilter:
    def __init__(self, size: int) -> None:
        self.samples: deque[tuple[float, float, float, float]] = deque(maxlen=max(1, size))

    def add(self, u: float, v: float, theta: float, width: float) -> None:
        self.samples.append((u, v, theta, width))

    def ready(self, min_samples: int) -> bool:
        return len(self.samples) >= min_samples

    def stable(self, min_samples: int, max_center_spread_px: float = 28.0) -> bool:
        if not self.ready(min_samples):
            return False
        arr = np.array(self.samples, dtype=np.float32)
        centers = arr[:, :2]
        center = np.median(centers, axis=0)
        spread = np.linalg.norm(centers - center, axis=1)
        return float(np.max(spread)) <= max_center_spread_px

    def mean(self) -> tuple[float, float, float, float]:
        arr = np.array(self.samples, dtype=np.float32)
        return tuple(float(v) for v in np.median(arr, axis=0))


def estimate_theta_deg(frame: np.ndarray, det: Detection) -> float:
    roi = frame[det.y1:det.y2, det.x1:det.x2]
    if roi.size == 0:
        return 0.0
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(gray, 60, 160)
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return 0.0
    contour = max(contours, key=cv2.contourArea)
    if cv2.contourArea(contour) < 40:
        return 0.0
    rect = cv2.minAreaRect(contour)
    angle = float(rect[2])
    w, h = rect[1]
    if w < h:
        angle += 90.0
    while angle > 90.0:
        angle -= 180.0
    while angle < -90.0:
        angle += 180.0
    return angle


def _pen_fallback_detection(frame: np.ndarray) -> Detection | None:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(gray, 40, 120)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 3))
    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=1)
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    h, w = frame.shape[:2]
    best: tuple[float, Detection] | None = None
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < 50 or area > (w * h) * 0.12:
            continue
        rect = cv2.minAreaRect(contour)
        (cx, cy), (rw, rh), angle = rect
        long_side = max(rw, rh)
        short_side = min(rw, rh)
        if short_side < 2 or long_side < 10:
            continue
        aspect = long_side / max(short_side, 1.0)
        if aspect < 1.35:
            continue
        fill_ratio = area / max(rw * rh, 1.0)
        score = aspect * 0.9 + fill_ratio * 1.0 + min(area / 1200.0, 1.5)
        x, y, bw, bh = cv2.boundingRect(contour)
        det = Detection(
            name="pen",
            x1=max(0, x),
            y1=max(0, y),
            x2=min(w - 1, x + bw),
            y2=min(h - 1, y + bh),
            conf=float(min(0.72, max(0.18, score / 3.5))),
        )
        if best is None or score > best[0]:
            best = (score, det)
    return best[1] if best is not None else None


def pen_fallback_detection(frame: np.ndarray) -> Detection | None:
    line = _pen_line_fallback_detection(frame)
    if line is not None:
        return line
    return _pen_fallback_detection(frame)


def _pen_line_fallback_detection(frame: np.ndarray) -> Detection | None:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(gray, 35, 110)
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=14, minLineLength=16, maxLineGap=8)
    if lines is None:
        return None

    h, w = frame.shape[:2]
    best: tuple[float, Detection] | None = None
    for entry in lines[:, 0, :]:
        x1, y1, x2, y2 = map(int, entry)
        length = float(math.hypot(x2 - x1, y2 - y1))
        if length < 16:
            continue
        angle = abs(math.degrees(math.atan2(y2 - y1, x2 - x1)))
        angle = min(angle, 180.0 - angle)
        box_w = abs(x2 - x1)
        box_h = abs(y2 - y1)
        box_area = max(1.0, float((box_w + 8) * (box_h + 8)))
        aspect = length / max(4.0, min(box_w + 8, box_h + 8))
        if aspect < 1.9:
            continue
        score = length / 40.0 + aspect * 0.8 + max(0.0, 1.0 - abs(angle - 45.0) / 90.0)
        x_min = max(0, min(x1, x2) - 6)
        y_min = max(0, min(y1, y2) - 6)
        x_max = min(w - 1, max(x1, x2) + 6)
        y_max = min(h - 1, max(y1, y2) + 6)
        det = Detection(
            name="pen",
            x1=x_min,
            y1=y_min,
            x2=x_max,
            y2=y_max,
            conf=float(min(0.62, max(0.16, score / 5.0))),
        )
        if best is None or score > best[0]:
            best = (score, det)
    return best[1] if best is not None else None


def apply_rotation_calibration(theta_deg: float) -> float:
    calib_path = PROJECT_DIR / "runtime" / "calibration" / "rotation_calibration.yaml"
    if not calib_path.exists():
        return theta_deg
    try:
        data = yaml.safe_load(calib_path.read_text(encoding="utf-8")) or {}
        theta_deg += float(data.get("offset_deg", 0.0))
    except Exception:
        return theta_deg
    while theta_deg > 180.0:
        theta_deg -= 360.0
    while theta_deg < -180.0:
        theta_deg += 360.0
    return theta_deg


def pixel_to_base_mm(u: float, v: float) -> tuple[float, float]:
    workspace_path = PROJECT_DIR / "codex_pickup_package" / "workspace_homography.yaml"
    homography = load_homography(workspace_path)
    x_table_m, y_table_m = project_center(homography, float(u), float(v))
    return x_table_m * 1000.0, y_table_m * 1000.0


def is_reachable(x_mm: float, y_mm: float) -> bool:
    cfg = get_xiaou_config()
    radius_mm = math.hypot(x_mm, y_mm)
    return (
        cfg.workspace_x_min_mm <= x_mm <= cfg.workspace_x_max_mm
        and cfg.workspace_y_min_mm <= y_mm <= cfg.workspace_y_max_mm
        and 250.0 <= radius_mm <= 430.0
    )


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def grip_plan_for_object(obj: str, width_mm: float) -> tuple[str, float, float, int]:
    cfg = get_xiaou_config()
    obj = str(obj).strip().lower()
    grip_type = "top_grip"
    if obj in {"cup", "cola", "bottle", "coffee cup", "mug", "wine glass"}:
        grip_type = "side_grip"
    elif obj in {"pen", "pencil", "earphone"}:
        grip_type = "top_grip"
    elif obj in {"book", "computer keyboard"}:
        grip_type = "flat_grip"

    gripper_open = clamp(width_mm + cfg.gripper_open_margin_mm, cfg.gripper_open_min_mm, cfg.gripper_open_max_mm)
    gripper_close = clamp(width_mm - cfg.gripper_close_margin_mm, 5.0, gripper_open)
    return grip_type, gripper_open, gripper_close, cfg.gripper_force_pct


def target_names_for(obj: str) -> set[str]:
    if get_xiaou_config().yolo_strict_target:
        return {obj}
    return TARGET_ALIASES.get(obj, {obj})


def _verify_pen_shape(roi: np.ndarray) -> bool:
    """Quick check: does this region look like a pen (long + thin + not gray)?"""
    if roi.size == 0:
        return False
    h, w = roi.shape[:2]
    aspect = max(w, h) / max(min(w, h), 1)
    if aspect < PEN_MIN_ASPECT:
        return False
    area = w * h
    if area < 120 or area > 90_000:
        return False
    return True


def _get_min_conf(target: str) -> float:
    """Get minimum confidence for a target, with per-class overrides."""
    return TARGET_CONF_OVERRIDES.get(target, get_xiaou_config().yolo_conf)


def candidate_score(det: Detection, frame_shape: tuple[int, int, int]) -> float:
    h, w = frame_shape[:2]
    center_x = w / 2.0
    center_y = h / 2.0
    dist = math.hypot((det.cx - center_x) / max(w, 1), (det.cy - center_y) / max(h, 1))
    center_bonus = max(0.0, 1.0 - dist)
    area_ratio = det.area / max(float(w * h), 1.0)
    return det.conf * 3.0 + area_ratio * 2.0 + center_bonus


def save_debug_frame(frame: np.ndarray, detections: list[Detection], target: str, name: str) -> None:
    if not get_xiaou_config().vision_debug:
        return
    DEBUG_DIR.mkdir(parents=True, exist_ok=True)
    image = frame.copy()
    for det in detections:
        color = (0, 255, 0) if det.name in target_names_for(target) else (120, 120, 120)
        cv2.rectangle(image, (det.x1, det.y1), (det.x2, det.y2), color, 2)
        label = f"{det.name} {det.conf:.2f}"
        cv2.putText(image, label, (det.x1, max(20, det.y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
    cv2.imwrite(str(DEBUG_DIR / name), image)


def find_stable_target(
    model: OpenCVDnnYolo,
    target: str,
    timeout_s: float | None = None,
    cap: cv2.VideoCapture | None = None,
    min_area_ratio: float | None = None,
    filter_size: int | None = None,
    min_stable: int | None = None,
) -> TargetResult:
    cfg = get_xiaou_config()
    timeout_s = timeout_s if timeout_s is not None else cfg.target_timeout_s
    min_area_ratio = min_area_ratio if min_area_ratio is not None else cfg.min_target_area_ratio
    filter_size = filter_size if filter_size is not None else cfg.target_filter_window
    min_stable = min_stable if min_stable is not None else cfg.target_stable_frames
    z_safe = cfg.z_safe_mm
    z_grab = cfg.z_grab_mm

    own_cap = cap is None
    cap = cap if cap is not None else cv2.VideoCapture(get_camera_index())
    if cap is None or not cap.isOpened():
        return TargetResult(False, "camera_failed", target)

    filt = SlidingTargetFilter(filter_size)
    deadline = time.time() + timeout_s
    last_seen = False
    last_reject_reason: str | None = None
    last_frame: np.ndarray | None = None
    last_detections: list[Detection] = []
    accepted_names = target_names_for(target)
    target_area_ratio = min_area_ratio * 0.25 if target in SMALL_OBJECT_TARGETS else min_area_ratio
    target_min_stable = max(2, min_stable - 2) if target in SMALL_OBJECT_TARGETS else min_stable
    try:
        while time.time() < deadline:
            ok, frame = cap.read()
            if not ok:
                continue
            last_frame = frame
            frame_area = frame.shape[0] * frame.shape[1]
            detections = model.detect(frame)
            last_detections = detections
            min_conf = _get_min_conf(target)
            name_candidates = [det for det in detections if det.name in accepted_names]
            if not name_candidates:
                continue
            last_seen = True

            candidates = [
                det for det in name_candidates
                if det.conf >= min_conf and det.area >= frame_area * target_area_ratio
            ]
            if not candidates:
                continue
            det = max(candidates, key=lambda item: candidate_score(item, frame.shape))
            theta = apply_rotation_calibration(estimate_theta_deg(frame, det))
            width_mm = max(det.x2 - det.x1, det.y2 - det.y1) * 0.50
            filt.add(det.cx, det.cy, theta, width_mm)
            if not filt.stable(target_min_stable):
                continue

            u, v, theta_mean, width_mean = filt.mean()
            try:
                x_mm, y_mm = pixel_to_base_mm(u, v)
            except (OSError, PlanningBlocked):
                save_debug_frame(frame, detections, target, "workspace_calibration_invalid.jpg")
                return TargetResult(
                    False,
                    "workspace_calibration_invalid",
                    target,
                    int(round(u)),
                    int(round(v)),
                )
            grip_type, gripper_open, gripper_close, force = grip_plan_for_object(target, width_mean)
            if not is_reachable(x_mm, y_mm):
                save_debug_frame(frame, detections, target, "last_out_of_range.jpg")
                return TargetResult(
                    False,
                    "out_of_range",
                    target,
                    int(round(u)),
                    int(round(v)),
                    x_mm,
                    y_mm,
                    theta_mean,
                    z_safe,
                    z_grab,
                    width_mean,
                    grip_type,
                    gripper_open,
                    gripper_close,
                    force,
                )
            save_debug_frame(frame, detections, target, "last_pick.jpg")
            return TargetResult(
                True,
                "ok",
                target,
                int(round(u)),
                int(round(v)),
                x_mm,
                y_mm,
                theta_mean,
                z_safe,
                z_grab,
                width_mean,
                grip_type,
                gripper_open,
                gripper_close,
                force,
            )
    finally:
        if own_cap:
            cap.release()

    if last_frame is not None:
        save_debug_frame(last_frame, last_detections, target, "last_not_found.jpg")
    return TargetResult(False, last_reject_reason or ("not_found" if last_seen else "target_lost"), target)

