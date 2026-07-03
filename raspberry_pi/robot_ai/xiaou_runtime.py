from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
import logging
from logging.handlers import RotatingFileHandler
import os
from pathlib import Path

from dotenv import load_dotenv


def _env_str(name: str, default: str = "") -> str:
    value = os.getenv(name)
    return default if value is None or value == "" else value.strip()


def _env_int(name: str, default: int) -> int:
    value = os.getenv(name, "")
    return default if not value.strip() else int(float(value))


def _env_float(name: str, default: float) -> float:
    value = os.getenv(name, "")
    return default if not value.strip() else float(value)


def _env_bool(name: str, default: bool) -> bool:
    value = os.getenv(name, "")
    if not value.strip():
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _env_list(name: str, default: str) -> tuple[str, ...]:
    raw = _env_str(name, default)
    return tuple(item.strip() for item in raw.split(",") if item.strip())


@dataclass(frozen=True)
class XiaouConfig:
    project_dir: Path
    runtime_dir: Path
    log_dir: Path

    camera_index: int
    camera_width: int
    camera_height: int
    camera_fps: int
    camera_warmup_frames: int
    camera_open_retries: int
    camera_open_retry_delay_s: float

    yolo_model: str
    yolo_image_size: int
    yolo_conf: float
    yolo_strict_target: bool
    target_objects: tuple[str, ...]

    vision_debug: bool
    target_timeout_s: float
    target_filter_window: int
    target_stable_frames: int
    min_target_area_ratio: float

    pixel_origin_u: float
    pixel_origin_v: float
    pixel_to_base_scale_x: float
    pixel_to_base_scale_y: float
    base_offset_x_mm: float
    base_offset_y_mm: float
    workspace_x_min_mm: float
    workspace_x_max_mm: float
    workspace_y_min_mm: float
    workspace_y_max_mm: float
    z_safe_mm: float
    z_grab_mm: float

    gripper_open_margin_mm: float
    gripper_close_margin_mm: float
    gripper_open_min_mm: float
    gripper_open_max_mm: float
    gripper_force_pct: int

    enable_serial: bool
    serial_port: str
    serial_baud: int
    serial_protocol: str


def load_xiaou_config() -> XiaouConfig:
    project_dir = Path(__file__).resolve().parents[1]
    load_dotenv(project_dir / "config.env")
    load_dotenv(project_dir / "config.demo.env", override=True)
    runtime_dir = project_dir / "runtime"
    log_dir = runtime_dir / "logs"
    return XiaouConfig(
        project_dir=project_dir,
        runtime_dir=runtime_dir,
        log_dir=log_dir,
        camera_index=_env_int("CAMERA_INDEX", 0),
        camera_width=_env_int("CAMERA_WIDTH", 640),
        camera_height=_env_int("CAMERA_HEIGHT", 480),
        camera_fps=_env_int("CAMERA_FPS", 15),
        camera_warmup_frames=_env_int("CAMERA_WARMUP_FRAMES", 3),
        camera_open_retries=_env_int("CAMERA_OPEN_RETRIES", 3),
        camera_open_retry_delay_s=_env_float("CAMERA_OPEN_RETRY_DELAY_S", 0.6),
        yolo_model=_env_str("YOLO_MODEL", "xiaou_all.onnx"),
        yolo_image_size=_env_int("YOLO_IMAGE_SIZE", 640),
        yolo_conf=_env_float("YOLO_CONF", 0.35),
        yolo_strict_target=_env_bool("YOLO_STRICT_TARGET", False),
        target_objects=_env_list("TARGET_OBJECTS", "pen,cup,bottle"),
        vision_debug=_env_bool("VISION_DEBUG", True),
        target_timeout_s=_env_float("TARGET_TIMEOUT_S", 5.0),
        target_filter_window=_env_int("TARGET_FILTER_WINDOW", 5),
        target_stable_frames=_env_int("TARGET_STABLE_FRAMES", 3),
        min_target_area_ratio=_env_float("MIN_TARGET_AREA_RATIO", 0.015),
        pixel_origin_u=_env_float("PIXEL_ORIGIN_U", 320.0),
        pixel_origin_v=_env_float("PIXEL_ORIGIN_V", 240.0),
        pixel_to_base_scale_x=_env_float("PIXEL_TO_BASE_SCALE_X", 0.50),
        pixel_to_base_scale_y=_env_float("PIXEL_TO_BASE_SCALE_Y", 0.50),
        base_offset_x_mm=_env_float("BASE_OFFSET_X_MM", 200.0),
        base_offset_y_mm=_env_float("BASE_OFFSET_Y_MM", 0.0),
        workspace_x_min_mm=_env_float("WORKSPACE_X_MIN_MM", 80.0),
        workspace_x_max_mm=_env_float("WORKSPACE_X_MAX_MM", 360.0),
        workspace_y_min_mm=_env_float("WORKSPACE_Y_MIN_MM", -180.0),
        workspace_y_max_mm=_env_float("WORKSPACE_Y_MAX_MM", 180.0),
        z_safe_mm=_env_float("Z_SAFE_MM", 80.0),
        z_grab_mm=_env_float("Z_GRAB_MM", 25.0),
        gripper_open_margin_mm=_env_float("GRIPPER_OPEN_MARGIN_MM", 15.0),
        gripper_close_margin_mm=_env_float("GRIPPER_CLOSE_MARGIN_MM", 8.0),
        gripper_open_min_mm=_env_float("GRIPPER_OPEN_MIN_MM", 35.0),
        gripper_open_max_mm=_env_float("GRIPPER_OPEN_MAX_MM", 95.0),
        gripper_force_pct=_env_int("GRIPPER_FORCE_PCT", 60),
        enable_serial=_env_bool("ENABLE_SERIAL", False),
        serial_port=_env_str("SERIAL_PORT", "/dev/serial0"),
        serial_baud=_env_int("SERIAL_BAUD", 115200),
        serial_protocol=_env_str("SERIAL_PROTOCOL", "binary"),
    )


@lru_cache(maxsize=1)
def get_xiaou_config() -> XiaouConfig:
    return load_xiaou_config()


def setup_logging(name: str = "xiaou", level: int | None = None) -> logging.Logger:
    cfg = get_xiaou_config()
    cfg.log_dir.mkdir(parents=True, exist_ok=True)

    logger = logging.getLogger(name)
    if logger.handlers:
        return logger

    root_level = level if level is not None else logging.INFO
    logger.setLevel(root_level)
    logger.propagate = False
    formatter = logging.Formatter("%(asctime)s %(levelname)s [%(name)s] %(message)s")

    console = logging.StreamHandler()
    console.setFormatter(formatter)
    console.setLevel(root_level)
    logger.addHandler(console)

    file_handler = RotatingFileHandler(
        cfg.log_dir / "xiaou.log",
        maxBytes=2_000_000,
        backupCount=3,
        encoding="utf-8",
    )
    file_handler.setFormatter(formatter)
    file_handler.setLevel(root_level)
    logger.addHandler(file_handler)
    return logger


def get_logger(name: str) -> logging.Logger:
    return setup_logging(name)
