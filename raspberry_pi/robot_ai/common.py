from __future__ import annotations

from functools import lru_cache
from pathlib import Path

from xiaou_runtime import XiaouConfig, get_logger, get_xiaou_config


PROJECT_DIR = Path(__file__).resolve().parents[1]
LOGGER = get_logger(__name__)


@lru_cache(maxsize=1)
def get_config() -> XiaouConfig:
    return get_xiaou_config()


def get_camera_index() -> int:
    return get_config().camera_index


def get_target_objects() -> list[str]:
    return list(get_config().target_objects)


def get_yolo_model() -> str:
    return get_config().yolo_model


def get_yolo_imgsz() -> int:
    return get_config().yolo_image_size


def get_yolo_conf() -> float:
    return get_config().yolo_conf


def normalize_object_name(text: str) -> str | None:
    lowered = text.lower()
    aliases = {
        "cup": ("cup", "coffee cup", "mug"),
        "bottle": ("bottle", "water bottle"),
        "pen": ("pen", "pencil"),
    }
    for name, keys in aliases.items():
        if any(key in lowered for key in keys):
            return name
    return None


def serial_enabled() -> bool:
    return get_config().enable_serial
