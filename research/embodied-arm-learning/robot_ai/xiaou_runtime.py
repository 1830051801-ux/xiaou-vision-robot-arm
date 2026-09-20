from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
import logging
from logging.handlers import RotatingFileHandler
import os
from pathlib import Path

from dotenv import load_dotenv


def _parse_bool(value: str, default: bool = False) -> bool:
    text = value.strip().lower()
    if not text:
        return default
    return text in {"1", "true", "yes", "on"}


def _env_str(name: str, default: str = "") -> str:
    value = os.getenv(name)
    return default if value is None or value == "" else value.strip()


def _env_int(name: str, default: int) -> int:
    value = os.getenv(name, "")
    if not value.strip():
        return default
    return int(float(value))


def _env_float(name: str, default: float) -> float:
    value = os.getenv(name, "")
    if not value.strip():
        return default
    return float(value)


def _env_bool(name: str, default: bool) -> bool:
    value = os.getenv(name, "")
    if not value.strip():
        return default
    return _parse_bool(value, default)


def _env_list(name: str, default: str) -> tuple[str, ...]:
    raw = _env_str(name, default)
    return tuple(item.strip() for item in raw.split(",") if item.strip())


@dataclass(frozen=True)
class XiaouConfig:
    project_dir: Path
    runtime_dir: Path
    log_dir: Path

    robot_name: str

    camera_index: int
    camera_width: int
    camera_height: int
    camera_fps: int
    camera_warmup_frames: int
    camera_open_retries: int
    camera_open_retry_delay_s: float

    voice_sample_rate: int
    record_seconds: float
    audio_input_device: str
    audio_output_device: str

    tts_engine: str
    tts_voice: str
    tts_pitch: str
    tts_rate: str
    tts_allow_robotic_fallback: bool
    espeak_voice: str
    espeak_speed: str

    ai_api_key: str
    ai_api_url: str
    ai_model: str
    ai_structured_output: bool
    ai_timeout_s: float

    baidu_asr_api_key: str
    baidu_asr_secret_key: str
    baidu_asr_dev_pid: int
    baidu_asr_rate: int

    wakeword_enabled: bool
    wakeword_require_second_stage: bool
    wakewords: tuple[str, ...]
    wake_chunk_seconds: float
    command_max_seconds: float
    command_min_seconds: float
    command_silence_seconds: float
    command_silence_rms: float
    demo_idle_sleep_s: float

    yolo_model: str
    yolo_image_size: int
    yolo_conf: float
    yolo_strict_target: bool
    vision_debug: bool
    target_timeout_s: float
    target_filter_window: int
    target_stable_frames: int
    min_target_area_ratio: float
    target_objects: tuple[str, ...]
    pixel_origin_u: float
    pixel_origin_v: float
    pixel_to_base_scale_x: float
    pixel_to_base_scale_y: float
    base_offset_x_mm: float
    base_offset_y_mm: float
    gripper_open_margin_mm: float
    gripper_close_margin_mm: float
    gripper_open_min_mm: float
    gripper_open_max_mm: float
    gripper_force_pct: int

    workspace_x_min_mm: float
    workspace_x_max_mm: float
    workspace_y_min_mm: float
    workspace_y_max_mm: float
    z_safe_mm: float
    z_grab_mm: float

    face_width: int
    face_height: int
    face_fullscreen: bool
    face_show_label: bool
    face_debug_border: bool
    face_remove_green_bg: bool
    face_max_frames: int
    face_frame_step: int

    deskpet_tts: bool
    deskpet_face_enabled: bool
    face_greet_cooldown_s: float
    face_greet_min_area: int
    face_greet_text: str


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
        robot_name=_env_str("ROBOT_NAME", "小U"),
        camera_index=_env_int("CAMERA_INDEX", 0),
        camera_width=_env_int("CAMERA_WIDTH", 640),
        camera_height=_env_int("CAMERA_HEIGHT", 480),
        camera_fps=_env_int("CAMERA_FPS", 15),
        camera_warmup_frames=_env_int("CAMERA_WARMUP_FRAMES", 3),
        camera_open_retries=_env_int("CAMERA_OPEN_RETRIES", 3),
        camera_open_retry_delay_s=_env_float("CAMERA_OPEN_RETRY_DELAY_S", 0.6),
        voice_sample_rate=_env_int("VOICE_SAMPLE_RATE", 16000),
        record_seconds=_env_float("RECORD_SECONDS", 5.0),
        audio_input_device=_env_str("AUDIO_INPUT_DEVICE", ""),
        audio_output_device=_env_str("AUDIO_OUTPUT_DEVICE", ""),
        tts_engine=_env_str("TTS_ENGINE", "local"),
        tts_voice=_env_str("TTS_VOICE", "zh-CN-XiaoxiaoNeural"),
        tts_pitch=_env_str("TTS_PITCH", ""),
        tts_rate=_env_str("TTS_RATE", "+0%"),
        tts_allow_robotic_fallback=_env_bool("TTS_ALLOW_ROBOTIC_FALLBACK", False),
        espeak_voice=_env_str("ESPEAK_VOICE", "zh"),
        espeak_speed=_env_str("ESPEAK_SPEED", "150"),
        ai_api_key=_env_str("AI_API_KEY", ""),
        ai_api_url=_env_str("AI_API_URL", ""),
        ai_model=_env_str("AI_MODEL", "deepseek-chat"),
        ai_structured_output=_env_bool("AI_STRUCTURED_OUTPUT", True),
        ai_timeout_s=_env_float("AI_TIMEOUT_S", 30.0),
        baidu_asr_api_key=_env_str("BAIDU_ASR_API_KEY", ""),
        baidu_asr_secret_key=_env_str("BAIDU_ASR_SECRET_KEY", ""),
        baidu_asr_dev_pid=_env_int("BAIDU_ASR_DEV_PID", 1537),
        baidu_asr_rate=_env_int("BAIDU_ASR_RATE", 16000),
        wakeword_enabled=_env_bool("WAKEWORD_ENABLED", True),
        wakeword_require_second_stage=_env_bool("WAKEWORD_REQUIRE_SECOND_STAGE", True),
        wakewords=_env_list("WAKEWORDS", "小U,xiao u"),
        wake_chunk_seconds=_env_float("WAKE_CHUNK_SECONDS", 1.4),
        command_max_seconds=_env_float("COMMAND_MAX_SECONDS", 8.0),
        command_min_seconds=_env_float("COMMAND_MIN_SECONDS", 1.0),
        command_silence_seconds=_env_float("COMMAND_SILENCE_SECONDS", 1.2),
        command_silence_rms=_env_float("COMMAND_SILENCE_RMS", 0.012),
        demo_idle_sleep_s=_env_float("DEMO_IDLE_SLEEP_S", 0.4),
        yolo_model=_env_str("YOLO_MODEL", "yolov5n.onnx"),
        yolo_image_size=_env_int("YOLO_IMAGE_SIZE", 640),
        yolo_conf=_env_float("YOLO_CONF", 0.35),
        yolo_strict_target=_env_bool("YOLO_STRICT_TARGET", False),
        vision_debug=_env_bool("VISION_DEBUG", True),
        target_timeout_s=_env_float("TARGET_TIMEOUT_S", 5.0),
        target_filter_window=_env_int("TARGET_FILTER_WINDOW", 5),
        target_stable_frames=_env_int("TARGET_STABLE_FRAMES", 3),
        min_target_area_ratio=_env_float("MIN_TARGET_AREA_RATIO", 0.015),
        target_objects=_env_list("TARGET_OBJECTS", "cup,bottle,pen,cell phone,book,keyboard,mouse"),
        pixel_origin_u=_env_float("PIXEL_ORIGIN_U", 320.0),
        pixel_origin_v=_env_float("PIXEL_ORIGIN_V", 240.0),
        pixel_to_base_scale_x=_env_float("PIXEL_TO_BASE_SCALE_X", 0.50),
        pixel_to_base_scale_y=_env_float("PIXEL_TO_BASE_SCALE_Y", 0.50),
        base_offset_x_mm=_env_float("BASE_OFFSET_X_MM", 200.0),
        base_offset_y_mm=_env_float("BASE_OFFSET_Y_MM", 0.0),
        gripper_open_margin_mm=_env_float("GRIPPER_OPEN_MARGIN_MM", 15.0),
        gripper_close_margin_mm=_env_float("GRIPPER_CLOSE_MARGIN_MM", 8.0),
        gripper_open_min_mm=_env_float("GRIPPER_OPEN_MIN_MM", 35.0),
        gripper_open_max_mm=_env_float("GRIPPER_OPEN_MAX_MM", 95.0),
        gripper_force_pct=_env_int("GRIPPER_FORCE_PCT", 60),
        workspace_x_min_mm=_env_float("WORKSPACE_X_MIN_MM", 80.0),
        workspace_x_max_mm=_env_float("WORKSPACE_X_MAX_MM", 360.0),
        workspace_y_min_mm=_env_float("WORKSPACE_Y_MIN_MM", -180.0),
        workspace_y_max_mm=_env_float("WORKSPACE_Y_MAX_MM", 180.0),
        z_safe_mm=_env_float("Z_SAFE_MM", 80.0),
        z_grab_mm=_env_float("Z_GRAB_MM", 25.0),
        face_width=_env_int("FACE_WIDTH", 360),
        face_height=_env_int("FACE_HEIGHT", 270),
        face_fullscreen=_env_bool("FACE_FULLSCREEN", False),
        face_show_label=_env_bool("FACE_SHOW_LABEL", False),
        face_debug_border=_env_bool("FACE_DEBUG_BORDER", False),
        face_remove_green_bg=_env_bool("FACE_REMOVE_GREEN_BG", False),
        face_max_frames=_env_int("FACE_MAX_FRAMES", 8),
        face_frame_step=_env_int("FACE_FRAME_STEP", 4),
        deskpet_tts=_env_bool("DESKPET_TTS", True),
        deskpet_face_enabled=_env_bool("DESKPET_FACE_ENABLED", False),
        face_greet_cooldown_s=_env_float("FACE_GREET_COOLDOWN_S", 8.0),
        face_greet_min_area=_env_int("FACE_GREET_MIN_AREA", 6500),
        face_greet_text=_env_str("FACE_GREET_TEXT", "Hello, I am XiaoU."),
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

    formatter = logging.Formatter(
        fmt="%(asctime)s %(levelname)s [%(name)s] %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )

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
