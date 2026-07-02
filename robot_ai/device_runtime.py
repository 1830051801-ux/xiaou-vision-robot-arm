from __future__ import annotations

import time

import cv2
import sounddevice as sd

from xiaou_runtime import get_xiaou_config


def _resolve_sounddevice(value: str, kind: str) -> int | None:
    value = value.strip()
    if not value:
        return None
    if value.isdigit():
        return int(value)

    key = "max_input_channels" if kind == "input" else "max_output_channels"
    needle = value.lower()
    matches = []
    for index, info in enumerate(sd.query_devices()):
        if info.get(key, 0) <= 0:
            continue
        if needle in str(info.get("name", "")).lower():
            matches.append(index)
    if matches:
        return matches[0]
    raise RuntimeError(f"{kind} audio device not found: {value}")


def configure_sounddevice() -> tuple[int | None, int | None]:
    cfg = get_xiaou_config()
    input_id = _resolve_sounddevice(cfg.audio_input_device, "input")
    output_id = _resolve_sounddevice(cfg.audio_output_device, "output")

    current_in, current_out = sd.default.device
    if input_id is None:
        input_id = current_in
    if output_id is None:
        output_id = current_out

    if input_id is not None or output_id is not None:
        sd.default.device = (input_id, output_id)
    if cfg.voice_sample_rate:
        sd.default.samplerate = cfg.voice_sample_rate
    return input_id, output_id


def open_cv_camera(index: int | None = None) -> cv2.VideoCapture | None:
    cfg = get_xiaou_config()
    cam_index = cfg.camera_index if index is None else index
    width = cfg.camera_width
    height = cfg.camera_height
    fps = cfg.camera_fps
    warmup_frames = cfg.camera_warmup_frames
    retries = cfg.camera_open_retries
    retry_delay_s = cfg.camera_open_retry_delay_s

    candidate_indices: list[int] = []
    if cam_index is not None:
        candidate_indices.append(int(cam_index))
    for fallback in range(4):
        if fallback not in candidate_indices:
            candidate_indices.append(fallback)

    cap = None
    for cam_index in candidate_indices:
        for attempt in range(max(1, retries)):
            cap = cv2.VideoCapture(cam_index, cv2.CAP_V4L2)
            if not cap.isOpened():
                cap.release()
                cap = cv2.VideoCapture(cam_index)
            if cap.isOpened():
                break
            cap.release()
            cap = None
            if attempt < retries - 1:
                time.sleep(retry_delay_s)
        if cap is not None and cap.isOpened():
            break
    if cap is None:
        return None

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    for _ in range(max(1, warmup_frames)):
        ok, _frame = cap.read()
        if ok:
            break
        time.sleep(0.05)
    return cap
