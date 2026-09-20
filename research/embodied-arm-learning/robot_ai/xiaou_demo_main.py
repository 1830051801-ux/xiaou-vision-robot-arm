from __future__ import annotations

import argparse
import json
import re
import time
from pathlib import Path

import cv2
import numpy as np
import sounddevice as sd
import soundfile as sf

from baidu_asr import baidu_asr_wav
from common import PROJECT_DIR, ask_cloud_intent
from device_runtime import configure_sounddevice, open_cv_camera
from dialog_emote_bridge import set_dialog_face
from face_state import set_face_state
from robot_ai_07_import import cloud_chat, speak_text
from emotion_state import xiaou_style_prompt
from xiaou_runtime import get_logger, get_xiaou_config
from vision_targeting import find_stable_target
from yolo_opencv import OpenCVDnnYolo


CFG = get_xiaou_config()
LOGGER = get_logger(__name__)

SAMPLE_RATE = CFG.voice_sample_rate
WAKE_CHUNK_SECONDS = CFG.wake_chunk_seconds
COMMAND_MAX_SECONDS = CFG.command_max_seconds
COMMAND_MIN_SECONDS = CFG.command_min_seconds
SILENCE_SECONDS = CFG.command_silence_seconds
SILENCE_RMS = CFG.command_silence_rms
IDLE_SLEEP_S = CFG.demo_idle_sleep_s
WAKEWORD_ENABLED = CFG.wakeword_enabled
WAKEWORD_REQUIRE_SECOND_STAGE = CFG.wakeword_require_second_stage
WAKEWORDS = [item.strip().lower() for item in CFG.wakewords if item.strip()]
ENABLE_DENOISE = getattr(CFG, 'enable_denoise', True)

try:
    from scipy.signal import butter, lfilter
    _HAS_SCIPY = True
except ImportError:
    _HAS_SCIPY = False


def _reduce_noise(audio: np.ndarray) -> np.ndarray:
    """Remove low-frequency hum and high-frequency hiss from speech audio."""
    if audio.size == 0:
        return audio
    if audio.ndim > 1:
        audio = audio.mean(axis=1)
    if _HAS_SCIPY:
        try:
            nyq = 0.5 * SAMPLE_RATE
            b, a = butter(4, [80.0 / nyq, 7500.0 / nyq], btype="band")
            return lfilter(b, a, audio).astype(np.float32)
        except Exception:
            pass
    try:
        window = max(2, int(SAMPLE_RATE / 80))
        kernel = np.ones(window) / window
        smoothed = np.convolve(audio, kernel, mode="same")
        smoothed -= np.mean(smoothed)
        return smoothed.astype(np.float32)
    except Exception:
        return audio

OBJECT_CN = {
    "Coffee cup": "水杯",
    "Bottle": "瓶子",
    "Pen": "笔",
    "Pencil": "铅笔",
    "Mobile phone": "手机",
    "Book": "书",
    "Computer keyboard": "键盘",
    "Computer mouse": "鼠标",
    "Scissors": "剪刀",
}


def write_wav(path: Path, audio: np.ndarray) -> None:
    if ENABLE_DENOISE:
        audio = _reduce_noise(audio)
    sf.write(path, audio, SAMPLE_RATE, subtype="PCM_16")


def record_fixed(path: Path, seconds: float) -> None:
    audio = sd.rec(int(seconds * SAMPLE_RATE), samplerate=SAMPLE_RATE, channels=1, dtype="float32")
    sd.wait()
    audio = audio.flatten()
    write_wav(path, audio)


def record_until_silence(path: Path) -> None:
    chunk_s = 0.2
    chunk_n = int(SAMPLE_RATE * chunk_s)
    max_chunks = int(COMMAND_MAX_SECONDS / chunk_s)
    min_chunks = int(COMMAND_MIN_SECONDS / chunk_s)
    silence_need = int(SILENCE_SECONDS / chunk_s)
    silence_count = 0
    chunks: list[np.ndarray] = []

    print("Command recording started. Stop speaking to finish.")
    for index in range(max_chunks):
        chunk = sd.rec(chunk_n, samplerate=SAMPLE_RATE, channels=1, dtype="float32")
        sd.wait()
        chunks.append(chunk.copy())
        rms = float(np.sqrt(np.mean(np.square(chunk))))
        if index >= min_chunks and rms < SILENCE_RMS:
            silence_count += 1
        else:
            silence_count = 0
        if index >= min_chunks and silence_count >= silence_need:
            break

    audio = np.concatenate(chunks, axis=0) if chunks else np.zeros((1, 1), dtype=np.float32)
    audio = audio.flatten()
    write_wav(path, audio)


def has_wakeword(text: str) -> tuple[bool, str]:
    if not WAKEWORD_ENABLED:
        return True, text
    lower = text.lower()
    for word in WAKEWORDS:
        if word and word in lower:
            cleaned = re.sub(re.escape(word), "", text, flags=re.I)
            cleaned = cleaned.strip(" \t\r\n,.;:!?，。！？、：；")
            return True, cleaned
    return False, text


def local_intent(text: str) -> dict:
    low = text.lower()
    obj = None
    if re.search(r"\u6c34\u676f|\u676f\u5b50|\u8336\u676f|cup", text, re.I):
        obj = "Coffee cup"
    elif re.search(r"\u74f6\u5b50|\u6c34\u74f6|\u996e\u6599\u74f6|bottle", text, re.I):
        obj = "Bottle"
    elif re.search(r"\u7b14|\u94a2\u7b14|\u5706\u73e0\u7b14|\u94c5\u7b14|pen|pencil", text, re.I):
        obj = "Pen"
    elif re.search(r"\u624b\u673a|phone", text, re.I):
        obj = "Mobile phone"
    elif re.search(r"\u4e66|book", text, re.I):
        obj = "Book"
    elif re.search(r"\u952e\u76d8|keyboard", text, re.I):
        obj = "Computer keyboard"
    elif re.search(r"\u9f20\u6807|mouse", text, re.I):
        obj = "Computer mouse"
    elif re.search(r"\u526a\u5200|scissors", text, re.I):
        obj = "Scissors"

    if re.search(r"\u505c\u6b62|\u6025\u505c|\u522b\u52a8|\u505c\u4e0b|stop", low):
        return {"action": "stop", "object": None, "reply": "stop"}
    if re.search(r"\u56de\u96f6|\u56de\u5bb6|\u590d\u4f4d|home", low):
        return {"action": "home", "object": None, "reply": "home"}
    if obj and re.search(r"\u62ff|\u53d6|\u6293|\u627e|\u7ed9\u6211|\u5e2e\u6211|pick|grab|fetch", low):
        return {"action": "pick", "object": obj, "reply": f"pick {obj}"}
    if obj:
        return {"action": "look", "object": obj, "reply": f"look {obj}"}
    return {"action": "chat", "object": None, "reply": ""}


def parse_intent(text: str) -> dict:
    try:
        intent = ask_cloud_intent(text)
        if isinstance(intent, dict) and intent.get("action"):
            return intent
    except Exception as exc:
        print("[intent] cloud failed:", exc)
    return local_intent(text)


def print_payload(payload: dict) -> None:
    print("arm_plan_request:", json.dumps(payload, ensure_ascii=False))
    if payload.get("cmd") == "pick":
        print(
            "plan:",
            f"object={payload.get('object')}",
            f"u={payload.get('u')}",
            f"v={payload.get('v')}",
            f"x={payload.get('x_base_mm')}mm",
            f"y={payload.get('y_base_mm')}mm",
            f"theta={payload.get('theta_deg')}deg",
            f"z_safe={payload.get('z_safe_mm')}mm",
            f"z_grab={payload.get('z_grab_mm')}mm",
        )


def reply_for(payload: dict, obj: str | None, user_text: str) -> str:
    cmd = payload.get("cmd")
    obj_name = OBJECT_CN.get(obj or "", obj or "target")
    if cmd == "pick":
        return f"好哒，我看到{obj_name}啦。坐标大概是 X {payload.get('x_base_mm')} 毫米，Y {payload.get('y_base_mm')} 毫米。"
    if cmd in {"target_lost", "not_found"}:
        return f"我还没有稳稳看到{obj_name}呢。你把它放到镜头中间一点点，我再试一次呀。"
    if cmd == "out_of_range":
        return f"我看见{obj_name}啦，不过它超出安全范围了呢。你稍微放近一点我再帮你。"
    if cmd == "camera_failed":
        return "摄像头好像有点忙，小U现在先看不清呢。"
    if cmd == "stop":
        return "收到呀，我先停下来。"
    if cmd == "home":
        return "收到呀，我先回到安全位置。"
    try:
        return cloud_chat(
            [
                {"role": "system", "content": xiaou_style_prompt(" 如果可以，就像一只软软的小猫助手那样回答，甜一点但别夸张。")},
                {"role": "user", "content": user_text},
            ]
        )
    except Exception:
        return "I heard you."


def handle_command(
    text: str,
    model: OpenCVDnnYolo,
    cap: cv2.VideoCapture | None,
    speak: bool,
) -> tuple[cv2.VideoCapture | None, str]:
    print("command_text:", text)
    emote = set_dialog_face(text)
    print("dialog_emote:", json.dumps(emote, ensure_ascii=False))

    set_face_state("thinking", "thinking")
    intent = parse_intent(text)
    print("intent:", json.dumps(intent, ensure_ascii=False))
    action = intent.get("action")
    obj = intent.get("object")

    if action in {"pick", "look"} and obj:
        set_face_state("searching", f"searching {OBJECT_CN.get(obj, obj)}")
        if cap is None or not cap.isOpened():
            print("Camera handle is not open. Reopening camera...")
            cap = open_camera()
        result = find_stable_target(model, obj, cap=cap)
        if not result.ok and result.reason == "camera_failed":
            print("Camera failed during target search. Retrying once...")
            if cap is not None:
                cap.release()
            cap = open_camera()
            result = find_stable_target(model, obj, cap=cap)
        payload = result.payload()
    elif action in {"stop", "home", "tidy"}:
        payload = {"cmd": action, "object": obj}
    else:
        payload = {"cmd": "chat", "object": obj}

    print_payload(payload)
    if payload.get("cmd") == "pick":
        set_face_state("happy", "target found")
    elif payload.get("cmd") in {"target_lost", "not_found", "out_of_range", "camera_failed"}:
        set_face_state("error", "target failed")
    elif payload.get("cmd") == "stop":
        set_face_state("stop", "stopped")

    print("planning only: ROS2/MoveIt six-axis request is not executed")

    reply = reply_for(payload, obj, text)
    print("XiaoU:", reply)
    if speak:
        set_face_state("speaking", "speaking")
        speak_text(reply)
    return cap, reply


def open_camera() -> cv2.VideoCapture | None:
    print("Opening camera...")
    cap = open_cv_camera()
    if cap is None:
        print("WARNING: camera open failed.")
        return None
    ok, _ = cap.read()
    print("Camera kept open:", bool(ok))
    return cap


def listen_for_wake_and_command(wake_path: Path, command_path: Path) -> str | None:
    set_face_state("idle", "say XiaoU")
    print(f"Wakeword mode: say one of {WAKEWORDS}")
    while True:
        set_face_state("listening", "listening")
        record_fixed(wake_path, WAKE_CHUNK_SECONDS)
        try:
            wake_text = baidu_asr_wav(wake_path).strip()
        except Exception as exc:
            print("wake ASR skip:", exc)
            time.sleep(IDLE_SLEEP_S)
            continue
        if not wake_text:
            continue
        print("wake_text:", wake_text)
        woke, rest = has_wakeword(wake_text)
        if not woke:
            continue
        set_face_state("happy", "wake")
        print("Wakeword detected.")
        if not WAKEWORD_REQUIRE_SECOND_STAGE and rest and local_intent(rest).get("action") != "chat":
            return rest
        set_face_state("listening", "command")
        record_until_silence(command_path)
        try:
            command_text = baidu_asr_wav(command_path).strip()
        except Exception as exc:
            print("command ASR failed:", exc)
            return None
        return command_text or None


def record_command_on_enter(command_path: Path) -> str | None:
    set_face_state("listening", "command")
    try:
        prompt = input("Press Enter to record a command, or q to quit: ").strip()
    except EOFError:
        return None
    if prompt.lower() == "q":
        return None
    print("Command recording started. Stop speaking to finish.")
    record_until_silence(command_path)
    try:
        command_text = baidu_asr_wav(command_path).strip()
    except Exception as exc:
        print("command ASR failed:", exc)
        return None
    return command_text or None


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--speak", action="store_true")
    parser.add_argument("--once", action="store_true")
    parser.add_argument("--text")
    parser.add_argument("--no-wakeword", action="store_true")
    parser.add_argument("--push-to-talk", action="store_true")
    args = parser.parse_args()

    print("XiaoU demo started.")
    print("Camera stays open. Wakeword starts command recording.")
    print("Loading YOLO...")
    configure_sounddevice()
    model = OpenCVDnnYolo()
    cap = open_camera()
    wake_path = PROJECT_DIR / "voice_wake.wav"
    command_path = PROJECT_DIR / "voice_command.wav"

    try:
        if args.text:
            cap, _reply = handle_command(args.text, model, cap, args.speak)
            return

        push_to_talk = args.push_to_talk or args.no_wakeword or not WAKEWORD_ENABLED
        while True:
            try:
                if push_to_talk:
                    command_text = record_command_on_enter(command_path)
                else:
                    command_text = listen_for_wake_and_command(wake_path, command_path)
                if command_text:
                    cap, _reply = handle_command(command_text, model, cap, args.speak)
                if args.once:
                    return
            except KeyboardInterrupt:
                set_face_state("idle", "standby")
                print("XiaoU demo stopped.")
                return
            except Exception as exc:
                set_face_state("error", "demo error")
                print("ERROR:", exc)
                time.sleep(1)
    finally:
        if cap is not None:
            cap.release()


if __name__ == "__main__":
    main()
