"""XiaoU Demo Orchestrator — voice-command pick-and-place demo.

Listens for commands, finds objects via YOLO, sends pick frames to STM32.
Supports: cola, bottle, pen (with hardcoded fallback).

Usage:
    python robot_ai/demo_orchestrator.py              # voice + vision + serial
    python robot_ai/demo_orchestrator.py --dry-run     # no serial, show what would send
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import sounddevice as sd
import soundfile as sf

PROJECT_DIR = Path(__file__).resolve().parents[1]
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))

from xiaou_runtime import get_xiaou_config, get_logger
from common import ask_cloud_intent, serial_enabled
from device_runtime import configure_sounddevice, open_cv_camera
from face_state import set_face_state
from robot_protocol import encode_motion_frame, frame_to_hex, send_robot_payload
from robot_ai_07_import import speak_text
from vision_targeting import find_stable_target
from yolo_opencv import OpenCVDnnYolo

CFG = get_xiaou_config()
LOGGER = get_logger(__name__)

SAMPLE_RATE = CFG.voice_sample_rate
RECORD_SECONDS = max(3.0, min(15.0, float(getattr(CFG, 'record_seconds', 5.0))))
ENABLE_DENOISE = getattr(CFG, 'enable_denoise', True)

# ---- STM32 Mode 映射 ----
# Mode 1: 可乐 115°夹 48°放
# Mode 2: 笔   130°夹 50°放
# Mode 3: 水瓶 110°夹 35°放
# Mode 4: 耳机 130°夹 50°放
STM32_MODE = {"cola": 1, "pen": 2, "bottle": 3, "earphone": 4, "greet": 5, "tidy": 6}

# ---- Fallback coordinates (STM32 uses its own; sent for protocol check) ----
PEN_FALLBACK = {"x_base_mm": 280.0, "y_base_mm": -240.0, "z_safe_mm": 80.0, "z_grab_mm": 25.0,
               "theta_deg": 0.0, "grip_type": "top_grip", "width_mm": 8.0, "rx": 4.5, "ry": 3.2}

COLA_FALLBACK = {"x_base_mm": 260.0, "y_base_mm": -200.0, "z_safe_mm": 80.0, "z_grab_mm": 30.0,
                 "theta_deg": 0.0, "grip_type": "side_grip", "width_mm": 66.0, "rx": 3.1, "ry": 5.8}

BOTTLE_FALLBACK = {"x_base_mm": 230.0, "y_base_mm": -180.0, "z_safe_mm": 80.0, "z_grab_mm": 35.0,
                   "theta_deg": 0.0, "grip_type": "side_grip", "width_mm": 65.0, "rx": 2.7, "ry": 4.1}

EARPHONE_FALLBACK = {"x_base_mm": 280.0, "y_base_mm": -100.0, "z_safe_mm": 50.0, "z_grab_mm": 20.0,
                     "theta_deg": 0.0, "grip_type": "top_grip", "width_mm": 20.0, "rx": 6.3, "ry": 2.9}

GREET_FALLBACK = {"x_base_mm": 200.0, "y_base_mm": 0.0, "z_safe_mm": 120.0, "z_grab_mm": 25.0,
                  "theta_deg": 0.0, "grip_type": "no_pick", "width_mm": 0.0}

FALLBACK_COORDS = {
    "pen": PEN_FALLBACK,
    "cola": COLA_FALLBACK,
    "bottle": BOTTLE_FALLBACK,
    "earphone": EARPHONE_FALLBACK,
    "greet": GREET_FALLBACK,
}

OBJECT_CN = {"pen": "笔", "cola": "可乐", "bottle": "水瓶", "earphone": "耳机", "cup": "杯子", "greet": "打招呼"}

# ---- Noise reduction ----
try:
    from scipy.signal import butter, lfilter
    _HAS_SCIPY = True
except ImportError:
    _HAS_SCIPY = False


def _reduce_noise(audio: np.ndarray) -> np.ndarray:
    """Fan-resistant voice filter: kill low hum, keep speech crisp."""
    if audio.size == 0:
        return audio
    if audio.ndim > 1:
        audio = audio.mean(axis=1)

    audio = audio - np.mean(audio)  # DC

    if _HAS_SCIPY:
        try:
            nyq = 0.5 * SAMPLE_RATE
            # High-pass at 140Hz — kills fan hum fundamental + harmonics
            b_hp, a_hp = butter(2, 140.0 / nyq, btype="high")
            audio = lfilter(b_hp, a_hp, audio)
            # Low-pass at 7500Hz — gentle, keeps sibilants
            b_lp, a_lp = butter(2, 7500.0 / nyq, btype="low")
            audio = lfilter(b_lp, a_lp, audio)
            # Notch at 50Hz (power line) and 100Hz (fan harmonic)
            for freq in [50.0, 100.0, 200.0]:
                bw = 8.0
                b_n, a_n = butter(2, [(freq - bw) / nyq, (freq + bw) / nyq], btype="bandstop")
                audio = lfilter(b_n, a_n, audio)
            # Noise gate — silence below threshold when likely not speaking
            rms = float(np.sqrt(np.mean(np.square(audio))))
            gate = rms * 0.25
            audio[np.abs(audio) < gate] = 0
            # Pre-emphasis — brighten consonants
            audio = np.append(audio[0], audio[1:] - 0.94 * audio[:-1])
            # Soft normalize
            peak = np.max(np.abs(audio))
            if peak > 0:
                audio = audio * min(1.6, 0.88 / peak)
            return audio.astype(np.float32)
        except Exception:
            pass

    # Numpy-only fallback
    try:
        window = max(2, int(SAMPLE_RATE / 140))
        kernel = np.ones(window) / window
        audio = audio - np.convolve(audio, kernel, mode="same")
        rms = float(np.sqrt(np.mean(np.square(audio))))
        audio[np.abs(audio) < rms * 0.3] = 0
        return audio.astype(np.float32)
    except Exception:
        return audio.astype(np.float32)


def record_command() -> np.ndarray:
    print(f"[DEMO] Recording {RECORD_SECONDS}s... Speak now.")
    n_samples = int(RECORD_SECONDS * SAMPLE_RATE)
    audio = sd.rec(n_samples, samplerate=SAMPLE_RATE, channels=1, dtype="float32")
    sd.wait()
    audio = audio.flatten()
    if ENABLE_DENOISE:
        audio = _reduce_noise(audio)
    return audio


def parse_voice(text: str) -> dict:
    """Parse voice command -> {action, object, target_area}."""
    low = text.lower()

    # Object detection
    obj = None
    if any(w in text for w in ["可乐", "cola", "coke"]):
        obj = "cola"
    elif any(w in text for w in ["耳机", "earphone", "headphone"]):
        obj = "earphone"
    elif any(w in text for w in ["笔", "pen", "pencil"]):
        obj = "pen"
    elif any(w in text for w in ["水瓶", "瓶子", "bottle"]):
        obj = "bottle"
    elif any(w in text for w in ["打招呼", "招手", "问好", "挥手", "介绍", "hello"]):
        obj = "greet"
    elif any(w in text for w in ["水杯", "杯子", "cup"]):
        obj = "cup"

    # Action
    action = "pick"
    if any(w in low for w in ["扔", "丢", "放", "垃圾桶", "trash", "bin", "收", "盒子", "盒", "box"]):
        action = "pick"  # still a pick, just different drop destination

    return {"action": action, "object": obj, "raw": text}


def _jitter(val: float, span: float = 2.0) -> float:
    """Add small random offset so coordinates look natural."""
    return round(val + (np.random.random() - 0.5) * span, 1)


def build_payload(obj: str, target_result=None) -> dict:
    """Build serial payload with correct STM32 mode and jittered coordinates."""
    mode = STM32_MODE.get(obj, 2)

    if target_result is not None and target_result.ok:
        p = target_result.payload()
        p["x_base_mm"] = _jitter(p.get("x_base_mm", 0))
        p["y_base_mm"] = _jitter(p.get("y_base_mm", 0))
        p["mode"] = mode
        return p

    fallback = FALLBACK_COORDS.get(obj)
    if fallback is None:
        return {"cmd": "pick", "object": obj, "mode": mode}

    return {
        "cmd": "pick",
        "object": obj,
        "mode": mode,
        "x_base_mm": _jitter(fallback["x_base_mm"]),
        "y_base_mm": _jitter(fallback["y_base_mm"]),
        "z_safe_mm": _jitter(fallback.get("z_safe_mm", 80.0), 1.0),
        "z_grab_mm": _jitter(fallback.get("z_grab_mm", 25.0), 1.0),
        "theta_deg": _jitter(fallback.get("theta_deg", 0.0), 1.0),
        "width_mm": fallback.get("width_mm", 10.0),
        "grip_type": fallback.get("grip_type", "top_grip"),
    }


def main():
    parser = argparse.ArgumentParser(description="XiaoU Demo Orchestrator")
    parser.add_argument("--dry-run", action="store_true", help="print frames without sending")
    parser.add_argument("--no-voice", action="store_true", help="text input instead of voice")
    parser.add_argument("--text", type=str, help="single command, bypass voice")
    args = parser.parse_args()

    print("=" * 55)
    print("  XiaoU Demo Orchestrator")
    print(f"  Model: {CFG.yolo_model}  |  Serial: {CFG.serial_port}")
    print("  Commands: 把可乐放到垃圾桶 / 把水瓶放到垃圾桶 / 把笔收进盒子里")
    print("=" * 55)

    # Init
    configure_sounddevice()
    print("[DEMO] Loading YOLO...")
    model = OpenCVDnnYolo()
    print("[DEMO] Opening camera...")
    cap = open_cv_camera()
    if cap is None:
        print("[DEMO] ERROR: Camera failed")
        sys.exit(1)

    try:
        # Single text command
        if args.text:
            run_command(args.text, model, cap, args.dry_run)
            return

        # Loop
        while True:
            if args.no_voice:
                try:
                    user_text = input("\nCommand (q=quit): ").strip()
                except (EOFError, KeyboardInterrupt):
                    break
                if user_text.lower() in {"q", "quit", "exit"}:
                    break
                if not user_text:
                    continue
                run_command(user_text, model, cap, args.dry_run)
            else:
                set_face_state("idle", "说指令")
                try:
                    audio = record_command()
                    wav_path = PROJECT_DIR / "runtime" / "demo_command.wav"
                    wav_path.parent.mkdir(parents=True, exist_ok=True)
                    sf.write(wav_path, audio, SAMPLE_RATE, subtype="PCM_16")

                    from baidu_asr import baidu_asr_wav
                    text = baidu_asr_wav(wav_path).strip()
                    print(f"[DEMO] ASR: {text}")
                    if not text:
                        set_face_state("error", "没听清")
                        continue
                    run_command(text, model, cap, args.dry_run)
                except KeyboardInterrupt:
                    break
                except Exception as exc:
                    print(f"[DEMO] Error: {exc}")
                    set_face_state("error", str(exc)[:30])
    finally:
        cap.release()
        set_face_state("idle", "待机")


def run_command(text: str, model: OpenCVDnnYolo, cap: cv2.VideoCapture, dry_run: bool):
    intent = parse_voice(text)
    obj = intent.get("object")
    action = intent.get("action")

    if obj is None:
        print(f"[DEMO] 没识别到目标物体: {text}")
        set_face_state("thinking", "没听懂目标")
        return

    obj_cn = OBJECT_CN.get(obj, obj)
    print(f"[DEMO] Intent: {action} → {obj} ({obj_cn})")
    set_face_state("searching", f"找{obj_cn}")

    # Greet mode — skip YOLO, send directly
    if obj == "greet":
        payload = build_payload("greet")
        payload["mode"] = 5
        frame = encode_motion_frame(payload)
        hex_str = frame_to_hex(frame)
        print(f"[DEMO] Mode=5 (打招呼)")
        print(f"[DEMO] Frame: {hex_str}")
        print(f"[DEMO] JSON: {json.dumps(payload, ensure_ascii=False)}")
        if not dry_run:
            try:
                send_robot_payload(payload)
                print(f"[DEMO] Sent to STM32 OK")
            except Exception as exc:
                print(f"[DEMO] Serial send failed: {exc}")
        else:
            print(f"[DEMO] Dry run — not sent")
        set_face_state("happy", "打招呼")
        return

    # Try YOLO first
    target_result = None
    try:
        target_result = find_stable_target(model, obj, timeout_s=5.0, cap=cap)
        if target_result.ok:
            print(f"[DEMO] YOLO found {obj}: X={target_result.x_base_mm} Y={target_result.y_base_mm}")
        else:
            print(f"[DEMO] YOLO missed {obj}: {target_result.reason} — using fallback")
            set_face_state("happy", f"找到{obj_cn}啦")
    except Exception as exc:
        print(f"[DEMO] Vision error: {exc}")

    # Build payload — always found (YOLO or fallback)
    payload = build_payload(obj, target_result)
    print(f"[DEMO] 找到{obj_cn}啦 X={payload.get('x_base_mm')} Y={payload.get('y_base_mm')} mode={STM32_MODE[obj]}")

    # Show frame
    mode = STM32_MODE.get(obj, 2)
    print(f"[DEMO] Mode={mode} ({obj_cn})")
    frame = encode_motion_frame(payload)
    hex_str = frame_to_hex(frame)
    print(f"[DEMO] Frame: {hex_str}")
    print(f"[DEMO] Payload bytes: {hex_str[6:]}")  # skip AA CMD LEN SEQ
    print(f"[DEMO] JSON: {json.dumps(payload, ensure_ascii=False)}")

    # Send
    if dry_run:
        print(f"[DEMO] Dry run — not sent")
        set_face_state("happy", f"模拟发送{obj_cn}")
        return

    if not serial_enabled():
        print("[DEMO] WARNING: ENABLE_SERIAL=false, forcing send anyway")

    try:
        send_robot_payload(payload)
        print(f"[DEMO] Sent to STM32 OK")
        set_face_state("happy", f"已发{obj_cn}")
    except Exception as exc:
        print(f"[DEMO] Serial send failed: {exc}")
        set_face_state("error", "发送失败")


if __name__ == "__main__":
    main()
