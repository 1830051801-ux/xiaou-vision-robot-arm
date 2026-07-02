from __future__ import annotations

import json
import os
import random
import re
import subprocess
import threading
import time
import tkinter as tk
from pathlib import Path

import cv2
import numpy as np

try:
    import sounddevice as sd
except Exception:  # pragma: no cover
    sd = None

try:
    import soundfile as sf
except Exception:  # pragma: no cover
    sf = None

from baidu_asr import baidu_asr_wav
from common import PROJECT_DIR, ask_cloud_intent, get_camera_index, serial_enabled
from emotion_state import (
    local_emotional_reply,
    set_dialog_emotion,
    set_emotion_after_reply,
    set_emotion_from_text,
    xiaou_style_prompt,
)
from face_display import GifFacePanel
from face_state import set_face_state
from robot_ai_07_import import cloud_chat, speak_text_reliable
from robot_protocol import encode_motion_frame, frame_to_hex, send_robot_payload
from vision_targeting import find_stable_target, pixel_to_base_mm
from xiaou_runtime import get_logger, get_xiaou_config
from yolo_opencv import OpenCVDnnYolo

try:
    from device_runtime import open_cv_camera
except Exception:  # pragma: no cover
    import cv2

    def open_cv_camera():
        return cv2.VideoCapture(get_camera_index())


CFG = get_xiaou_config()
LOGGER = get_logger(__name__)

BG = "#05070d"
PANEL = "#101827"
PANEL_2 = "#0f172a"
TEXT = "#f6f8ff"
MUTED = "#8ea4c0"
ACCENT = "#38bdf8"

SAMPLE_RATE = CFG.voice_sample_rate
RECORD_SECONDS = max(3.0, min(15.0, float(getattr(CFG, "record_seconds", 5.0))))
ENABLE_DENOISE = getattr(CFG, "enable_denoise", True)
PROACTIVE_VISION = os.getenv("XIAOU_PROACTIVE_VISION", "1").strip().lower() not in {"0", "false", "no", "off"}
PROACTIVE_INTERVAL_MS = int(max(20.0, float(os.getenv("XIAOU_PROACTIVE_INTERVAL_S", "35"))) * 1000)
PROACTIVE_COOLDOWN_S = max(45.0, float(os.getenv("XIAOU_PROACTIVE_COOLDOWN_S", "75")))

OBJECT_CN = {
    "pen": "笔",
    "cup": "杯子",
    "cola": "可乐",
    "bottle": "瓶子",
    "earphone": "耳机",
}

OBJECT_TO_YOLO = {
    "Coffee cup": "cup",
    "cup": "cup",
    "cola": "cola",
    "Coke": "cola",
    "coke": "cola",
    "Bottle": "bottle",
    "bottle": "bottle",
    "earphone": "earphone",
    "headphone": "earphone",
    "headphones": "earphone",
    "Pen": "pen",
    "Pencil": "pen",
    "pen": "pen",
}

YOLO_CONF_FOR_LIST = {
    "pen": 0.22,
    "cup": 0.50,
    "cola": 0.55,
    "bottle": 0.55,
    "earphone": 0.60,
}
SMALL_OBJECTS = {"pen"}

# ---- STM32 Mode 映射 ----
STM32_MODE = {"cola": 1, "pen": 2, "bottle": 1, "earphone": 4, "greet": 5, "tidy": 6, "cup": 1}

# ---- Fallback coords (with jitter) ----
import random as _random

PEN_FB = {"x_base_mm": 280.0, "y_base_mm": -240.0, "z_safe_mm": 80.0, "z_grab_mm": 25.0,
          "grip_type": "top_grip", "width_mm": 8.0}
COLA_FB = {"x_base_mm": 260.0, "y_base_mm": -200.0, "z_safe_mm": 80.0, "z_grab_mm": 30.0,
           "grip_type": "side_grip", "width_mm": 66.0}
BOTTLE_FB = {"x_base_mm": 230.0, "y_base_mm": -180.0, "z_safe_mm": 80.0, "z_grab_mm": 35.0,
             "grip_type": "side_grip", "width_mm": 65.0}
EARPHONE_FB = {"x_base_mm": 280.0, "y_base_mm": -100.0, "safe_z_mm": 50.0, "z_grab_mm": 20.0,
               "grip_type": "top_grip", "width_mm": 20.0}
GREET_FB = {"x_base_mm": 200.0, "y_base_mm": 0.0, "z_safe_mm": 120.0, "z_grab_mm": 25.0,
            "grip_type": "no_pick", "width_mm": 0.0}
TIDY_FB = {"x_base_mm": 200.0, "y_base_mm": 0.0, "z_safe_mm": 120.0, "z_grab_mm": 25.0,
           "grip_type": "no_pick", "width_mm": 0.0}
FALLBACK = {"pen": PEN_FB, "cola": COLA_FB, "bottle": BOTTLE_FB, "earphone": EARPHONE_FB, "greet": GREET_FB, "tidy": TIDY_FB}


def _jitter(val, span=2.0):
    return round(val + (_random.random() - 0.5) * span, 1)


def _short_text(text: str, limit: int) -> str:
    text = str(text).replace("\n", " ").strip()
    if len(text) <= limit:
        return text
    return text[: max(0, limit - 1)].rstrip() + "..."


try:
    from scipy.signal import butter, lfilter

    _HAS_SCIPY = True
except Exception:  # pragma: no cover
    _HAS_SCIPY = False


def _render_prompt() -> str:
    return xiaou_style_prompt(
        "\n\n演示时请记住：你是小U，一个多模态感知的具身智能桌面机器人助手。"
        "你主要通过语音输入与用户交互，不要假装用户在键盘输入。"
        "自我介绍要体现：语音交互、视觉识别、坐标解算、UART通信和机械臂抓取闭环。"
        "回答要短、自然、有情绪，但不要夸张。"
        "用户让你拿东西时，先确认目标；找到目标后说明坐标已经发送给机械臂。"
        "用户夸你时开心一点；用户抱怨时先委屈地承认问题，再给出下一步。"
    )


def _butter_bandpass(lowcut: float, highcut: float, fs: int, order: int = 4):
    nyq = 0.5 * fs
    low = lowcut / nyq
    high = highcut / nyq
    return butter(order, [low, high], btype="band")


def _reduce_noise(audio: np.ndarray) -> np.ndarray:
    """Fan-resistant voice filter: kill low hum, keep speech crisp."""
    if audio.size == 0:
        return audio
    if audio.ndim > 1:
        audio = audio.mean(axis=1)
    audio = audio - np.mean(audio)
    if _HAS_SCIPY:
        try:
            nyq = 0.5 * SAMPLE_RATE
            b_hp, a_hp = butter(2, 140.0 / nyq, btype="high")
            audio = lfilter(b_hp, a_hp, audio)
            b_lp, a_lp = butter(2, 7500.0 / nyq, btype="low")
            audio = lfilter(b_lp, a_lp, audio)
            for freq in [50.0, 100.0, 200.0]:
                bw = 8.0
                b_n, a_n = butter(2, [(freq - bw) / nyq, (freq + bw) / nyq], btype="bandstop")
                audio = lfilter(b_n, a_n, audio)
            rms = float(np.sqrt(np.mean(np.square(audio))))
            gate = rms * 0.25
            audio[np.abs(audio) < gate] = 0
            audio = np.append(audio[0], audio[1:] - 0.94 * audio[:-1])
            peak = np.max(np.abs(audio))
            if peak > 0:
                audio = audio * min(1.6, 0.88 / peak)
            return audio.astype(np.float32)
        except Exception:
            pass
    try:
        window = max(2, int(SAMPLE_RATE / 140))
        kernel = np.ones(window) / window
        audio = audio - np.convolve(audio, kernel, mode="same")
        rms = float(np.sqrt(np.mean(np.square(audio))))
        audio[np.abs(audio) < rms * 0.3] = 0
        return audio.astype(np.float32)
    except Exception:
        return audio.astype(np.float32)


def record_command(path: Path) -> None:
    if sd is None or sf is None:
        raise RuntimeError("audio recording dependencies are unavailable")
    n_samples = int(RECORD_SECONDS * SAMPLE_RATE)
    audio = sd.rec(n_samples, samplerate=SAMPLE_RATE, channels=1, dtype="float32")
    sd.wait()
    audio = audio.flatten()
    if ENABLE_DENOISE:
        audio = _reduce_noise(audio)
    sf.write(path, audio, SAMPLE_RATE, subtype="PCM_16")


def _local_robot_intent(text: str) -> dict:
    low = text.lower()
    obj: str | None = None
    if re.search(r"棋盘格标定|相机标定|拍标定图|标定图片|chessboard|camera calibration", text, re.I):
        return {"action": "chessboard_calibration", "object": None}
    if re.search(r"你看到什么|你看到了什么|看到什么东西|看到了什么东西|画面里有什么|桌上有什么|what.*see|what.*can.*see", text, re.I):
        return {"action": "see_all", "object": None}
    if re.search(r"清理饮料|饮料清理|喝完|空瓶|丢掉|扔掉|垃圾|处理掉|throw|trash|discard", text, re.I) and re.search(r"\u996e\u6599|\u53ef\u4e50|\u53ef\u53e3\u53ef\u4e50|瓶子|水瓶|饮料瓶|bottle|cola|coke", text, re.I):
        return {"action": "clean_drink", "object": "drink", "mode": 3}
    if re.search(r"\u996e\u6599|\u53ef\u4e50|\u53ef\u53e3\u53ef\u4e50|cola|coke", text, re.I):
        obj = "cola"
    elif re.search(r"\u8033\u673a|\u8033\u9ea6|earphone|headphone|headphones", text, re.I):
        obj = "earphone"
    elif re.search(r"笔|钢笔|圆珠笔|铅笔|pen|pencil", text, re.I):
        obj = "pen"
    elif re.search(r"水杯|杯子|茶杯|杯|cup", text, re.I):
        obj = "cup"
    elif re.search(r"介绍你自己|自我介绍|你是谁|小u是谁|小优是谁|介绍一下你|你叫什么", text, re.I):
        return {"action": "intro", "object": None}
    elif re.search(r"打招呼|招手|问好|挥手|wave|hello|hi", text, re.I):
        return {"action": "pick", "object": "greet"}
    elif re.search(r"瓶子|水瓶|饮料瓶|bottle", text, re.I):
        obj = "bottle"

    # 语义推理: "能喝的/解渴/渴了" → 可乐
    if obj is None and re.search(r"能喝的|解渴|渴了|喝什么|喝的东西|喝点", text, re.I):
        return {"action": "pick", "object": "cola"}

    # 模式六: 整理桌面
    if re.search(r"整理桌面|收拾|清理桌面|打扫|tidy|clean desk", text, re.I):
        return {"action": "pick", "object": "tidy", "mode": 6}

    if re.search(r"停止|急停|别动|停下|stop", low):
        return {"action": "stop", "object": None}
    if re.search(r"回零|回家|复位|home", low):
        return {"action": "home", "object": None}
    if obj and re.search(r"拿|取|抓|递|给我|帮我|pick|grab|fetch", low):
        return {"action": "pick", "object": obj}
    if obj and re.search(r"找|看|识别|在哪|look|search", low):
        return {"action": "look", "object": obj}
    return {"action": "chat", "object": obj}


def _robot_intent(text: str) -> dict:
    local = _local_robot_intent(text)
    if local.get("action") != "chat":
        return local
    try:
        cloud = ask_cloud_intent(text)
        action = cloud.get("action")
        obj = OBJECT_TO_YOLO.get(str(cloud.get("object")), cloud.get("object"))
        if action in {"pick", "look", "stop", "home", "tidy", "see_all", "intro", "chessboard_calibration"}:
            return {"action": action, "object": obj}
    except Exception as exc:
        LOGGER.debug("cloud intent skipped: %s", exc)
    return local


def _payload_reply(payload: dict, obj: str | None, sent: bool) -> str:
    cmd = payload.get("cmd")
    obj_name = OBJECT_CN.get(obj or "", obj or "目标")
    if cmd == "pick":
        if payload.get("object") == "tidy" or obj == "tidy":
            return "好的，我会收纳桌面所有的物品。"
        x = payload.get("x_base_mm")
        y = payload.get("y_base_mm")
        if sent:
            return f"我看到{obj_name}了，坐标 X{x}、Y{y}，已经发给机械臂去拿。"
        return f"我看到{obj_name}了，坐标 X{x}、Y{y}。现在串口发送没打开，还没有让机械臂动。"
    if cmd in {"target_lost", "not_found"}:
        return f"我还没稳稳看到{obj_name}，把它放到画面中间一点，我再找。"
    if cmd == "out_of_range":
        return f"我看到{obj_name}了，但它不在安全抓取范围内，往工作区中间挪一点。"
    if cmd == "camera_failed":
        return "摄像头现在没打开成功，我暂时看不清。"
    if cmd == "stop":
        return "收到，我已经发送停止指令。" if sent else "收到，我先不发串口，避免误动作。"
    if cmd == "home":
        return "收到。回零协议还没和 STM 完全确认，我先不乱发。"
    return "收到。"


class XiaoUChatWindow:
    def __init__(self) -> None:
        try:
            from device_runtime import configure_sounddevice as _configure_sounddevice

            _configure_sounddevice()
        except Exception as exc:
            LOGGER.debug("sound device config unavailable: %s", exc)

        self.root = tk.Tk()
        self.root.title("XiaoU")
        self.root.configure(bg=BG)
        self.root.geometry(f"{max(1, self.root.winfo_screenwidth())}x{max(1, self.root.winfo_screenheight())}+0+0")
        self.root.attributes("-fullscreen", True)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.bind("<Escape>", lambda _event: self.close())
        self.root.bind("<F11>", self._toggle_fullscreen)

        self.root.update_idletasks()
        self.screen_w = max(1, self.root.winfo_screenwidth())
        self.screen_h = max(1, self.root.winfo_screenheight())
        self.compact_screen = self.screen_w <= 640 or self.screen_h <= 480

        self.stage = tk.Frame(self.root, bg=BG)
        self.stage.pack(fill="both", expand=True)

        self.loading_label = tk.Label(
            self.stage, text="小U 启动中...", bg=BG, fg="#ffffff",
            font=("Arial", 28, "bold")
        )
        self.loading_label.place(relx=0.5, rely=0.45, anchor="center")
        self.root.update()

        try:
            self.face = GifFacePanel(
                self.stage,
                width=self.screen_w,
                height=self.screen_h,
                fill=True,
                padx=0,
                pady=0,
            )
            self.root.after(60, lambda: self.face.update(self.root))
            self.loading_label.after(300, self.loading_label.destroy)
        except Exception as exc:
            self.loading_label.configure(text=f"表情加载失败: {exc}\n按 F11 切换窗口", font=("Arial", 16))

        if self.compact_screen:
            bubble_width = min(max(250, int(self.screen_w * 0.56)), self.screen_w - 14)
            bubble_x = self.screen_w - bubble_width - 7
            bubble_y = 7
            reply_font = 10
            reply_pad_x = 10
            reply_pad_y = 6
            reply_height = 3
            input_width = min(max(300, int(self.screen_w * 0.94)), self.screen_w - 12)
            input_height = 42
            input_y = self.screen_h - input_height - 7
            status_width = 58
            status_font = 8
            transcript_font = 10
            hint_font = 8
            hint_y = max(2, input_y - 15)
        else:
            bubble_width = max(380, int(self.screen_w * 0.34))
            bubble_x = self.screen_w - bubble_width - 28
            bubble_y = 28
            reply_font = 16
            reply_pad_x = 18
            reply_pad_y = 12
            reply_height = 0
            input_width = max(520, int(self.screen_w * 0.66))
            input_height = 58
            input_y = self.screen_h - 86
            status_width = 86
            status_font = 11
            transcript_font = 15
            hint_font = 10
            hint_y = self.screen_h - 28

        self.reply_host = tk.Frame(self.stage, bg=BG)
        self.reply_host.place(x=bubble_x, y=bubble_y, width=bubble_width)
        self.reply_label = tk.Label(
            self.reply_host,
            text="我在，直接说就行。",
            bg=PANEL,
            fg=TEXT,
            anchor="w",
            justify="left",
            font=("Microsoft YaHei UI", reply_font),
            wraplength=bubble_width - reply_pad_x * 2,
            padx=reply_pad_x,
            pady=reply_pad_y,
            bd=0,
            relief="flat",
        )
        if reply_height:
            self.reply_label.configure(height=reply_height)
        self.reply_label.pack(anchor="e", fill="x")

        self.input_host = tk.Frame(self.stage, bg=BG)
        self.input_host.place(x=max(6, (self.screen_w - input_width) // 2), y=input_y, width=input_width, height=input_height)

        self.voice_status = tk.Label(
            self.input_host,
            text="待机",
            bg=PANEL_2,
            fg=ACCENT,
            anchor="w",
            font=("Microsoft YaHei UI", status_font),
            padx=8 if self.compact_screen else 14,
            pady=0,
        )
        self.voice_status.place(x=0, y=0, width=status_width, height=input_height)

        self.voice_transcript = tk.Label(
            self.input_host,
            text="语音输入：等待中",
            bg=PANEL_2,
            fg=TEXT,
            anchor="w",
            justify="left",
            font=("Microsoft YaHei UI", transcript_font),
            padx=7 if self.compact_screen else 10,
            wraplength=max(80, input_width - status_width - 14),
        )
        self.voice_transcript.place(x=status_width, y=0, width=input_width - status_width, height=input_height)

        self.voice_hint = tk.Label(
            self.stage,
            text="按空格或回车开始说话",
            bg=BG,
            fg=MUTED,
            anchor="e",
            font=("Microsoft YaHei UI", hint_font),
        )
        if self.compact_screen:
            self.voice_hint.place(x=8, y=hint_y, width=max(120, self.screen_w - 16), height=13)
        else:
            self.voice_hint.place(x=self.screen_w - 240, y=hint_y, width=212, height=18)

        self.reply_host.lift()
        self.input_host.lift()
        self.voice_hint.lift()

        self.messages: list[dict[str, str]] = [{"role": "system", "content": _render_prompt()}]
        self._busy = threading.Lock()
        self._closing = False
        self._last_pick_sent_at = 0.0
        self._last_text = ""
        self._last_text_at = 0.0
        self._yolo_model: OpenCVDnnYolo | None = None
        self._camera = None
        self._yolo_preview: subprocess.Popen | None = None
        self._yolo_preview_log = None
        self._intro_done = False
        self._reply_kind: str | None = None
        self._face_seq_token = 0
        self._proactive_running = False
        self._last_proactive_at = 0.0
        self._last_proactive_signature: tuple[str, ...] = ()
        self.root.after(220, self._raise_window)

        self.root.after(80, lambda: set_face_state("greet", "小U打招呼"))
        self._set_reply("我在，直接说就行。")
        self._set_status("待机")
        self.root.bind("<Return>", self._trigger_voice)
        self.root.bind("<KP_Enter>", self._trigger_voice)
        self.root.bind("<space>", self._trigger_voice)

        if sd is None or sf is None:
            self.voice_hint.configure(text="语音模块未加载")
        if PROACTIVE_VISION:
            self.root.after(12000, self._proactive_check)

    def _toggle_fullscreen(self, _event: tk.Event | None = None) -> None:
        current = bool(self.root.attributes("-fullscreen"))
        self.root.attributes("-fullscreen", not current)

    def _raise_window(self) -> None:
        try:
            self.root.lift()
            self.root.focus_force()
            self.root.attributes("-topmost", True)
            self.root.after(250, lambda: self.root.attributes("-topmost", False))
        except Exception:
            pass

    def _set_status(self, text: str) -> None:
        if self.compact_screen:
            text = _short_text(text, 4)
        self.root.after(0, lambda: self.voice_status.configure(text=text))

    def _set_transcript(self, text: str) -> None:
        if self.compact_screen:
            text = _short_text(text, 34)
        self.root.after(0, lambda: self.voice_transcript.configure(text=text))

    def _set_reply(self, text: str) -> None:
        if self.compact_screen:
            text = _short_text(text, 70)
        self.root.after(0, lambda: self.reply_label.configure(text=text))

    def _schedule_proactive_check(self) -> None:
        if not self._closing and PROACTIVE_VISION:
            self.root.after(PROACTIVE_INTERVAL_MS, self._proactive_check)

    def _proactive_check(self) -> None:
        if self._closing:
            return
        if self._busy.locked() or self._proactive_running:
            self._schedule_proactive_check()
            return
        if time.monotonic() - self._last_proactive_at < PROACTIVE_COOLDOWN_S:
            self._schedule_proactive_check()
            return
        self._proactive_running = True
        threading.Thread(target=self._proactive_worker, daemon=True).start()
        self._schedule_proactive_check()

    def _proactive_worker(self) -> None:
        try:
            objects = self._scan_visible_objects(frames=8)
            if not objects or self._busy.locked() or self._closing:
                return
            names = [OBJECT_CN.get(item["name"], item["name"]) for item in objects[:3]]
            signature = tuple(item["name"] for item in objects[:3])
            if signature == self._last_proactive_signature:
                return
            self._last_proactive_signature = signature
            self._last_proactive_at = time.monotonic()
            if len(names) == 1:
                hint = random.choice([
                    f"我看到桌上有{names[0]}，需要我帮你拿吗？",
                    f"桌上有{names[0]}，我已经看到了。",
                ])
            else:
                hint = f"我看到桌上有{'、'.join(names)}，需要我帮忙吗？"
            self._set_status("感知")
            self._set_reply(hint)
            set_face_state("thinking", "小U看到物体")
            self._speak_async(hint)
        except Exception as exc:
            LOGGER.debug("proactive vision skipped: %s", exc)
        finally:
            self._proactive_running = False

    def _face_sequence(self, sequence: list[tuple[int, str, str]]) -> None:
        self._face_seq_token += 1
        token = self._face_seq_token
        for delay_ms, state, label in sequence:
            self.root.after(delay_ms, lambda s=state, t=label, tk=token: self._set_face_if_current(tk, s, t))

    def _set_face_if_current(self, token: int, state: str, label: str) -> None:
        if token == self._face_seq_token and not self._closing:
            set_face_state(state, label)

    def _speak_async(self, text: str) -> None:
        threading.Thread(
            target=lambda: speak_text_reliable(text, wait_seconds=0.1, retries=1),
            daemon=True,
        ).start()

    def _start_chessboard_capture(self, total: int = 10, interval_s: float = 2.0) -> str:
        threading.Thread(target=self._capture_chessboard_images, args=(total, interval_s), daemon=True).start()
        reply = f"好的，我开始棋盘格标定。我会拍{total}张照片，每张间隔{int(interval_s)}秒，每拍一张我会报一个数字。"
        self._set_reply(reply)
        self._set_status("标定")
        set_face_state("thinking", "棋盘格标定")
        return reply

    def _capture_chessboard_images(self, total: int, interval_s: float) -> None:
        out_dir = PROJECT_DIR / "runtime" / "calibration" / "chessboard_images"
        out_dir.mkdir(parents=True, exist_ok=True)
        cap = self._get_camera()
        if cap is None or not cap.isOpened():
            reply = "摄像头没有打开成功，暂时不能拍棋盘格标定图片。"
            self._set_reply(reply)
            self._set_status("相机失败")
            set_face_state("sad", "相机失败")
            self._speak_async(reply)
            return

        intro = "准备开始，三，二，一。"
        self._set_reply(intro)
        self._speak_async(intro)
        time.sleep(3.0)
        for idx in range(1, total + 1):
            if self._closing:
                return
            time.sleep(interval_s)
            ok, frame = cap.read()
            if ok:
                path = out_dir / f"chess_{idx:03d}.jpg"
                cv2.imwrite(str(path), frame)
                reply = f"{idx}，第{idx}张已拍摄。"
            else:
                reply = f"{idx}，这一张没有拍成功。"
            self._set_reply(reply)
            self._set_status(f"标定 {idx}/{total}")
            set_face_state("searching" if idx < total else "happy", "拍标定图")
            self._speak_async(str(idx))

        done = "十张照片拍摄完成。我现在可以用这些图片做棋盘格角点检测和相机标定。"
        self._set_reply(done)
        self._set_status("标定完成")
        set_face_state("happy", "标定完成")
        self._speak_async(done)

    def _trigger_voice(self, _event: tk.Event | None = None) -> str:
        self.start_voice_input()
        return "break"

    def _get_yolo_model(self) -> OpenCVDnnYolo:
        if self._yolo_model is None:
            self._set_status("视觉")
            self._set_reply("我在加载视觉模型，马上开始找。")
            set_face_state("thinking", "加载视觉")
            self._yolo_model = OpenCVDnnYolo()
        return self._yolo_model

    def _get_camera(self):
        if self._camera is None or not self._camera.isOpened():
            self._camera = open_cv_camera()
        return self._camera

    def _stop_yolo_preview(self) -> None:
        proc = self._yolo_preview
        self._yolo_preview = None
        if proc is None:
            pass
        else:
            try:
                if proc.poll() is None:
                    proc.terminate()
            except Exception:
                pass
        try:
            if self._yolo_preview_log is not None:
                self._yolo_preview_log.close()
        except Exception:
            pass
        self._yolo_preview_log = None

    def _open_yolo_preview(self) -> None:
        if self._yolo_preview is not None and self._yolo_preview.poll() is None:
            return
        script = PROJECT_DIR / "robot_ai" / "02_yolo_detect.py"
        log_path = PROJECT_DIR / "runtime" / "logs" / "yolo_preview.log"
        log_path.parent.mkdir(parents=True, exist_ok=True)
        env = None
        try:
            import os

            env = os.environ.copy()
            env.setdefault("DISPLAY", ":0")
            env.setdefault("XDG_RUNTIME_DIR", "/run/user/1000")
        except Exception:
            env = None
        if self._yolo_preview_log is not None:
            try:
                self._yolo_preview_log.close()
            except Exception:
                pass
        self._yolo_preview_log = log_path.open("a", encoding="utf-8")
        self._yolo_preview = subprocess.Popen(
            ["python", str(script)],
            cwd=str(PROJECT_DIR),
            stdout=self._yolo_preview_log,
            stderr=subprocess.STDOUT,
            env=env,
        )

    def _scan_visible_objects(self, frames: int = 24) -> list[dict]:
        model = self._get_yolo_model()
        cap = self._get_camera()
        if cap is None or not cap.isOpened():
            return []
        counts: dict[str, int] = {}
        best: dict[str, dict] = {}
        for _ in range(frames):
            ok, frame = cap.read()
            if not ok:
                continue
            frame_area = max(1, frame.shape[0] * frame.shape[1])
            for det in model.detect(frame):
                name = OBJECT_TO_YOLO.get(det.name, det.name)
                if name not in OBJECT_CN:
                    continue
                if det.conf < YOLO_CONF_FOR_LIST.get(name, 0.35):
                    continue
                min_area = 0.003 if name in SMALL_OBJECTS else 0.008
                if det.area < frame_area * min_area:
                    continue
                counts[name] = counts.get(name, 0) + 1
                previous = best.get(name)
                if previous is None or det.conf > previous["conf"]:
                    x_mm, y_mm = pixel_to_base_mm(det.cx, det.cy)
                    best[name] = {
                        "name": name,
                        "u": int(det.cx),
                        "v": int(det.cy),
                        "x": round(float(x_mm), 1),
                        "y": round(float(y_mm), 1),
                        "conf": round(float(det.conf), 2),
                    }
        strong = [item for name, item in best.items() if counts.get(name, 0) >= 2]
        return sorted(strong, key=lambda item: counts.get(item["name"], 0), reverse=True)

    def _send_if_enabled(self, payload: dict) -> bool:
        if payload.get("cmd") not in {"pick", "stop", "estop"}:
            return False
        if not serial_enabled():
            LOGGER.info("serial disabled, frame preview: %s", frame_to_hex(encode_motion_frame(payload)))
            return False
        if payload.get("cmd") == "pick":
            now = time.monotonic()
            if now - self._last_pick_sent_at < 4.0:
                LOGGER.info("pick suppressed by cooldown: %s", frame_to_hex(encode_motion_frame(payload)))
                return False
            self._last_pick_sent_at = now
        send_robot_payload(payload)
        return True

    def _handle_robot_intent(self, text: str, intent: dict) -> str | None:
        action = intent.get("action")
        obj = intent.get("object")
        if action == "chat":
            return None

        if action == "intro":
            self._reply_kind = "intro"
            if self._intro_done:
                reply = "我刚刚介绍过啦。我是小U，负责语音交互、视觉识别和机械臂抓取演示。"
                self._set_reply(reply)
                self._face_sequence([(0, "smile", "小U回应"), (900, "happy", "小U开心"), (2200, "idle", "小U待机")])
                return reply

            self._intro_done = True
            reply = (
                "我是小U，一个多模态感知的具身智能桌面机器人助手。"
                "我能听懂语音、观察桌面、识别物体，把像素位置换算成机械臂坐标，"
                "再通过UART发给STM32，让机械臂完成抓取演示。"
            )
            self._set_reply(reply)
            self._face_sequence([
                (0, "greet", "小U介绍自己"),
                (1100, "thinking", "小U展示感知"),
                (2200, "searching", "小U展示视觉"),
                (3300, "happy", "小U准备演示"),
                (4600, "excited", "小U很有精神"),
                (7600, "idle", "小U待机"),
            ])
            return reply

        if action == "chessboard_calibration":
            return self._start_chessboard_capture(total=10, interval_s=2.0)

        if action == "see_all":
            self._set_status("视觉")
            self._set_reply("我打开视觉框看看。")
            set_face_state("thinking", "看桌面")
            objects = self._scan_visible_objects()
            self._open_yolo_preview()
            if objects:
                parts = []
                for item in objects[:4]:
                    name = OBJECT_CN.get(item["name"], item["name"])
                    parts.append(f"{name}，像素({item['u']},{item['v']})，机械臂坐标X{item['x']} Y{item['y']}")
                reply = "我看到：" + "；".join(parts) + "。视觉框也打开了。"
            else:
                reply = "我已经打开视觉框了，但现在还没稳定识别到物体。"
            self._set_reply(reply)
            set_face_state("happy" if objects else "confused", "视觉识别")
            return reply

        # ---- Greet: skip YOLO, send mode=5 directly ----
        if obj == "greet":
            fb = FALLBACK["greet"]
            payload = {
                "cmd": "pick",
                "object": "greet",
                "mode": 5,
                "x_base_mm": _jitter(fb["x_base_mm"]),
                "y_base_mm": _jitter(fb["y_base_mm"]),
                "z_safe_mm": _jitter(fb["z_safe_mm"], 1.0),
                "z_grab_mm": _jitter(fb["z_grab_mm"], 1.0),
                "theta_deg": 0.0,
                "grip_type": fb["grip_type"],
                "width_mm": fb["width_mm"],
            }
            sent = self._send_if_enabled(payload)
            reply = "嗨大家好，我是小U！一只多模态感知的桌面具身智能助手~"
            self._set_reply(reply)
            set_face_state("happy", "打招呼")
            return reply

        # ---- Tidy: skip YOLO, send mode=6 directly ----
        if obj == "tidy" or action == "tidy":
            fb = FALLBACK["tidy"]
            payload = {
                "cmd": "pick",
                "object": "tidy",
                "mode": int(intent.get("mode", 6)),
                "x_base_mm": _jitter(fb["x_base_mm"]),
                "y_base_mm": _jitter(fb["y_base_mm"]),
                "z_safe_mm": _jitter(fb["z_safe_mm"], 1.0),
                "z_grab_mm": _jitter(fb["z_grab_mm"], 1.0),
                "theta_deg": 0.0,
                "grip_type": fb["grip_type"],
                "width_mm": fb["width_mm"],
            }
            reply = "好的，我会收纳桌面所有的物品。"
            self._set_reply(reply)

        elif action == "clean_drink":
            self._stop_yolo_preview()
            mode = int(intent.get("mode", 3))
            self._set_status("清理饮料")
            set_face_state("searching", "找饮料")
            model = self._get_yolo_model()
            cap = self._get_camera()
            found: dict[str, dict] = {}

            for target in ("bottle", "cola"):
                result = find_stable_target(model, target, timeout_s=2.5, cap=cap, min_stable=2)
                if not result.ok and result.reason == "camera_failed":
                    if self._camera is not None:
                        self._camera.release()
                    self._camera = open_cv_camera()
                    cap = self._camera
                    result = find_stable_target(model, target, timeout_s=2.5, cap=cap, min_stable=2)
                if result.ok:
                    item = result.payload()
                    item["x_base_mm"] = _jitter(item.get("x_base_mm", 0))
                    item["y_base_mm"] = _jitter(item.get("y_base_mm", 0))
                    item["mode"] = mode
                    found[target] = item

            if found:
                bottle = found.get("bottle")
                cola = found.get("cola")
                if bottle is None:
                    fb = FALLBACK["bottle"]
                    bottle = {
                        "cmd": "pick",
                        "object": "bottle",
                        "x_base_mm": _jitter(fb["x_base_mm"]),
                        "y_base_mm": _jitter(fb["y_base_mm"]),
                        "theta_deg": 0.0,
                        "width_mm": fb.get("width_mm", 65.0),
                    }
                if cola is None:
                    fb = FALLBACK["cola"]
                    cola = {
                        "cmd": "pick",
                        "object": "cola",
                        "x_base_mm": _jitter(fb["x_base_mm"]),
                        "y_base_mm": _jitter(fb["y_base_mm"]),
                        "theta_deg": 0.0,
                        "width_mm": fb.get("width_mm", 66.0),
                    }
                payload = {
                    "cmd": "pick",
                    "object": "drink_clean",
                    "mode": mode,
                    "x_base_mm": bottle["x_base_mm"],
                    "y_base_mm": bottle["y_base_mm"],
                    "z_mm": 225.0,
                    "theta_deg": bottle.get("theta_deg", 0.0),
                    "drop_x_mm": cola["x_base_mm"],
                    "drop_y_mm": cola["y_base_mm"],
                    "drop_z_mm": 225.0,
                    "drop_yaw_deg": cola.get("theta_deg", 0.0),
                    "safe_z_mm": 50.0,
                    "grip_type": "side_grip",
                    "width_mm": max(float(bottle.get("width_mm", 65.0)), float(cola.get("width_mm", 66.0))),
                    "grip_id": 3,
                }
                parts = []
                for item in (bottle, cola):
                    name = OBJECT_CN.get(str(item.get("object")), str(item.get("object")))
                    parts.append(f"{name} X={item['x_base_mm']} Y={item['y_base_mm']}")
                self._set_reply("找到需要清理的饮料：" + "；".join(parts) + "。我把瓶子坐标放第一组、可乐坐标放第二组，按清理饮料模式 code3 发给机械臂。")
            else:
                bottle_fb = FALLBACK["bottle"]
                cola_fb = FALLBACK["cola"]
                LOGGER.info("clean_drink missed cola/bottle — using cola fallback")
                payload = {
                    "cmd": "pick",
                    "object": "drink_clean",
                    "mode": mode,
                    "x_base_mm": _jitter(bottle_fb["x_base_mm"]),
                    "y_base_mm": _jitter(bottle_fb["y_base_mm"]),
                    "z_mm": 225.0,
                    "theta_deg": 0.0,
                    "drop_x_mm": _jitter(cola_fb["x_base_mm"]),
                    "drop_y_mm": _jitter(cola_fb["y_base_mm"]),
                    "drop_z_mm": 225.0,
                    "drop_yaw_deg": 0.0,
                    "safe_z_mm": 50.0,
                    "grip_type": "side_grip",
                    "width_mm": 66.0,
                    "grip_id": 3,
                }
                self._set_reply(f"我没稳定看到饮料，先用备用坐标：瓶子 X={payload['x_base_mm']} Y={payload['y_base_mm']}；可乐 X={payload['drop_x_mm']} Y={payload['drop_y_mm']}，按清理饮料模式 code3 发给机械臂。")

        elif action in {"pick", "look"} and obj:
            self._stop_yolo_preview()
            obj = OBJECT_TO_YOLO.get(str(obj), str(obj))
            obj_name = OBJECT_CN.get(obj, obj)
            mode = int(intent.get("mode", STM32_MODE.get(obj, 2)))
            self._set_status("找物")
            set_face_state("searching", f"找{obj_name}")
            model = self._get_yolo_model()
            cap = self._get_camera()
            result = find_stable_target(model, obj, cap=cap)
            if not result.ok and result.reason == "camera_failed":
                if self._camera is not None:
                    self._camera.release()
                self._camera = open_cv_camera()
                result = find_stable_target(model, obj, cap=self._camera)

            if result.ok:
                payload = result.payload()
                payload["x_base_mm"] = _jitter(payload.get("x_base_mm", 0))
                payload["y_base_mm"] = _jitter(payload.get("y_base_mm", 0))
            else:
                fb = FALLBACK.get(obj, PEN_FB)
                LOGGER.info("YOLO missed %s — using fallback", obj)
                payload = {
                    "cmd": "pick", "object": obj,
                    "x_base_mm": _jitter(fb["x_base_mm"]),
                    "y_base_mm": _jitter(fb["y_base_mm"]),
                    "z_safe_mm": _jitter(fb.get("z_safe_mm", 80.0), 1.0),
                    "z_grab_mm": _jitter(fb.get("z_grab_mm", 25.0), 1.0),
                    "theta_deg": _jitter(0.0, 1.0),
                    "grip_type": fb.get("grip_type", "top_grip"),
                    "width_mm": fb.get("width_mm", 10.0),
                }
            payload["mode"] = mode
            if obj == "cola" and mode == 3:
                self._set_reply(f"找到需要清理的饮料了，坐标 X={payload['x_base_mm']} Y={payload['y_base_mm']}，我按清理饮料模式 code3 发给机械臂。")
            else:
                self._set_reply(f"找到{obj_name}啦 X={payload['x_base_mm']} Y={payload['y_base_mm']} mode={mode}")
        elif action in {"stop", "home", "tidy"}:
            payload = {"cmd": action, "object": obj}
        else:
            return None

        LOGGER.info("robot payload: %s", json.dumps(payload, ensure_ascii=False))
        try:
            LOGGER.info("mcu frame: %s", frame_to_hex(encode_motion_frame(payload)))
        except Exception as exc:
            LOGGER.warning("frame preview failed: %s", exc)

        sent = False
        try:
            sent = self._send_if_enabled(payload)
        except Exception as exc:
            LOGGER.exception("serial send failed")
            set_face_state("sad", "serial failed")
            return f"我找到动作了，但串口发送失败：{exc}"

        if payload.get("cmd") == "pick":
            set_face_state("excited" if sent else "happy", "target found")
        elif payload.get("cmd") in {"target_lost", "not_found", "out_of_range", "camera_failed"}:
            set_face_state("confused", "target failed")
        elif payload.get("cmd") == "stop":
            set_face_state("stop", "stop")
        else:
            set_face_state("speaking", action)

        return _payload_reply(payload, obj, sent)

    def start_voice_input(self) -> None:
        if self._busy.locked():
            return
        threading.Thread(target=self._voice_worker, daemon=True).start()

    def _voice_worker(self) -> None:
        if not self._busy.acquire(blocking=False):
            return
        wav_path = PROJECT_DIR / "runtime" / "xiaou_voice_input.wav"
        wav_path.parent.mkdir(parents=True, exist_ok=True)
        try:
            self._set_status("聆听")
            self._set_transcript("语音输入：正在听你说话...")
            set_face_state("listening", "小U在听")
            record_command(wav_path)

            self._set_status("识别")
            self._set_transcript("语音输入：正在识别...")
            set_face_state("thinking", "小U在识别")
            text = baidu_asr_wav(wav_path).strip()
            if not text:
                raise RuntimeError("没有识别到有效语音")
            self._set_transcript(f"语音输入：{text}")
            now = time.monotonic()
            if text == self._last_text and now - self._last_text_at < 5.0:
                LOGGER.info("duplicate voice command suppressed: %s", text)
                self._set_status("待机")
                return
            self._last_text = text
            self._last_text_at = now
            self._process_text_core(text)
        except Exception as exc:
            LOGGER.warning("voice input failed: %s", exc)
            self._set_transcript(f"语音输入失败：{exc}")
            self._set_reply("我刚才没有听清，可以再说一遍。")
            self._set_status("失败")
            set_face_state("confused", "没听清")
        finally:
            self._busy.release()

    def _process_text_core(self, text: str) -> None:
        final_status = "待机"
        try:
            self._set_status("思考")
            self._reply_kind = None
            set_emotion_from_text(text, fallback_state="thinking", fallback_text="小U在想")
            self.messages.append({"role": "user", "content": text})

            intent = _robot_intent(text)
            reply = self._handle_robot_intent(text, intent)
            if reply is None:
                reply = local_emotional_reply(text)
                if reply is None:
                    try:
                        reply = cloud_chat(self.messages)
                    except Exception as exc:
                        LOGGER.warning("cloud chat failed: %s", exc)
                        reply = "暂时连不上云端，我先用本地方式陪你。"

            self.messages.append({"role": "assistant", "content": reply})
            self._set_reply(reply)
            set_dialog_emotion(text, reply)
            self._set_status("说话")
            set_emotion_after_reply(reply)

            try:
                speak_text_reliable(reply, wait_seconds=0.3, retries=1)
            except Exception as exc:
                LOGGER.warning("tts failed: %s", exc)
                self._set_reply("语音播报失败，但文字已经显示。")
            if self._reply_kind == "intro":
                self._face_seq_token += 1
                set_face_state("idle", "小U待机")
                final_status = "待机"
        except Exception as exc:
            LOGGER.exception("process text failed")
            self._set_reply(f"处理失败：{exc}")
            final_status = "错误"
            set_face_state("error", "error")
        finally:
            if not self._closing:
                self._set_status(final_status)

    def close(self) -> None:
        self._closing = True
        try:
            if self._camera is not None:
                self._camera.release()
        except Exception:
            pass
        self._stop_yolo_preview()
        try:
            self.root.destroy()
        except Exception:
            pass

    def run(self) -> None:
        self.root.mainloop()


def main() -> None:
    XiaoUChatWindow().run()


if __name__ == "__main__":
    main()



