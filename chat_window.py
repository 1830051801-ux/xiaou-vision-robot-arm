from __future__ import annotations

import json
import re
import threading
import time
import tkinter as tk
from pathlib import Path

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
from emotion_state import local_emotional_reply, set_dialog_emotion, set_emotion_from_text, xiaou_style_prompt
from face_display import GifFacePanel
from face_state import set_face_state
from robot_ai_07_import import cloud_chat, speak_text_reliable
from robot_protocol import encode_motion_frame, frame_to_hex, send_robot_payload
from vision_targeting import find_stable_target
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

BG = "#000000"
PANEL = "#10141c"
PANEL_2 = "#171d27"
TEXT = "#f6f8ff"
MUTED = "#93a1b5"
ACCENT = "#7dd3fc"

SAMPLE_RATE = CFG.voice_sample_rate
RECORD_SECONDS = max(3.0, min(15.0, float(getattr(CFG, "record_seconds", 5.0))))
ENABLE_DENOISE = getattr(CFG, "enable_denoise", True)

OBJECT_CN = {
    "pen": "笔",
    "cup": "杯子",
    "bottle": "瓶子",
    "Coffee cup": "杯子",
    "Bottle": "瓶子",
    "Pen": "笔",
}

OBJECT_TO_YOLO = {
    "Coffee cup": "cup",
    "cup": "cup",
    "Bottle": "bottle",
    "bottle": "bottle",
    "Pen": "pen",
    "Pencil": "pen",
    "pen": "pen",
}


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
        "回答要短、自然、有情绪，但不要夸张。"
        "用户让你拿东西时，先确认动作；找到目标后说明已经发送给机械臂。"
        "用户夸你时开心一点；用户抱怨时先委屈地承认问题，再给出下一步。"
    )


def _butter_bandpass(lowcut: float, highcut: float, fs: int, order: int = 4):
    nyq = 0.5 * fs
    low = lowcut / nyq
    high = highcut / nyq
    return butter(order, [low, high], btype="band")


def _reduce_noise(audio: np.ndarray) -> np.ndarray:
    if audio.size == 0:
        return audio
    if _HAS_SCIPY:
        try:
            b, a = _butter_bandpass(80.0, 7500.0, SAMPLE_RATE, order=4)
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
    if re.search(r"\u53ef\u4e50|\u53ef\u53e3\u53ef\u4e50|cola|coke", text, re.I):
        obj = "cola"
    elif re.search(r"\u8033\u673a|\u8033\u9ea6|earphone|headphone|headphones", text, re.I):
        obj = "earphone"
    elif re.search(r"笔|钢笔|圆珠笔|铅笔|pen|pencil", text, re.I):
        obj = "pen"
    elif re.search(r"水杯|杯子|茶杯|杯|cup", text, re.I):
        obj = "cup"
    elif re.search(r"瓶子|水瓶|饮料瓶|bottle", text, re.I):
        obj = "bottle"

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
        if action in {"pick", "look", "stop", "home", "tidy"}:
            return {"action": action, "object": obj}
    except Exception as exc:
        LOGGER.debug("cloud intent skipped: %s", exc)
    return local


def _payload_reply(payload: dict, obj: str | None, sent: bool) -> str:
    cmd = payload.get("cmd")
    obj_name = OBJECT_CN.get(obj or "", obj or "目标")
    if cmd == "pick":
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

        self.face = GifFacePanel(
            self.stage,
            width=self.screen_w,
            height=self.screen_h,
            fill=True,
            padx=0,
            pady=0,
        )
        self.root.after(60, lambda: self.face.update(self.root))

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
        self._yolo_model: OpenCVDnnYolo | None = None
        self._camera = None
        self.root.after(220, self._raise_window)

        self.root.after(80, lambda: set_face_state("greet", "小U打招呼"))
        self._set_reply("我在，直接说就行。")
        self._set_status("待机")
        self.root.bind("<Return>", self._trigger_voice)
        self.root.bind("<KP_Enter>", self._trigger_voice)
        self.root.bind("<space>", self._trigger_voice)

        if sd is None or sf is None:
            self.voice_hint.configure(text="语音模块未加载")

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

        if action in {"pick", "look"} and obj:
            obj = OBJECT_TO_YOLO.get(str(obj), str(obj))
            obj_name = OBJECT_CN.get(obj, obj)
            self._set_status("找物")
            self._set_reply(f"我先找一下{obj_name}。")
            set_face_state("searching", f"searching {obj_name}")
            model = self._get_yolo_model()
            cap = self._get_camera()
            result = find_stable_target(model, obj, cap=cap)
            if not result.ok and result.reason == "camera_failed":
                if self._camera is not None:
                    self._camera.release()
                self._camera = open_cv_camera()
                result = find_stable_target(model, obj, cap=self._camera)
            payload = result.payload()
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
            set_face_state("speaking", reply)

            try:
                speak_text_reliable(reply, wait_seconds=0.8, retries=2)
            except Exception as exc:
                LOGGER.warning("tts failed: %s", exc)
                self._set_reply("语音播报失败，但文字已经显示。")

            set_dialog_emotion(text, reply)
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


