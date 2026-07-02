from __future__ import annotations

import base64
import io
import json
import threading
import tkinter as tk
from pathlib import Path
from tkinter import scrolledtext

import numpy as np
import sounddevice as sd
import soundfile as sf
from PIL import Image, ImageSequence

from baidu_asr import baidu_asr_wav
from common import PROJECT_DIR, ask_cloud_intent, normalize_object_name, serial_enabled
from device_runtime import configure_sounddevice, open_cv_camera
from emotion_state import local_emotional_reply, set_dialog_emotion, set_emotion_from_text, xiaou_style_prompt
from face_state import get_face_state, set_face_state
from robot_ai_07_import import cloud_chat, speak_text
from robot_protocol import send_robot_payload
from vision_targeting import find_stable_target
from yolo_opencv import OpenCVDnnYolo


SAMPLE_RATE = 16000
CHUNK_SECONDS = 0.2
MAX_SECONDS = 8.0
MIN_SECONDS = 1.0
SILENCE_SECONDS = 1.2
SILENCE_RMS = 0.012

FACE_ASSET_DIR = PROJECT_DIR / "robot_ai" / "emote_assets" / "gif"
FACE_STATE_TO_GIF = {
    "idle": "uh_huh_ok.gif",
    "listening": "uh_huh_ok.gif",
    "thinking": "robot_what.gif",
    "searching": "nervous_oh_no.gif",
    "happy": "great_job_thumbs_up.gif",
    "sad": "sad_cry.gif",
    "error": "sad_head_pat.gif",
    "stop": "sad_head_pat.gif",
    "speaking": "you_got_this_u_da_man.gif",
}

OBJECT_CN = {
    "Coffee cup": "杯子",
    "Bottle": "瓶子",
    "Pen": "笔",
    "Mobile phone": "手机",
    "Book": "书",
    "Computer keyboard": "键盘",
    "Computer mouse": "鼠标",
    "Scissors": "剪刀",
}

PICK_WORDS = ("拿", "取", "抓", "递", "帮我拿", "帮我取", "fetch", "grab", "pick")
LOOK_WORDS = ("找", "看看", "看一下", "search", "look")
STOP_WORDS = ("停止", "停下", "别动", "急停", "stop")
HOME_WORDS = ("回零", "回家", "回到原点", "待机", "home")


def direct_robot_intent(user_text: str) -> dict | None:
    obj = normalize_object_name(user_text)
    text = user_text.lower()
    if any(word in text for word in STOP_WORDS):
        return {"action": "stop", "object": None, "reply": "收到，我先停下。"}
    if any(word in text for word in HOME_WORDS):
        return {"action": "home", "object": None, "reply": "收到，我回到安全位置。"}
    if obj and any(word in text for word in PICK_WORDS):
        return {"action": "pick", "object": obj, "reply": f"收到，我去拿{obj}。"}
    if obj and any(word in text for word in LOOK_WORDS):
        return {"action": "look", "object": obj, "reply": f"收到，我去找{obj}。"}
    return None


def resolve_robot_intent(user_text: str, cloud_intent: dict) -> dict:
    action = str(cloud_intent.get("action") or "").strip().lower()
    obj = normalize_object_name(str(cloud_intent.get("object") or "")) or normalize_object_name(user_text)
    text = user_text.lower()

    if any(word in text for word in STOP_WORDS):
        return {"action": "stop", "object": None, "reply": cloud_intent.get("reply") or "收到，我先停下。"}
    if any(word in text for word in HOME_WORDS):
        return {"action": "home", "object": None, "reply": cloud_intent.get("reply") or "收到，我回到安全位置。"}

    if obj:
        if any(word in text for word in PICK_WORDS):
            return {"action": "pick", "object": obj, "reply": cloud_intent.get("reply") or f"收到，我去拿{obj}。"}
        if any(word in text for word in LOOK_WORDS):
            return {"action": "look", "object": obj, "reply": cloud_intent.get("reply") or f"收到，我去找{obj}。"}

    if obj and action in {"chat", "think", "answer", ""}:
        return {"action": "look", "object": obj, "reply": cloud_intent.get("reply") or f"收到，我去看看{obj}。"}

    if obj and action in {"pick", "look"}:
        cloud_intent["object"] = obj
        return cloud_intent

    if obj:
        cloud_intent["object"] = obj
    return cloud_intent


def record_until_silence(path: Path) -> None:
    chunk_n = int(SAMPLE_RATE * CHUNK_SECONDS)
    max_chunks = int(MAX_SECONDS / CHUNK_SECONDS)
    min_chunks = int(MIN_SECONDS / CHUNK_SECONDS)
    silence_need = max(1, int(SILENCE_SECONDS / CHUNK_SECONDS))
    silence_count = 0
    chunks: list[np.ndarray] = []

    for index in range(max_chunks):
        audio = sd.rec(chunk_n, samplerate=SAMPLE_RATE, channels=1, dtype="float32")
        sd.wait()
        chunks.append(audio.copy())
        rms = float(np.sqrt(np.mean(np.square(audio))))
        if index >= min_chunks and rms < SILENCE_RMS:
            silence_count += 1
        else:
            silence_count = 0
        if index >= min_chunks and silence_count >= silence_need:
            break

    merged = np.concatenate(chunks, axis=0) if chunks else np.zeros((1, 1), dtype=np.float32)
    sf.write(path, merged, SAMPLE_RATE, subtype="PCM_16")


def _fit_face(image: Image.Image, size: tuple[int, int]) -> Image.Image:
    image = image.convert("RGBA")
    background = Image.new("RGBA", image.size, (0, 0, 0, 255))
    background.alpha_composite(image)
    image = background.convert("RGB")
    image.thumbnail(size, Image.Resampling.LANCZOS)
    canvas = Image.new("RGB", size, (0, 0, 0))
    canvas.paste(image, ((size[0] - image.width) // 2, (size[1] - image.height) // 2))
    return canvas


class FacePanel:
    def __init__(self, parent: tk.Misc, width: int = 420, height: int = 300) -> None:
        self.width = width
        self.height = height
        self._gif_name = ""
        self._frame_index = 0
        self._frames: list[tk.PhotoImage] = []
        self._durations: list[int] = []

        frame = tk.Frame(parent, bg="#0f0f0f", highlightthickness=1, highlightbackground="#2a2a2a")
        frame.pack(fill=tk.X, padx=10, pady=(10, 6))
        self.canvas = tk.Canvas(frame, width=width, height=height, bg="#0f0f0f", highlightthickness=0)
        self.canvas.pack()
        self.image_item = self.canvas.create_image(width // 2, height // 2, anchor=tk.CENTER)
        self.placeholder = tk.PhotoImage(width=width, height=height)
        self.canvas.itemconfigure(self.image_item, image=self.placeholder)

    def _load_animation(self, gif_name: str) -> None:
        path = FACE_ASSET_DIR / gif_name
        frames: list[tk.PhotoImage] = []
        durations: list[int] = []
        with Image.open(path) as gif:
            for frame in ImageSequence.Iterator(gif):
                fitted = _fit_face(frame.copy(), (self.width, self.height))
                buf = io.BytesIO()
                fitted.save(buf, format="GIF")
                frames.append(tk.PhotoImage(data=base64.b64encode(buf.getvalue()).decode("ascii"), format="gif"))
                durations.append(max(int(frame.info.get("duration", 80)), 30))
                if len(frames) >= 48:
                    break
        if not frames:
            raise RuntimeError(f"No frames in {path}")
        self._frames = frames
        self._durations = durations
        self._frame_index = 0
        self._gif_name = gif_name

    def update(self, root: tk.Tk) -> None:
        state = get_face_state().get("state", "idle")
        gif_name = FACE_STATE_TO_GIF.get(state, "idle.gif")
        if gif_name != self._gif_name:
            try:
                self._load_animation(gif_name)
            except Exception:
                self._frames = [self.placeholder]
                self._durations = [120]
                self._gif_name = gif_name

        if self._frames:
            self.canvas.itemconfigure(self.image_item, image=self._frames[self._frame_index])
            delay = self._durations[self._frame_index] if self._durations else 120
            self._frame_index = (self._frame_index + 1) % len(self._frames)
            root.after(delay, lambda: self.update(root))


def build_pick_reply(payload: dict, obj: str) -> str:
    obj_name = OBJECT_CN.get(obj, obj)
    cmd = payload.get("cmd")
    if cmd == "pick":
        return (
            f"好哒，我看到{obj_name}啦。"
            f"坐标大概是 X {payload.get('x_base_mm')} 毫米，Y {payload.get('y_base_mm')} 毫米。"
        )
    if cmd in {"target_lost", "not_found"}:
        return f"我还没稳稳看到{obj_name}呢。你把它放到镜头中间一点，我再试一次。"
    if cmd == "out_of_range":
        return f"我看见{obj_name}了，不过它超出安全范围了。你稍微放近一点我再帮你。"
    if cmd == "camera_failed":
        return "摄像头现在没准备好，我先不乱猜。"
    return f"我先去看看{obj_name}。"


class XiaoUVoiceGui:
    def __init__(self) -> None:
        configure_sounddevice()

        self.root = tk.Tk()
        self.root.title("XiaoU")
        self.root.geometry("900x720")
        self.root.configure(bg="#111111")
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.face_panel = FacePanel(self.root, width=420, height=300)

        self.log = scrolledtext.ScrolledText(
            self.root,
            wrap=tk.WORD,
            bg="#111111",
            fg="white",
            insertbackground="white",
            relief=tk.FLAT,
            padx=12,
            pady=12,
        )
        self.log.pack(fill=tk.BOTH, expand=True, padx=10, pady=(0, 8))
        self.log.configure(state="disabled")

        bottom = tk.Frame(self.root, bg="#111111")
        bottom.pack(fill=tk.X, padx=10, pady=(0, 10))

        self.status = tk.Label(bottom, text="按 Enter 开始录音", bg="#111111", fg="#9bdcff", anchor="w")
        self.status.pack(side=tk.LEFT, fill=tk.X, expand=True)

        self.button = tk.Button(
            bottom,
            text="开始录音",
            command=self.on_record,
            bg="#2a2a2a",
            fg="white",
            relief=tk.FLAT,
            padx=14,
            pady=6,
        )
        self.button.pack(side=tk.RIGHT)

        self.root.bind("<Return>", self.on_record)
        self.root.bind("<KP_Enter>", self.on_record)

        self.stop_event = threading.Event()
        self.record_lock = threading.RLock()
        self.camera_lock = threading.RLock()
        self.busy = False
        self.camera = open_cv_camera()
        try:
            self.model: OpenCVDnnYolo | None = OpenCVDnnYolo()
        except Exception as exc:
            self.model = None
            self.append("System", f"YOLO 后端不可用：{exc}")

        self.messages: list[dict[str, str]] = [
            {
                "role": "system",
                "content": xiaou_style_prompt(
                    "如果用户明确要求拿、抓、递某个物品，只确认你听到了，不要说已经完成动作。"
                    "说话要自然、软一点，像一只认真但可爱的小猫助手。"
                    "如果用户抱怨、责骂或失落，先委屈、安慰、缓和，不要逗乐。"
                ),
            }
        ]
        self.append("System", "按 Enter 开始录音。说完后停下来，小U会识别并回复。")
        self.root.after(80, lambda: self.face_panel.update(self.root))
        self.root.after(200, self.raise_windows)

    def append(self, who: str, text: str) -> None:
        def _append() -> None:
            self.log.configure(state="normal")
            self.log.insert(tk.END, f"{who}: {text}\n")
            self.log.see(tk.END)
            self.log.configure(state="disabled")

        self.root.after(0, _append)

    def set_status(self, text: str) -> None:
        self.root.after(0, lambda: self.status.configure(text=text))

    def raise_windows(self) -> None:
        try:
            self.root.deiconify()
            self.root.lift()
            self.root.focus_force()
            self.root.attributes("-topmost", True)
            self.root.after(400, lambda: self.root.attributes("-topmost", False))
        except Exception:
            pass

    def on_record(self, _event: object | None = None) -> None:
        with self.record_lock:
            if self.busy:
                return
            self.busy = True
        self.button.configure(state=tk.DISABLED)
        self.set_status("录音中...")
        threading.Thread(target=self.record_and_reply, daemon=True).start()

    def _ensure_camera(self) -> None:
        with self.camera_lock:
            if self.camera is None or not self.camera.isOpened():
                self.camera = open_cv_camera()

    def _run_pick(self, obj: str) -> dict:
        self.set_status(f"寻找 {OBJECT_CN.get(obj, obj)} ...")
        set_face_state("searching", f"正在找{OBJECT_CN.get(obj, obj)}")
        self.append("Vision", f"YOLO search for: {obj}")
        self._ensure_camera()
        if self.model is None:
            return {"cmd": "camera_failed", "object": obj}
        camera = open_cv_camera()
        if camera is None or not camera.isOpened():
            return {"cmd": "camera_failed", "object": obj}
        try:
            result = find_stable_target(
                self.model,
                obj,
                timeout_s=4.0,
                cap=camera,
                min_area_ratio=0.008,
                filter_size=3,
                min_stable=1,
            )
            payload = result.payload()
            if payload.get("cmd") in {"not_found", "target_lost"}:
                result = find_stable_target(
                    self.model,
                    obj,
                    timeout_s=6.0,
                    cap=camera,
                )
                payload = result.payload()
        finally:
            with self.camera_lock:
                try:
                    camera.release()
                except Exception:
                    pass
        if payload.get("cmd") == "camera_failed":
            with self.camera_lock:
                try:
                    self.camera.release()
                except Exception:
                    pass
                self.camera = None
        if serial_enabled():
            try:
                send_robot_payload(payload)
            except Exception as exc:
                self.append("Serial", f"发送失败：{exc}")
        return payload

    def _chat_reply(self, user_text: str) -> str:
        reply = local_emotional_reply(user_text)
        if reply is not None:
            return reply
        self.messages.append({"role": "user", "content": user_text})
        try:
            reply = cloud_chat(self.messages)
        except Exception as exc:
            reply = f"Cloud AI request failed: {exc}"
        self.messages.append({"role": "assistant", "content": reply})
        return reply

    def record_and_reply(self) -> None:
        wav_path = PROJECT_DIR / "voice_ai_input.wav"
        try:
            self.append("System", "开始录音，等你说完后自动停。")
            record_until_silence(wav_path)
            self.set_status("识别中...")
            text = baidu_asr_wav(wav_path).strip()
            if not text:
                raise RuntimeError("没有识别到内容")

            self.append("Me", text)
            set_emotion_from_text(text)

            intent = direct_robot_intent(text) or resolve_robot_intent(text, ask_cloud_intent(text))
            self.append("Intent", json.dumps(intent, ensure_ascii=False))
            action = intent.get("action")
            obj = normalize_object_name(str(intent.get("object") or "")) or None
            if obj is None and action in {"pick", "look"}:
                obj = normalize_object_name(text)
            if obj is not None:
                intent["object"] = obj

            if action in {"pick", "look"} and obj:
                payload = self._run_pick(obj)
                reply = build_pick_reply(payload, obj)
                if payload.get("cmd") == "pick":
                    set_face_state("happy", f"找到{OBJECT_CN.get(obj, obj)}")
                elif payload.get("cmd") in {"target_lost", "not_found", "out_of_range", "camera_failed"}:
                    set_face_state("error", "没有稳稳看到目标")
            elif action == "stop":
                payload = {"cmd": "stop", "object": obj}
                reply = "收到呀，我先停下来。"
                set_face_state("stop", "已停止")
                if serial_enabled():
                    try:
                        send_robot_payload(payload)
                    except Exception as exc:
                        self.append("Serial", f"发送失败：{exc}")
            elif action == "home":
                payload = {"cmd": "home", "object": obj}
                reply = "收到呀，我先回到安全位置。"
                set_face_state("idle", "回到待机")
                if serial_enabled():
                    try:
                        send_robot_payload(payload)
                    except Exception as exc:
                        self.append("Serial", f"发送失败：{exc}")
            else:
                reply = self._chat_reply(text)
                set_dialog_emotion(text, reply)

            self.append("XiaoU", reply)
            self.set_status("播报中...")
            speak_text(reply)
            set_dialog_emotion(text, reply)
        except Exception as exc:
            self.append("Error", str(exc))
            self.set_status(f"出错: {exc}")
            set_face_state("error", "出错了")
        finally:
            self.root.after(0, lambda: self.button.configure(state=tk.NORMAL))
            with self.record_lock:
                self.busy = False
            self.set_status("按 Enter 开始录音")

    def close(self) -> None:
        if self.stop_event.is_set():
            return
        self.stop_event.set()
        with self.camera_lock:
            if self.camera is not None:
                self.camera.release()
                self.camera = None
        try:
            self.root.destroy()
        except Exception:
            pass

    def run(self) -> None:
        self.root.mainloop()


def main() -> None:
    XiaoUVoiceGui().run()


if __name__ == "__main__":
    main()
