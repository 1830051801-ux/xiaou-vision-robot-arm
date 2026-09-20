"""Small XiaoU voice/vision GUI.

The GUI creates a six-axis arm planning request and never owns hardware
execution. ROS2/MoveIt and its safety gate are the only execution boundary.
"""

from __future__ import annotations

import json
import threading
import tkinter as tk
from pathlib import Path
from tkinter import scrolledtext

import sounddevice as sd
import soundfile as sf

from baidu_asr import baidu_asr_wav
from common import PROJECT_DIR, ask_cloud_intent, normalize_object_name
from device_runtime import open_cv_camera
from emotion_state import local_emotional_reply, set_dialog_emotion
from face_state import set_face_state
from robot_ai_07_import import cloud_chat, speak_text
from vision_targeting import find_stable_target
from yolo_opencv import OpenCVDnnYolo


SAMPLE_RATE = 16000
INPUT_SECONDS = 6


class XiaoUGui:
    def __init__(self) -> None:
        self.root = tk.Tk()
        self.root.title("XiaoU | Six-axis planning preview")
        self.root.geometry("760x520")
        self.output = scrolledtext.ScrolledText(self.root, wrap=tk.WORD, state=tk.DISABLED)
        self.output.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        self.entry = tk.Entry(self.root)
        self.entry.pack(fill=tk.X, padx=10)
        self.entry.bind("<Return>", lambda _event: self.start_text())
        buttons = tk.Frame(self.root)
        buttons.pack(fill=tk.X, padx=10, pady=8)
        tk.Button(buttons, text="Plan text", command=self.start_text).pack(side=tk.LEFT)
        tk.Button(buttons, text="Record voice", command=self.start_voice).pack(side=tk.LEFT, padx=8)
        tk.Button(buttons, text="Quit", command=self.root.destroy).pack(side=tk.RIGHT)
        self.model = None
        self.camera = None
        self.busy = False

    def append(self, label: str, value: object) -> None:
        self.output.configure(state=tk.NORMAL)
        self.output.insert(tk.END, f"{label}: {value}\n")
        self.output.see(tk.END)
        self.output.configure(state=tk.DISABLED)

    def start_text(self) -> None:
        text = self.entry.get().strip()
        self.entry.delete(0, tk.END)
        if text:
            self._start(text)

    def start_voice(self) -> None:
        self._start(None)

    def _start(self, text: str | None) -> None:
        if self.busy:
            return
        self.busy = True
        threading.Thread(target=self._worker, args=(text,), daemon=True).start()

    def _worker(self, text: str | None) -> None:
        try:
            if text is None:
                path = PROJECT_DIR / "runtime" / "gui_voice_input.wav"
                path.parent.mkdir(parents=True, exist_ok=True)
                self.append("System", "recording")
                audio = sd.rec(INPUT_SECONDS * SAMPLE_RATE, samplerate=SAMPLE_RATE, channels=1, dtype="float32")
                sd.wait()
                sf.write(path, audio, SAMPLE_RATE, subtype="PCM_16")
                text = baidu_asr_wav(path).strip()
            self.append("User", text)
            intent = ask_cloud_intent(text)
            self.append("Intent", json.dumps(intent, ensure_ascii=False))
            action = str(intent.get("action") or "chat")
            obj = normalize_object_name(str(intent.get("object") or ""))
            if action in {"pick", "look"} and obj:
                plan = self._plan_target(obj)
            else:
                plan = {"cmd": action, "object": obj, "execution": "ros2_moveit"}
            self.append("arm_plan_request", json.dumps(plan, ensure_ascii=False))
            reply = self._reply(text, plan, obj)
            self.append("XiaoU", reply)
            set_dialog_emotion(text, reply)
            speak_text(reply)
        except Exception as exc:
            self.append("Error", exc)
            set_face_state("error", "planning error")
        finally:
            self.busy = False

    def _plan_target(self, obj: str) -> dict:
        set_face_state("searching", f"searching {obj}")
        if self.model is None:
            self.model = OpenCVDnnYolo()
        if self.camera is None or not self.camera.isOpened():
            self.camera = open_cv_camera()
        if self.camera is None or not self.camera.isOpened():
            return {"cmd": "camera_failed", "object": obj, "execution": "ros2_moveit"}
        result = find_stable_target(self.model, obj, cap=self.camera)
        plan = result.payload()
        plan["execution"] = "ros2_moveit"
        plan["six_axis"] = True
        return plan

    @staticmethod
    def _reply(text: str, plan: dict, obj: str | None) -> str:
        cmd = plan.get("cmd")
        if cmd == "pick":
            return f"Target {obj} found. A six-axis ROS2 plan was prepared; nothing was executed."
        if cmd in {"target_lost", "not_found", "out_of_range", "camera_failed"}:
            return f"Target planning failed: {cmd}."
        reply = local_emotional_reply(text)
        if reply:
            return reply
        try:
            return cloud_chat([{"role": "user", "content": text}])
        except Exception:
            return "I heard you."

    def run(self) -> None:
        self.root.mainloop()


def main() -> None:
    XiaoUGui().run()


if __name__ == "__main__":
    main()
