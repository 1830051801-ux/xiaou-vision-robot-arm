from __future__ import annotations

import argparse
import time
from pathlib import Path

import sounddevice as sd
import soundfile as sf

from baidu_asr import baidu_asr_wav
from common import PROJECT_DIR, ask_cloud_intent, command_json, print_json
from emotion_state import local_emotional_reply, set_dialog_emotion, set_emotion_after_reply, set_emotion_from_text
from face_state import set_face_state
from robot_ai_07_import import cloud_chat, speak_text
from xiaou_runtime import get_xiaou_config
from vision_targeting import find_stable_target
from yolo_opencv import OpenCVDnnYolo


CFG = get_xiaou_config()
SAMPLE_RATE = CFG.voice_sample_rate
RECORD_SECONDS = 5

OBJECT_CN = {
    "Coffee cup": "水杯",
    "Bottle": "瓶子",
    "Pen": "笔",
    "Mobile phone": "手机",
    "Book": "书",
    "Computer keyboard": "键盘",
    "Computer mouse": "鼠标",
    "Scissors": "剪刀",
}


def record_wav(path: Path) -> None:
    print(f"Recording {RECORD_SECONDS}s. Speak now...")
    audio = sd.rec(int(RECORD_SECONDS * SAMPLE_RATE), samplerate=SAMPLE_RATE, channels=1, dtype="float32")
    sd.wait()
    sf.write(path, audio, SAMPLE_RATE, subtype="PCM_16")


def make_reply(action: str | None, obj: str | None, payload: dict) -> str:
    obj_cn = OBJECT_CN.get(obj or "", obj or "目标")
    cmd = payload.get("cmd")
    if cmd == "pick":
        return (
            f"收到，我看到{obj_cn}了。"
            f"坐标 X {payload.get('x_base_mm')} 毫米，Y {payload.get('y_base_mm')} 毫米。"
        )
    if cmd in {"not_found", "target_lost"}:
        return f"我还没有稳定看到{obj_cn}，你把它放到镜头中间一点。"
    if cmd == "out_of_range":
        return f"我看到{obj_cn}了，但是它超出机械臂安全范围。"
    if cmd == "camera_failed":
        return "摄像头打开失败，我现在看不到画面。"
    if action == "stop":
        return "收到，我已经停止。"
    if action == "home":
        return "收到，我准备回到初始位置。"
    if action == "tidy":
        return "收到，我准备整理桌面。"
    return "收到。"


def print_plan(payload: dict) -> None:
    print_json("robot_cmd", payload)
    if payload.get("cmd") != "pick":
        return
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
        f"width={payload.get('width_mm')}mm",
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--speak", action="store_true")
    args = parser.parse_args()

    robot_name = CFG.robot_name
    wav_path = PROJECT_DIR / "voice_robot_input.wav"

    set_face_state("idle", f"{robot_name}待机中")
    print("=" * 48)
    print(f"{robot_name} desktop robot assistant")
    print("voice -> Baidu ASR -> intent/chat -> YOLO -> ROS2 arm plan")
    print("=" * 48)
    print("Loading YOLO...")
    model = OpenCVDnnYolo()
    print("Ready. Press Enter and speak. Type q to quit.")

    while True:
        cmd = input("\nEnter to speak/q> ").strip().lower()
        if cmd in {"q", "quit", "exit"}:
            break

        try:
            set_face_state("listening", "我在听")
            record_wav(wav_path)
            set_face_state("thinking", "正在识别")
            user_text = baidu_asr_wav(wav_path)
            print(f"ASR: {user_text}")
            set_emotion_from_text(user_text)
        except Exception as exc:
            set_face_state("error", "没有听清")
            print(f"ASR failed: {exc}")
            continue

        set_face_state("thinking", "正在理解")
        intent = ask_cloud_intent(user_text)
        print_json("intent", intent)
        action = intent.get("action")
        obj = intent.get("object")

        if action == "pick" and obj:
            set_face_state("searching", f"正在找{OBJECT_CN.get(obj, obj)}")
            payload = find_stable_target(model, obj).payload()
        elif action == "tidy":
            payload = command_json("tidy", obj)
        elif action in {"stop", "home"}:
            payload = command_json(action, obj)
        elif action == "look" and obj:
            set_face_state("searching", f"正在找{OBJECT_CN.get(obj, obj)}")
            payload = find_stable_target(model, obj).payload()
        else:
            payload = command_json("chat", obj)

        print_plan(payload)

        if payload.get("cmd") == "pick":
            set_face_state("happy", f"找到{OBJECT_CN.get(obj or '', obj or '目标')}")
        elif payload.get("cmd") in {"stop", "home"}:
            set_face_state("stop", "已停止" if payload.get("cmd") == "stop" else "回到初始")
        elif payload.get("cmd") in {"not_found", "target_lost", "out_of_range", "camera_failed"}:
            set_face_state("error", "没有找到目标")

        print("planning only: ROS2/MoveIt is the six-axis execution boundary")

        reply = make_reply(action, obj, payload)
        if payload.get("cmd") == "chat":
            reply = local_emotional_reply(user_text)
            if reply is None:
                try:
                    reply = cloud_chat(
                        [
                            {
                                "role": "system",
                                "content": "你是桌面机械臂助手小U。回答简短自然。用户骂你时不要开心，不要反击，要委屈但礼貌；用户难过时要安慰；用户开心时一起开心。",
                            },
                            {"role": "user", "content": user_text},
                        ]
                    )
                except Exception as exc:
                    reply = f"云端聊天失败：{exc}"
                    set_face_state("error", "网络不太舒服")
            set_dialog_emotion(user_text, reply)

        print(f"{robot_name}: {reply}")
        if args.speak:
            set_face_state("speaking", "正在回复")
            speak_text(reply)
            if payload.get("cmd") == "chat":
                set_dialog_emotion(user_text, reply)
        if payload.get("cmd") in {"stop", "home", "tidy"}:
            time.sleep(1.5)
            set_face_state("idle", f"{robot_name}待机中")


if __name__ == "__main__":
    main()
