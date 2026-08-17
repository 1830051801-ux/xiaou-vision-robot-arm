from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import sounddevice as sd
import soundfile as sf

from baidu_asr import baidu_asr_wav
from common import PROJECT_DIR, ask_cloud_intent, print_json
from dialog_emote_bridge import set_dialog_face
from emotion_state import local_emotional_reply, set_dialog_emotion, set_emotion_from_text
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
    print(f"[1/6] 录音 {RECORD_SECONDS} 秒，请说话...")
    audio = sd.rec(
        int(RECORD_SECONDS * SAMPLE_RATE),
        samplerate=SAMPLE_RATE,
        channels=1,
        dtype="float32",
    )
    sd.wait()
    sf.write(path, audio, SAMPLE_RATE, subtype="PCM_16")
    print(f"      saved: {path}")


def robot_reply(payload: dict, obj: str | None) -> str:
    obj_cn = OBJECT_CN.get(obj or "", obj or "目标")
    cmd = payload.get("cmd")
    if cmd == "pick":
        return (
            f"收到，我看到{obj_cn}了。"
            f"坐标 X {payload.get('x_base_mm')} 毫米，Y {payload.get('y_base_mm')} 毫米，"
            f"角度 {payload.get('theta_deg')} 度。"
        )
    if cmd in {"target_lost", "not_found"}:
        return f"我还没稳定看到{obj_cn}，把它放到画面中间再试一次。"
    if cmd == "out_of_range":
        return f"我看到{obj_cn}了，但是它超出机械臂安全范围。"
    if cmd == "camera_failed":
        return "摄像头打开失败，我现在看不到画面。"
    if cmd == "stop":
        return "收到，已停止。"
    if cmd == "home":
        return "收到，准备回到初始位置。"
    if cmd == "tidy":
        return "收到，准备整理桌面。"
    return "收到。"


def print_pick_summary(payload: dict) -> None:
    print_json("[4/6] robot_cmd", payload)
    if payload.get("cmd") != "pick":
        return
    print(
        "[4/6] plan:",
        f"object={payload.get('object')}",
        f"u={payload.get('u')}",
        f"v={payload.get('v')}",
        f"x={payload.get('x_base_mm')}mm",
        f"y={payload.get('y_base_mm')}mm",
        f"theta={payload.get('theta_deg')}deg",
        f"z_safe={payload.get('z_safe_mm')}mm",
        f"z_grab={payload.get('z_grab_mm')}mm",
        f"open={payload.get('gripper_open_mm')}mm",
        f"close={payload.get('gripper_close_mm')}mm",
    )
    print("[5/6] planning only: ROS2/MoveIt six-axis request is not executed")


def chat_reply(user_text: str) -> str:
    local_reply = local_emotional_reply(user_text)
    if local_reply:
        return local_reply
    return cloud_chat(
        [
            {
                "role": "system",
                "content": (
                    "你是桌面机械臂助手小U。回答要简短自然。"
                    "用户骂你时不要开心，要委屈但礼貌。"
                    "用户难过时安慰。用户开心或夸你时开心回应。"
                ),
            },
            {"role": "user", "content": user_text},
        ]
    )


def handle_text(user_text: str, model: OpenCVDnnYolo, speak: bool) -> None:
    print(f"[2/6] ASR/text: {user_text}")
    emote_result = set_dialog_face(user_text)
    print_json("[2/6] dialog_emote", emote_result)
    set_emotion_from_text(user_text)

    set_face_state("thinking", "理解中")
    intent = ask_cloud_intent(user_text)
    print_json("[3/6] intent", intent)

    action = intent.get("action")
    obj = intent.get("object")

    if action in {"pick", "look"} and obj:
        set_face_state("searching", f"找{OBJECT_CN.get(obj, obj)}")
        payload = find_stable_target(model, obj).payload()
    elif action in {"stop", "home", "tidy"}:
        payload = {"cmd": action, "object": obj}
    else:
        payload = {"cmd": "chat", "object": obj}

    print_pick_summary(payload)

    if payload.get("cmd") == "pick":
        set_face_state("happy", f"找到{OBJECT_CN.get(obj or '', obj or '目标')}")
    elif payload.get("cmd") in {"target_lost", "not_found", "out_of_range", "camera_failed"}:
        set_face_state("error", "没找到目标")
    elif payload.get("cmd") == "stop":
        set_face_state("stop", "已停止")

    print("[5/6] planning only: execute through ROS2 MoveIt after measured calibration")

    if payload.get("cmd") == "chat":
        try:
            reply = chat_reply(user_text)
        except Exception as exc:
            reply = f"云端聊天失败：{exc}"
            set_face_state("error", "网络异常")
        set_dialog_emotion(user_text, reply)
    else:
        reply = robot_reply(payload, obj)

    print(f"[6/6] 小U: {reply}")
    if speak:
        set_face_state("speaking", "回复中")
        speak_text(reply)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--speak", action="store_true", help="voice reply")
    parser.add_argument("--text", help="skip microphone and test this text directly")
    args = parser.parse_args()

    print("=" * 60)
    print("小U一体化测试：语音/文字 -> 意图 -> YOLO -> 坐标 -> ROS2/MoveIt六轴规划请求")
    print("=" * 60)
    print("Loading YOLO...")
    model = OpenCVDnnYolo()
    print("Ready.")
    set_face_state("idle", "小U待机")

    wav_path = PROJECT_DIR / "voice_robot_input.wav"
    if args.text:
        handle_text(args.text, model, args.speak)
        return

    while True:
        value = input("\n回车开始说话；输入文字直接测试；q退出 > ").strip()
        if value.lower() in {"q", "quit", "exit"}:
            break
        try:
            if value:
                user_text = value
            else:
                set_face_state("listening", "我在听")
                record_wav(wav_path)
                set_face_state("thinking", "识别中")
                user_text = baidu_asr_wav(wav_path)
            handle_text(user_text, model, args.speak)
        except KeyboardInterrupt:
            raise
        except Exception as exc:
            set_face_state("error", "出错了")
            print("ERROR:", exc)
        time.sleep(0.2)


if __name__ == "__main__":
    main()
