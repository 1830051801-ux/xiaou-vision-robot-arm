from __future__ import annotations

from pathlib import Path

import sounddevice as sd
import soundfile as sf

from baidu_asr import baidu_asr_wav
from common import PROJECT_DIR
from emotion_state import local_emotional_reply, set_dialog_emotion, set_emotion_from_text
from face_state import set_face_state
from robot_ai_07_import import cloud_chat, speak_text


SAMPLE_RATE = 16000
RECORD_SECONDS = 5


def record_wav(path: Path) -> None:
    print(f"Recording {RECORD_SECONDS}s. Speak now...")
    audio = sd.rec(int(RECORD_SECONDS * SAMPLE_RATE), samplerate=SAMPLE_RATE, channels=1, dtype="float32")
    sd.wait()
    sf.write(path, audio, SAMPLE_RATE, subtype="PCM_16")


def main() -> None:
    print("小U中文语音AI对话。按回车开始录音，输入 q 退出。")
    messages = [
        {
            "role": "system",
            "content": (
                "你是桌面机械臂助手小U。回答要短、自然、有情绪。"
                "如果用户难过，要安慰；如果用户开心，要一起开心；如果用户骂你，不要反击，委屈但礼貌。"
                "如果用户让你拿物品，只确认你听到了，不要说已经完成。"
            ),
        }
    ]
    wav_path = PROJECT_DIR / "voice_ai_input.wav"
    while True:
        cmd = input("Enter to speak/q> ").strip().lower()
        if cmd in {"q", "quit", "exit"}:
            break
        try:
            set_face_state("listening", "我在听")
            record_wav(wav_path)
            set_face_state("thinking", "正在识别")
            text = baidu_asr_wav(wav_path)
            print(f"ASR: {text}")
            set_emotion_from_text(text)
        except Exception as exc:
            set_face_state("error", "没有听清")
            print(f"ASR failed: {exc}")
            continue

        messages.append({"role": "user", "content": text})
        reply = local_emotional_reply(text)
        if reply is None:
            try:
                reply = cloud_chat(messages)
            except Exception as exc:
                reply = f"Cloud AI request failed: {exc}"
                set_face_state("error", "网络不太舒服")
        messages.append({"role": "assistant", "content": reply})
        print(f"AI: {reply}")
        set_dialog_emotion(text, reply)
        speak_text(reply)
        set_dialog_emotion(text, reply)


if __name__ == "__main__":
    main()
