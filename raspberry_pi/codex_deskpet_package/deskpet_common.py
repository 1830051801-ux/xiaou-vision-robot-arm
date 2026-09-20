from __future__ import annotations

import json
import os
import subprocess
import time
from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parent
PROJECT_DIR = PACKAGE_DIR.parent
RUNTIME_DIR = PROJECT_DIR / "runtime"
FACE_STATE_FILE = RUNTIME_DIR / "face_state.json"
EVENT_LOG = RUNTIME_DIR / "deskpet_events.jsonl"
EMOTION_DB = RUNTIME_DIR / "deskpet_emotion.json"


def write_face_state(state: str, text: str = "") -> None:
    RUNTIME_DIR.mkdir(parents=True, exist_ok=True)
    payload = {"state": state, "text": text, "ts": time.time()}
    tmp = FACE_STATE_FILE.with_suffix(".tmp")
    tmp.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")
    tmp.replace(FACE_STATE_FILE)


def append_event(event: str, **kwargs) -> None:
    RUNTIME_DIR.mkdir(parents=True, exist_ok=True)
    payload = {"ts": time.time(), "event": event, **kwargs}
    with EVENT_LOG.open("a", encoding="utf-8") as file:
        file.write(json.dumps(payload, ensure_ascii=False) + "\n")


def speak(text: str) -> dict:
    if os.getenv("DESKPET_TTS", "false").strip().lower() not in {"1", "true", "yes", "on"}:
        return {"ok": True, "mode": "disabled"}
    commands = [
        ["espeak", "-s140", text],
        ["spd-say", text],
    ]
    for cmd in commands:
        try:
            result = subprocess.run(cmd, check=False, timeout=10, capture_output=True, text=True)
            if result.returncode == 0:
                return {"ok": True, "mode": cmd[0]}
        except Exception:
            continue
    return {"ok": False, "mode": "missing_tts", "hint": "Install espeak or set DESKPET_TTS=false."}


def load_emotion() -> dict:
    if EMOTION_DB.exists():
        try:
            return json.loads(EMOTION_DB.read_text(encoding="utf-8"))
        except Exception:
            pass
    return {"happy": 0, "sad": 0, "last": 0}


def update_emotion(delta: dict[str, int | float]) -> dict:
    RUNTIME_DIR.mkdir(parents=True, exist_ok=True)
    state = load_emotion()
    for key, value in delta.items():
        state[key] = float(state.get(key, 0)) + float(value)
    state["last"] = time.time()
    EMOTION_DB.write_text(json.dumps(state, ensure_ascii=False, indent=2), encoding="utf-8")
    return state
