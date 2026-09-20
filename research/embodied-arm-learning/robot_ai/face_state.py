from __future__ import annotations

import json
import os
import time
import uuid

from common import PROJECT_DIR


RUNTIME_DIR = PROJECT_DIR / "runtime"
FACE_STATE_FILE = RUNTIME_DIR / "face_state.json"


def set_face_state(state: str, text: str = "") -> None:
    RUNTIME_DIR.mkdir(parents=True, exist_ok=True)
    payload = {
        "state": state,
        "text": text,
        "ts": time.time(),
    }
    tmp = FACE_STATE_FILE.with_name(f"{FACE_STATE_FILE.name}.{os.getpid()}.{uuid.uuid4().hex}.tmp")
    tmp.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")
    tmp.replace(FACE_STATE_FILE)


def get_face_state() -> dict:
    try:
        return json.loads(FACE_STATE_FILE.read_text(encoding="utf-8"))
    except Exception:
        return {"state": "idle", "text": "\u5c0fU\u5f85\u673a\u4e2d", "ts": 0}
