from __future__ import annotations

import sys

from common import PROJECT_DIR
from face_state import set_face_state


PACKAGE_DIR = PROJECT_DIR / "codex_emote_ai_package"
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))


EMOTE_TO_FACE_STATE = {
    "idle": "idle",
    "happy": "happy",
    "thankful": "happy",
    "apologize": "sad",
    "agree": "happy",
    "disagree": "stop",
    "thinking": "thinking",
    "confused": "thinking",
    "celebrate": "happy",
    "sad": "sad",
}


def set_dialog_face(text: str) -> dict:
    try:
        from dialog_manager import map_dialog_to_emote

        result = map_dialog_to_emote(text)
    except Exception as exc:
        result = {"intent": "idle", "emote": "idle", "loop": True, "priority": 0, "error": str(exc)}
    face_state = EMOTE_TO_FACE_STATE.get(result.get("emote", "idle"), "idle")
    set_face_state(face_state, f"{result.get('intent', 'idle')} / {result.get('emote', 'idle')}")
    return result
