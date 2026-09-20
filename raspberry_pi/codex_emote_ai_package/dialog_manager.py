from __future__ import annotations

import json
import re
from pathlib import Path
from typing import Any


PACKAGE_DIR = Path(__file__).resolve().parent
MAPPING_PATH = PACKAGE_DIR / "emote_mapping.json"


def load_mapping(path: str | Path = MAPPING_PATH) -> dict[str, Any]:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def _normalize(text: str) -> str:
    return " ".join(text.casefold().strip().split())


def _intent_from_json(text: str, mapping: dict[str, Any]) -> str | None:
    try:
        data = json.loads(text)
    except Exception:
        return None
    intent = str(data.get("intent", "")).strip()
    return intent if intent in mapping["intents"] else None


def _score_keyword(text: str, keyword: str) -> int:
    keyword = _normalize(keyword)
    if not keyword:
        return 0
    if keyword == text:
        return 1000 + len(keyword)
    if keyword.isascii() and keyword.replace(" ", "").isalpha():
        pattern = r"(?<![a-z0-9])" + re.escape(keyword) + r"(?![a-z0-9])"
        if re.search(pattern, text):
            return 500 + len(keyword)
    elif keyword in text:
        return 500 + len(keyword)
    # Basic stem-like support: "thanks" should match "thank".
    if keyword.endswith("s") and keyword[:-1] in text:
        return 300 + len(keyword)
    if keyword.endswith("ing") and keyword[:-3] in text:
        return 250 + len(keyword)
    return 0


def detect_intent(text: str, mapping: dict[str, Any] | None = None) -> str:
    mapping = mapping or load_mapping()
    json_intent = _intent_from_json(text, mapping)
    if json_intent:
        return json_intent

    normalized = _normalize(text)
    best_intent = "idle"
    best_score = -1
    for intent, keywords in mapping.get("keyword_map", {}).items():
        if intent not in mapping["intents"]:
            continue
        priority = int(mapping["intents"][intent].get("priority", 0))
        for keyword in keywords:
            match_score = _score_keyword(normalized, str(keyword))
            if match_score <= 0:
                continue
            score = priority * 10000 + match_score
            if score > best_score:
                best_score = score
                best_intent = intent
    return best_intent


def map_dialog_to_emote(text: str, mapping: dict[str, Any] | None = None) -> dict[str, Any]:
    """Rule-based dialog -> intent -> emote mapping.

    Extend behavior by editing emote_mapping.json:
    - intents controls emote, loop and priority.
    - keyword_map controls case-insensitive phrase matching.
    """
    mapping = mapping or load_mapping()
    intent = detect_intent(text, mapping)
    spec = mapping["intents"].get(intent, mapping["intents"]["idle"])
    return {
        "intent": intent,
        "emote": spec.get("emote", "idle"),
        "loop": bool(spec.get("loop", False)),
        "priority": int(spec.get("priority", 0)),
    }


if __name__ == "__main__":
    import sys

    query = " ".join(sys.argv[1:]) or "Hello"
    print(json.dumps(map_dialog_to_emote(query), ensure_ascii=False, indent=2))
