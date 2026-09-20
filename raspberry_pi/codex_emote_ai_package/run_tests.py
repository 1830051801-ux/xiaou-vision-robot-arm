from __future__ import annotations

import json
import os
from pathlib import Path

from dialog_manager import map_dialog_to_emote
from emote_player import EMOTE_TO_ASSET, asset_path_for_emote, play_emote


PACKAGE_DIR = Path(__file__).resolve().parent
REPORT_PATH = PACKAGE_DIR / "codex_emote_report.json"

TESTS = [
    ("Hello", "greeting", "happy"),
    ("Hi, how are you?", "greeting", "happy"),
    ("Thanks a lot", "thanks", "thankful"),
    ("Sorry, my bad", "sorry", "apologize"),
    ("Yes, please do it", "confirm", "agree"),
    ("No, not that one", "deny", "disagree"),
    ("Wait, I'm thinking", "thinking", "thinking"),
    ("What do you mean?", "confused", "confused"),
    ("Done, finished", "success", "celebrate"),
    ("I can't do that", "fail", "sad"),
]


def main() -> None:
    details = []
    missing = []
    playback_modes = set()
    headless = not bool(os.getenv("DISPLAY"))

    for text, expected_intent, expected_emote in TESTS:
        actual = map_dialog_to_emote(text)
        playback = play_emote(actual["emote"], actual["loop"], headless=headless)
        playback_modes.add(playback.get("mode", "unknown"))
        passed = actual["intent"] == expected_intent and actual["emote"] == expected_emote
        reason = ""
        if not passed:
            reason = "intent/emote mismatch; check keyword_map priority or missing phrase"
        if playback.get("mode") == "missing_asset":
            missing.append(actual["emote"])
        details.append(
            {
                "input": text,
                "expected_intent": expected_intent,
                "actual_intent": actual["intent"],
                "pass": bool(passed),
                "expected_emote": expected_emote,
                "actual_emote": actual["emote"],
                "playback": playback,
                "reason": reason,
            }
        )

    # Extra acceptance check: LLM JSON intent input.
    json_check = map_dialog_to_emote('{"intent":"greeting"}')
    if json_check["intent"] != "greeting":
        details.append(
            {
                "input": '{"intent":"greeting"}',
                "expected_intent": "greeting",
                "actual_intent": json_check["intent"],
                "pass": False,
                "expected_emote": "happy",
                "actual_emote": json_check["emote"],
                "reason": "LLM JSON intent parsing failed",
            }
        )

    passed_count = sum(1 for item in details[: len(TESTS)] if item["pass"])
    failed_count = len(TESTS) - passed_count

    expected_emotes = sorted({emote for _, _, emote in TESTS})
    missing_emotes = sorted({emote for emote in expected_emotes if asset_path_for_emote(emote) is None})
    missing = sorted(set(missing + missing_emotes))

    notes = []
    if headless:
        notes.append("No DISPLAY detected; playback used headless fallback and did not open a window.")
    if missing:
        notes.append("Missing emote assets: " + ", ".join(missing))
    if not missing:
        notes.append("All required test emotes resolve to existing GIF/PNG assets.")
    notes.append("Playback modes: " + ", ".join(sorted(playback_modes)))

    report = {
        "created": [
            "emote_mapping.json",
            "dialog_manager.py",
            "emote_player.py",
            "run_demo.py",
            "run_tests.py",
            "requirements.txt",
        ],
        "tests": {
            "total": len(TESTS),
            "passed": passed_count,
            "failed": failed_count,
            "details": details,
        },
        "notes": " ".join(notes),
    }
    REPORT_PATH.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    if passed_count < 8:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
