from __future__ import annotations

import argparse
import json
import time
import uuid
from pathlib import Path

from deskpet_common import PROJECT_DIR, append_event, speak, update_emotion, write_face_state


REMINDER_DB = PROJECT_DIR / "runtime" / "deskpet_reminders.json"


def load_reminders() -> list[dict]:
    if REMINDER_DB.exists():
        try:
            data = json.loads(REMINDER_DB.read_text(encoding="utf-8"))
            if isinstance(data, list):
                return data
        except Exception:
            pass
    return []


def save_reminders(items: list[dict]) -> None:
    REMINDER_DB.parent.mkdir(parents=True, exist_ok=True)
    REMINDER_DB.write_text(json.dumps(items, ensure_ascii=False, indent=2), encoding="utf-8")


def add_reminder(in_seconds: float, message: str, emote: str = "thinking") -> dict:
    items = load_reminders()
    item = {
        "id": str(uuid.uuid4()),
        "at_ts": time.time() + float(in_seconds),
        "message": message,
        "emote": emote,
        "created": time.time(),
    }
    items.append(item)
    save_reminders(items)
    return item


def trigger_reminder(item: dict) -> None:
    emote = str(item.get("emote") or "thinking")
    message = str(item.get("message") or "提醒时间到了")
    state = {
        "thinking": "thinking",
        "happy": "happy",
        "sad": "sad",
        "idle": "idle",
        "success": "happy",
        "celebrate": "happy",
    }.get(emote, "thinking")
    append_event("reminder", id=item.get("id"), message=message, emote=emote)
    update_emotion({"happy": 0})
    write_face_state(state, message)
    print("[scheduler] triggered:", message)
    speak(message)


def run_loop(poll_s: float = 1.0, once: bool = False) -> None:
    print("scheduler started:", REMINDER_DB)
    while True:
        items = load_reminders()
        now = time.time()
        remaining = []
        for item in items:
            if float(item.get("at_ts", 0)) <= now:
                trigger_reminder(item)
            else:
                remaining.append(item)
        if len(remaining) != len(items):
            save_reminders(remaining)
        if once:
            return
        time.sleep(poll_s)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--add", action="store_true")
    parser.add_argument("--in", dest="in_s", type=float)
    parser.add_argument("--message", default="提醒时间到了")
    parser.add_argument("--emote", default="thinking")
    parser.add_argument("--run", action="store_true")
    parser.add_argument("--once", action="store_true")
    parser.add_argument("--poll", type=float, default=1.0)
    args = parser.parse_args()

    if args.add:
        if args.in_s is None:
            raise SystemExit("Use --add --in seconds --message text")
        item = add_reminder(args.in_s, args.message, args.emote)
        print(json.dumps(item, ensure_ascii=False, indent=2))
        return
    if args.run or args.once:
        run_loop(args.poll, once=args.once)
        return
    parser.print_help()


if __name__ == "__main__":
    main()
