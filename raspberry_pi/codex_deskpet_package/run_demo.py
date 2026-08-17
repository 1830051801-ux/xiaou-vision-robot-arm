from __future__ import annotations

import argparse
import os
import threading
import time

from deskpet_common import write_face_state
from face_greet import face_greet_loop
from scheduler import add_reminder, run_loop


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--face", action="store_true", help="Enable face greeting if camera is free")
    parser.add_argument("--sample-reminder", action="store_true")
    args = parser.parse_args()

    write_face_state("idle", "\u5c0fU\u5f85\u673a")
    if args.sample_reminder:
        add_reminder(30, "\u8be5\u4f11\u606f\u4e00\u4e0b\u5566", "thinking")

    threads: list[threading.Thread] = []
    face_enabled = args.face or os.getenv("DESKPET_FACE_ENABLED", "false").strip().lower() in {"1", "true", "yes", "on"}
    if face_enabled:
        threads.append(threading.Thread(target=face_greet_loop, kwargs={"camera": args.camera}, daemon=True))
    threads.append(threading.Thread(target=run_loop, daemon=True))

    for thread in threads:
        thread.start()

    print("deskpet demo started. Ctrl+C to stop.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("deskpet demo stopped")


if __name__ == "__main__":
    main()
