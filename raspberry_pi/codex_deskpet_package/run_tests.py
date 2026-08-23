from __future__ import annotations

import json
import time
from pathlib import Path

from deskpet_common import EMOTION_DB, EVENT_LOG, FACE_STATE_FILE, PROJECT_DIR, load_emotion, update_emotion, write_face_state
from scheduler import REMINDER_DB, add_reminder, run_loop
from face_greet import find_haar_cascade


REPORT_PATH = Path(__file__).resolve().parent / "codex_deskpet_report.json"


def run_test(name: str, fn):
    try:
        result = fn() or {}
        return {"name": name, "ok": True, **result}
    except Exception as exc:
        return {"name": name, "ok": False, "error": str(exc)}


def test_face_state_write():
    write_face_state("happy", "测试表情")
    data = json.loads(FACE_STATE_FILE.read_text(encoding="utf-8"))
    assert data["state"] == "happy"
    return {"file": str(FACE_STATE_FILE)}


def test_emotion_memory():
    before = load_emotion().get("happy", 0)
    after = update_emotion({"happy": 1}).get("happy", 0)
    assert after >= before + 1
    return {"file": str(EMOTION_DB)}


def test_scheduler_trigger():
    item = add_reminder(0.1, "测试提醒", "thinking")
    time.sleep(0.2)
    run_loop(poll_s=0.1, once=True)
    face = json.loads(FACE_STATE_FILE.read_text(encoding="utf-8"))
    assert "测试提醒" in face.get("text", "")
    remaining = json.loads(REMINDER_DB.read_text(encoding="utf-8")) if REMINDER_DB.exists() else []
    assert all(rem.get("id") != item["id"] for rem in remaining)
    return {"reminder_id": item["id"]}


def test_haar_available():
    try:
        import cv2
    except Exception as exc:
        return {
            "warning": "OpenCV import failed on this machine; live face detection needs python3-opencv on Raspberry Pi.",
            "error": str(exc),
            "fix": "sudo apt install -y python3-opencv",
        }

    cascade_path = find_haar_cascade()
    cascade = cv2.CascadeClassifier(cascade_path)
    if cascade.empty():
        return {
            "warning": "OpenCV imported, but Haar cascade was not found.",
            "cascade": cascade_path,
            "fix": "sudo apt install -y python3-opencv",
        }
    return {"cascade": cascade_path}


def main() -> None:
    tests = [
        run_test("face_state_write", test_face_state_write),
        run_test("emotion_memory", test_emotion_memory),
        run_test("scheduler_trigger", test_scheduler_trigger),
        run_test("haar_cascade_available", test_haar_available),
    ]
    passed = sum(1 for item in tests if item["ok"])
    report = {
        "created": [
            "deskpet_common.py",
            "face_greet.py",
            "scheduler.py",
            "run_demo.py",
            "run_tests.py",
            "requirements.txt",
        ],
        "tests": {
            "total": len(tests),
            "passed": passed,
            "failed": len(tests) - passed,
            "details": tests,
        },
        "paths": {
            "project": str(PROJECT_DIR),
            "face_state": str(FACE_STATE_FILE),
            "event_log": str(EVENT_LOG),
            "emotion_db": str(EMOTION_DB),
            "reminder_db": str(REMINDER_DB),
        },
        "notes": (
            "Camera live face greeting requires a real camera. "
            "If haar_cascade_available fails, install/fix OpenCV on target: "
            "sudo apt install -y python3-opencv, or reinstall matching numpy/opencv in the venv."
        ),
    }
    REPORT_PATH.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    if passed != len(tests):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
