import argparse

from common import (
    ask_cloud_intent,
    command_json,
    print_json,
    serial_enabled,
)
from robot_protocol import send_robot_payload
from vision_targeting import find_stable_target
from yolo_opencv import OpenCVDnnYolo


def send_serial(payload: dict) -> None:
    send_robot_payload(payload)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--send-serial", action="store_true", help="send JSON command to STM32 UART")
    args = parser.parse_args()

    print("Loading YOLO...")
    model = OpenCVDnnYolo()
    print("Ready. Example commands: 帮我拿水杯 / 整理桌面 / 停止 / 回零")
    print("Type q to quit.")

    while True:
        user_text = input("> ").strip()
        if user_text.lower() in {"q", "quit", "exit"}:
            break

        intent = ask_cloud_intent(user_text)
        print_json("intent", intent)
        action = intent.get("action")
        obj = intent.get("object")

        if action == "pick" and obj:
            payload = find_stable_target(model, obj).payload()
        elif action == "tidy":
            payload = command_json("tidy", obj)
        elif action in {"stop", "home"}:
            payload = command_json(action, obj)
        else:
            payload = command_json(action or "chat", obj)

        print_json("robot_cmd", payload)
        if args.send_serial or serial_enabled():
            send_serial(payload)


if __name__ == "__main__":
    main()
