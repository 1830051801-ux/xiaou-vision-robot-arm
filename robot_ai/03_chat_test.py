import json

from common import ask_cloud_intent


def main() -> None:
    print("Input Chinese command. Example: 帮我拿水杯")
    print("Type q to quit.")
    while True:
        text = input("> ").strip()
        if text.lower() in {"q", "quit", "exit"}:
            break
        result = ask_cloud_intent(text)
        print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
