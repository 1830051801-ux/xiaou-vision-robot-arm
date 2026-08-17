from __future__ import annotations

import json

from dialog_manager import map_dialog_to_emote
from emote_player import play_emote


def main() -> None:
    samples = [
        "Hello",
        "Thanks a lot",
        "Sorry, my bad",
        "Done, finished",
        '{"intent":"greeting"}',
    ]
    for text in samples:
        result = map_dialog_to_emote(text)
        playback = play_emote(result["emote"], result["loop"])
        print(json.dumps({"input": text, "result": result, "playback": playback}, ensure_ascii=False))


if __name__ == "__main__":
    main()
