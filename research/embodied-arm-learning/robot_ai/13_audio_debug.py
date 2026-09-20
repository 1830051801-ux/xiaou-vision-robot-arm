from __future__ import annotations

import argparse
import subprocess
from pathlib import Path

import sounddevice as sd
import soundfile as sf

from common import PROJECT_DIR
from device_runtime import configure_sounddevice
from robot_ai_07_import import speak_text


def describe_devices() -> None:
    print("Audio devices:")
    for idx, info in enumerate(sd.query_devices()):
        if info.get("max_input_channels", 0) > 0 or info.get("max_output_channels", 0) > 0:
            print(
                f"  {idx}: {info['name']} | in={info.get('max_input_channels', 0)} "
                f"out={info.get('max_output_channels', 0)}"
            )


def play_wav(path: Path) -> None:
    try:
        subprocess.run(["aplay", str(path)], check=False)
    except FileNotFoundError:
        print("aplay not found, skipping file playback.")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=3.0)
    parser.add_argument("--speak", action="store_true")
    args = parser.parse_args()

    runtime_dir = PROJECT_DIR / "runtime"
    runtime_dir.mkdir(parents=True, exist_ok=True)

    input_id, output_id = configure_sounddevice()
    print(f"default input={input_id} output={output_id}")
    describe_devices()

    sample_rate = int(sd.default.samplerate or 16000)
    out = runtime_dir / "audio_debug.wav"
    print(f"Recording {args.seconds:.1f}s from microphone...")
    audio = sd.rec(int(args.seconds * sample_rate), samplerate=sample_rate, channels=1, dtype="float32")
    sd.wait()
    sf.write(out, audio, sample_rate, subtype="PCM_16")
    print(f"Saved recording: {out}")

    print("Playing back recorded audio...")
    play_wav(out)

    if args.speak:
        print("Speaking test phrase...")
        speak_text("音响测试完成")


if __name__ == "__main__":
    main()
