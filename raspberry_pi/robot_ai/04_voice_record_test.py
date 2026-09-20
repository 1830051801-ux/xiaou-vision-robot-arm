from pathlib import Path

import sounddevice as sd
import soundfile as sf

from common import PROJECT_DIR


def main() -> None:
    seconds = 5
    sample_rate = 16000
    out = PROJECT_DIR / "voice_test.wav"
    print(f"Recording {seconds}s audio...")
    audio = sd.rec(int(seconds * sample_rate), samplerate=sample_rate, channels=1, dtype="float32")
    sd.wait()
    sf.write(out, audio, sample_rate)
    print(f"Saved: {out}")
    print("This test only checks microphone recording. Speech-to-text can be added after camera and cloud chat work.")


if __name__ == "__main__":
    main()
