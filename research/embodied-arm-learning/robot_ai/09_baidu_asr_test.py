from pathlib import Path

import sounddevice as sd
import soundfile as sf

from common import PROJECT_DIR
from baidu_asr import baidu_asr_wav


def main() -> None:
    seconds = 5
    sample_rate = 16000
    out = PROJECT_DIR / "baidu_asr_test.wav"
    print(f"录音 {seconds} 秒，请说中文...")
    audio = sd.rec(int(seconds * sample_rate), samplerate=sample_rate, channels=1, dtype="float32")
    sd.wait()
    sf.write(out, audio, sample_rate, subtype="PCM_16")
    print(f"已保存: {out}")
    print("正在调用百度ASR...")
    text = baidu_asr_wav(out)
    print(f"识别结果: {text}")


if __name__ == "__main__":
    main()
