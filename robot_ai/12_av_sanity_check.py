from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import sounddevice as sd
import soundfile as sf

from common import PROJECT_DIR
from device_runtime import configure_sounddevice, open_cv_camera
from robot_ai_07_import import speak_text
from yolo_opencv import OpenCVDnnYolo


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--record-seconds", type=float, default=2.0)
    parser.add_argument("--speak", action="store_true")
    parser.add_argument("--yolo", action="store_true")
    args = parser.parse_args()

    runtime_dir = PROJECT_DIR / "runtime"
    runtime_dir.mkdir(parents=True, exist_ok=True)

    input_id, output_id = configure_sounddevice()
    print(f"sounddevice default input={input_id} output={output_id}")

    devices = sd.query_devices()
    chosen = {}
    if input_id is not None and 0 <= int(input_id) < len(devices):
        chosen["input"] = devices[int(input_id)]
    if output_id is not None and 0 <= int(output_id) < len(devices):
        chosen["output"] = devices[int(output_id)]
    print("audio_devices:", json.dumps(chosen, ensure_ascii=False, default=str))

    samples = int(args.record_seconds * int(sd.default.samplerate or 16000))
    audio = sd.rec(samples, samplerate=int(sd.default.samplerate or 16000), channels=1, dtype="float32")
    sd.wait()
    wav_path = runtime_dir / "av_test_input.wav"
    sf.write(wav_path, audio, int(sd.default.samplerate or 16000), subtype="PCM_16")
    print(f"saved_mic_test: {wav_path}")

    if args.speak:
        speak_text("XiaoU audio output test.")

    cap = open_cv_camera()
    if cap is None or not cap.isOpened():
        raise RuntimeError("Camera open failed in av sanity check.")

    ok, frame = cap.read()
    if not ok:
        cap.release()
        raise RuntimeError("Camera frame read failed in av sanity check.")

    cam_path = runtime_dir / "av_test_camera.jpg"
    cv2.imwrite(str(cam_path), frame)
    print(f"saved_camera_test: {cam_path}")

    if args.yolo:
        model = OpenCVDnnYolo()
        detections = []
        annotated = frame.copy()
        for det in model.detect(frame):
            detections.append(
                {
                    "name": det.name,
                    "conf": round(float(det.conf), 3),
                    "cx": int(det.cx),
                    "cy": int(det.cy),
                }
            )
            cv2.rectangle(annotated, (det.x1, det.y1), (det.x2, det.y2), (0, 255, 0), 2)
            cv2.putText(
                annotated,
                f"{det.name} {det.conf:.2f}",
                (det.x1, max(20, det.y1 - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
            )
        yolo_path = runtime_dir / "av_test_yolo.jpg"
        cv2.imwrite(str(yolo_path), annotated)
        print(f"saved_yolo_test: {yolo_path}")
        print("detections:", json.dumps(detections, ensure_ascii=False))

    cap.release()


if __name__ == "__main__":
    main()
