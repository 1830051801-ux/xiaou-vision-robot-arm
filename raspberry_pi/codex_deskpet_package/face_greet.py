from __future__ import annotations

import argparse
import os
import time

import cv2

from deskpet_common import append_event, speak, update_emotion, write_face_state


def find_haar_cascade() -> str:
    candidates: list[str] = []
    data = getattr(cv2, "data", None)
    haar_dir = getattr(data, "haarcascades", "") if data is not None else ""
    if haar_dir:
        candidates.append(haar_dir + "haarcascade_frontalface_default.xml")
    candidates.extend(
        [
            "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml",
            "/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml",
            "/usr/local/share/opencv4/haarcascades/haarcascade_frontalface_default.xml",
            "/usr/local/share/opencv/haarcascades/haarcascade_frontalface_default.xml",
        ]
    )
    for path in candidates:
        if os.path.exists(path):
            return path
    raise RuntimeError(
        "Cannot find haarcascade_frontalface_default.xml. "
        "Install it with: sudo apt install -y opencv-data python3-opencv"
    )


def detect_faces(frame, min_area: int) -> list[tuple[int, int, int, int]]:
    cascade_path = find_haar_cascade()
    cascade = cv2.CascadeClassifier(cascade_path)
    if cascade.empty():
        raise RuntimeError(f"Cannot load Haar cascade: {cascade_path}")
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.equalizeHist(gray)
    faces = cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(40, 40))
    return [tuple(map(int, face)) for face in faces if int(face[2]) * int(face[3]) >= min_area]


def face_greet_loop(
    camera: int = 0,
    cooldown_s: float = 8.0,
    min_area: int = 6500,
    greeting: str = "\u4f60\u597d\uff0c\u6211\u662f\u5c0fU\uff0c\u770b\u5230\u4f60\u5566\u3002",
    once: bool = False,
) -> None:
    cap = cv2.VideoCapture(int(camera))
    if not cap.isOpened():
        print(f"face_greet skipped: cannot open camera {camera}")
        return

    last_greet = 0.0
    write_face_state("idle", "\u5c0fU\u5f85\u673a")
    print("face_greet started")
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.1)
                continue
            try:
                faces = detect_faces(frame, min_area=min_area)
            except Exception as exc:
                print(f"face_greet skipped: {exc}")
                return
            now = time.time()
            if faces and now - last_greet >= cooldown_s:
                last_greet = now
                largest = max(faces, key=lambda item: item[2] * item[3])
                append_event("face_greet", faces=len(faces), largest_area=largest[2] * largest[3])
                update_emotion({"happy": 1})
                write_face_state("happy", "\u4f60\u597d\u5440")
                print(f"face_greet: faces={len(faces)} area={largest[2] * largest[3]}")
                speak(greeting)
                if once:
                    return
            time.sleep(0.05)
    finally:
        cap.release()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", type=int, default=int(os.getenv("CAMERA_INDEX", "0")))
    parser.add_argument("--cooldown", type=float, default=float(os.getenv("FACE_GREET_COOLDOWN_S", "8")))
    parser.add_argument("--min-area", type=int, default=int(os.getenv("FACE_GREET_MIN_AREA", "6500")))
    parser.add_argument("--greeting", default=os.getenv("FACE_GREET_TEXT", "\u4f60\u597d\uff0c\u6211\u662f\u5c0fU\uff0c\u770b\u5230\u4f60\u5566\u3002"))
    parser.add_argument("--once", action="store_true")
    args = parser.parse_args()
    face_greet_loop(args.camera, args.cooldown, args.min_area, args.greeting, args.once)


if __name__ == "__main__":
    main()
