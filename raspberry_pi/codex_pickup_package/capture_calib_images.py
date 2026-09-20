from __future__ import annotations

import argparse
from pathlib import Path

import cv2


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--outdir", default="calib_images")
    parser.add_argument("--prefix", default="calib")
    args = parser.parse_args()

    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        raise SystemExit(f"Cannot open camera index {args.camera}")

    print("Press s to save frame, q to quit.")
    count = len(list(outdir.glob(f"{args.prefix}_*.jpg")))
    while True:
        ok, frame = cap.read()
        if not ok:
            print("camera read failed")
            break
        cv2.putText(frame, f"saved={count}  s=save  q=quit", (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.imshow("calibration capture", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("s"):
            path = outdir / f"{args.prefix}_{count:03d}.jpg"
            cv2.imwrite(str(path), frame)
            print("saved", path)
            count += 1
        elif key == ord("q"):
            break
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
