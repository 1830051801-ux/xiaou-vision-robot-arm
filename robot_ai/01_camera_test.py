import time

import cv2

from common import PROJECT_DIR
from device_runtime import open_cv_camera


def main() -> None:
    cap = open_cv_camera()
    if cap is None or not cap.isOpened():
        raise RuntimeError("Camera open failed. Check USB camera or CAMERA_INDEX in config.env.")

    print("Camera opened. Press q in the image window to quit.")
    last_frame = None
    for _ in range(300):
        ok, frame = cap.read()
        if not ok:
            print("Frame read failed.")
            time.sleep(0.1)
            continue
        last_frame = frame
        cv2.putText(frame, "camera ok", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        try:
            cv2.imshow("camera_test", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        except cv2.error:
            break

    if last_frame is not None:
        out = PROJECT_DIR / "last_camera.jpg"
        cv2.imwrite(str(out), last_frame)
        print(f"Saved frame: {out}")
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
