import cv2

from common import PROJECT_DIR, get_camera_index, get_target_objects, get_yolo_model
from device_runtime import open_cv_camera
from yolo_opencv import OpenCVDnnYolo


MIN_BOX_AREA_RATIO = 0.010
SMALL_OBJECT_AREA_RATIO = 0.004
STABLE_FRAMES = 6
PEN_STABLE_FRAMES = 4
CLASS_CONF_OVERRIDES = {
    "cola": 0.55,
    "bottle": 0.55,
    "Bottle": 0.55,
    "cup": 0.50,
    "Coffee cup": 0.50,
    "earphone": 0.60,
    "headphone": 0.60,
    "headphones": 0.60,
    "pen": 0.22,
    "Pen": 0.22,
}
SMALL_OBJECTS = {"pen", "Pen"}


def main() -> None:
    model_name = get_yolo_model()
    targets = {item for item in get_target_objects() if item != "all"}
    print(f"Loading YOLO model: {model_name}")
    print("Backend: OpenCV DNN ONNX, stable demo mode")
    print(f"Target objects: {sorted(targets)}")
    model = OpenCVDnnYolo()

    cap = open_cv_camera(get_camera_index())
    if cap is None or not cap.isOpened():
        raise RuntimeError("Camera open failed.")

    last_frame = None
    stable_counts: dict[str, int] = {}
    print("Running YOLO. Press q to quit.")
    while True:
        ok, frame = cap.read()
        if not ok:
            print("Frame read failed.")
            break
        annotated = frame.copy()
        frame_area = frame.shape[0] * frame.shape[1]
        raw = []
        seen_names = set()
        for det in model.detect(frame):
            if det.name not in targets:
                continue
            if det.conf < CLASS_CONF_OVERRIDES.get(det.name, 0.0):
                continue
            min_area_ratio = SMALL_OBJECT_AREA_RATIO if det.name in SMALL_OBJECTS else MIN_BOX_AREA_RATIO
            if det.area < frame_area * min_area_ratio:
                continue
            raw.append(det)
            seen_names.add(det.name)

        for name in list(stable_counts):
            if name not in seen_names:
                stable_counts[name] = max(0, stable_counts[name] - 1)
        for name in seen_names:
            stable_counts[name] = stable_counts.get(name, 0) + 1

        detections = []
        for det in raw:
            min_frames = PEN_STABLE_FRAMES if det.name in SMALL_OBJECTS else STABLE_FRAMES
            if stable_counts.get(det.name, 0) < min_frames:
                continue
            detections.append((det.name, det.cx, det.cy, round(det.conf, 3)))
            cv2.rectangle(annotated, (det.x1, det.y1), (det.x2, det.y2), (0, 255, 0), 2)
            cv2.putText(annotated, f"{det.name} {det.cx},{det.cy}", (det.x1, max(30, det.y1 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        if detections:
            print(detections)
        last_frame = annotated
        try:
            cv2.imshow("yolo_detect", annotated)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        except cv2.error:
            break

    if last_frame is not None:
        out = PROJECT_DIR / "last_yolo.jpg"
        cv2.imwrite(str(out), last_frame)
        print(f"Saved frame: {out}")
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
