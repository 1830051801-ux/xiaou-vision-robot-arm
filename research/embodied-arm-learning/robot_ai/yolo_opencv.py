from __future__ import annotations

import json
import urllib.request
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from common import PROJECT_DIR, get_yolo_conf, get_yolo_imgsz, get_yolo_model


def _load_oiv7_names() -> list[str]:
    """Load Open Images V7 class names from runtime/oiv7_names.json.
    Falls back to COCO names if the file doesn't exist (legacy models)."""
    names_path = PROJECT_DIR / "runtime" / "oiv7_names.json"
    if names_path.exists():
        data = json.loads(names_path.read_text(encoding="utf-8"))
        # data is {"0": "name1", "1": "name2", ...}
        return [data[str(i)] for i in range(len(data))]
    # Fallback: original COCO names
    return COCO_NAMES_LEGACY


def _load_sidecar_names(model_path: Path) -> list[str] | None:
    candidates = [
        model_path.with_suffix(".names"),
        PROJECT_DIR / "models" / f"{model_path.stem}.names",
    ]
    for path in candidates:
        if not path.exists():
            continue
        names = [line.strip() for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
        if names:
            return names
    return None

COCO_NAMES_LEGACY = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
    "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
    "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
    "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
    "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
    "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
    "hair drier", "toothbrush",
]

# Active class names — loaded from model metadata or legacy fallback
COCO_NAMES = _load_oiv7_names()


MODEL_URLS = {
    "yolov5n.onnx": "https://github.com/ultralytics/yolov5/releases/download/v7.0/yolov5n.onnx",
    "yolov5s.onnx": "https://github.com/ultralytics/yolov5/releases/download/v7.0/yolov5s.onnx",
}


@dataclass
class Detection:
    name: str
    x1: int
    y1: int
    x2: int
    y2: int
    conf: float

    @property
    def cx(self) -> int:
        return (self.x1 + self.x2) // 2

    @property
    def cy(self) -> int:
        return (self.y1 + self.y2) // 2

    @property
    def area(self) -> int:
        return max(0, self.x2 - self.x1) * max(0, self.y2 - self.y1)


class OpenCVDnnYolo:
    def __init__(self) -> None:
        self.imgsz = get_yolo_imgsz()
        self.conf_thres = get_yolo_conf()
        self.model_path = self._ensure_model()
        if self.model_path.name in {"yolov5n.onnx", "yolov5s.onnx"} and self.imgsz != 640:
            print(f"{self.model_path.name} expects 640 input. Forcing YOLO_IMAGE_SIZE=640.")
            self.imgsz = 640
        self.names = _load_sidecar_names(self.model_path) or COCO_NAMES
        self.net = cv2.dnn.readNetFromONNX(str(self.model_path))
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

    def _letterbox(self, frame: np.ndarray) -> tuple[np.ndarray, float, int, int]:
        h, w = frame.shape[:2]
        scale = min(self.imgsz / w, self.imgsz / h)
        new_w = int(round(w * scale))
        new_h = int(round(h * scale))
        resized = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
        canvas = np.full((self.imgsz, self.imgsz, 3), 114, dtype=np.uint8)
        pad_x = (self.imgsz - new_w) // 2
        pad_y = (self.imgsz - new_h) // 2
        canvas[pad_y:pad_y + new_h, pad_x:pad_x + new_w] = resized
        return canvas, scale, pad_x, pad_y

    def _ensure_model(self) -> Path:
        raw = get_yolo_model()
        path = Path(raw)
        if not path.is_absolute():
            path = PROJECT_DIR / "models" / raw
        path.parent.mkdir(parents=True, exist_ok=True)
        if path.exists():
            return path
        url = MODEL_URLS.get(path.name)
        if not url:
            raise FileNotFoundError(f"Model not found: {path}")
        print(f"Downloading small ONNX model: {url}")
        urllib.request.urlretrieve(url, path)
        return path

    def detect(self, frame: np.ndarray) -> list[Detection]:
        h, w = frame.shape[:2]
        image, scale, pad_x, pad_y = self._letterbox(frame)
        blob = cv2.dnn.blobFromImage(
            image,
            1 / 255.0,
            (self.imgsz, self.imgsz),
            swapRB=True,
            crop=False,
        )
        self.net.setInput(blob)
        pred = self.net.forward()
        pred = np.squeeze(pred)
        if pred.ndim == 1:
            pred = pred.reshape(1, -1)
        if pred.ndim == 2 and pred.shape[0] < pred.shape[1] and pred.shape[0] <= 256:
            pred = pred.T
        if pred.ndim != 2 or pred.shape[1] < 6:
            return []

        boxes: list[list[int]] = []
        scores: list[float] = []
        class_ids: list[int] = []

        # YOLOv8 format: [cx,cy,w,h, class_0, class_1, ..., class_N]
        # No separate objectness — use max class score directly
        for row in pred:
            if pred.shape[1] == len(self.names) + 5:
                obj_conf = float(row[4])
                class_scores = row[5:]
                class_id = int(np.argmax(class_scores))
                score = obj_conf * float(class_scores[class_id])
            else:
                class_scores = row[4:]
                class_id = int(np.argmax(class_scores))
                score = float(class_scores[class_id])
            if score < self.conf_thres:
                continue

            cx, cy, bw, bh = row[0], row[1], row[2], row[3]
            x1 = int((cx - bw / 2 - pad_x) / scale)
            y1 = int((cy - bh / 2 - pad_y) / scale)
            box_w = int(bw / scale)
            box_h = int(bh / scale)
            boxes.append([x1, y1, box_w, box_h])
            scores.append(score)
            class_ids.append(class_id)

        keep = cv2.dnn.NMSBoxes(boxes, scores, self.conf_thres, 0.45)
        detections: list[Detection] = []
        if len(keep) == 0:
            return detections

        for idx in np.array(keep).flatten():
            x, y, bw, bh = boxes[int(idx)]
            class_id = class_ids[int(idx)]
            name = self.names[class_id] if class_id < len(self.names) else str(class_id)
            detections.append(
                Detection(
                    name=name,
                    x1=max(0, x),
                    y1=max(0, y),
                    x2=min(w - 1, x + bw),
                    y2=min(h - 1, y + bh),
                    conf=float(scores[int(idx)]),
                )
            )
        return detections
