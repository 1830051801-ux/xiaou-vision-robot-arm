from __future__ import annotations

import ast
import json
import hashlib
import os
import shutil
import tempfile
import urllib.request
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

try:
    from common import PROJECT_DIR, get_yolo_conf, get_yolo_imgsz, get_yolo_model
except ModuleNotFoundError:  # Package import from the workspace root.
    try:
        from robot_ai.common import PROJECT_DIR, get_yolo_conf, get_yolo_imgsz, get_yolo_model
    except ModuleNotFoundError:  # Minimal Pi archive without the voice/config stack.
        PROJECT_DIR = Path(__file__).resolve().parents[1]

        def get_yolo_conf() -> float:
            return 0.35

        def get_yolo_imgsz() -> int:
            return 640

        def get_yolo_model() -> str:
            return "xiaou_objects_gpu_deep.onnx"


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


def _parse_onnx_names_metadata(raw: str | None) -> list[str] | None:
    """Parse Ultralytics ``names`` metadata without executing its contents."""
    if not raw:
        return None
    try:
        value = ast.literal_eval(raw)
    except (SyntaxError, ValueError):
        try:
            value = json.loads(raw)
        except (TypeError, json.JSONDecodeError):
            return None
    if isinstance(value, dict):
        try:
            normalized = {int(key): str(name).strip() for key, name in value.items()}
        except (TypeError, ValueError):
            return None
        if sorted(normalized) != list(range(len(normalized))):
            return None
        names = [normalized[index] for index in range(len(normalized))]
    elif isinstance(value, (list, tuple)):
        names = [str(name).strip() for name in value]
    else:
        return None
    return names if names and all(names) and len(set(names)) == len(names) else None


def _load_onnx_metadata_names(model_path: Path) -> list[str] | None:
    """Read embedded class names when a deployable sidecar is unavailable.

    ``onnx`` is preferred on the desktop because it reads the protobuf without
    creating an inference session.  The Pi package has ``onnxruntime`` instead,
    so it remains a fallback.  This path is only used when ``.names`` is absent.
    """
    if model_path.suffix.lower() != ".onnx":
        return None
    metadata: dict[str, str] = {}
    try:
        import onnx  # type: ignore

        model = onnx.load(str(model_path), load_external_data=False)
        metadata = {entry.key: entry.value for entry in model.metadata_props}
    except Exception:
        try:
            import onnxruntime as ort  # type: ignore

            session = ort.InferenceSession(str(model_path), providers=["CPUExecutionProvider"])
            metadata = dict(session.get_modelmeta().custom_metadata_map)
            del session
        except Exception:
            return None
    return _parse_onnx_names_metadata(metadata.get("names"))

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

# Legacy names used only when neither a sidecar nor ONNX metadata is available.
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
    def __init__(
        self,
        model_path: str | Path | None = None,
        imgsz: int | None = None,
        conf_thres: float | None = None,
    ) -> None:
        self.imgsz = int(imgsz if imgsz is not None else get_yolo_imgsz())
        self.conf_thres = float(conf_thres if conf_thres is not None else get_yolo_conf())
        self.model_path = self._ensure_model(model_path)
        if self.model_path.name in {"yolov5n.onnx", "yolov5s.onnx"} and self.imgsz != 640:
            print(f"{self.model_path.name} expects 640 input. Forcing YOLO_IMAGE_SIZE=640.")
            self.imgsz = 640
        self.names = _load_sidecar_names(self.model_path) or _load_onnx_metadata_names(self.model_path) or COCO_NAMES
        self.net = self._read_net(self.model_path)
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

    @staticmethod
    def _read_net(path: Path):
        """Load ONNX and work around OpenCV's Windows Unicode path limitation."""
        try:
            return cv2.dnn.readNetFromONNX(str(path))
        except cv2.error:
            if os.name != "nt" or str(path).isascii():
                raise
            digest = hashlib.sha1(path.read_bytes()).hexdigest()[:16]
            cache_dir = Path(tempfile.gettempdir()) / "xiaou_onnx_ascii_cache"
            cache_dir.mkdir(parents=True, exist_ok=True)
            cached = cache_dir / f"{path.stem}_{digest}{path.suffix}"
            if not cached.exists():
                shutil.copy2(path, cached)
            return cv2.dnn.readNetFromONNX(str(cached))

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

    def _ensure_model(self, model_path: str | Path | None = None) -> Path:
        raw = model_path if model_path is not None else get_yolo_model()
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
