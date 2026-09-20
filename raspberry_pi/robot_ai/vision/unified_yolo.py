#!/usr/bin/env python3
"""One detection contract for desktop GPU and Raspberry Pi ONNX YOLO models.

Model selection is deliberately explicit.  A comma-separated model list runs
an ensemble and merges overlapping boxes by canonical class name; it does not
send commands to ROS or to the arm.  ``.pt`` models use Ultralytics on the
desktop GPU, while ``.onnx`` models use the existing OpenCV DNN CPU backend.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np

try:
    from common import get_yolo_conf, get_yolo_imgsz, get_yolo_model
    from yolo_opencv import Detection, OpenCVDnnYolo
except ModuleNotFoundError:  # Package import from the workspace root.
    try:
        from robot_ai.common import get_yolo_conf, get_yolo_imgsz, get_yolo_model
        from robot_ai.yolo_opencv import Detection, OpenCVDnnYolo
    except ModuleNotFoundError:  # Minimal Pi archive without the voice/config stack.
        from robot_ai.yolo_opencv import Detection, OpenCVDnnYolo, get_yolo_conf, get_yolo_imgsz, get_yolo_model


ALIASES = {
    "bottle": "bottle",
    "bottles": "bottle",
    "water bottle": "bottle",
    "cola": "cola",
    "coke": "cola",
    "cup": "cup",
    "coffee cup": "cup",
    "mug": "cup",
    "pen": "pen",
    "pencil": "pen",
    "earphone": "earphone",
    "headphone": "earphone",
    "headphones": "earphone",
}


def canonical_name(name: str) -> str:
    key = " ".join(str(name).strip().lower().split())
    return ALIASES.get(key, key)


def _iou(a: Detection, b: Detection) -> float:
    x1 = max(a.x1, b.x1)
    y1 = max(a.y1, b.y1)
    x2 = min(a.x2, b.x2)
    y2 = min(a.y2, b.y2)
    overlap = max(0, x2 - x1) * max(0, y2 - y1)
    union = a.area + b.area - overlap
    return overlap / union if union > 0 else 0.0


class _Backend:
    name = "unknown"
    supported_classes: set[str] = set()

    def detect(self, frame: np.ndarray) -> list[Detection]:
        raise NotImplementedError


class _OpenCVBackend(_Backend):
    name = "opencv-onnx-cpu"

    def __init__(self, path: Path, imgsz: int, conf: float) -> None:
        self.model = OpenCVDnnYolo(model_path=path, imgsz=imgsz, conf_thres=conf)
        self.supported_classes = {canonical_name(name) for name in self.model.names}

    def detect(self, frame: np.ndarray) -> list[Detection]:
        return self.model.detect(frame)


class _UltralyticsBackend(_Backend):
    name = "ultralytics-pytorch"

    def __init__(self, path: Path, imgsz: int, conf: float, device: str) -> None:
        try:
            from ultralytics import YOLO  # type: ignore
        except Exception as exc:  # pragma: no cover - desktop dependency
            raise RuntimeError(".pt inference requires ultralytics in the desktop GPU venv") from exc
        self.model = YOLO(str(path))
        self.imgsz = imgsz
        self.conf = conf
        self.device = device
        names = getattr(self.model, "names", {})
        self.supported_classes = {
            canonical_name(str(name)) for name in (names.values() if isinstance(names, dict) else names)
        }

    def detect(self, frame: np.ndarray) -> list[Detection]:
        results = self.model.predict(
            source=frame,
            imgsz=self.imgsz,
            conf=self.conf,
            device=self.device,
            verbose=False,
            stream=False,
        )
        if not results:
            return []
        result = results[0]
        boxes = getattr(result, "boxes", None)
        if boxes is None:
            return []
        names = getattr(result, "names", None) or getattr(self.model, "names", {})
        output: list[Detection] = []
        for xyxy, conf, class_id in zip(boxes.xyxy, boxes.conf, boxes.cls):
            coords = [int(round(float(value))) for value in xyxy.tolist()]
            index = int(class_id.item())
            raw_name = names.get(index, str(index)) if isinstance(names, dict) else names[index]
            output.append(
                Detection(
                    name=canonical_name(str(raw_name)),
                    x1=max(0, coords[0]),
                    y1=max(0, coords[1]),
                    x2=min(frame.shape[1] - 1, coords[2]),
                    y2=min(frame.shape[0] - 1, coords[3]),
                    conf=float(conf.item()),
                )
            )
        return output


def _resolve_path(raw: str | Path) -> Path:
    path = Path(raw)
    if not path.is_absolute():
        try:
            from common import PROJECT_DIR
        except ModuleNotFoundError:
            try:
                from robot_ai.common import PROJECT_DIR
            except ModuleNotFoundError:
                PROJECT_DIR = Path(__file__).resolve().parents[2]

        project_relative = PROJECT_DIR / path
        path = project_relative if project_relative.exists() else PROJECT_DIR / "models" / path
    return path


@dataclass
class UnifiedYolo:
    """Backend-neutral detector with optional model-list ensembling."""

    backends: list[_Backend]
    iou_merge: float = 0.55
    fallback_uncovered_classes: bool = True

    @classmethod
    def from_profile(
        cls,
        profile_name: str | None = None,
        *,
        registry_path: str | Path | None = None,
        imgsz: int | None = None,
        conf: float | None = None,
        device: str = "cpu",
        iou_merge: float = 0.55,
        fallback_uncovered_classes: bool = True,
    ) -> "UnifiedYolo":
        try:
            from .model_registry import DEFAULT_REGISTRY, resolve_profile
        except ImportError:  # Imported as ``vision.unified_yolo`` on the Pi.
            from vision.model_registry import DEFAULT_REGISTRY, resolve_profile

        profile = resolve_profile(profile_name, registry_path=registry_path or DEFAULT_REGISTRY)
        return cls.from_config(
            str(profile.model_path),
            imgsz=imgsz,
            conf=profile.confidence if conf is None else conf,
            device=device,
            iou_merge=iou_merge,
            fallback_uncovered_classes=fallback_uncovered_classes,
        )

    @classmethod
    def from_config(
        cls,
        model_spec: str | None = None,
        *,
        imgsz: int | None = None,
        conf: float | None = None,
        device: str = "cpu",
        iou_merge: float = 0.55,
        fallback_uncovered_classes: bool = True,
    ) -> "UnifiedYolo":
        raw_spec = model_spec if model_spec is not None else get_yolo_model()
        paths = [_resolve_path(item.strip()) for item in str(raw_spec).split(",") if item.strip()]
        if not paths:
            raise ValueError("YOLO model specification is empty")
        size = int(imgsz if imgsz is not None else get_yolo_imgsz())
        threshold = float(conf if conf is not None else get_yolo_conf())
        backends: list[_Backend] = []
        for path in paths:
            if not path.is_file():
                raise FileNotFoundError(f"YOLO model not found: {path}")
            suffix = path.suffix.lower()
            if suffix == ".onnx":
                backends.append(_OpenCVBackend(path, size, threshold))
            elif suffix in {".pt", ".pth"}:
                backends.append(_UltralyticsBackend(path, size, threshold, device))
            else:
                raise ValueError(f"Unsupported YOLO model type: {path.suffix}")
        return cls(
            backends=backends,
            iou_merge=iou_merge,
            fallback_uncovered_classes=fallback_uncovered_classes,
        )

    @property
    def backend_names(self) -> list[str]:
        return [backend.name for backend in self.backends]

    def detect(self, frame: np.ndarray) -> list[Detection]:
        candidates: list[Detection] = []
        covered_classes: set[str] = set()
        for index, backend in enumerate(self.backends):
            detections = backend.detect(frame)
            if index == 0 or not self.fallback_uncovered_classes:
                candidates.extend(detections)
            else:
                candidates.extend(
                    item for item in detections if canonical_name(item.name) not in covered_classes
                )
            covered_classes.update(backend.supported_classes)
        candidates.sort(key=lambda item: item.conf, reverse=True)
        merged: list[Detection] = []
        for detection in candidates:
            normalized = Detection(
                name=canonical_name(detection.name),
                x1=detection.x1,
                y1=detection.y1,
                x2=detection.x2,
                y2=detection.y2,
                conf=detection.conf,
            )
            if any(item.name == normalized.name and _iou(item, normalized) >= self.iou_merge for item in merged):
                continue
            merged.append(normalized)
        return merged


__all__ = ["UnifiedYolo", "canonical_name"]
