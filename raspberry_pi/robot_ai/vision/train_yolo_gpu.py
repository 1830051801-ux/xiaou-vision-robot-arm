#!/usr/bin/env python3
"""Prepare the local camera dataset, train YOLO on CUDA, and export ONNX.

This is a desktop/offline tool.  It never opens the arm serial port and never
publishes a motion command.  The exported ONNX model is consumed by the
existing Raspberry Pi ``OpenCVDnnYolo`` path.

The current labels come from two old datasets that use these source IDs:
``0=pen, 2=bottle, 3=cola, 4=earphone``.  They are remapped to a compact
four-class detector.  There are currently no cup labels; the report says so
explicitly instead of claiming cup accuracy.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import random
import shutil
import sys
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_SOURCE = PROJECT_ROOT / "my_all_data"
DEFAULT_MODEL = PROJECT_ROOT / "my_objects_data" / "yolov8n.pt"
DEFAULT_DATASET = PROJECT_ROOT / "runtime" / "yolo_train" / "xiaou_objects_dataset"
DEFAULT_PROJECT = PROJECT_ROOT / "runtime" / "yolo_train"
DEFAULT_RUN_NAME = "xiaou_objects_gpu"
DEFAULT_EXPORT = PROJECT_ROOT / "models" / "xiaou_objects_gpu.onnx"

CLASS_NAMES = ("pen", "bottle", "cola", "earphone")
SOURCE_CLASS_MAP = {0: 0, 2: 1, 3: 2, 4: 3}
IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}

# Keep augmentation explicit so a training report is reproducible.  These
# values are conservative for tabletop objects: modest in-plane rotation and
# perspective, no vertical flip, and enough color/scale variation for camera
# exposure changes without inventing a new object geometry.
AUGMENTATION_CONFIG = {
    "hsv_h": 0.010,
    "hsv_s": 0.50,
    "hsv_v": 0.30,
    "degrees": 3.0,
    "translate": 0.05,
    "scale": 0.25,
    "shear": 1.0,
    "perspective": 0.0002,
    "flipud": 0.0,
    "fliplr": 0.50,
    "mosaic": 0.80,
    "mixup": 0.0,
    "copy_paste": 0.0,
}


@dataclass(frozen=True)
class Pair:
    image: Path
    label: Path
    rows: tuple[tuple[int, float, float, float, float], ...]


def _parse_label(path: Path) -> tuple[tuple[int, float, float, float, float], ...]:
    rows: list[tuple[int, float, float, float, float]] = []
    for line_no, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        text = raw.strip()
        if not text:
            continue
        fields = text.split()
        if len(fields) != 5:
            raise ValueError(f"{path}:{line_no}: expected 5 YOLO fields, got {len(fields)}")
        source_id = int(fields[0])
        if source_id not in SOURCE_CLASS_MAP:
            raise ValueError(
                f"{path}:{line_no}: unsupported source class {source_id}; "
                f"supported IDs are {sorted(SOURCE_CLASS_MAP)}"
            )
        values = tuple(float(value) for value in fields[1:])
        if any(value < 0.0 or value > 1.0 for value in values):
            raise ValueError(f"{path}:{line_no}: normalized box values must be in [0, 1]")
        if values[2] <= 0.0 or values[3] <= 0.0:
            raise ValueError(f"{path}:{line_no}: box width and height must be positive")
        rows.append((SOURCE_CLASS_MAP[source_id], *values))
    return tuple(rows)


def discover_pairs(source: Path) -> list[Pair]:
    image_dir = source / "images"
    label_dir = source / "labels"
    if not image_dir.is_dir() or not label_dir.is_dir():
        raise FileNotFoundError(f"Expected {image_dir} and {label_dir}")
    pairs: list[Pair] = []
    for image in sorted(image_dir.iterdir()):
        if image.suffix.lower() not in IMAGE_SUFFIXES:
            continue
        label = label_dir / f"{image.stem}.txt"
        if not label.is_file():
            raise FileNotFoundError(f"Missing label for {image.name}: {label}")
        pairs.append(Pair(image=image, label=label, rows=_parse_label(label)))
    if not pairs:
        raise ValueError(f"No images found in {image_dir}")
    return pairs


def _stratified_split(pairs: Iterable[Pair], val_ratio: float, seed: int) -> tuple[list[Pair], list[Pair]]:
    if not 0.05 <= val_ratio < 0.5:
        raise ValueError("val_ratio must be in [0.05, 0.5)")
    groups: dict[tuple[int, ...], list[Pair]] = defaultdict(list)
    for pair in pairs:
        key = tuple(sorted({row[0] for row in pair.rows}))
        groups[key].append(pair)
    rng = random.Random(seed)
    train: list[Pair] = []
    val: list[Pair] = []
    for group in groups.values():
        rng.shuffle(group)
        if len(group) == 1:
            train.extend(group)
            continue
        count = max(1, int(round(len(group) * val_ratio)))
        count = min(count, len(group) - 1)
        val.extend(group[:count])
        train.extend(group[count:])
    rng.shuffle(train)
    rng.shuffle(val)
    if not val:
        raise ValueError("The split produced no validation images")
    return train, val


def _write_yaml(path: Path, dataset_root: Path) -> None:
    root = dataset_root.resolve().as_posix()
    lines = [
        f"path: '{root}'",
        "train: images/train",
        "val: images/val",
        "names:",
    ]
    lines.extend(f"  {index}: {name}" for index, name in enumerate(CLASS_NAMES))
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _copy_split(pairs: Iterable[Pair], dataset_root: Path, split: str) -> Counter[str]:
    image_out = dataset_root / "images" / split
    label_out = dataset_root / "labels" / split
    image_out.mkdir(parents=True, exist_ok=True)
    label_out.mkdir(parents=True, exist_ok=True)
    counts: Counter[str] = Counter()
    for pair in pairs:
        shutil.copy2(pair.image, image_out / pair.image.name)
        lines = []
        for class_id, cx, cy, width, height in pair.rows:
            lines.append(f"{class_id} {cx:.6f} {cy:.6f} {width:.6f} {height:.6f}")
            counts[CLASS_NAMES[class_id]] += 1
        (label_out / pair.label.name).write_text("\n".join(lines) + "\n", encoding="utf-8")
    return counts


def prepare_dataset(source: Path, destination: Path, val_ratio: float, seed: int, force: bool) -> dict:
    source = source.resolve()
    destination = destination.resolve()
    pairs = discover_pairs(source)
    train, val = _stratified_split(pairs, val_ratio, seed)
    if destination.exists() and force:
        shutil.rmtree(destination)
    destination.mkdir(parents=True, exist_ok=True)
    train_counts = _copy_split(train, destination, "train")
    val_counts = _copy_split(val, destination, "val")
    yaml_path = destination / "dataset.yaml"
    _write_yaml(yaml_path, destination)
    report = {
        "source": str(source),
        "dataset": str(destination),
        "yaml": str(yaml_path),
        "classes": list(CLASS_NAMES),
        "source_class_map": {str(key): CLASS_NAMES[value] for key, value in SOURCE_CLASS_MAP.items()},
        "train_images": len(train),
        "val_images": len(val),
        "train_instances": dict(train_counts),
        "val_instances": dict(val_counts),
        "seed": seed,
        "val_ratio": val_ratio,
        "missing_classes": [name for name in ("cup",) if name not in CLASS_NAMES],
        "warning": "No cup labels are present in the current source dataset.",
    }
    (destination / "manifest.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return report


def _require_ultralytics():
    try:
        from ultralytics import YOLO  # type: ignore
    except Exception as exc:  # pragma: no cover - depends on desktop environment
        raise RuntimeError(
            "ultralytics is required in the GPU venv. Install with "
            "<venv>\\Scripts\\python.exe -m pip install ultralytics"
        ) from exc
    return YOLO


def train_and_export(args: argparse.Namespace, dataset_report: dict) -> dict:
    YOLO = _require_ultralytics()
    if args.checkpoint:
        best_path = Path(args.checkpoint).resolve()
        if not best_path.is_file():
            raise FileNotFoundError(f"Checkpoint not found: {best_path}")
        model_path = best_path
    else:
        model_path = Path(args.model).resolve()
        if not model_path.is_file():
            raise FileNotFoundError(f"Base model not found: {model_path}")
        model = YOLO(str(model_path))
        results = model.train(
            data=dataset_report["yaml"],
            epochs=args.epochs,
            imgsz=args.imgsz,
            batch=args.batch,
            device=args.device,
            workers=args.workers,
            project=str(Path(args.project).resolve()),
            name=args.name,
            exist_ok=True,
            patience=args.patience,
            optimizer=args.optimizer,
            amp=True,
            cache=False,
            plots=True,
            verbose=True,
            close_mosaic=min(args.close_mosaic, max(0, args.epochs - 1)),
            **AUGMENTATION_CONFIG,
        )
        best_path = Path(getattr(results, "save_dir", Path(args.project) / args.name)) / "weights" / "best.pt"
    if not best_path.is_file():
        raise FileNotFoundError(f"Training finished but best checkpoint is missing: {best_path}")
    if importlib.util.find_spec("onnx") is None:
        raise RuntimeError(
            "ONNX export requires the desktop dependency. Install on the GPU venv with "
            "python -m pip install onnx protobuf ml_dtypes"
        )
    best = YOLO(str(best_path))
    export_result = best.export(
        format="onnx",
        imgsz=args.imgsz,
        opset=12,
        simplify=False,
        dynamic=False,
        device=args.device,
    )
    exported = Path(str(export_result))
    if not exported.is_file():
        raise FileNotFoundError(f"Ultralytics export did not produce a file: {exported}")
    target = Path(args.export).resolve()
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(exported, target)
    names_path = target.with_suffix(".names")
    names_path.write_text("\n".join(CLASS_NAMES) + "\n", encoding="utf-8")
    try:
        import torch

        cuda_available = bool(torch.cuda.is_available())
        gpu_name = torch.cuda.get_device_name(0) if cuda_available else None
    except Exception:
        cuda_available = False
        gpu_name = None
    return {
        "dataset": dataset_report,
        "base_model": str(model_path),
        "best_checkpoint": str(best_path.resolve()),
        "onnx": str(target),
        "names": str(names_path),
        "device_requested": args.device,
        "cuda_available": cuda_available,
        "gpu_name": gpu_name,
        "epochs": args.epochs,
        "imgsz": args.imgsz,
        "batch": args.batch,
        "export_only": bool(args.checkpoint),
        "offline": True,
        "hardware_motion": False,
        "warning": "This detector has no cup training examples; collect and label cups before using cup as a target.",
        "augmentation": dict(AUGMENTATION_CONFIG),
        "augmentation_note": "Explicit tabletop-safe Ultralytics augmentation; vertical flips and copy-paste are disabled.",
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--dataset", type=Path, default=DEFAULT_DATASET)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--checkpoint", type=Path, help="skip training and export this .pt checkpoint")
    parser.add_argument("--project", type=Path, default=DEFAULT_PROJECT)
    parser.add_argument("--name", default=DEFAULT_RUN_NAME)
    parser.add_argument("--export", type=Path, default=DEFAULT_EXPORT)
    parser.add_argument("--val-ratio", type=float, default=0.2)
    parser.add_argument("--seed", type=int, default=20260809)
    parser.add_argument("--force", action="store_true", help="replace a prepared dataset with the same destination")
    parser.add_argument("--prepare-only", action="store_true")
    parser.add_argument("--export-only", action="store_true", help="alias for --checkpoint; kept for readable commands")
    parser.add_argument("--epochs", type=int, default=80)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--batch", type=int, default=4)
    parser.add_argument("--workers", type=int, default=0, help="0 is the stable Windows setting")
    parser.add_argument("--patience", type=int, default=20)
    parser.add_argument("--close-mosaic", type=int, default=10)
    parser.add_argument("--optimizer", default="AdamW")
    parser.add_argument("--device", default="0", help="CUDA device, e.g. 0; use cpu only for a deliberate fallback")
    args = parser.parse_args()
    if args.export_only and not args.checkpoint:
        parser.error("--export-only requires --checkpoint")
    if args.epochs < 1 or args.imgsz < 64 or args.batch < 1:
        parser.error("epochs, imgsz, and batch must be positive")
    report = prepare_dataset(args.source, args.dataset, args.val_ratio, args.seed, args.force)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    if args.prepare_only:
        return 0
    result = train_and_export(args, report)
    report_path = Path(args.project).resolve() / args.name / "gpu_training_report.json"
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
