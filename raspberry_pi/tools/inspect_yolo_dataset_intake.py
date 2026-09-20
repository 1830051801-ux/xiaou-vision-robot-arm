#!/usr/bin/env python3
"""Audit an incoming YOLO image batch without merging or training it.

The source may contain ``images/**`` plus matching ``labels/**`` trees, or it
may be an image-only directory awaiting annotation.  The command validates the
versioned class contract, image readability, normalized boxes, label pairing,
and duplicate hashes.  It writes only a JSON report and never changes the
incoming batch or the active training dataset.
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path
import sys

import cv2
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SCHEMA = PROJECT_ROOT / "robot_ai/vision/config/yolo_class_schema_v1.json"
IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def _resolve(path: Path) -> Path:
    return path if path.is_absolute() else PROJECT_ROOT / path


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _decode(path: Path) -> np.ndarray | None:
    raw = np.fromfile(str(path), dtype=np.uint8)
    return cv2.imdecode(raw, cv2.IMREAD_COLOR) if raw.size else None


def _load_schema(path: Path) -> tuple[dict, list[str]]:
    schema = json.loads(path.read_text(encoding="utf-8"))
    classes = schema.get("classes")
    if not isinstance(classes, list) or not classes:
        raise ValueError("schema.classes must be a non-empty list")
    ids = [int(row["id"]) for row in classes]
    names = [str(row["name"]).strip() for row in classes]
    if ids != list(range(len(classes))) or any(not name for name in names) or len(set(names)) != len(names):
        raise ValueError("class IDs must be contiguous from zero and names must be unique")
    return schema, names


def _comparison_hashes(paths: list[Path]) -> dict[str, list[str]]:
    hashes: dict[str, list[str]] = defaultdict(list)
    for root in paths:
        if not root.exists():
            continue
        candidates = [root] if root.is_file() else root.rglob("*")
        for path in candidates:
            if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES:
                hashes[_sha256(path)].append(str(path.resolve()))
    return hashes


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--schema", type=Path, default=DEFAULT_SCHEMA)
    parser.add_argument("--batch-name", required=True)
    parser.add_argument("--mode", choices=("auto", "labeled", "unlabeled"), default="auto")
    parser.add_argument("--compare-images", default="", help="comma-separated files or directories for duplicate checks")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    source = _resolve(args.source).resolve()
    schema_path = _resolve(args.schema).resolve()
    if not source.is_dir() or not schema_path.is_file():
        raise FileNotFoundError(f"source={source}, schema={schema_path}")
    schema, class_names = _load_schema(schema_path)

    images_root = source / "images" if (source / "images").is_dir() else source
    labels_root = source / "labels" if (source / "labels").is_dir() else None
    images = sorted(path for path in images_root.rglob("*") if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES)
    if not images:
        raise FileNotFoundError(f"no supported images under {images_root}")

    compare_paths = [
        _resolve(Path(value.strip())).resolve()
        for value in args.compare_images.split(",")
        if value.strip()
    ]
    known_hashes = _comparison_hashes(compare_paths)
    batch_hashes: dict[str, list[str]] = defaultdict(list)
    errors: list[str] = []
    warnings: list[str] = []
    class_counts: Counter[str] = Counter()
    dimensions: Counter[str] = Counter()
    missing_labels: list[str] = []
    unreadable_images: list[str] = []
    rows = []

    for image_path in images:
        relative = image_path.relative_to(images_root)
        digest = _sha256(image_path)
        batch_hashes[digest].append(str(relative))
        frame = _decode(image_path)
        if frame is None:
            unreadable_images.append(str(relative))
            errors.append(f"unreadable image: {relative}")
            continue
        height, width = frame.shape[:2]
        dimensions[f"{width}x{height}"] += 1
        label_path = labels_root / relative.with_suffix(".txt") if labels_root is not None else None
        label_count = 0
        if label_path is None or not label_path.is_file():
            missing_labels.append(str(relative))
        else:
            for line_no, raw in enumerate(label_path.read_text(encoding="utf-8").splitlines(), 1):
                fields = raw.split()
                if not fields:
                    continue
                if len(fields) != 5:
                    errors.append(f"{label_path}:{line_no}: expected 5 fields")
                    continue
                try:
                    class_id = int(fields[0])
                    values = [float(value) for value in fields[1:]]
                except ValueError:
                    errors.append(f"{label_path}:{line_no}: non-numeric label")
                    continue
                if class_id < 0 or class_id >= len(class_names):
                    errors.append(f"{label_path}:{line_no}: class id {class_id} is outside schema")
                    continue
                if any(value < 0.0 or value > 1.0 for value in values) or values[2] <= 0.0 or values[3] <= 0.0:
                    errors.append(f"{label_path}:{line_no}: invalid normalized box")
                    continue
                class_counts[class_names[class_id]] += 1
                label_count += 1
        rows.append({
            "image": str(relative),
            "sha256": digest,
            "width": width,
            "height": height,
            "label_count": label_count,
        })

    internal_duplicates = [paths for paths in batch_hashes.values() if len(paths) > 1]
    known_duplicates = [
        {"incoming": incoming, "existing": known_hashes[digest]}
        for digest, incoming in batch_hashes.items()
        if digest in known_hashes
    ]
    if internal_duplicates:
        warnings.append(f"{len(internal_duplicates)} duplicate hash groups exist inside the batch")
    if known_duplicates:
        warnings.append(f"{len(known_duplicates)} incoming hash groups already exist in comparison data")

    inferred_mode = "labeled" if len(missing_labels) == 0 else "unlabeled"
    effective_mode = inferred_mode if args.mode == "auto" else args.mode
    if effective_mode == "labeled" and missing_labels:
        errors.append(f"labeled mode requires labels for all images; missing={len(missing_labels)}")
    if effective_mode == "unlabeled" and len(missing_labels) != len(images):
        warnings.append("unlabeled mode contains a mixture of labeled and unlabeled images")

    report = {
        "offline": True,
        "read_only_source": True,
        "hardware_motion": False,
        "camera_opened": False,
        "batch_name": args.batch_name,
        "source": str(source),
        "mode": effective_mode,
        "schema": {
            "path": str(schema_path),
            "name": schema.get("schema_name"),
            "version": schema.get("schema_version"),
            "status": schema.get("status"),
            "classes": class_names,
        },
        "image_count": len(images),
        "labeled_image_count": len(images) - len(missing_labels),
        "missing_label_count": len(missing_labels),
        "unreadable_image_count": len(unreadable_images),
        "class_instances": dict(class_counts),
        "dimensions": dict(dimensions),
        "internal_duplicate_groups": internal_duplicates,
        "known_duplicate_groups": known_duplicates,
        "errors": errors,
        "warnings": warnings,
        "training_ready": effective_mode == "labeled" and not errors and not internal_duplicates and not known_duplicates,
        "next_action": "label and re-audit" if effective_mode == "unlabeled" else "build a versioned split without duplicate leakage",
        "files": rows,
    }
    output = _resolve(args.output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if not errors else 1


if __name__ == "__main__":
    raise SystemExit(main())
