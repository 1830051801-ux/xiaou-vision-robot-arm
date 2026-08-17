#!/usr/bin/env python3
"""Validate YOLO images, labels, class coverage, and dataset YAML offline.

The checker is deliberately independent of Ultralytics.  It catches stale
class maps, orphan files, malformed normalized boxes, unreadable images, and
classes that are named but have no actual labels.  A missing class is a
warning by default because an incomplete dataset can still be useful for a
focused detector; pass ``--require-classes`` to make it a hard failure.
"""

from __future__ import annotations

import argparse
import json
import math
from collections import Counter
from pathlib import Path
import sys
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[1]
IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def _load_yaml(path: Path) -> dict[str, Any]:
    try:
        import yaml  # type: ignore

        value = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        if not isinstance(value, dict):
            raise ValueError("dataset YAML must contain a mapping")
        return value
    except ImportError:
        # The Pi does not need this tool, but a tiny parser keeps the checker
        # usable in a minimal desktop environment.
        data: dict[str, Any] = {}
        names: dict[int, str] = {}
        for raw in path.read_text(encoding="utf-8").splitlines():
            line = raw.strip()
            if not line or line.startswith("#") or ":" not in line:
                continue
            key, value = line.split(":", 1)
            key = key.strip()
            value = value.strip().strip("'\"")
            if key.isdigit():
                names[int(key)] = value
            elif key == "names":
                continue
            else:
                data[key] = value
        if names:
            data["names"] = names
        return data


def _names(raw: Any) -> list[str]:
    if isinstance(raw, list):
        return [str(item) for item in raw]
    if isinstance(raw, dict):
        pairs = sorted((int(key), str(value)) for key, value in raw.items())
        if pairs and [key for key, _ in pairs] != list(range(len(pairs))):
            raise ValueError("class IDs in names must be contiguous from zero")
        return [value for _, value in pairs]
    raise ValueError("dataset YAML has no usable names mapping")


def _resolve_root(dataset_yaml: Path, value: Any) -> Path:
    path = Path(str(value))
    if path.is_absolute():
        return path.resolve()
    return (dataset_yaml.parent / path).resolve()


def _split_values(value: Any) -> list[Any]:
    if isinstance(value, list):
        return value
    return [part.strip() for part in str(value).split(",") if part.strip()]


def _image_label_dirs(image_dir: Path) -> tuple[Path, Path]:
    parts = list(image_dir.parts)
    try:
        index = next(i for i, item in enumerate(parts) if item.lower() == "images")
    except StopIteration:
        return image_dir, image_dir.parent / "labels"
    label_parts = parts[:]
    label_parts[index] = "labels"
    return image_dir, Path(*label_parts)


def _decode_ok(path: Path) -> bool:
    try:
        import cv2  # type: ignore
        import numpy as np  # type: ignore

        raw = np.fromfile(str(path), dtype=np.uint8)
        return bool(raw.size and cv2.imdecode(raw, cv2.IMREAD_COLOR) is not None)
    except ImportError:
        return path.stat().st_size > 0


def _validate_label(path: Path, class_count: int) -> tuple[Counter[int], list[str]]:
    counts: Counter[int] = Counter()
    errors: list[str] = []
    for line_no, raw in enumerate(path.read_text(encoding="utf-8-sig").splitlines(), 1):
        text = raw.strip()
        if not text:
            continue
        fields = text.split()
        if len(fields) != 5:
            errors.append(f"{path}:{line_no}: expected 5 fields, got {len(fields)}")
            continue
        try:
            class_id = int(fields[0])
            values = [float(item) for item in fields[1:]]
        except ValueError:
            errors.append(f"{path}:{line_no}: class or box is not numeric")
            continue
        if not 0 <= class_id < class_count:
            errors.append(f"{path}:{line_no}: class {class_id} outside [0, {class_count - 1}]")
        if any(not math.isfinite(item) for item in values):
            errors.append(f"{path}:{line_no}: non-finite box value")
        elif any(item < 0.0 or item > 1.0 for item in values):
            errors.append(f"{path}:{line_no}: normalized box value outside [0, 1]")
        elif values[2] <= 0.0 or values[3] <= 0.0:
            errors.append(f"{path}:{line_no}: width and height must be positive")
        elif values[0] - values[2] / 2.0 < -1e-6 or values[0] + values[2] / 2.0 > 1.000001 or values[1] - values[3] / 2.0 < -1e-6 or values[1] + values[3] / 2.0 > 1.000001:
            errors.append(f"{path}:{line_no}: box extends outside image bounds")
        counts[class_id] += 1
    return counts, errors


def validate(dataset_yaml: Path, *, expected: list[str] | None, require_classes: bool) -> dict[str, Any]:
    config = _load_yaml(dataset_yaml)
    names = _names(config.get("names"))
    root_value = config.get("path", dataset_yaml.parent)
    root = _resolve_root(dataset_yaml, root_value)
    errors: list[str] = []
    warnings: list[str] = []
    splits: dict[str, Any] = {}
    total_counts: Counter[int] = Counter()
    for split in ("train", "val", "test"):
        if split not in config:
            continue
        images: list[Path] = []
        for raw_dir in _split_values(config[split]):
            image_dir = _resolve_root(dataset_yaml, raw_dir)
            if not image_dir.is_dir():
                errors.append(f"{split}: missing image directory {image_dir}")
                continue
            images.extend(sorted(path for path in image_dir.rglob("*") if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES))
        image_set = {path.resolve() for path in images}
        label_dirs = {_image_label_dirs(path.parent)[1] for path in images}
        labels = sorted({label for directory in label_dirs for label in directory.glob("*.txt") if label.is_file()})
        label_set = {path.resolve() for path in labels}
        split_counts: Counter[int] = Counter()
        split_errors: list[str] = []
        unreadable: list[str] = []
        missing_labels: list[str] = []
        for image in images:
            label = _image_label_dirs(image.parent)[1] / f"{image.stem}.txt"
            if not label.is_file():
                missing_labels.append(str(image))
                continue
            if not _decode_ok(image):
                unreadable.append(str(image))
            counts, label_errors = _validate_label(label, len(names))
            split_counts.update(counts)
            split_errors.extend(label_errors)
        orphan_labels = sorted(str(path) for path in label_set if not any(path.stem == image.stem for image in images))
        if missing_labels:
            split_errors.extend(f"{item}: missing matching label" for item in missing_labels)
        if unreadable:
            split_errors.extend(f"{item}: image cannot be decoded" for item in unreadable)
        if orphan_labels:
            split_errors.extend(f"{item}: label has no matching image" for item in orphan_labels)
        errors.extend(split_errors)
        total_counts.update(split_counts)
        splits[split] = {
            "images": len(images),
            "labels": len(labels),
            "instances": {names[key] if 0 <= key < len(names) else str(key): value for key, value in sorted(split_counts.items())},
            "missing_labels": len(missing_labels),
            "orphan_labels": len(orphan_labels),
            "unreadable_images": len(unreadable),
            "errors": len(split_errors),
        }
    observed = {names[key] for key in total_counts if 0 <= key < len(names)}
    expected_names = expected if expected is not None else names
    missing_classes = [name for name in expected_names if name not in observed]
    if missing_classes:
        warnings.append("classes named or expected but without labels: " + ", ".join(missing_classes))
        if require_classes:
            errors.append(warnings[-1])
    unknown_expected = [name for name in expected_names if name not in names]
    if unknown_expected:
        errors.append("expected classes absent from YAML names: " + ", ".join(unknown_expected))
    return {
        "offline": True,
        "hardware_motion": False,
        "yaml": str(dataset_yaml.resolve()),
        "root": str(root),
        "names": names,
        "expected_classes": expected_names,
        "observed_classes": sorted(observed),
        "missing_classes": missing_classes,
        "instances": {names[key] if 0 <= key < len(names) else str(key): value for key, value in sorted(total_counts.items())},
        "splits": splits,
        "errors": errors,
        "warnings": warnings,
        "passed": not errors,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--yaml", type=Path, required=True)
    parser.add_argument("--expected-classes", default="", help="comma-separated class names")
    parser.add_argument("--require-classes", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    dataset_yaml = args.yaml if args.yaml.is_absolute() else PROJECT_ROOT / args.yaml
    if not dataset_yaml.is_file():
        raise FileNotFoundError(dataset_yaml)
    expected = [item.strip() for item in args.expected_classes.split(",") if item.strip()] or None
    report = validate(dataset_yaml.resolve(), expected=expected, require_classes=args.require_classes)
    output = args.output
    if output:
        output = output if output.is_absolute() else PROJECT_ROOT / output
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
