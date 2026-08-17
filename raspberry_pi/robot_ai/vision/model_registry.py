"""Versioned, hash-checked YOLO profile registry.

Resolving a profile selects perception weights only.  It never opens a camera,
starts ROS, changes the active production model, or enables arm motion.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_REGISTRY = PROJECT_ROOT / "robot_ai/vision/config/yolo_model_registry.json"


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


@dataclass(frozen=True)
class YoloModelProfile:
    name: str
    model_path: Path
    class_schema_path: Path
    classes: tuple[str, ...]
    confidence: float
    role: str
    sha256: str

    def as_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "model": str(self.model_path),
            "class_schema": str(self.class_schema_path),
            "classes": list(self.classes),
            "confidence": self.confidence,
            "role": self.role,
            "sha256": self.sha256,
            "motion_enabled": False,
        }


def load_registry(path: str | Path = DEFAULT_REGISTRY) -> dict[str, Any]:
    registry_path = Path(path)
    if not registry_path.is_absolute():
        registry_path = PROJECT_ROOT / registry_path
    data = json.loads(registry_path.read_text(encoding="utf-8"))
    if data.get("schema_version") != 1 or not isinstance(data.get("profiles"), dict):
        raise ValueError(f"unsupported YOLO registry schema: {registry_path}")
    if data.get("automatic_switching") is not False:
        raise ValueError("automatic YOLO model switching must remain disabled")
    return data


def resolve_profile(
    name: str | None = None,
    *,
    registry_path: str | Path = DEFAULT_REGISTRY,
    project_root: str | Path = PROJECT_ROOT,
) -> YoloModelProfile:
    root = Path(project_root).resolve()
    data = load_registry(registry_path)
    profile_name = str(name or data.get("default_profile") or "").strip()
    profiles = data["profiles"]
    if profile_name not in profiles:
        raise KeyError(f"unknown YOLO profile {profile_name!r}; available={sorted(profiles)}")
    row = profiles[profile_name]
    if not isinstance(row, dict):
        raise ValueError(f"invalid YOLO profile: {profile_name}")
    model_path = (root / str(row.get("model") or "")).resolve()
    schema_path = (root / str(row.get("class_schema") or "")).resolve()
    try:
        model_path.relative_to(root)
        schema_path.relative_to(root)
    except ValueError as exc:
        raise ValueError(f"YOLO profile escapes project root: {profile_name}") from exc
    if not model_path.is_file() or not schema_path.is_file():
        raise FileNotFoundError(f"profile={profile_name}, model={model_path}, schema={schema_path}")
    expected_hash = str(row.get("sha256") or "").lower()
    actual_hash = _sha256(model_path)
    if len(expected_hash) != 64 or actual_hash != expected_hash:
        raise ValueError(f"YOLO model hash mismatch for profile {profile_name}: {actual_hash}")
    classes = tuple(str(value).strip() for value in (row.get("classes") or []))
    if not classes or any(not value for value in classes) or len(set(classes)) != len(classes):
        raise ValueError(f"invalid class list for profile {profile_name}")
    confidence = float(row.get("confidence"))
    if not 0.0 < confidence < 1.0:
        raise ValueError(f"invalid confidence for profile {profile_name}")
    schema = json.loads(schema_path.read_text(encoding="utf-8"))
    schema_classes = tuple(str(item["name"]) for item in schema.get("classes") or [])
    if schema_classes != classes:
        raise ValueError(f"class schema mismatch for profile {profile_name}")
    return YoloModelProfile(
        name=profile_name,
        model_path=model_path,
        class_schema_path=schema_path,
        classes=classes,
        confidence=confidence,
        role=str(row.get("role") or ""),
        sha256=actual_hash,
    )


__all__ = ["DEFAULT_REGISTRY", "YoloModelProfile", "load_registry", "resolve_profile"]
