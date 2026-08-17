from __future__ import annotations

import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from robot_ai.vision.model_registry import load_registry, resolve_profile


class YoloModelRegistryTests(unittest.TestCase):
    def _registry(self, root: Path, digest: str) -> Path:
        (root / "classes.json").write_text(
            json.dumps({"classes": [{"id": 0, "name": "pen"}]}), encoding="utf-8"
        )
        path = root / "registry.json"
        path.write_text(
            json.dumps({
                "schema_version": 1,
                "default_profile": "safe",
                "automatic_switching": False,
                "profiles": {
                    "safe": {
                        "model": "model.onnx",
                        "sha256": digest,
                        "class_schema": "classes.json",
                        "classes": ["pen"],
                        "confidence": 0.2,
                        "role": "test",
                    }
                },
            }),
            encoding="utf-8",
        )
        return path

    def test_profile_resolution_checks_hash_and_schema(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            model = root / "model.onnx"
            model.write_bytes(b"offline-model")
            digest = hashlib.sha256(model.read_bytes()).hexdigest()
            profile = resolve_profile(registry_path=self._registry(root, digest), project_root=root)
            self.assertEqual(profile.name, "safe")
            self.assertEqual(profile.classes, ("pen",))

    def test_unknown_profile_fails_closed(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            model = root / "model.onnx"
            model.write_bytes(b"offline-model")
            digest = hashlib.sha256(model.read_bytes()).hexdigest()
            with self.assertRaises(KeyError):
                resolve_profile("missing", registry_path=self._registry(root, digest), project_root=root)

    def test_changed_model_hash_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "model.onnx").write_bytes(b"changed")
            with self.assertRaises(ValueError):
                resolve_profile(registry_path=self._registry(root, "0" * 64), project_root=root)

    def test_automatic_switching_must_remain_disabled(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "registry.json"
            path.write_text(json.dumps({"schema_version": 1, "profiles": {}, "automatic_switching": True}), encoding="utf-8")
            with self.assertRaises(ValueError):
                load_registry(path)


if __name__ == "__main__":
    unittest.main()
