from __future__ import annotations

import hashlib
import io
import os
from pathlib import Path
import sys
import tarfile
import tempfile
import unittest


PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR))

from tools.build_pi_deployment import build
from tools.install_pi_inference_release import install
from tools.verify_pi_deployment_package import verify as verify_archive


class PiInferenceReleaseTests(unittest.TestCase):
    def test_built_archive_contains_verified_release_manifest(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            archive = root / "xiaou.tar.gz"
            report = build(archive, root / "build.json")
            self.assertTrue(report["passed"])
            self.assertGreater(report["file_count"], 30)
            verification = verify_archive(
                archive,
                PROJECT_DIR / "runtime/yolo_train/xiaou_objects_dataset_deep/images/val/pen_calib_001.jpg",
            )
            self.assertTrue(verification["passed"], verification)
            self.assertTrue(verification["release_verification"]["passed"])
            self.assertIn("tools/preview_object_teach_record.py", [
                row["path"] for row in report["files"]
            ])
            activated = install(
                archive,
                root / "releases",
                expected_sha256=report["archive_sha256"],
                activate=os.name != "nt",
            )
            self.assertEqual(activated["action"], "activated" if os.name != "nt" else "staged")
            if os.name != "nt":
                self.assertTrue((root / "releases" / "current").is_symlink())
            self.assertFalse(activated["hardware_motion"])

    def test_installer_rejects_archive_content_outside_release_root(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            archive = root / "unsafe.tar.gz"
            with tarfile.open(archive, "w:gz") as bundle:
                payload = b"not a release"
                info = tarfile.TarInfo("unexpected.txt")
                info.size = len(payload)
                bundle.addfile(info, io.BytesIO(payload))
            digest = hashlib.sha256(archive.read_bytes()).hexdigest()
            with self.assertRaisesRegex(ValueError, "outside xiaou_pi"):
                install(archive, root / "releases", expected_sha256=digest, activate=False)


if __name__ == "__main__":
    unittest.main()
