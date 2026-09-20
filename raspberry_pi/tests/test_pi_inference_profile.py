from __future__ import annotations

from pathlib import Path
import sys
import unittest
from unittest.mock import patch


PROJECT_DIR = Path(__file__).resolve().parents[1]
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))

import tools.profile_pi_inference_runtime as profiler  # noqa: E402


class PiInferenceProfileTests(unittest.TestCase):
    def test_missing_rss_cannot_pass_the_pi_memory_gate(self) -> None:
        with patch.object(profiler, "_rss_mb", return_value=None):
            report = profiler.profile(
                PROJECT_DIR / "models/xiaou_objects_gpu_deep.onnx",
                PROJECT_DIR / "runtime/decision/transformer_policy_safety_final_20260813.int8.onnx",
                PROJECT_DIR / "runtime/yolo_train/xiaou_objects_dataset_deep/images/val/pen_calib_001.jpg",
                iterations=1,
                warmup=0,
                cv_threads=1,
                max_rss_mb=1024.0,
                task_class="cola",
            )
        self.assertFalse(report["passed"])
        self.assertFalse(report["deployment_readiness"]["pi_linux_rss_measured"])
        self.assertEqual(report["deployment_readiness"]["status"], "desktop_reference_only")
        self.assertIsNone(report["rss_mb"]["within_limit"])


if __name__ == "__main__":
    unittest.main()
