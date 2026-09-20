from __future__ import annotations

import unittest
from pathlib import Path

import numpy as np
import yaml


PROJECT_DIR = Path(__file__).resolve().parents[1]


class WorkspaceHomographyTests(unittest.TestCase):
    def test_saved_calibration_reprojects_within_limit(self) -> None:
        path = PROJECT_DIR / "codex_pickup_package" / "workspace_homography.yaml"
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
        homography = np.asarray(data["homography"], dtype=np.float64)
        pixels = np.asarray(data["pixel_points"], dtype=np.float64)
        expected = np.asarray(data["base_points_mm"], dtype=np.float64)

        homogeneous = np.column_stack([pixels, np.ones(len(pixels))])
        projected = (homography @ homogeneous.T).T
        projected = projected[:, :2] / projected[:, 2:3]
        errors = np.linalg.norm(projected - expected, axis=1)

        self.assertLess(float(errors.mean()), 1.0)
        self.assertLess(float(errors.max()), 1.5)
        self.assertAlmostEqual(float(errors.mean()), float(data["mean_error_mm"]), places=4)
        self.assertAlmostEqual(float(errors.max()), float(data["max_error_mm"]), places=4)


if __name__ == "__main__":
    unittest.main()
