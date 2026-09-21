#!/usr/bin/env python3
"""Read-only audit for XiaoU's fixed-camera workspace homography.

The audit intentionally does not rewrite calibration or widen the runtime
workspace.  It compares the checked-in calibration's own units and metadata
with the points it maps and with the configured workspace gate, then emits a
machine-readable report for offline verification.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path
from typing import Any

import numpy as np
import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CALIBRATION = PROJECT_ROOT / "codex_pickup_package" / "workspace_homography.yaml"


def _finite_matrix(value: Any, shape: tuple[int, int], label: str) -> np.ndarray:
    matrix = np.asarray(value, dtype=np.float64)
    if matrix.shape != shape or not np.isfinite(matrix).all():
        raise ValueError(f"{label} must be a finite {shape[0]}x{shape[1]} matrix")
    return matrix


def _project(matrix: np.ndarray, pixels: np.ndarray) -> np.ndarray:
    homogeneous = np.concatenate((pixels, np.ones((pixels.shape[0], 1))), axis=1)
    mapped = (matrix @ homogeneous.T).T
    denominator = mapped[:, 2]
    if np.any(np.abs(denominator) < 1e-12):
        raise ValueError("homography maps a calibration point to infinity")
    result = mapped[:, :2] / denominator[:, None]
    if not np.isfinite(result).all():
        raise ValueError("homography produced non-finite coordinates")
    return result


def _env_float(name: str, default: float) -> float:
    raw = os.getenv(name)
    if raw is None or not raw.strip():
        return default
    return float(raw)


def audit_homography(
    path: str | Path = DEFAULT_CALIBRATION,
    *,
    expected_resolution: tuple[int, int] = (1920, 1080),
    workspace_mm: tuple[float, float, float, float] | None = None,
) -> dict[str, Any]:
    """Return a truthful calibration audit without touching hardware."""

    calibration_path = Path(path)
    data = yaml.safe_load(calibration_path.read_text(encoding="utf-8")) or {}
    if not isinstance(data, dict):
        raise ValueError("calibration YAML must contain a mapping")
    pixels = _finite_matrix(data.get("pixel_points"), (9, 2), "pixel_points")
    base_mm = _finite_matrix(data.get("base_points_mm"), (9, 2), "base_points_mm")
    matrix = _finite_matrix(data.get("homography"), (3, 3), "homography")
    mapped = _project(matrix, pixels)
    residual = mapped - base_mm
    residual_norm = np.linalg.norm(residual, axis=1)
    if workspace_mm is None:
        workspace_mm = (
            _env_float("WORKSPACE_X_MIN_MM", 80.0),
            _env_float("WORKSPACE_X_MAX_MM", 360.0),
            _env_float("WORKSPACE_Y_MIN_MM", -180.0),
            _env_float("WORKSPACE_Y_MAX_MM", 180.0),
        )
    x_min, x_max, y_min, y_max = (float(value) for value in workspace_mm)
    in_gate = (
        (base_mm[:, 0] >= x_min)
        & (base_mm[:, 0] <= x_max)
        & (base_mm[:, 1] >= y_min)
        & (base_mm[:, 1] <= y_max)
    )
    metadata_resolution = (
        data.get("image_width_px"),
        data.get("image_height_px"),
    )
    issues: list[str] = []
    if metadata_resolution != expected_resolution:
        issues.append("missing_or_mismatched_camera_resolution")
    if data.get("output_frame") != "robot_base_table":
        issues.append("missing_or_mismatched_output_frame")
    if data.get("output_unit") != "m":
        issues.append("output_unit_declares_not_meters")
    if not bool(np.all(in_gate)):
        issues.append("calibration_points_outside_runtime_workspace_gate")
    declared_max = data.get("max_error_mm")
    declared_mean = data.get("mean_error_mm")
    if isinstance(declared_max, (int, float)) and abs(float(declared_max) - float(np.max(residual_norm))) > 1e-3:
        issues.append("declared_max_error_does_not_match_recomputed")
    if isinstance(declared_mean, (int, float)) and abs(float(declared_mean) - float(np.mean(residual_norm))) > 1e-3:
        issues.append("declared_mean_error_does_not_match_recomputed")
    return {
        "schema": "xiaou_homography_workspace_audit_v1",
        "offline": True,
        "hardware_motion": False,
        "calibration_path": str(calibration_path),
        "expected_camera_resolution_px": list(expected_resolution),
        "declared_camera_resolution_px": list(metadata_resolution),
        "declared_output_frame": data.get("output_frame"),
        "declared_output_unit": data.get("output_unit"),
        "recomputed_projection_unit": "mm (matched against base_points_mm)",
        "workspace_gate_mm": {
            "x": [x_min, x_max],
            "y": [y_min, y_max],
        },
        "calibration_point_count": int(base_mm.shape[0]),
        "points_inside_workspace_gate": int(np.count_nonzero(in_gate)),
        "points_outside_workspace_gate": int(np.count_nonzero(~in_gate)),
        "reprojection_error_mm": {
            "max": float(np.max(residual_norm)),
            "mean": float(np.mean(residual_norm)),
            "p95": float(np.percentile(residual_norm, 95)),
        },
        "mapped_base_points_mm": mapped.tolist(),
        "issues": issues,
        "status": "pass" if not issues else "needs_recalibration",
        "recommendation": (
            "Use only after adding fixed-camera metadata, declaring SI output units, "
            "and confirming all calibration points lie inside the runtime gate."
            if issues
            else "Calibration metadata and workspace gate are internally consistent."
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    try:
        report = audit_homography(args.calibration)
    except (OSError, ValueError, yaml.YAMLError) as exc:
        print(json.dumps({"status": "invalid", "error": str(exc)}, ensure_ascii=False, indent=2))
        return 2
    payload = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload, encoding="utf-8")
    print(payload, end="")
    return 0 if report["status"] == "pass" else 2


if __name__ == "__main__":
    raise SystemExit(main())
