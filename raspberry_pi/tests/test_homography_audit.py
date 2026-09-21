from __future__ import annotations

from pathlib import Path

import yaml

from tools.audit_homography_workspace import audit_homography


def _calibration(tmp_path: Path, *, metadata: bool = True, points_inside: bool = True) -> Path:
    base = [[100.0, -50.0], [120.0, -40.0], [140.0, -30.0]] * 3
    pixels = [[10.0, 10.0], [12.0, 11.0], [14.0, 12.0]] * 3
    # x=10*u, y=10*v-150; all values are expressed in mm for this audit.
    matrix = [[10.0, 0.0, 0.0], [0.0, 10.0, -150.0], [0.0, 0.0, 1.0]]
    if not points_inside:
        base = [[100.0, -250.0], [120.0, -240.0], [140.0, -230.0]] * 3
    payload: dict[str, object] = {
        "base_points_mm": base,
        "pixel_points": pixels,
        "homography": matrix,
        "max_error_mm": 0.0,
        "mean_error_mm": 0.0,
    }
    if metadata:
        payload.update(
            {
                "image_width_px": 1920,
                "image_height_px": 1080,
                "output_frame": "robot_base_table",
                "output_unit": "m",
            }
        )
    path = tmp_path / "workspace.yaml"
    path.write_text(yaml.safe_dump(payload), encoding="utf-8")
    return path


def test_audit_reports_consistent_calibration(tmp_path: Path) -> None:
    report = audit_homography(_calibration(tmp_path))
    assert report["status"] == "pass"
    assert report["points_outside_workspace_gate"] == 0
    assert report["reprojection_error_mm"]["max"] == 0.0


def test_audit_exposes_stale_metadata_and_gate_conflict(tmp_path: Path) -> None:
    report = audit_homography(_calibration(tmp_path, metadata=False, points_inside=False))
    assert report["status"] == "needs_recalibration"
    assert "missing_or_mismatched_camera_resolution" in report["issues"]
    assert "calibration_points_outside_runtime_workspace_gate" in report["issues"]
