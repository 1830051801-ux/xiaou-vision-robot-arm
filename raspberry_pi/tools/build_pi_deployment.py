#!/usr/bin/env python3
"""Build and audit a small Raspberry Pi inference/no-motion package.

The archive intentionally contains ONNX models and protocol/safety code, not
PyTorch, Ultralytics, desktop datasets, MuJoCo assets, or hardware-enable
configuration.  It is a staging artifact; it does not SSH to or modify a Pi.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import tarfile
from typing import Iterable


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ARCHIVE = PROJECT_ROOT / "runtime/deployment/xiaou_pi_inference_final_20260813.tar.gz"
DEFAULT_REPORT = PROJECT_ROOT / "runtime/deployment/xiaou_pi_inference_final_20260813.json"


DUAL_CANDIDATE_FILES = [
    "runtime/yolo_candidates/xiaou_objects_precision_candidate_20260809.onnx",
    "runtime/yolo_candidates/xiaou_objects_precision_candidate_20260809.names",
    "runtime/decision/transformer_policy_safety_final_20260813.int8.onnx",
    "runtime/decision/transformer_policy_safety_final_20260813.int8.json",
    "robot_ai/vision/model_registry.py",
    "robot_ai/vision/config/yolo_model_registry.json",
    "robot_ai/vision/config/yolo_class_schema_v1.json",
]

PRESENTATION_FILES = [
    "robot_ai/face_state.py",
    "robot_ai/face_animation.py",
    "robot_ai/face_display.py",
    "robot_ai/face_presentation.py",
    "robot_ai/dialog_emote_bridge.py",
    "robot_ai/xiaou_runtime.py",
    "codex_emote_ai_package/dialog_manager.py",
    "codex_emote_ai_package/emote_player.py",
    "codex_emote_ai_package/emote_mapping.json",
    "scripts/run_xiaou_face_screen.sh",
    "tools/verify_xiaou_presentation.py",
]
PRESENTATION_FILES.extend(
    f"robot_ai/emote_assets/gif/{name}"
    for name in ("angry.gif", "idle.gif", "investigate.gif", "laugh.gif", "mock.gif", "ponder.gif", "question.gif", "sad.gif", "shocked.gif", "smile.gif")
)


def _files(profile_set: str = "production") -> list[Path]:
    relative: list[str] = [
        "requirements_pi_inference.txt",
        "README_使用说明.md",
        "codex_pickup_package/workspace_homography.yaml",
        "models/xiaou_objects_gpu_deep.onnx",
        "models/xiaou_objects_gpu_deep.names",
        "runtime/decision/transformer_policy_safety_final_20260813.int8.onnx",
        "runtime/decision/transformer_policy_safety_final_20260813.int8.json",
        "robot_ai/common.py",
        "robot_ai/sitecustomize.py",
        "robot_ai/yolo_opencv.py",
        "robot_ai/vision/__init__.py",
        "robot_ai/vision/unified_yolo.py",
        "robot_ai/decision/__init__.py",
        "robot_ai/decision/onnx_policy.py",
        "robot_ai/decision/transformer_policy.py",
        "robot_ai/status_dashboard.py",
        "robot_ai/xiaou_status_dashboard.py",
        *PRESENTATION_FILES,
        "robot_ai/arm_control/__init__.py",
        "robot_ai/arm_control/can_protocol.py",
        "robot_ai/arm_control/can_passive.py",
        "robot_ai/arm_control/feedback_quality.py",
        "robot_ai/arm_control/grasp_families.py",
        "robot_ai/arm_control/joint_status_probe.py",
        "robot_ai/arm_control/kinematics.py",
        "robot_ai/arm_control/lie.py",
        "robot_ai/arm_control/model.py",
        "robot_ai/arm_control/motor_feedback_diagnostic.py",
        "robot_ai/arm_control/safety.py",
        "robot_ai/arm_control/task_planner.py",
        "robot_ai/arm_control/taught_grasp.py",
        "robot_ai/arm_control/teach_registry.py",
        "robot_ai/arm_control/taught_waypoint_plan.py",
        "robot_ai/arm_control/trajectory.py",
        "robot_ai/arm_control/uart_protocol.py",
        "robot_ai/arm_control/config/arm_model.json",
        "robot_ai/arm_control/config/hardware_calibration.json",
        "robot_ai/arm_control/config/grasp_family_registry.json",
        "robot_ai/arm_control/config/object_teach_registry.json",
        "robot_ai/arm_control/config/object_grasp_profiles.json",
        "scripts/verify_pi_inference_release.sh",
        "scripts/setup_pi_inference_env.sh",
        "scripts/install_pi_inference_release.sh",
        "scripts/run_xiaou_status_dashboard.sh",
        "tools/verify_pi_inference_release.py",
        "tools/install_pi_inference_release.py",
        "tools/profile_pi_inference_runtime.py",
        "tools/preview_object_teach_record.py",
        "docs/Pi_离线推理包_原子部署回滚_20260809.md",
        "docs/小U_ControlTower_交互与安全呈现_20260809.md",
        "docs/小U_表情外观与ControlTower协作_20260809.md",
    ]
    if profile_set == "dual-candidate":
        relative.extend(DUAL_CANDIDATE_FILES)
    elif profile_set != "production":
        raise ValueError(f"unsupported profile set: {profile_set}")
    files = [(PROJECT_ROOT / item).resolve() for item in relative]
    missing = [path for path in files if not path.is_file()]
    if missing:
        raise FileNotFoundError("missing deployment files: " + ", ".join(str(item) for item in missing))
    return files


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def build(archive: Path, report_path: Path, profile_set: str = "production") -> dict:
    files = _files(profile_set)
    forbidden = [path for path in files if path.suffix.lower() in {".pt", ".pth"}]
    if forbidden:
        raise RuntimeError(f"Pi archive cannot contain PyTorch weights: {forbidden}")
    archive.parent.mkdir(parents=True, exist_ok=True)
    rows = []
    with tarfile.open(archive, "w:gz", format=tarfile.PAX_FORMAT) as bundle:
        for path in sorted(files):
            relative = path.relative_to(PROJECT_ROOT).as_posix()
            bundle.add(path, arcname=f"xiaou_pi/{relative}", recursive=False)
            rows.append({"path": relative, "bytes": path.stat().st_size, "sha256": _sha256(path)})
        manifest = {
            "schema_version": 1,
            "profile_set": profile_set,
            "hardware_motion": False,
            "files": rows,
        }
        manifest_bytes = (json.dumps(manifest, ensure_ascii=False, indent=2) + "\n").encode("utf-8")
        manifest_info = tarfile.TarInfo("xiaou_pi/DEPLOYMENT_MANIFEST.json")
        manifest_info.size = len(manifest_bytes)
        manifest_info.mode = 0o644
        bundle.addfile(manifest_info, io.BytesIO(manifest_bytes))

    requirements = [
        line.strip().split("#", 1)[0].strip()
        for line in (PROJECT_ROOT / "requirements_pi_inference.txt").read_text(encoding="utf-8").splitlines()
        if line.strip() and not line.lstrip().startswith("#")
    ]
    runtime_modules = {name: bool(importlib.util.find_spec(name)) for name in ("cv2", "numpy", "onnxruntime", "PIL")}
    package_bytes = sum(int(row["bytes"]) for row in rows) + len(manifest_bytes)
    # Conservative planning figures for ONNX Runtime + OpenCV + ROS2 Python
    # nodes on a 2 GB Pi.  These are a budget gate, not a measured RSS claim.
    ram_budget = {
        "ram_limit_mb": 2048,
        "reserved_os_ros2_mb": 512,
        "estimated_peak_process_mb": 864,
        "required_headroom_mb": 256,
    }
    ram_budget["estimated_total_mb"] = ram_budget["reserved_os_ros2_mb"] + ram_budget["estimated_peak_process_mb"]
    ram_budget["passes_2gb_budget"] = ram_budget["estimated_total_mb"] + ram_budget["required_headroom_mb"] <= ram_budget["ram_limit_mb"]
    report = {
        "offline": True,
        "hardware_motion": False,
        "ssh_used": False,
        "serial_opened": False,
        "can_opened": False,
        "archive": str(archive.resolve()),
        "profile_set": profile_set,
        "archive_bytes": archive.stat().st_size,
        "archive_sha256": _sha256(archive),
        "file_count": len(rows) + 1,
        "payload_bytes_uncompressed": package_bytes,
        "requirements": requirements,
        "host_import_probe": runtime_modules,
        "forbidden_desktop_artifacts": ["*.pt", "*.pth", "Ultralytics", "PyTorch", "MuJoCo", "camera capture"],
        "opencv_install_policy": "Raspberry Pi OS python3-opencv is required before creating the --system-site-packages inference venv",
        "ram_budget": ram_budget,
        "runtime_model_policy": "load exactly one YOLO profile at a time; keep transformer_policy_safety_final_20260813 INT8 on CPU; route object classes through the grasp-family registry before planning",
        "files": rows,
        "manifest": "DEPLOYMENT_MANIFEST.json",
        "switch_policy": "stage to a new release directory, verify manifest and hardware lock, atomically update current symlink, keep previous symlink for rollback",
        "passed": ram_budget["passes_2gb_budget"] and not forbidden,
    }
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive", type=Path, default=DEFAULT_ARCHIVE)
    parser.add_argument("--report", type=Path, default=DEFAULT_REPORT)
    parser.add_argument("--profile-set", choices=("production", "dual-candidate"), default="production")
    args = parser.parse_args()
    archive = args.archive if args.archive.is_absolute() else PROJECT_ROOT / args.archive
    report = args.report if args.report.is_absolute() else PROJECT_ROOT / args.report
    result = build(archive.resolve(), report.resolve(), args.profile_set)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
