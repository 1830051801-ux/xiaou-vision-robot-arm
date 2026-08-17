#!/usr/bin/env python3
"""Fine-tune and rank two offline YOLO candidates without touching hardware.

Both candidates start from the same local checkpoint.  The script evaluates
the unchanged baseline in the same run, trains two conservative augmentation
profiles, validates all models on the configured validation split, and exports
only the best non-regressing candidate to an isolated candidate directory.
It never opens a camera, serial port, CAN interface, ROS graph, or motion path.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import shutil
import sys
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[1]

PROFILES: tuple[dict[str, Any], ...] = (
    {
        "name": "polish",
        "seed": 20260824,
        "lr0": 2e-4,
        "lrf": 0.10,
        "hsv_h": 0.005,
        "hsv_s": 0.25,
        "hsv_v": 0.15,
        "degrees": 1.5,
        "translate": 0.02,
        "scale": 0.10,
        "shear": 0.25,
        "perspective": 0.0,
        "mosaic": 0.10,
        "erasing": 0.05,
    },
    {
        "name": "balanced",
        "seed": 20260821,
        "lr0": 8e-4,
        "lrf": 0.05,
        "hsv_h": 0.010,
        "hsv_s": 0.50,
        "hsv_v": 0.30,
        "degrees": 3.0,
        "translate": 0.05,
        "scale": 0.20,
        "shear": 0.5,
        "perspective": 0.0001,
        "mosaic": 0.35,
        "erasing": 0.10,
    },
    {
        "name": "robust",
        "seed": 20260822,
        "lr0": 5e-4,
        "lrf": 0.10,
        "hsv_h": 0.012,
        "hsv_s": 0.60,
        "hsv_v": 0.35,
        "degrees": 5.0,
        "translate": 0.08,
        "scale": 0.30,
        "shear": 1.0,
        "perspective": 0.0002,
        "mosaic": 0.60,
        "erasing": 0.15,
    },
)


def _resolve(path: Path) -> Path:
    return path if path.is_absolute() else PROJECT_ROOT / path


def _metrics(result: Any) -> dict[str, float]:
    return {
        "precision": float(result.box.mp),
        "recall": float(result.box.mr),
        "map50": float(result.box.map50),
        "map50_95": float(result.box.map),
    }


def _validate(YOLO: Any, checkpoint: Path, data: Path, project: Path, name: str, args: argparse.Namespace) -> dict[str, Any]:
    result = YOLO(str(checkpoint)).val(
        data=str(data),
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        workers=0,
        project=str(project),
        name=name,
        exist_ok=True,
        plots=False,
        verbose=False,
    )
    return {
        "checkpoint": str(checkpoint.resolve()),
        "metrics": _metrics(result),
        "validation_dir": str(Path(result.save_dir).resolve()),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base", type=Path, required=True)
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--project", type=Path, default=PROJECT_ROOT / "runtime/yolo_train/improved_candidates_20260809")
    parser.add_argument("--candidate-dir", type=Path, default=PROJECT_ROOT / "runtime/yolo_candidates")
    parser.add_argument("--epochs", type=int, default=40)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--batch", type=int, default=4)
    parser.add_argument("--device", default="0")
    parser.add_argument("--profiles", default=",".join(profile["name"] for profile in PROFILES))
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()
    if args.epochs < 1 or args.imgsz < 64 or args.batch < 1:
        parser.error("epochs, imgsz, and batch must be positive")

    base = _resolve(args.base).resolve()
    data = _resolve(args.data).resolve()
    project = _resolve(args.project).resolve()
    candidate_dir = _resolve(args.candidate_dir).resolve()
    report_path = _resolve(args.report).resolve()
    if not base.is_file() or not data.is_file():
        raise FileNotFoundError(f"base={base}, data={data}")

    from ultralytics import YOLO
    import torch

    requested_profiles = {value.strip() for value in args.profiles.split(",") if value.strip()}
    profiles = [profile for profile in PROFILES if profile["name"] in requested_profiles]
    unknown_profiles = requested_profiles - {profile["name"] for profile in PROFILES}
    if unknown_profiles or not profiles:
        parser.error(f"profiles must be drawn from {[profile['name'] for profile in PROFILES]}")

    project.mkdir(parents=True, exist_ok=True)
    candidate_dir.mkdir(parents=True, exist_ok=True)
    baseline = _validate(YOLO, base, data, project, "baseline_validation", args)
    candidates: list[dict[str, Any]] = []

    for profile in profiles:
        run_name = f"finetune_{profile['name']}"
        model = YOLO(str(base))
        result = model.train(
            data=str(data),
            epochs=args.epochs,
            imgsz=args.imgsz,
            batch=args.batch,
            device=args.device,
            workers=0,
            project=str(project),
            name=run_name,
            exist_ok=True,
            patience=max(10, args.epochs // 3),
            optimizer="AdamW",
            lr0=profile["lr0"],
            lrf=profile["lrf"],
            weight_decay=5e-4,
            warmup_epochs=1.0,
            cos_lr=True,
            amp=True,
            cache=False,
            plots=False,
            verbose=False,
            deterministic=True,
            seed=profile["seed"],
            close_mosaic=min(10, max(0, args.epochs - 1)),
            hsv_h=profile["hsv_h"],
            hsv_s=profile["hsv_s"],
            hsv_v=profile["hsv_v"],
            degrees=profile["degrees"],
            translate=profile["translate"],
            scale=profile["scale"],
            shear=profile["shear"],
            perspective=profile["perspective"],
            flipud=0.0,
            fliplr=0.5,
            mosaic=profile["mosaic"],
            mixup=0.0,
            cutmix=0.0,
            copy_paste=0.0,
            erasing=profile["erasing"],
        )
        save_dir = Path(result.save_dir).resolve()
        best = save_dir / "weights/best.pt"
        if not best.is_file():
            raise FileNotFoundError(best)
        validation = _validate(YOLO, best, data, project, f"{run_name}_validation", args)
        candidates.append({
            "profile": dict(profile),
            "training_dir": str(save_dir),
            **validation,
        })

    ranked = sorted(
        candidates,
        key=lambda row: (
            -row["metrics"]["map50_95"],
            -row["metrics"]["map50"],
            -row["metrics"]["recall"],
            -row["metrics"]["precision"],
        ),
    )
    best = ranked[0]
    baseline_metrics = baseline["metrics"]
    best_metrics = best["metrics"]
    non_regressing = (
        best_metrics["map50_95"] >= baseline_metrics["map50_95"]
        and best_metrics["map50"] >= baseline_metrics["map50"] - 0.002
    )

    exported: dict[str, Any] | None = None
    if non_regressing:
        best_checkpoint = Path(best["checkpoint"])
        export_result = YOLO(str(best_checkpoint)).export(
            format="onnx",
            imgsz=args.imgsz,
            opset=12,
            simplify=False,
            dynamic=False,
            device=args.device,
        )
        source_onnx = Path(str(export_result)).resolve()
        target_stem = candidate_dir / "xiaou_objects_improved_best_20260809"
        target_pt = target_stem.with_suffix(".pt")
        target_onnx = target_stem.with_suffix(".onnx")
        target_names = target_stem.with_suffix(".names")
        shutil.copy2(best_checkpoint, target_pt)
        shutil.copy2(source_onnx, target_onnx)
        model_names = YOLO(str(best_checkpoint)).names
        names = [str(model_names[index]) for index in sorted(model_names)]
        target_names.write_text("\n".join(names) + "\n", encoding="utf-8")
        exported = {
            "profile": best["profile"]["name"],
            "checkpoint": str(target_pt),
            "onnx": str(target_onnx),
            "names": str(target_names),
            "classes": names,
        }

    report = {
        "offline": True,
        "hardware_motion": False,
        "camera_opened": False,
        "serial_opened": False,
        "can_opened": False,
        "gpu": torch.cuda.get_device_name(0) if torch.cuda.is_available() else None,
        "data": str(data),
        "epochs_per_candidate": args.epochs,
        "baseline": baseline,
        "candidates": candidates,
        "best_candidate": best,
        "non_regressing": non_regressing,
        "exported": exported,
        "selection_rule": "mAP50-95, mAP50, recall, precision; export only if baseline guards pass",
        "warning": "The 20-image validation split is small and has no real cup or tissue labels.",
    }
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if non_regressing else 2


if __name__ == "__main__":
    raise SystemExit(main())
