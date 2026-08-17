from __future__ import annotations

import argparse
from pathlib import Path
import sys

import yaml


PROJECT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_DATASET_YAML = PROJECT_DIR / "my_all_data" / "dataset.yaml"


def write_dataset_template(path: Path) -> None:
    template = {
        "path": "/absolute/path/to/your/dataset",
        "train": "images/train",
        "val": "images/val",
        "test": "images/test",
        "names": ["cup", "bottle", "pen"],
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(template, sort_keys=False, allow_unicode=True), encoding="utf-8")


def train_one(args: argparse.Namespace) -> None:
    try:
        from ultralytics import YOLO
    except Exception as exc:  # pragma: no cover - depends on local training env
        raise SystemExit(
            "ultralytics is not installed. Install it on the training machine with:\n"
            "  python -m pip install ultralytics\n"
            f"Import error: {exc}"
        ) from exc

    data_path = Path(args.data)
    if not data_path.exists():
        raise SystemExit(
            f"Dataset YAML not found: {data_path}\n"
            f"Generate a template first with:\n  python {Path(__file__).name} --write-template"
        )

    model = YOLO(args.model)
    model.train(
        data=str(data_path),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        workers=args.workers,
        project=args.project,
        name=args.name,
        exist_ok=True,
        patience=args.patience,
        optimizer=args.optimizer,
        lr0=args.lr0,
        lrf=args.lrf,
        cos_lr=True,
        close_mosaic=args.close_mosaic,
        hsv_h=0.015,
        hsv_s=0.75,
        hsv_v=0.5,
        degrees=14.0,
        translate=0.14,
        scale=0.65,
        shear=2.0,
        perspective=0.0005,
        fliplr=0.5,
        mosaic=1.0,
        mixup=0.05,
        copy_paste=0.15,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Fine-tune YOLO for pen / cup / cola / bottle / earphone.")
    parser.add_argument("--data", default=str(DEFAULT_DATASET_YAML), help="dataset yaml path")
    parser.add_argument("--model", default="yolov8s.pt", help="pretrained base model")
    parser.add_argument("--epochs", type=int, default=120, help="training epochs")
    parser.add_argument("--imgsz", type=int, default=1024, help="training image size")
    parser.add_argument("--batch", type=int, default=16, help="batch size")
    parser.add_argument("--device", default="", help="cuda device id or empty for auto")
    parser.add_argument("--workers", type=int, default=4, help="dataloader workers")
    parser.add_argument("--project", default=str(PROJECT_DIR / "runtime" / "yolo_train"), help="output folder")
    parser.add_argument("--name", default="cup_pen", help="run name")
    parser.add_argument("--patience", type=int, default=30, help="early stop patience")
    parser.add_argument("--optimizer", default="AdamW", help="optimizer")
    parser.add_argument("--lr0", type=float, default=0.0015, help="initial learning rate")
    parser.add_argument("--lrf", type=float, default=0.01, help="final lr ratio")
    parser.add_argument("--close-mosaic", type=int, default=15, help="close mosaic augmentation N epochs before end")
    parser.add_argument("--cos-lr", action="store_true", default=True, help="use cosine LR scheduler")
    parser.add_argument("--write-template", action="store_true", help="write a dataset yaml template and exit")
    args = parser.parse_args()

    if args.write_template:
        write_dataset_template(Path(args.data))
        print(f"Wrote dataset template to: {args.data}")
        return

    train_one(args)


if __name__ == "__main__":
    main()
