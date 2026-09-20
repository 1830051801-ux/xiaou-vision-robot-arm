from __future__ import annotations

import argparse
from pathlib import Path
import statistics

import yaml

from common import PROJECT_DIR


def _wrap_deg(angle: float) -> float:
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Create a simple image-angle to robot-yaw calibration file."
    )
    parser.add_argument(
        "--pair",
        action="append",
        nargs=2,
        type=float,
        metavar=("IMAGE_THETA_DEG", "ROBOT_RZ_DEG"),
        help="One calibration pair. Repeat 3-8 times. Example: --pair 12 18",
    )
    parser.add_argument(
        "--output",
        default=str(PROJECT_DIR / "runtime" / "calibration" / "rotation_calibration.yaml"),
    )
    args = parser.parse_args()

    if not args.pair:
        raise SystemExit(
            "Need at least one --pair IMAGE_THETA_DEG ROBOT_RZ_DEG. "
            "Better collect 3-8 pairs from different object rotations."
        )

    offsets = [_wrap_deg(robot_rz - image_theta) for image_theta, robot_rz in args.pair]
    offset_deg = statistics.fmean(offsets)
    max_error = max(abs(_wrap_deg(item - offset_deg)) for item in offsets)

    data = {
        "type": "image_theta_to_robot_rz",
        "offset_deg": float(offset_deg),
        "max_pair_error_deg": float(max_error),
        "pairs": [
            {"image_theta_deg": float(image_theta), "robot_rz_deg": float(robot_rz)}
            for image_theta, robot_rz in args.pair
        ],
        "formula": "robot_rz_deg = image_theta_deg + offset_deg",
    }

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(yaml.safe_dump(data, allow_unicode=False), encoding="utf-8")
    print(f"Wrote {output}")
    print(f"offset_deg={offset_deg:.2f}, max_pair_error_deg={max_error:.2f}")


if __name__ == "__main__":
    main()
