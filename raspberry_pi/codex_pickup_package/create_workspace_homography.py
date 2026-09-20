from __future__ import annotations

import argparse

import cv2
import numpy as np
import yaml


def parse_points(values: list[float], label: str) -> np.ndarray:
    if len(values) < 8 or len(values) % 2 != 0:
        raise SystemExit(f"{label} needs an even number of values and at least 4 points")
    return np.asarray(values, dtype=np.float32).reshape(-1, 2)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Create pixel-to-robot-table homography from 4 or more known points."
    )
    parser.add_argument(
        "--pixels",
        nargs="+",
        type=float,
        required=True,
        help="Image pixel points: u1 v1 u2 v2 ... Same order as --base_mm.",
    )
    parser.add_argument(
        "--base_mm",
        nargs="+",
        type=float,
        required=True,
        help="Robot base table points in millimeters: x1 y1 x2 y2 ... Same order as --pixels.",
    )
    parser.add_argument("--output", default="workspace_homography.yaml")
    parser.add_argument("--image-width-px", type=int, required=True)
    parser.add_argument("--image-height-px", type=int, required=True)
    args = parser.parse_args()

    if (args.image_width_px, args.image_height_px) != (1920, 1080):
        raise SystemExit("This formal F407 UART path accepts only fixed 1920x1080 calibration images.")

    pixels = parse_points(args.pixels, "--pixels")
    base_mm = parse_points(args.base_mm, "--base_mm")
    if len(pixels) != len(base_mm):
        raise SystemExit(f"Point count mismatch: pixels={len(pixels)} base_mm={len(base_mm)}")

    base_m = base_mm / 1000.0
    homography, mask = cv2.findHomography(pixels, base_m, method=0)
    if homography is None:
        raise SystemExit("Failed to compute homography. Check point order.")

    projected_m = cv2.perspectiveTransform(pixels.reshape(-1, 1, 2), homography).reshape(-1, 2)
    errors_mm = np.linalg.norm(projected_m - base_m, axis=1) * 1000.0

    data = {
        "type": "pixel_to_robot_base_table_m",
        "image_width_px": args.image_width_px,
        "image_height_px": args.image_height_px,
        "output_frame": "robot_base_table",
        "output_unit": "m",
        "pixel_points": pixels.tolist(),
        "base_points_mm": base_mm.tolist(),
        "base_points_m": base_m.tolist(),
        "homography": homography.tolist(),
        "mean_error_mm": float(np.mean(errors_mm)),
        "max_error_mm": float(np.max(errors_mm)),
        "point_order": "Use the same physical order for both lists. For 9 points, use row-major order: top-left to top-right, then middle row, then bottom row.",
    }
    with open(args.output, "w", encoding="utf-8") as file:
        yaml.safe_dump(data, file, allow_unicode=False)

    print("Wrote", args.output)
    print(f"points: {len(pixels)}")
    print(f"mean_error_mm: {np.mean(errors_mm):.3f}")
    print(f"max_error_mm: {np.max(errors_mm):.3f}")
    for p in pixels:
        src = np.asarray([p[0], p[1], 1.0], dtype=np.float64)
        dst = homography @ src
        dst = dst[:2] / dst[2]
        print(f"pixel ({p[0]:.1f}, {p[1]:.1f}) -> base ({dst[0] * 1000.0:.1f}, {dst[1] * 1000.0:.1f}) mm")


if __name__ == "__main__":
    main()
