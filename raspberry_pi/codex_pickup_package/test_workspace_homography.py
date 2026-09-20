from __future__ import annotations

import argparse

import numpy as np
import yaml


def pixel_to_base_mm(homography: np.ndarray, u: float, v: float) -> tuple[float, float]:
    src = np.asarray([u, v, 1.0], dtype=np.float64)
    dst = homography @ src
    if abs(dst[2]) < 1e-9:
        raise ValueError("Invalid homography result")
    x, y = dst[:2] / dst[2]
    return float(x), float(y)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace", default="workspace_homography.yaml")
    parser.add_argument("--u", type=float, required=True)
    parser.add_argument("--v", type=float, required=True)
    args = parser.parse_args()

    data = yaml.safe_load(open(args.workspace, encoding="utf-8"))
    homography = np.asarray(data["homography"], dtype=np.float64)
    x, y = pixel_to_base_mm(homography, args.u, args.v)
    print(f"pixel u={args.u:.1f} v={args.v:.1f}")
    print(f"base  X={x:.1f}mm Y={y:.1f}mm")


if __name__ == "__main__":
    main()
