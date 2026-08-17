from __future__ import annotations

import argparse
import glob
import sys

import cv2
import numpy as np
import yaml


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--images", required=True)
    parser.add_argument("--cols", type=int, default=7, help="inner corners per row")
    parser.add_argument("--rows", type=int, default=9, help="inner corners per column")
    parser.add_argument("--square_size", type=float, default=0.01, help="square size in meters")
    parser.add_argument("--output", default="camera.yaml")
    parser.add_argument("--debug_dir", default="chess_debug")
    args = parser.parse_args()

    image_paths = sorted(glob.glob(args.images))
    if not image_paths:
        raise SystemExit(f"No images found: {args.images}")

    pattern_size = (args.cols, args.rows)
    objp = np.zeros((args.rows * args.cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0 : args.cols, 0 : args.rows].T.reshape(-1, 2)
    objp *= args.square_size

    objpoints = []
    imgpoints = []
    image_size = None

    import os

    os.makedirs(args.debug_dir, exist_ok=True)

    for path in image_paths:
        image = cv2.imread(path)
        if image is None:
            continue
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        image_size = gray.shape[::-1]

        found, corners = cv2.findChessboardCornersSB(gray, pattern_size)
        if not found:
            found, corners = cv2.findChessboardCorners(gray, pattern_size)
        if not found:
            print("No chessboard:", path)
            continue

        criteria = (
            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
            30,
            0.001,
        )
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        objpoints.append(objp)
        imgpoints.append(corners)

        debug = image.copy()
        cv2.drawChessboardCorners(debug, pattern_size, corners, found)
        out_name = os.path.join(args.debug_dir, os.path.basename(path))
        cv2.imwrite(out_name, debug)
        print("Detected:", path, len(corners))

    if not objpoints or image_size is None:
        raise SystemExit("No valid chessboard corners collected")

    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints,
        imgpoints,
        image_size,
        None,
        None,
    )

    data = {
        "camera_matrix": np.asarray(camera_matrix).tolist(),
        "dist_coeff": np.asarray(dist_coeffs).tolist(),
        "reprojection_error": float(ret),
        "pattern": {
            "type": "chessboard",
            "cols": args.cols,
            "rows": args.rows,
            "square_size_m": args.square_size,
        },
    }
    with open(args.output, "w", encoding="utf-8") as file:
        yaml.safe_dump(data, file)
    print("Wrote", args.output)
    print("reprojection_error:", ret)


if __name__ == "__main__":
    main()
