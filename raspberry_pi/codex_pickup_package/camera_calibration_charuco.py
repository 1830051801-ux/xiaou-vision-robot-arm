from __future__ import annotations

import argparse
import glob
import sys

import cv2
import numpy as np
import yaml


def make_charuco_board(squares_x: int, squares_y: int, square_length: float, marker_length: float):
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
    if hasattr(cv2.aruco, "CharucoBoard"):
        board = cv2.aruco.CharucoBoard((squares_x, squares_y), square_length, marker_length, aruco_dict)
    else:
        board = cv2.aruco.CharucoBoard_create(squares_x, squares_y, square_length, marker_length, aruco_dict)
    return aruco_dict, board


def detect_charuco(images_glob: str, squares_x: int, squares_y: int, square_length: float, marker_length: float):
    aruco_dict, board = make_charuco_board(squares_x, squares_y, square_length, marker_length)
    params = cv2.aruco.DetectorParameters()
    image_paths = sorted(glob.glob(images_glob))
    if not image_paths:
        print("No images found:", images_glob)
        sys.exit(2)

    all_corners = []
    all_ids = []
    image_size = None
    for path in image_paths:
        image = cv2.imread(path)
        if image is None:
            continue
        image_size = image.shape[:2][::-1]
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=params)
        if ids is None:
            print("No markers:", path)
            continue
        _, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board)
        if charuco_corners is not None and charuco_ids is not None and len(charuco_corners) > 3:
            all_corners.append(charuco_corners)
            all_ids.append(charuco_ids)
            print("Detected:", path, len(charuco_corners))

    return board, all_corners, all_ids, image_size


def calibrate(board, corners, ids, image_size):
    if not corners or image_size is None:
        raise SystemExit("No valid Charuco corners collected")
    ret, camera_matrix, dist_coeffs, *_ = cv2.aruco.calibrateCameraCharucoExtended(
        charucoCorners=corners,
        charucoIds=ids,
        board=board,
        imageSize=image_size,
        cameraMatrix=None,
        distCoeffs=None,
        flags=0,
    )
    return ret, camera_matrix, dist_coeffs


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--images", required=True)
    parser.add_argument("--squaresX", type=int, default=8, help="number of squares along X")
    parser.add_argument("--squaresY", type=int, default=10, help="number of squares along Y")
    parser.add_argument("--square_length", type=float, default=0.01, help="square size in meters, 10mm = 0.01m")
    parser.add_argument("--marker_length", type=float, default=0.005, help="marker size in meters, 5mm = 0.005m")
    parser.add_argument("--output", default="camera.yaml")
    args = parser.parse_args()

    board, corners, ids, image_size = detect_charuco(
        args.images,
        args.squaresX,
        args.squaresY,
        args.square_length,
        args.marker_length,
    )
    err, camera_matrix, dist_coeff = calibrate(board, corners, ids, image_size)
    data = {
        "camera_matrix": np.asarray(camera_matrix).tolist(),
        "dist_coeff": np.asarray(dist_coeff).tolist(),
        "reprojection_error": float(err),
    }
    with open(args.output, "w", encoding="utf-8") as file:
        yaml.safe_dump(data, file)
    print("Wrote", args.output)
    print("reprojection_error:", err)


if __name__ == "__main__":
    main()
