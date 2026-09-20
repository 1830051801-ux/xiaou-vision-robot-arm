from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import yaml


def load_camera(path: str) -> dict[str, float]:
    data = yaml.safe_load(open(path, encoding="utf-8"))
    matrix = data["camera_matrix"]
    return {
        "fx": float(matrix[0][0]),
        "fy": float(matrix[1][1]),
        "cx": float(matrix[0][2]),
        "cy": float(matrix[1][2]),
    }


def bbox_center_to_camera_xyz(bbox: list[float], camera: dict[str, float], cam_height: float):
    xmin, ymin, xmax, ymax = bbox
    u = (xmin + xmax) / 2.0
    v = (ymin + ymax) / 2.0
    z = cam_height
    x = (u - camera["cx"]) * z / camera["fx"]
    y = (v - camera["cy"]) * z / camera["fy"]
    return x, y, z, [u, v]


def load_workspace(path: str) -> np.ndarray | None:
    if not Path(path).exists():
        return None
    data = yaml.safe_load(open(path, encoding="utf-8"))
    return np.asarray(data["homography"], dtype=np.float64)


def pixel_to_base_m(homography: np.ndarray, u: float, v: float) -> tuple[float, float]:
    src = np.asarray([u, v, 1.0], dtype=np.float64)
    dst = homography @ src
    if abs(dst[2]) < 1e-9:
        raise ValueError("Invalid workspace homography result")
    x_mm, y_mm = dst[:2] / dst[2]
    return float(x_mm / 1000.0), float(y_mm / 1000.0)


def estimate_theta(image_path: str, bbox: list[float]) -> float:
    image = cv2.imread(image_path)
    if image is None:
        return 0.0
    xmin, ymin, xmax, ymax = map(int, bbox)
    h, w = image.shape[:2]
    xmin, xmax = max(0, xmin), min(w - 1, xmax)
    ymin, ymax = max(0, ymin), min(h - 1, ymax)
    crop = image[ymin:ymax, xmin:xmax]
    if crop.size == 0:
        return 0.0
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return 0.0
    rect = cv2.minAreaRect(max(contours, key=cv2.contourArea))
    angle = float(rect[2])
    return angle if rect[1][0] < rect[1][1] else angle + 90.0


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", required=True)
    parser.add_argument("--workspace", default="workspace_homography.yaml")
    parser.add_argument("--image", required=True)
    parser.add_argument("--bbox", nargs=4, type=float, required=True)
    parser.add_argument("--cam_height", type=float, required=True)
    parser.add_argument("--out", default="pose.json")
    args = parser.parse_args()

    camera = load_camera(args.camera)
    x_cam, y_cam, z, pixel = bbox_center_to_camera_xyz(args.bbox, camera, args.cam_height)
    workspace = load_workspace(args.workspace)
    if workspace is not None:
        x, y = pixel_to_base_m(workspace, pixel[0], pixel[1])
        source = "workspace_homography"
    else:
        x, y = x_cam, y_cam
        source = "camera_intrinsics_height"
    pose = {
        "x_m": x,
        "y_m": y,
        "z_m": z,
        "theta_deg": estimate_theta(args.image, args.bbox),
        "pixel": pixel,
        "source": source,
    }
    with open(args.out, "w", encoding="utf-8") as file:
        json.dump(pose, file, indent=2)
    print("Wrote", args.out)
    print(pose)


if __name__ == "__main__":
    main()
