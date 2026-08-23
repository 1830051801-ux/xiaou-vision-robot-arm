from __future__ import annotations

import argparse

import cv2


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--squaresX", type=int, default=8, help="number of squares along X")
    parser.add_argument("--squaresY", type=int, default=10, help="number of squares along Y")
    parser.add_argument("--square_length", type=float, default=0.01, help="square size in meters, 10mm = 0.01m")
    parser.add_argument("--marker_length", type=float, default=0.005, help="marker size in meters, 5mm = 0.005m")
    parser.add_argument("--width_px", type=int, default=1200)
    parser.add_argument("--height_px", type=int, default=1600)
    parser.add_argument("--out", default="charuco_board.png")
    args = parser.parse_args()

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
    if hasattr(cv2.aruco, "CharucoBoard"):
        board = cv2.aruco.CharucoBoard((args.squaresX, args.squaresY), args.square_length, args.marker_length, aruco_dict)
        image = board.generateImage((args.width_px, args.height_px))
    else:
        board = cv2.aruco.CharucoBoard_create(args.squaresX, args.squaresY, args.square_length, args.marker_length, aruco_dict)
        image = board.draw((args.width_px, args.height_px))
    cv2.imwrite(args.out, image)
    print("Wrote", args.out)


if __name__ == "__main__":
    main()
