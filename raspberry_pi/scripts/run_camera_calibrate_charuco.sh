#!/usr/bin/env bash
set -euo pipefail

cd "$HOME/raspi_robot_ai"
source .venv/bin/activate
source scripts/pyenv.sh

mkdir -p runtime/calibration
python codex_pickup_package/camera_calibration_charuco.py \
  --images "runtime/calibration/chessboard_images/*.jpg" \
  --squaresX "${CHARUCO_SQUARES_X:-8}" \
  --squaresY "${CHARUCO_SQUARES_Y:-10}" \
  --square_length "${CHARUCO_SQUARE_M:-0.01}" \
  --marker_length "${CHARUCO_MARKER_M:-0.005}" \
  --output runtime/calibration/camera_charuco.yaml
