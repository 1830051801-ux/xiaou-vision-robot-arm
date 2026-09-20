#!/usr/bin/env bash
set -euo pipefail

cd "$HOME/raspi_robot_ai"
source .venv/bin/activate
source scripts/pyenv.sh

mkdir -p runtime/calibration
# 8x10 个格子 -> 7x9 个内角点；每格 10mm -> 0.01m
python codex_pickup_package/camera_calibration_chessboard.py \
  --images "runtime/calibration/chessboard_images/*.jpg" \
  --cols "${CHESSBOARD_COLS:-7}" \
  --rows "${CHESSBOARD_ROWS:-9}" \
  --square_size "${CHESSBOARD_SQUARE_M:-0.01}" \
  --output runtime/calibration/camera.yaml \
  --debug_dir runtime/calibration/chessboard_debug
