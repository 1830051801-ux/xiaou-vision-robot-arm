#!/usr/bin/env bash
set -euo pipefail

cd "$HOME/raspi_robot_ai"
source .venv/bin/activate
source scripts/pyenv.sh

mkdir -p runtime/calibration/chessboard_images
python codex_pickup_package/capture_calib_images.py \
  --camera "${CAMERA_INDEX:-0}" \
  --outdir runtime/calibration/chessboard_images \
  --prefix chess
