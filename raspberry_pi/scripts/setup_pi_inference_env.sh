#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="${1:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
VENV_ROOT="${XIAOU_INFERENCE_VENV:-$HOME/xiaou_releases/.venv_inference}"
PYTHON_BOOTSTRAP="${PYTHON_BOOTSTRAP:-python3}"

"$PYTHON_BOOTSTRAP" -m venv --system-site-packages "$VENV_ROOT"
"$VENV_ROOT/bin/python" -m pip install --upgrade pip
"$VENV_ROOT/bin/python" -m pip install -r "$PROJECT_ROOT/requirements_pi_inference.txt"
if ! "$VENV_ROOT/bin/python" -c 'import cv2' >/dev/null 2>&1; then
  echo "OpenCV is missing. Install the Raspberry Pi OS package, then rerun:" >&2
  echo "  sudo apt-get update && sudo apt-get install -y python3-opencv" >&2
  exit 2
fi
"$VENV_ROOT/bin/python" -c 'import cv2, numpy, onnxruntime, PIL; print("xiaou_inference_env=OK")'
echo "XiaoU inference environment ready: $VENV_ROOT"
