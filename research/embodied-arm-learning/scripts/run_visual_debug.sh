#!/usr/bin/env bash
set -e

cd "$HOME/raspi_robot_ai"
source .venv/bin/activate
source scripts/pyenv.sh

set -a
[ -f config.demo.env ] && source config.demo.env
set +a

mkdir -p runtime/logs

echo "[debug] stopping old face/yolo camera processes..."
pkill -f "robot_ai/face_display.py" >/dev/null 2>&1 || true
pkill -f "robot_ai/02_yolo_detect.py" >/dev/null 2>&1 || true
pkill -f "robot_ai/01_camera_test.py" >/dev/null 2>&1 || true
pkill -f "robot_ai/12_av_sanity_check.py" >/dev/null 2>&1 || true
pkill -f "robot_ai/xiaou_demo_main.py" >/dev/null 2>&1 || true
pkill -f "robot_ai/xiaou_voice_demo.py" >/dev/null 2>&1 || true
pkill -f "scripts/run_demo_all.sh" >/dev/null 2>&1 || true
pkill -f "robot_ai/12_av_sanity_check.py" >/dev/null 2>&1 || true
pkill -f "robot_ai/01_camera_test.py" >/dev/null 2>&1 || true
pkill -f "rpicam" >/dev/null 2>&1 || true
pkill -f "libcamera" >/dev/null 2>&1 || true
sleep 1

echo "=============================================="
echo "Visual debug started"
echo "Face display: small window"
echo "YOLO preview disabled"
echo "Press Esc or q on the face window to quit"
echo "=============================================="

FACE_FULLSCREEN=false FACE_SHOW_LABEL=false FACE_DEBUG_BORDER=false FACE_WIDTH="${FACE_WIDTH:-240}" FACE_HEIGHT="${FACE_HEIGHT:-180}" python robot_ai/face_display.py
