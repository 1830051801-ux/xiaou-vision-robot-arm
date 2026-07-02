#!/usr/bin/env bash
set -e

cd "$HOME/raspi_robot_ai"
source .venv/bin/activate
source scripts/pyenv.sh

set -a
[ -f config.demo.env ] && source config.demo.env
set +a

mkdir -p runtime/logs

echo "[demo] stopping old demo/display/camera processes..."
pkill -f "robot_ai/xiaou_voice_demo.py" >/dev/null 2>&1 || true
pkill -f "robot_ai/xiaou_gui_demo.py" >/dev/null 2>&1 || true
pkill -f "robot_ai/02_yolo_detect.py" >/dev/null 2>&1 || true
pkill -f "robot_ai/01_camera_test.py" >/dev/null 2>&1 || true
pkill -f "robot_ai/12_av_sanity_check.py" >/dev/null 2>&1 || true
sleep 1

echo "=============================================="
echo "XiaoU voice dialog demo started"
echo "Mic: press Enter to record"
echo "Face display: embedded in dialog window"
echo "One dialog window with embedded face panel"
echo "Exit: close the windows"
echo "=============================================="

python robot_ai/xiaou_gui_demo.py "$@"
