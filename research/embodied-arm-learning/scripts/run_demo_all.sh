#!/usr/bin/env bash
set -euo pipefail

cd "$HOME/raspi_robot_ai"
source .venv/bin/activate
source scripts/pyenv.sh

set -a
[ -f config.demo.env ] && source config.demo.env
set +a

mkdir -p runtime/logs

echo "[demo] checking model, calibration and six-axis stack..."
python robot_ai/preflight_check.py

echo "[demo] stopping old demo/display/camera processes..."
pkill -f "robot_ai/xiaou_voice_demo.py" >/dev/null 2>&1 || true
pkill -f "robot_ai/xiaou_gui_demo.py" >/dev/null 2>&1 || true
pkill -f "robot_ai/02_yolo_detect.py" >/dev/null 2>&1 || true
pkill -f "robot_ai/01_camera_test.py" >/dev/null 2>&1 || true
pkill -f "robot_ai/12_av_sanity_check.py" >/dev/null 2>&1 || true
sleep 1

echo "=============================================="
echo "XiaoU demo started"
echo "Chat + face: xiaou_gui_demo.py"
echo "YOLO preview: disabled by default for smoother XiaoU UI"
echo "Exit: close the chat window"
echo "=============================================="

YOLO_PID=""
cleanup() {
    if [ -n "$YOLO_PID" ] && kill -0 "$YOLO_PID" >/dev/null 2>&1; then
        kill "$YOLO_PID" >/dev/null 2>&1 || true
    fi
}
trap cleanup EXIT INT TERM

if [ "${XIAOU_OPEN_YOLO:-0}" = "1" ]; then
    echo "[demo] starting YOLO preview in background..."
    python robot_ai/02_yolo_detect.py > runtime/logs/yolo_preview.log 2>&1 &
    YOLO_PID=$!
    sleep 1
fi

python robot_ai/xiaou_gui_demo.py "$@"
