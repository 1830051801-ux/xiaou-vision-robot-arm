#!/usr/bin/env bash
set -euo pipefail

cd "$HOME/raspi_robot_ai"
source .venv/bin/activate
export PYTHONPATH="$HOME/raspi_robot_ai:$HOME/raspi_robot_ai/robot_ai${PYTHONPATH:+:$PYTHONPATH}"

set -a
[ -f config.demo.env ] && source config.demo.env
set +a

mkdir -p runtime/logs

echo "[demo] stopping old demo/display/camera processes..."
pkill -f "robot_ai/02_yolo_detect.py" >/dev/null 2>&1 || true
pkill -f "robot_ai/01_camera_test.py" >/dev/null 2>&1 || true
sleep 1

echo "=============================================="
echo "XiaoU coordinate grasp demo started"
echo "Pipeline: camera -> YOLO -> calibrated XY -> UART -> STM32"
echo "Default is dry-run. Add --send-serial after checking the target coordinates."
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

python robot_ai/coordinate_grasp_demo.py "$@"
