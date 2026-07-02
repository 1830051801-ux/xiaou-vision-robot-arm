#!/usr/bin/env bash
set -e

cd "$HOME/raspi_robot_ai"
source .venv/bin/activate

set -a
[ -f config.demo.env ] && source config.demo.env
set +a

mkdir -p runtime/logs

pkill -f "robot_ai/face_display.py" >/dev/null 2>&1 || true
FACE_FULLSCREEN="${FACE_FULLSCREEN:-false}" \
FACE_SHOW_LABEL="${FACE_SHOW_LABEL:-true}" \
FACE_DEBUG_BORDER="${FACE_DEBUG_BORDER:-false}" \
python robot_ai/face_display.py > runtime/logs/face_display_av.log 2>&1 &
FACE_PID=$!

cleanup() {
  kill "$FACE_PID" >/dev/null 2>&1 || true
}
trap cleanup EXIT

python robot_ai/12_av_sanity_check.py --record-seconds 2 --speak --yolo "$@"
