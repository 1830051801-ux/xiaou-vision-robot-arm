#!/usr/bin/env bash
set -e
cd "$HOME/raspi_robot_ai"
source .venv/bin/activate

pkill -f "robot_ai/face_display.py" >/dev/null 2>&1 || true
exec bash scripts/run_demo_all.sh "$@"
