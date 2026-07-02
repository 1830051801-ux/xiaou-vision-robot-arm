#!/usr/bin/env bash
set -e
cd "$HOME/raspi_robot_ai"
source .venv/bin/activate
set -a
[ -f config.demo.env ] && source config.demo.env
set +a
python robot_ai/face_display.py
