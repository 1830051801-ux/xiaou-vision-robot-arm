#!/usr/bin/env bash
set -e
cd "$HOME/raspi_robot_ai"
source .venv/bin/activate
source scripts/pyenv.sh
set -a
[ -f config.demo.env ] && source config.demo.env
set +a
python robot_ai/xiaou_gui_demo.py
