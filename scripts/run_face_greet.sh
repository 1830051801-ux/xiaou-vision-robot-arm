#!/usr/bin/env bash
set -e
cd "$HOME/raspi_robot_ai"
source .venv/bin/activate
python codex_deskpet_package/face_greet.py "$@"
