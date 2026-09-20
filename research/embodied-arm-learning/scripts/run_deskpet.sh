#!/usr/bin/env bash
set -e
cd "$HOME/raspi_robot_ai"
source .venv/bin/activate
source scripts/pyenv.sh
python codex_deskpet_package/run_demo.py "$@"
