#!/usr/bin/env bash
set -e
cd "$HOME/raspi_robot_ai"
source .venv/bin/activate
source scripts/pyenv.sh
exec bash scripts/run_demo_all.sh "$@"
