#!/usr/bin/env bash
set -e
cd "$HOME/raspi_robot_ai"
source .venv/bin/activate
export PYTHONPATH="$HOME/raspi_robot_ai:$HOME/raspi_robot_ai/robot_ai${PYTHONPATH:+:$PYTHONPATH}"
exec bash scripts/run_demo_all.sh "$@"
