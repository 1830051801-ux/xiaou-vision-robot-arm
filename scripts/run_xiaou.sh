#!/usr/bin/env bash
set -e
cd "$HOME/raspi_robot_ai"
source .venv/bin/activate
exec bash scripts/run_demo_all.sh "$@"
