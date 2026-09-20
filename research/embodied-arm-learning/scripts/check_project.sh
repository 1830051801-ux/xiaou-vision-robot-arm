#!/usr/bin/env bash
set -euo pipefail

cd "$HOME/raspi_robot_ai"
source .venv/bin/activate
source scripts/pyenv.sh

python -m compileall -q robot_ai tests
python -m unittest discover -s tests -v
python robot_ai/preflight_check.py "$@"
