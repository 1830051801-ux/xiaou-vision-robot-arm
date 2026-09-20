#!/usr/bin/env bash
set -euo pipefail
cd "$HOME/raspi_robot_ai"
source .venv/bin/activate
source scripts/pyenv.sh

set -a
[ -f config.demo.env ] && source config.demo.env
set +a

exec python robot_ai/xiaou_demo_main.py "$@"
