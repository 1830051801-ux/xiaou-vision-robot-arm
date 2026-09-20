#!/usr/bin/env bash
set -e

cd "$HOME/raspi_robot_ai"
source .venv/bin/activate

set -a
[ -f config.demo.env ] && source config.demo.env
set +a

python robot_ai/13_audio_debug.py --seconds "${AUDIO_TEST_SECONDS:-3}" --speak
