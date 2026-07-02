#!/usr/bin/env bash
set -e

cd "$HOME/raspi_robot_ai"
source .venv/bin/activate

python robot_ai/16_stm_motion_send_loop.py --port /dev/serial0 --baud 115200 --interval 5
