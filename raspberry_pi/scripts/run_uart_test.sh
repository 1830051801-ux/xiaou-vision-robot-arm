#!/usr/bin/env bash
set -e

cd "$HOME/raspi_robot_ai"
source .venv/bin/activate

PORT="/dev/serial0"
if [ ! -e "$PORT" ]; then
  if [ -e "/dev/ttyAMA10" ]; then
    PORT="/dev/ttyAMA10"
  elif [ -e "/dev/ttyAMA0" ]; then
    PORT="/dev/ttyAMA0"
  fi
fi

python robot_ai/14_uart_test_thread.py --port "$PORT" --rx FF --reply FF
