#!/usr/bin/env bash
set -euo pipefail

cd "$HOME/raspi_robot_ai"
source .venv/bin/activate
source scripts/pyenv.sh

if [ "$#" -lt 4 ] || [ $(( $# % 2 )) -ne 0 ]; then
  echo "Usage:"
  echo "  bash scripts/run_rotation_calibration.sh image_theta1 robot_rz1 image_theta2 robot_rz2 ..."
  echo ""
  echo "Example:"
  echo "  bash scripts/run_rotation_calibration.sh 10 15 -20 -14 35 40"
  exit 2
fi

args=()
while [ "$#" -gt 0 ]; do
  args+=(--pair "$1" "$2")
  shift 2
done

python robot_ai/17_rotation_calibration.py "${args[@]}"
