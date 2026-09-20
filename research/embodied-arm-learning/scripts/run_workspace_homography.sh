#!/usr/bin/env bash
set -euo pipefail

cd "$HOME/raspi_robot_ai"
source .venv/bin/activate
source scripts/pyenv.sh

if [ "$#" -ne 16 ]; then
  echo "Usage:"
  echo "  bash scripts/run_workspace_homography.sh u1 v1 u2 v2 u3 v3 u4 v4 x1 y1 x2 y2 x3 y3 x4 y4"
  echo ""
  echo "Point order must match, for example:"
  echo "  left_top right_top right_bottom left_bottom"
  exit 2
fi

python codex_pickup_package/create_workspace_homography.py \
  --pixels "$1" "$2" "$3" "$4" "$5" "$6" "$7" "$8" \
  --base_mm "$9" "${10}" "${11}" "${12}" "${13}" "${14}" "${15}" "${16}" \
  --output codex_pickup_package/workspace_homography.yaml
