#!/usr/bin/env bash
set -euo pipefail

cd "$HOME/raspi_robot_ai"
source .venv/bin/activate
source scripts/pyenv.sh

if [ "$#" -ne 36 ]; then
  echo "Usage:"
  echo "  bash scripts/run_workspace_9point.sh \\"
  echo "    u1 v1 u2 v2 u3 v3 u4 v4 u5 v5 u6 v6 u7 v7 u8 v8 u9 v9 \\"
  echo "    x1 y1 x2 y2 x3 y3 x4 y4 x5 y5 x6 y6 x7 y7 x8 y8 x9 y9"
  echo ""
  echo "Point order must match. Recommended 3x3 row-major order:"
  echo "  top-left top-middle top-right middle-left center middle-right bottom-left bottom-middle bottom-right"
  exit 2
fi

python codex_pickup_package/create_workspace_homography.py \
  --pixels "$1" "$2" "$3" "$4" "$5" "$6" "$7" "$8" "$9" "${10}" "${11}" "${12}" "${13}" "${14}" "${15}" "${16}" "${17}" "${18}" \
  --base_mm "${19}" "${20}" "${21}" "${22}" "${23}" "${24}" "${25}" "${26}" "${27}" "${28}" "${29}" "${30}" "${31}" "${32}" "${33}" "${34}" "${35}" "${36}" \
  --output codex_pickup_package/workspace_homography.yaml
