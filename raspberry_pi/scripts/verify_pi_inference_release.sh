#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="${1:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
if [[ -z "${PYTHON_BIN:-}" && -x "$HOME/xiaou_releases/.venv_inference/bin/python" ]]; then
  PYTHON_BIN="$HOME/xiaou_releases/.venv_inference/bin/python"
else
  PYTHON_BIN="${PYTHON_BIN:-python3}"
fi
exec "$PYTHON_BIN" "$PROJECT_ROOT/tools/verify_pi_inference_release.py" --root "$PROJECT_ROOT"
