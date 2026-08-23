#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="${XIAOU_PROJECT_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
if [[ -z "${PYTHON_BIN:-}" && -x "$HOME/xiaou_releases/.venv_inference/bin/python" ]]; then
  PYTHON_BIN="$HOME/xiaou_releases/.venv_inference/bin/python"
else
  PYTHON_BIN="${PYTHON_BIN:-python3}"
fi
exec "$PYTHON_BIN" "$PROJECT_ROOT/robot_ai/xiaou_status_dashboard.py" "$@"
