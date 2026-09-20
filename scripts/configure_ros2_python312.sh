#!/usr/bin/env bash
set -euo pipefail

# ROS 2 Jazzy binary archives are built for Python 3.12. Debian 13 ships
# Python 3.13, so keep the system interpreter untouched and isolate ROS 2.
ROS2_PREFIX="${ROS2_PREFIX:-$HOME/ros2/jazzy}"
TOOLS_VENV="${TOOLS_VENV:-$HOME/.venvs/ros2-tools}"
ENV_FILE="$HOME/.config/xiaou/ros2_env.sh"

if [[ ! -f "$ROS2_PREFIX/setup.bash" ]]; then
  echo "ROS 2 setup file not found: $ROS2_PREFIX/setup.bash" >&2
  exit 2
fi

PY312=""
if command -v python3.12 >/dev/null 2>&1; then
  PY312="$(command -v python3.12)"
elif PY312="$(compgen -G "$HOME/.local/share/uv/python/cpython-3.12"*/bin/python3.12 | head -n 1)" && [[ -x "$PY312" ]]; then
  :
else
  python3 -m venv "$TOOLS_VENV"
  "$TOOLS_VENV/bin/pip" install --quiet uv
  "$TOOLS_VENV/bin/uv" python install 3.12
  PY312="$("$TOOLS_VENV/bin/uv" python find 3.12)"
fi

if [[ -z "$PY312" || ! -x "$PY312" ]]; then
  echo "Python 3.12 runtime was not found" >&2
  exit 3
fi

# Replace absolute /usr/bin/python3 shebangs in ROS 2 console launchers. This
# keeps rclpy and all generated ROS executables on the matching CPython ABI.
while IFS= read -r -d '' launcher; do
  if head -n 1 "$launcher" | grep -qE '^#!.*python3([[:space:]]|$)'; then
    sed -i "1c#!$PY312" "$launcher"
  fi
done < <(find "$ROS2_PREFIX/bin" -maxdepth 1 -type f -perm -u+x -print0)

mkdir -p "$HOME/.local/bin" "$(dirname "$ENV_FILE")"
cat > "$HOME/.local/bin/ros2" <<EOF
#!/usr/bin/env bash
exec "$PY312" "$ROS2_PREFIX/bin/ros2" "\$@"
EOF
cat > "$HOME/.local/bin/colcon" <<EOF
#!/usr/bin/env bash
exec "$PY312" "$ROS2_PREFIX/bin/colcon" "\$@"
EOF
chmod +x "$HOME/.local/bin/ros2" "$HOME/.local/bin/colcon"

cat > "$ENV_FILE" <<EOF
export ROS2_PREFIX="$ROS2_PREFIX"
export XIAOU_PROJECT_ROOT="${XIAOU_PROJECT_ROOT:-$HOME/raspi_robot_ai}"
source "$ROS2_PREFIX/setup.bash"
export PATH="$HOME/.local/bin:\$PATH"
export ROS_PYTHON_VERSION=3.12
EOF

if ! grep -q 'XIAOU_ROS2_PY312_ENV' "$HOME/.bashrc" 2>/dev/null; then
  cat >> "$HOME/.bashrc" <<'EOF'

# XIAOU_ROS2_PY312_ENV
if [ -f "$HOME/.config/xiaou/ros2_env.sh" ]; then
  . "$HOME/.config/xiaou/ros2_env.sh"
fi
EOF
fi

echo "ROS2_PREFIX=$ROS2_PREFIX"
echo "PY312=$PY312"
"$PY312" -c 'import sys, rclpy; print(sys.version); print("rclpy=OK")'
"$HOME/.local/bin/ros2" pkg list >/dev/null
echo "ROS 2 Python 3.12 environment is ready."
