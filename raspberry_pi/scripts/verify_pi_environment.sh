#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="${XIAOU_PROJECT_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
ROS2_PREFIX="${ROS2_PREFIX:-$HOME/ros2/jazzy}"
ENV_FILE="$HOME/.config/xiaou/ros2_env.sh"
[[ -f "$ENV_FILE" ]] && source "$ENV_FILE"
source "$ROS2_PREFIX/setup.bash"

PY312="$(command -v python3.12 || true)"
if [[ -z "$PY312" ]]; then
  PY312="$(find "$HOME/.local/share/uv/python" -path '*/bin/python3.12' -type f 2>/dev/null | head -n 1 || true)"
fi
[[ -n "$PY312" && -x "$PY312" ]] || { echo "Python 3.12 missing" >&2; exit 2; }

echo "== versions =="
"$PY312" --version
"$PY312" -c 'import rclpy; print("rclpy=OK")'
ros2 --help >/dev/null
echo "ros2_cli=OK"

echo "== package index =="
for package in rclcpp rclpy geometry_msgs sensor_msgs xacro robot_state_publisher; do
  ros2 pkg prefix "$package" >/dev/null
  echo "$package=OK"
done

echo "== workspace =="
cd "$PROJECT_ROOT/ros2_ws"
colcon list
python3 -m compileall -q "$PROJECT_ROOT/robot_ai" "$PROJECT_ROOT/tools"
python3 -m pytest -q "$PROJECT_ROOT/tests"

echo "== safety checks =="
! ip link show can0 >/dev/null 2>&1 || { echo "can0 exists; do not bring it up in this verification"; }
grep -q '"motion_enabled": false' "$PROJECT_ROOT/robot_ai/arm_control/config/hardware_calibration.json"
echo "motion_gate=LOCKED"
echo "Offline environment verification passed. No CAN frame was sent."
