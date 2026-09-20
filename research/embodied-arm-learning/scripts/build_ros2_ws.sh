#!/usr/bin/env bash
set -eo pipefail

DISTRO="${1:-${ROS_DISTRO:-jazzy}}"
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS2_PREFIX="${ROS2_PREFIX:-$HOME/ros2/${DISTRO}}"
if [[ ! -f "${ROS2_PREFIX}/setup.bash" ]]; then
  echo "ROS 2 setup file not found: ${ROS2_PREFIX}/setup.bash" >&2
  exit 2
fi
source "${ROS2_PREFIX}/setup.bash"
set -u
cd "${PROJECT_ROOT}/ros2_ws"
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --event-handlers console_direct+
echo "Build complete. Source: ${PROJECT_ROOT}/ros2_ws/install/setup.bash"
