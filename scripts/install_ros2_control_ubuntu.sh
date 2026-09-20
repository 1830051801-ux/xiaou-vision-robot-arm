#!/usr/bin/env bash
set -euo pipefail

# Ubuntu 24.04 -> Jazzy; Ubuntu 22.04 -> Humble.
DISTRO="${1:-${ROS_DISTRO:-jazzy}}"
case "$DISTRO" in
  humble|jazzy) ;;
  *) echo "supported ROS2 distros: humble or jazzy" >&2; exit 2 ;;
esac

sudo apt-get update
sudo apt-get install -y software-properties-common curl ca-certificates
sudo add-apt-repository universe -y
if [ -f /etc/apt/sources.list.d/ros2.sources ]; then
  # Newer ROS tooling installs deb822 ros2.sources with an embedded key.
  sudo rm -f /etc/apt/sources.list.d/ros2.list
else
  sudo curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo \"$UBUNTU_CODENAME\") main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
fi
sudo apt-get update
sudo apt-get install -y \
  "ros-${DISTRO}-ros2-control" \
  "ros-${DISTRO}-ros2-controllers" \
  "ros-${DISTRO}-controller-manager" \
  "ros-${DISTRO}-hardware-interface" \
  "ros-${DISTRO}-xacro" \
  "python3-rosdep" \
  "python3-colcon-common-extensions"

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init || true
fi
rosdep update

echo "Installed ROS2 ${DISTRO} control dependencies."
