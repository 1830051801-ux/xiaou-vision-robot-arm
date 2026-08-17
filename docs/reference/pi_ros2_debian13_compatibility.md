# Pi ROS 2 Jazzy Deployment Note

The selected archive is the official `ros2-jazzy-20260618-linux-noble-arm64`
build. Its native Python extension is CPython 3.12. Debian 13 on the Pi uses
CPython 3.13, so sourcing the archive alone is insufficient: `rclpy` will fail
with a missing `_rclpy_pybind11.cpython-313` module.

Run `scripts/configure_ros2_python312.sh` after the archive has been verified
and extracted to `/home/pi/ros2/jazzy`. The script installs an isolated Python
3.12 runtime with `uv`, patches only ROS 2 console launcher shebangs, and writes
`~/.config/xiaou/ros2_env.sh`. It never changes `/usr/bin/python3` and never
adds an Ubuntu apt repository.

Run `scripts/verify_pi_environment.sh` next. A successful result must include
`rclpy=OK`, the core package checks, project unit tests, and
`motion_gate=LOCKED`. A successful CLI check alone is not enough.
