# Pi Deployment Manifest

Prepared on 2026-08-06. The complete project archive is generated beside the
workspace as `D:\机械臂\raspi_robot_ai_upload_20260806_final.tar.gz`. Verify its
SHA-256 from the deployment log before uploading; the archive is not trusted
until that check passes.

## Tomorrow's order

1. Boot the Pi on the same phone hotspot and confirm `ssh pi@172.20.10.3`.
2. Keep `/home/pi/raspi_robot_ai_backup_20260806_2045`; do not delete it.
3. Upload the archive, verify its SHA-256 on the Pi, and extract it over a new
   staging directory before switching the project symlink.
4. Verify the ROS 2 archive hash:
   `e672764081b53e4c0f414de832a841c610631f96eadc8f156dc773409694f155`.
5. Run `bash scripts/configure_ros2_python312.sh`.
6. Run `bash scripts/verify_pi_environment.sh` and save its output under
   `runtime/deployment_logs/`.
7. Build only after the offline checks pass:
   `source ~/.config/xiaou/ros2_env.sh && cd ros2_ws && colcon build --symlink-install`.
8. Do not bring up `can0`, start the CAN hardware plugin, or send any CAN/UART
   actuator command during this deployment verification.

The project is ready for STM32 loopback and passive identification, but the
motion gate must remain closed until IDs, encoder signs, zero offsets, limits,
feedback watchdog behavior, and E-stop response are measured.
