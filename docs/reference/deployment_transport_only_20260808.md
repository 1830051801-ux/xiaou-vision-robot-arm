# Transport-only deployment package

Archive: `D:\机械臂\raspi_robot_ai_upload_20260808_transport_only.tar.gz`

SHA-256: calculate it locally with `Get-FileHash` before upload and compare it
with `D:\机械臂\raspi_robot_ai_upload_20260808_transport_only.sha256`.

The archive contains the Raspberry Pi source tree, ROS2 source packages,
offline tests, UART protocol tools, and safety documentation. It excludes
ROS2 build/install/log trees, Python caches, and simulation result files.

## Upload from Windows

```powershell
scp D:\机械臂\raspi_robot_ai_upload_20260808_transport_only.tar.gz pi@<PI_IP>:/home/pi/
ssh pi@<PI_IP> "sha256sum /home/pi/raspi_robot_ai_upload_20260808_transport_only.tar.gz"
```

The printed hash must match the external `.sha256` file before extraction. Keep the old
Pi project as a backup and extract into a new staging directory:

```bash
mkdir -p /home/pi/raspi_robot_ai_staging_20260808
tar -xzf /home/pi/raspi_robot_ai_upload_20260808_transport_only.tar.gz -C /home/pi/raspi_robot_ai_staging_20260808
mv /home/pi/raspi_robot_ai_staging_20260808/raspi_robot_ai_safety_20260808 /home/pi/raspi_robot_ai_transport_only_20260808
cd /home/pi/raspi_robot_ai_transport_only_20260808
python3 -m compileall -q robot_ai tests tools
python3 -m unittest discover -s tests -v
python3 tools/verify_six_axis_stack.py
```

Do not bring up `can0`, open `/dev/serial0`, run `--execute`, or enable the
hardware motion gate during this deployment check.
