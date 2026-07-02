#!/usr/bin/env bash
set -e

PROJECT="$HOME/raspi_robot_ai"
SERVICE_DIR="$HOME/.config/systemd/user"
SERVICE_FILE="$SERVICE_DIR/xiaou-demo.service"

mkdir -p "$SERVICE_DIR"

cat > "$SERVICE_FILE" <<EOF
[Unit]
Description=XiaoU robotic deskpet demo
After=default.target

[Service]
Type=simple
WorkingDirectory=$PROJECT
Environment=DISPLAY=:0
Environment=XAUTHORITY=$HOME/.Xauthority
ExecStart=/bin/bash $PROJECT/scripts/run_demo_all.sh
Restart=on-failure
RestartSec=3

[Install]
WantedBy=default.target
EOF

systemctl --user daemon-reload
systemctl --user enable xiaou-demo.service

echo "Installed user service: $SERVICE_FILE"
echo "Start:   systemctl --user start xiaou-demo.service"
echo "Stop:    systemctl --user stop xiaou-demo.service"
echo "Logs:    journalctl --user -u xiaou-demo.service -f"
