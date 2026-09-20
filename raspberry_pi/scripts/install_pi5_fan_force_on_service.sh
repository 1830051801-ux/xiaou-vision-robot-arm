#!/usr/bin/env bash
set -e

SERVICE="/etc/systemd/system/pi5-fan-force-on.service"
SCRIPT="/usr/local/bin/pi5-fan-force-on.sh"

echo "Installing Raspberry Pi 5 fan force-on service..."

sudo tee "$SCRIPT" >/dev/null <<'EOF'
#!/usr/bin/env bash
set -e

# Raspberry Pi 5 FAN_PWM: output low = fan full speed on.
if command -v pinctrl >/dev/null 2>&1; then
  pinctrl FAN_PWM op dl
fi

# Keep process alive so systemd can restart it if needed.
while true; do
  sleep 60
  if command -v pinctrl >/dev/null 2>&1; then
    pinctrl FAN_PWM op dl
  fi
done
EOF

sudo chmod +x "$SCRIPT"

sudo tee "$SERVICE" >/dev/null <<EOF
[Unit]
Description=Force Raspberry Pi 5 fan always on
After=multi-user.target

[Service]
Type=simple
ExecStart=$SCRIPT
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable pi5-fan-force-on.service
sudo systemctl restart pi5-fan-force-on.service

echo "Done."
echo "Check status:"
echo "systemctl status pi5-fan-force-on.service --no-pager"
