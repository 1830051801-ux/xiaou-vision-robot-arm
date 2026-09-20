#!/usr/bin/env bash
set -e

CONFIG="/boot/firmware/config.txt"
BACKUP="/boot/firmware/config.txt.bak.$(date +%Y%m%d_%H%M%S)"

if [ ! -f "$CONFIG" ]; then
  echo "Config not found: $CONFIG"
  exit 1
fi

echo "Backing up $CONFIG to $BACKUP"
sudo cp "$CONFIG" "$BACKUP"

TMP="$(mktemp)"
sudo grep -v '^dtparam=fan_temp[0-3]' "$CONFIG" | sudo grep -v '^dtparam=cooling_fan=' | sudo tee "$TMP" >/dev/null

cat <<'EOF' | sudo tee -a "$TMP" >/dev/null

# Robot project: keep Raspberry Pi 5 4-pin fan almost always on.
# Values are in millicelsius; speed is PWM 0-255.
dtparam=cooling_fan=on
dtparam=fan_temp0=1000
dtparam=fan_temp0_hyst=0
dtparam=fan_temp0_speed=255
dtparam=fan_temp1=50000
dtparam=fan_temp1_hyst=5000
dtparam=fan_temp1_speed=255
dtparam=fan_temp2=60000
dtparam=fan_temp2_hyst=5000
dtparam=fan_temp2_speed=255
dtparam=fan_temp3=70000
dtparam=fan_temp3_hyst=5000
dtparam=fan_temp3_speed=255
EOF

sudo cp "$TMP" "$CONFIG"
rm -f "$TMP"

echo "Trying to force fan full speed immediately..."
if command -v pinctrl >/dev/null 2>&1; then
  sudo pinctrl FAN_PWM op dl || true
else
  echo "pinctrl not found; config will apply after reboot."
fi

echo "Done. Reboot is required:"
echo "sudo reboot"
echo ""
echo "After reboot, check temperature:"
echo "vcgencmd measure_temp"
