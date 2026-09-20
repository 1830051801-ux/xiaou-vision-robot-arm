#!/usr/bin/env bash
set -e

cd "$HOME/raspi_robot_ai"
CONFIG_FILE="config.env"

if [ ! -f "$CONFIG_FILE" ]; then
  cp config.env.example "$CONFIG_FILE"
fi

set_default() {
  local key="$1"
  local value="$2"
  if grep -q "^${key}=" "$CONFIG_FILE"; then
    sed -i "s|^${key}=.*|${key}=${value}|" "$CONFIG_FILE"
  else
    printf "%s=%s\n" "$key" "$value" >> "$CONFIG_FILE"
  fi
}

set_default ROBOT_NAME 小U
set_default PIXEL_ORIGIN_U 320
set_default PIXEL_ORIGIN_V 240
set_default PIXEL_TO_BASE_SCALE_X 0.50
set_default PIXEL_TO_BASE_SCALE_Y 0.50
set_default BASE_OFFSET_X_MM 200
set_default BASE_OFFSET_Y_MM 0
set_default WORKSPACE_X_MIN_MM 80
set_default WORKSPACE_X_MAX_MM 360
set_default WORKSPACE_Y_MIN_MM -180
set_default WORKSPACE_Y_MAX_MM 180
set_default Z_SAFE_MM 80
set_default Z_GRAB_MM 25
set_default MIN_TARGET_AREA_RATIO 0.015
set_default TARGET_FILTER_WINDOW 5
set_default TARGET_STABLE_FRAMES 3
set_default TARGET_TIMEOUT_S 5
set_default GRIPPER_OPEN_MARGIN_MM 15
set_default GRIPPER_CLOSE_MARGIN_MM 8
set_default GRIPPER_OPEN_MIN_MM 35
set_default GRIPPER_OPEN_MAX_MM 95
set_default GRIPPER_FORCE_PCT 60
set_default TTS_ENGINE local

echo "Robot safety defaults applied:"
grep 'ROBOT_NAME\|PIXEL_ORIGIN\|PIXEL_TO_BASE\|BASE_OFFSET\|WORKSPACE\|Z_SAFE\|Z_GRAB\|TARGET_\|GRIPPER_' "$CONFIG_FILE"
