#!/usr/bin/env bash
set -e

USER_NAME="$(whoami)"
FILE="/etc/sudoers.d/010_${USER_NAME}_nopasswd"

echo "$USER_NAME ALL=(ALL) NOPASSWD:ALL" | sudo tee "$FILE" >/dev/null
sudo chmod 440 "$FILE"
sudo visudo -cf "$FILE"

echo "Passwordless sudo enabled for: $USER_NAME"
