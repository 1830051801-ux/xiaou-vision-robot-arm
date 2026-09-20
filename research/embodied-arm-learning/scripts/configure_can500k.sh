#!/usr/bin/env bash
set -euo pipefail

INTERFACE="${1:-can0}"
sudo ip link set "$INTERFACE" down 2>/dev/null || true
sudo ip link set "$INTERFACE" type can bitrate 500000 restart-ms 100
sudo ip link set "$INTERFACE" up
ip -details link show "$INTERFACE"
