#!/usr/bin/env bash
set -e

cd "$HOME/raspi_robot_ai"
CONFIG_FILE="config.env"

if [ ! -f "$CONFIG_FILE" ]; then
  cp config.env.example "$CONFIG_FILE"
fi

echo "Baidu ASR config repair"
echo "This will remove old broken BAIDU_ASR lines and write clean lines."
echo ""
read -rp "Paste Baidu API Key: " BAIDU_API_KEY
read -rsp "Paste Baidu Secret Key: " BAIDU_SECRET_KEY
echo ""

BAIDU_API_KEY="$(printf '%s' "$BAIDU_API_KEY" | tr -d '[:space:]')"
BAIDU_SECRET_KEY="$(printf '%s' "$BAIDU_SECRET_KEY" | tr -d '[:space:]')"

if [ -z "$BAIDU_API_KEY" ] || [ -z "$BAIDU_SECRET_KEY" ]; then
  echo "API Key or Secret Key is empty. Abort."
  exit 1
fi

if [ "${#BAIDU_API_KEY}" -lt 10 ] || [ "${#BAIDU_SECRET_KEY}" -lt 10 ]; then
  echo "Key looks too short. Abort."
  exit 1
fi

if [ "${#BAIDU_SECRET_KEY}" -gt 80 ]; then
  echo "Secret Key looks too long. You may have pasted it repeatedly. Abort."
  echo "API Key length: ${#BAIDU_API_KEY}"
  echo "Secret Key length: ${#BAIDU_SECRET_KEY}"
  exit 1
fi

TMP_FILE="$(mktemp)"
grep -v 'BAIDU_ASR_' "$CONFIG_FILE" | sed 's/\\n/\n/g' > "$TMP_FILE"
mv "$TMP_FILE" "$CONFIG_FILE"

cat >> "$CONFIG_FILE" <<EOF

BAIDU_ASR_API_KEY=$BAIDU_API_KEY
BAIDU_ASR_SECRET_KEY=$BAIDU_SECRET_KEY
BAIDU_ASR_DEV_PID=1537
BAIDU_ASR_RATE=16000
EOF

echo ""
echo "Done. Current Baidu ASR config:"
grep 'BAIDU_ASR_' "$CONFIG_FILE" | sed -E 's/(BAIDU_ASR_API_KEY=).+/\1****/; s/(BAIDU_ASR_SECRET_KEY=).+/\1****/'
echo ""
echo "Next:"
echo "bash scripts/run_baidu_asr_test.sh"
