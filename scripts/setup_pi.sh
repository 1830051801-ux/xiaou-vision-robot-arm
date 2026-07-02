#!/usr/bin/env bash
set -e

PROJECT_DIR="$HOME/raspi_robot_ai"
VENV_DIR="$PROJECT_DIR/.venv"

echo "[1/8] Checking project directory..."
if [ ! -d "$PROJECT_DIR" ]; then
  echo "Project directory not found: $PROJECT_DIR"
  echo "Copy raspi_robot_ai to the Raspberry Pi home directory first."
  exit 1
fi

echo "[2/8] Updating apt packages..."
sudo apt update

echo "[3/8] Installing system dependencies..."
sudo apt install -y \
  python3-venv \
  python3-pip \
  python3-tk \
  python3-opencv \
  python3-picamera2 \
  python3-pil \
  python3-pil.imagetk \
  python3-pygame \
  libopenblas-dev \
  portaudio19-dev \
  ffmpeg \
  mpg123 \
  v4l-utils \
  alsa-utils \
  espeak-ng \
  rpicam-apps \
  git

echo "[4/8] Adding current user to hardware groups..."
sudo usermod -aG dialout,video,audio "$USER" || true

echo "[5/8] Creating Python virtual environment..."
cd "$PROJECT_DIR"
if [ ! -d "$VENV_DIR" ]; then
  python3 -m venv --system-site-packages "$VENV_DIR"
fi

echo "[6/8] Installing Python packages..."
source "$VENV_DIR/bin/activate"
python -m pip install --upgrade pip
pip install -r requirements.txt

echo "[7/8] Creating config.env if missing..."
if [ ! -f "$PROJECT_DIR/config.env" ]; then
  cp "$PROJECT_DIR/config.env.example" "$PROJECT_DIR/config.env"
fi

echo "[8/8] Making scripts executable..."
chmod +x "$PROJECT_DIR"/scripts/*.sh

echo "Done."
echo ""
echo "Next commands:"
echo "cd ~/raspi_robot_ai"
echo "bash scripts/run_menu.sh"
