#!/usr/bin/env bash
set -e

cd "$HOME/raspi_robot_ai"
source .venv/bin/activate

echo "ALSA capture devices:"
arecord -l || true
echo ""
echo "ALSA playback devices:"
aplay -l || true
echo ""
echo "Sounddevice devices:"
python - <<'PY'
import sounddevice as sd
for i, d in enumerate(sd.query_devices()):
    if d.get("max_input_channels", 0) > 0 or d.get("max_output_channels", 0) > 0:
        print(i, d["name"], "| in=", d.get("max_input_channels", 0), "out=", d.get("max_output_channels", 0))
PY
echo ""
echo "To pin devices, edit config.demo.env and set:"
echo "AUDIO_INPUT_DEVICE=<index or substring>"
echo "AUDIO_OUTPUT_DEVICE=<index or substring>"
