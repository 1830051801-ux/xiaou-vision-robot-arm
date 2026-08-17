#!/usr/bin/env bash
set -e
cd "$HOME/raspi_robot_ai"
source .venv/bin/activate
python - <<'PY'
from pathlib import Path
from PIL import Image, ImageSequence

project = Path.home() / "raspi_robot_ai"
src = project / "robot_ai" / "emote_assets" / "gif" / "idle.gif"
out = project / "face_debug_idle.png"

def green_to_black(image):
    image = image.convert("RGB")
    pixels = image.load()
    width, height = image.size
    for y in range(height):
        for x in range(width):
            r, g, b = pixels[x, y]
            if g > 150 and g > r * 1.4 and g > b * 1.4:
                pixels[x, y] = (0, 0, 0)
    return image

with Image.open(src) as gif:
    frame = next(ImageSequence.Iterator(gif)).copy()
    image = green_to_black(frame)
    image.thumbnail((480, 320), Image.Resampling.NEAREST)
    canvas = Image.new("RGB", (480, 320), (0, 0, 0))
    canvas.paste(image, ((480 - image.width) // 2, (320 - image.height) // 2))
    canvas.save(out)

print(out)
PY
