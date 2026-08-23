from __future__ import annotations

import json
import time
from pathlib import Path

from kalman_filter import OneDKalman


IN_PATH = Path("/tmp/det.json")
OUT_PATH = Path("/tmp/det_filtered.json")
kx = OneDKalman()
ky = OneDKalman()
kt = OneDKalman()


def bbox_center(bbox: list[float]) -> tuple[float, float]:
    xmin, ymin, xmax, ymax = bbox
    return (xmin + xmax) / 2.0, (ymin + ymax) / 2.0


def main_once() -> None:
    if not IN_PATH.exists():
        print(f"missing {IN_PATH}")
        return
    det = json.loads(IN_PATH.read_text(encoding="utf-8"))
    bbox = det.get("bbox")
    if not bbox:
        print("missing bbox")
        return
    u, v = bbox_center(bbox)
    out = {
        "name": det.get("name"),
        "bbox": bbox,
        "image": det.get("image", "/tmp/frame.jpg"),
        "pixel_filtered": [kx.update(u), ky.update(v)],
        "theta_deg": kt.update(float(det.get("theta_deg", 0.0))),
        "timestamp": time.time(),
    }
    OUT_PATH.write_text(json.dumps(out, ensure_ascii=False), encoding="utf-8")
    print("filtered ->", out)


if __name__ == "__main__":
    main_once()
