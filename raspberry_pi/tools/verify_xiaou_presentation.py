#!/usr/bin/env python3
"""Offline audit for XiaoU's face assets, mappings, and Control Tower bridge."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Any

from PIL import Image, ImageSequence


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT / "robot_ai") not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT / "robot_ai"))
if str(PROJECT_ROOT / "codex_emote_ai_package") not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT / "codex_emote_ai_package"))

from dialog_emote_bridge import EMOTE_TO_FACE_STATE
from emote_player import EMOTE_TO_ASSET
from face_animation import STYLE_MAP
from face_display import FACE_STATE_TO_GIF


def audit() -> dict[str, Any]:
    failures: list[str] = []
    assets: list[dict[str, Any]] = []
    root = PROJECT_ROOT / "robot_ai/emote_assets/gif"
    for state, filename in sorted(FACE_STATE_TO_GIF.items()):
        path = root / filename
        if not path.is_file():
            failures.append(f"face_state_asset_missing:{state}:{filename}")
            continue
        try:
            with Image.open(path) as image:
                frame_count = sum(1 for _ in ImageSequence.Iterator(image))
                assets.append({"state": state, "file": filename, "bytes": path.stat().st_size, "frames": frame_count, "size": list(image.size)})
                if frame_count < 1:
                    failures.append(f"face_state_asset_empty:{state}")
        except Exception as exc:
            failures.append(f"face_state_asset_invalid:{state}:{exc}")
    unsupported_canvas_states = sorted(set(FACE_STATE_TO_GIF) - set(STYLE_MAP))
    if unsupported_canvas_states:
        failures.append("canvas_state_missing:" + ",".join(unsupported_canvas_states))
    dialog_states = sorted(set(EMOTE_TO_FACE_STATE.values()) - set(FACE_STATE_TO_GIF))
    if dialog_states:
        failures.append("dialog_face_state_missing:" + ",".join(dialog_states))
    missing_emote_assets = sorted(
        emote for emote, filename in EMOTE_TO_ASSET.items() if not (root / filename).is_file()
    )
    if missing_emote_assets:
        failures.append("dialog_emote_asset_missing:" + ",".join(missing_emote_assets))
    required_files = (
        "robot_ai/face_state.py",
        "robot_ai/face_animation.py",
        "robot_ai/face_display.py",
        "robot_ai/face_presentation.py",
        "robot_ai/xiaou_status_dashboard.py",
        "scripts/run_face_screen.sh",
        "scripts/run_xiaou_face_screen.sh",
    )
    missing_files = [path for path in required_files if not (PROJECT_ROOT / path).is_file()]
    failures.extend("required_file_missing:" + path for path in missing_files)
    dashboard_source = (PROJECT_ROOT / "robot_ai/xiaou_status_dashboard.py").read_text(encoding="utf-8")
    for required in ("CanvasFaceAnimator", "recommend_face", "实时表情", "任务表情预览"):
        if required not in dashboard_source:
            failures.append(f"control_tower_face_bridge_missing:{required}")
    return {
        "offline": True,
        "hardware_motion": False,
        "serial_opened": False,
        "can_opened": False,
        "face_state_count": len(FACE_STATE_TO_GIF),
        "gif_file_count": len(list(root.glob("*.gif"))),
        "assets": assets,
        "canvas_state_count": len(STYLE_MAP),
        "unsupported_canvas_states": unsupported_canvas_states,
        "missing_emote_assets": missing_emote_assets,
        "missing_required_files": missing_files,
        "failures": failures,
        "passed": not failures,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=PROJECT_ROOT / "runtime/simulations/presentation_audit_20260809.json")
    args = parser.parse_args()
    result = audit()
    output = args.output if args.output.is_absolute() else PROJECT_ROOT / args.output
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
