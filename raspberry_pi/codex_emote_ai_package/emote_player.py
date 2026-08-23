from __future__ import annotations

import os
from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parent
PROJECT_DIR = PACKAGE_DIR.parent
GIF_ASSET_DIR = PROJECT_DIR / "robot_ai" / "emote_assets" / "gif"
PNG_ASSET_DIR = PROJECT_DIR / "robot_ai" / "emote_assets" / "png"

EMOTE_TO_ASSET = {
    "idle": "idle.gif",
    "happy": "laugh.gif",
    "thankful": "smile.gif",
    "apologize": "sad.gif",
    "agree": "smile.gif",
    "disagree": "angry.gif",
    "thinking": "ponder.gif",
    "confused": "question.gif",
    "celebrate": "laugh.gif",
    "sad": "sad.gif",
}


def asset_path_for_emote(emote: str) -> Path | None:
    filename = EMOTE_TO_ASSET.get(emote, f"{emote}.gif")
    gif_path = GIF_ASSET_DIR / filename
    if gif_path.exists():
        return gif_path
    png_dir = PNG_ASSET_DIR / emote
    if png_dir.exists():
        return png_dir
    return None


def play_emote(emote: str, loop: bool = False, headless: bool | None = None) -> dict:
    """Best-effort emote playback.

    In headless SSH/test environments this never raises. It returns a status
    dictionary so automated tests can continue even when display assets are
    missing or no DISPLAY is available.
    """
    headless = headless if headless is not None else not bool(os.getenv("DISPLAY"))
    asset = asset_path_for_emote(emote)
    if asset is None:
        return {"ok": False, "mode": "missing_asset", "emote": emote, "asset": None}
    if headless:
        return {"ok": True, "mode": "headless_fallback", "emote": emote, "asset": str(asset), "loop": loop}
    try:
        # The main project face_display.py owns real playback. This package only
        # validates dialog->emote decisions and avoids crashing without display.
        return {"ok": True, "mode": "external_face_display", "emote": emote, "asset": str(asset), "loop": loop}
    except Exception as exc:
        return {"ok": False, "mode": "fallback_after_error", "emote": emote, "asset": str(asset), "error": str(exc)}
