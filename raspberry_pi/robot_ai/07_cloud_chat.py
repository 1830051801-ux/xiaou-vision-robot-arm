from __future__ import annotations

import io
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import requests

from common import extract_json, get_config
from emotion_state import local_emotional_reply, set_dialog_emotion, set_emotion_from_text, xiaou_style_prompt
from face_state import set_face_state
from xiaou_runtime import get_logger


LOGGER = get_logger(__name__)
TTS_GENERATE_TIMEOUT_S = 20.0
TTS_PLAYBACK_TIMEOUT_S = 20.0


def _tts_cache_path(text: str) -> Path:
    cfg = get_config()
    cache_dir = cfg.runtime_dir / "tts_cache"
    cache_dir.mkdir(parents=True, exist_ok=True)
    key_src = f"{cfg.tts_voice}|{cfg.tts_pitch}|{cfg.tts_rate}|{text}".encode("utf-8", errors="ignore")
    key = hashlib.sha1(key_src).hexdigest()
    return cache_dir / f"{key}.mp3"


def _text_timeout(text: str, minimum: float = 8.0, maximum: float = 25.0) -> float:
    return max(minimum, min(maximum, 4.0 + len(text) * 0.18))


def speak_text_fallback(text: str) -> bool:
    cfg = get_config()
    if not shutil.which("espeak-ng"):
        LOGGER.error("Local TTS unavailable. Install: sudo apt install -y espeak-ng")
        return False
    try:
        LOGGER.warning("Using fallback TTS (espeak-ng).")
        subprocess.run(
            ["espeak-ng", "-v", cfg.espeak_voice, "-s", cfg.espeak_speed, text],
            check=False,
            timeout=_text_timeout(text),
        )
        return True
    except subprocess.TimeoutExpired:
        LOGGER.error("Local TTS timed out.")
        return False
    except FileNotFoundError:
        LOGGER.error("Local TTS unavailable. Install: sudo apt install -y espeak-ng")
        return False


def _play_audio_file(path: Path) -> bool:
    if not path.exists() or path.stat().st_size == 0:
        LOGGER.error("TTS output file is missing or empty: %s", path)
        return False

    for candidate in ("ffplay", "mpg123", "mpv"):
        if not shutil.which(candidate):
            continue
        try:
            if candidate == "ffplay":
                subprocess.run(
                    ["ffplay", "-nodisp", "-autoexit", "-loglevel", "quiet", str(path)],
                    check=False,
                    timeout=TTS_PLAYBACK_TIMEOUT_S,
                )
            elif candidate == "mpg123":
                subprocess.run(["mpg123", "-q", str(path)], check=False, timeout=TTS_PLAYBACK_TIMEOUT_S)
            else:
                subprocess.run(
                    ["mpv", "--really-quiet", "--no-video", str(path)],
                    check=False,
                    timeout=TTS_PLAYBACK_TIMEOUT_S,
                )
            return True
        except subprocess.TimeoutExpired:
            LOGGER.warning("Audio playback timed out with %s.", candidate)
            return False

    try:
        import pygame

        if not pygame.mixer.get_init():
            pygame.mixer.init()
        pygame.mixer.music.load(str(path))
        pygame.mixer.music.play()
        deadline = time.monotonic() + TTS_PLAYBACK_TIMEOUT_S
        while pygame.mixer.music.get_busy():
            if time.monotonic() >= deadline:
                pygame.mixer.music.stop()
                LOGGER.warning("pygame audio playback timed out.")
                return False
            pygame.time.wait(50)
        return True
    except Exception as exc:
        LOGGER.debug("pygame playback failed: %s", exc)

    if shutil.which("aplay") and shutil.which("ffmpeg"):
        try:
            proc = subprocess.run(
                ["ffmpeg", "-nostdin", "-hide_banner", "-loglevel", "error", "-i", str(path), "-f", "wav", "-"],
                check=True,
                capture_output=True,
                timeout=TTS_GENERATE_TIMEOUT_S,
            )
            aplay = subprocess.run(["aplay", "-q"], input=proc.stdout, check=False, timeout=TTS_PLAYBACK_TIMEOUT_S)
            return aplay.returncode == 0
        except subprocess.TimeoutExpired as exc:
            LOGGER.warning("Audio playback via ffmpeg+aplay timed out: %s", exc)
            return False
        except Exception as exc:
            LOGGER.warning("Audio playback via ffmpeg+aplay failed: %s", exc)
            return False

    return False


def speak_text(text: str) -> bool:
    cfg = get_config()
    if cfg.tts_engine.lower() in {"local", "espeak", "espeak-ng", "offline"}:
        return speak_text_fallback(text)

    last_error: Exception | None = None
    out_path = _tts_cache_path(text)
    if out_path.exists() and out_path.stat().st_size > 0:
        LOGGER.info("Speaking from TTS cache: %s", out_path.name)
        if _play_audio_file(out_path):
            return True
        try:
            out_path.unlink(missing_ok=True)
        except Exception:
            pass

    try:
        fd, tmp_name = tempfile.mkstemp(prefix="robot_ai_reply_", suffix=".mp3", dir=str(out_path.parent))
        os.close(fd)
        tmp_path = Path(tmp_name)
        LOGGER.info("Speaking with edge-tts voice=%s", cfg.tts_voice)
        cmd = ["edge-tts", "--voice", cfg.tts_voice]
        if cfg.tts_pitch:
            cmd.extend(["--pitch", cfg.tts_pitch])
        cmd.extend(["--rate", cfg.tts_rate, "--text", text, "--write-media", str(tmp_path)])

        for attempt in range(1, 2):
            try:
                subprocess.run(cmd, check=True, timeout=TTS_GENERATE_TIMEOUT_S)
                if tmp_path.exists() and tmp_path.stat().st_size > 0:
                    tmp_path.replace(out_path)
                if _play_audio_file(out_path):
                    return True
                last_error = RuntimeError("Edge TTS generated audio but playback failed.")
                LOGGER.warning("Edge TTS playback failed on attempt %s.", attempt)
            except FileNotFoundError as exc:
                last_error = exc
                LOGGER.error("TTS tool missing: %s", exc)
                break
            except subprocess.TimeoutExpired as exc:
                last_error = exc
                LOGGER.error("TTS timed out on attempt %s.", attempt)
            except subprocess.CalledProcessError as exc:
                last_error = exc
                LOGGER.error("TTS generation failed on attempt %s: %s", attempt, exc)
    finally:
        try:
            if "tmp_path" in locals():
                tmp_path.unlink(missing_ok=True)
        except Exception:
            pass

    if last_error is not None:
        LOGGER.error("Normal voice failed: %s", last_error)
    return speak_text_fallback(text)

def speak_text_reliable(text: str, wait_seconds: float = 0.8, retries: int = 2) -> bool:
    text = text.strip()
    if not text:
        return False
    if wait_seconds > 0:
        time.sleep(wait_seconds)
    for attempt in range(1, max(1, retries) + 1):
        if speak_text(text):
            return True
        LOGGER.warning("TTS attempt %s/%s failed; retrying once more.", attempt, retries)
        if attempt < retries:
            time.sleep(0.6)
    LOGGER.warning("TTS failed after retries; skip duplicate fallback playback.")
    return False


def cloud_chat(messages: list[dict[str, str]]) -> str:
    cfg = get_config()
    if not cfg.ai_api_key or "replace_with" in cfg.ai_api_key or not cfg.ai_api_url:
        return "Cloud AI is not configured. Please set AI_API_KEY, AI_API_URL and AI_MODEL in config.env."

    payload = {
        "model": cfg.ai_model,
        "messages": messages,
        "temperature": 0.7,
    }
    headers = {"Authorization": f"Bearer {cfg.ai_api_key}", "Content-Type": "application/json"}
    resp = requests.post(cfg.ai_api_url, headers=headers, json=payload, timeout=cfg.ai_timeout_s)
    resp.raise_for_status()
    data = resp.json()
    return data["choices"][0]["message"]["content"].strip()


def maybe_robot_intent(text: str) -> str | None:
    parsed = extract_json(text)
    if not parsed:
        return None
    action = parsed.get("action")
    obj = parsed.get("object")
    if action:
        return f"robot_intent: action={action}, object={obj}"
    return None


def main() -> None:
    voice_reply = "--voice" in sys.argv
    cfg = get_config()
    set_face_state("idle", f"{cfg.robot_name}在这儿")
    LOGGER.info("%s cloud chat started. Type q to quit.", cfg.robot_name)
    LOGGER.info("Using AI_API_KEY / AI_API_URL / AI_MODEL from config.env.")
    if voice_reply:
        LOGGER.info("Voice reply: ON")
    messages = [
        {
            "role": "system",
            "content": xiaou_style_prompt(
                "如果用户明确要求拿、取、递某个物品，只确认你听到了，不要说已经完成动作。"
                "说话要自然一点，像一个很乖的小助理。"
                "如果用户抱怨、责备或失落，先安慰、再回答，不要插科打诨。"
            ),
        }
    ]
    while True:
        user_text = input("> ").strip()
        if user_text.lower() in {"q", "quit", "exit"}:
            break
        set_emotion_from_text(user_text)
        messages.append({"role": "user", "content": user_text})
        reply = local_emotional_reply(user_text)
        if reply is None:
            try:
                reply = cloud_chat(messages)
            except Exception as exc:
                reply = f"Cloud AI request failed: {exc}"
                set_face_state("error", "网络有点不稳")
                LOGGER.warning("cloud chat failed: %s", exc)
        messages.append({"role": "assistant", "content": reply})
        print(reply)
        set_dialog_emotion(user_text, reply)
        if voice_reply:
            speak_text(reply)
        intent = maybe_robot_intent(reply)
        if intent:
            LOGGER.info("%s", intent)


if __name__ == "__main__":
    main()
