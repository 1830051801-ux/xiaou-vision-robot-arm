from __future__ import annotations

import base64
import uuid
from pathlib import Path

import requests

from xiaou_runtime import get_xiaou_config


TOKEN_URL = "https://aip.baidubce.com/oauth/2.0/token"
ASR_URL = "http://vop.baidu.com/server_api"


def _get_config() -> tuple[str, str, int, int]:
    cfg = get_xiaou_config()
    return cfg.baidu_asr_api_key, cfg.baidu_asr_secret_key, cfg.baidu_asr_dev_pid, cfg.baidu_asr_rate


def get_baidu_access_token() -> str:
    api_key, secret_key, _, _ = _get_config()
    if not api_key or "replace_with" in api_key:
        raise RuntimeError("娌℃湁閰嶇疆 BAIDU_ASR_API_KEY")
    if not secret_key or "replace_with" in secret_key:
        raise RuntimeError("娌℃湁閰嶇疆 BAIDU_ASR_SECRET_KEY")

    params = {
        "grant_type": "client_credentials",
        "client_id": api_key,
        "client_secret": secret_key,
    }
    resp = requests.post(TOKEN_URL, params=params, timeout=20)
    resp.raise_for_status()
    data = resp.json()
    token = data.get("access_token")
    if not token:
        raise RuntimeError(f"鐧惧害token鑾峰彇澶辫触: {data}")
    return token


def baidu_asr_wav(wav_path: str | Path) -> str:
    path = Path(wav_path)
    audio = path.read_bytes()
    token = get_baidu_access_token()
    _, _, dev_pid, rate = _get_config()
    cuid = f"raspi-{uuid.getnode():x}"

    payload = {
        "format": "wav",
        "rate": rate,
        "channel": 1,
        "cuid": cuid,
        "token": token,
        "dev_pid": dev_pid,
        "len": len(audio),
        "speech": base64.b64encode(audio).decode("ascii"),
    }
    resp = requests.post(ASR_URL, json=payload, timeout=30)
    resp.raise_for_status()
    data = resp.json()
    if data.get("err_no") != 0:
        raise RuntimeError(f"鐧惧害ASR璇嗗埆澶辫触: {data}")
    result = data.get("result") or []
    if not result:
        raise RuntimeError(f"鐧惧害ASR娌℃湁杩斿洖鏂囧瓧: {data}")
    return result[0].strip()
