from __future__ import annotations

import json
from functools import lru_cache
from pathlib import Path
from typing import Any

import requests

from xiaou_runtime import XiaouConfig, get_logger, get_xiaou_config


PROJECT_DIR = Path(__file__).resolve().parents[1]
LOGGER = get_logger(__name__)


@lru_cache(maxsize=1)
def get_config() -> XiaouConfig:
    return get_xiaou_config()


def get_camera_index() -> int:
    return get_config().camera_index


def get_target_objects() -> list[str]:
    return list(get_config().target_objects)


def get_yolo_model() -> str:
    return get_config().yolo_model


def get_yolo_imgsz() -> int:
    return get_config().yolo_image_size


def get_yolo_conf() -> float:
    return get_config().yolo_conf


def normalize_object_name(text: str) -> str | None:
    lowered = text.lower()
    aliases = {
        "Coffee cup": ("coffee cup", "cup", "杯子", "水杯", "茶杯", "杯"),
        "Bottle": ("bottle", "瓶子", "水瓶", "饮料瓶", "瓶"),
        "cola": ("cola", "coke", "可乐", "可口可乐", "饮料"),
        "earphone": ("earphone", "headphone", "headphones", "耳机", "耳麦"),
        "Pen": ("pen", "pencil", "笔", "钢笔", "圆珠笔", "铅笔"),
        "Mobile phone": ("mobile phone", "cell phone", "phone", "手机", "电话", "手機", "電話"),
        "Book": ("book", "书", "书本", "本子"),
        "Computer keyboard": ("computer keyboard", "keyboard", "键盘"),
        "Computer mouse": ("computer mouse", "mouse", "鼠标"),
        "Scissors": ("scissors", "剪刀"),
        "medicine_box": ("medicine", "medicine_box", "药盒", "药箱"),
        "key": ("key", "钥匙"),
    }
    for name, keys in aliases.items():
        for key in keys:
            if key.lower() in lowered or key in text:
                return name
    return None


def fallback_intent(text: str) -> dict:
    text_lower = text.lower()
    if any(word in text_lower for word in ("cola", "coke", "可乐", "可口可乐", "饮料")):
        obj = "cola"
    elif any(word in text_lower for word in ("earphone", "headphone", "headphones", "耳机", "耳麦")):
        obj = "earphone"
    else:
        obj = normalize_object_name(text)
    pick_words = ["拿", "取", "抓", "递", "帮我拿", "帮我取", "pick", "grab", "fetch"]
    look_words = ["找", "看看", "看一下", "search", "look"]
    tidy_words = ["整理", "归位", "收拾", "桌面整理", "tidy", "clean"]
    stop_words = ["停止", "急停", "停下", "别动", "stop"]
    home_words = ["回零", "回到初始", "回家", "待机", "home"]

    if any(word in text_lower for word in stop_words):
        return {"action": "stop", "object": None, "reply": "收到，我先停下。"}
    if any(word in text_lower for word in home_words):
        return {"action": "home", "object": None, "reply": "收到，我回到安全位置。"}
    if any(word in text_lower for word in tidy_words):
        return {"action": "tidy", "object": obj, "reply": "收到，我开始整理。"}
    if obj and any(word in text_lower for word in pick_words):
        return {"action": "pick", "object": obj, "reply": f"收到，我去拿{obj}。"}
    if obj and any(word in text_lower for word in look_words):
        return {"action": "look", "object": obj, "reply": f"收到，我去找{obj}。"}
    if obj:
        return {"action": "look", "object": obj, "reply": f"收到，我去看看{obj}。"}
    return {"action": "chat", "object": None, "reply": "我听到了，不过没找到明确目标。"}


def extract_json(text: str) -> dict | None:
    text = text.strip()
    if not text:
        return None
    try:
        parsed = json.loads(text)
    except Exception:
        return None
    return parsed if isinstance(parsed, dict) else None


def _parse_structured_response(data: dict) -> dict | None:
    if not isinstance(data, dict):
        return None
    if {"action", "object"}.issubset(data.keys()):
        return data

    choices = data.get("choices")
    if not isinstance(choices, list) or not choices:
        return None
    message = choices[0].get("message", {})
    if not isinstance(message, dict):
        return None

    content = message.get("content")
    if isinstance(content, dict) and {"action", "object"}.issubset(content.keys()):
        return content
    if isinstance(content, str):
        parsed = extract_json(content)
        if parsed is not None:
            return parsed

    tool_calls = message.get("tool_calls")
    if isinstance(tool_calls, list) and tool_calls:
        first = tool_calls[0]
        if isinstance(first, dict):
            function = first.get("function", {})
            if isinstance(function, dict):
                arguments = function.get("arguments")
                if isinstance(arguments, str):
                    parsed = extract_json(arguments)
                    if parsed is not None:
                        return parsed

    return None


def ask_cloud_intent(user_text: str) -> dict:
    cfg = get_config()
    if not cfg.ai_api_key or "replace_with" in cfg.ai_api_key or not cfg.ai_api_url:
        return fallback_intent(user_text)

    system_prompt = (
        "Return a JSON object only with this schema: "
        '{"action":"pick|look|tidy|stop|home|chat",'
        '"object":"Coffee cup|Bottle|cola|earphone|Pen|Mobile phone|Book|Computer keyboard|Computer mouse|Scissors|null",'
        '"reply":"short Chinese reply"}. '
        "Map Chinese object names: 饮料→cola, 水杯→Coffee cup, 瓶子→Bottle, 笔→Pen, 手机→Mobile phone, "
        "书→Book, 键盘→Computer keyboard, 鼠标→Computer mouse, 剪刀→Scissors. "
        "If the user asks to fetch, take, grab, or hand over an object, use action pick. "
        "If the user asks to clean or organize the desk, use action tidy. "
        "If the user says stop, use action stop. "
        "If the user asks reset/home/standby, use action home. "
        "Map Chinese object names to the target object names above."
    )

    payload: dict[str, Any] = {
        "model": cfg.ai_model,
        "messages": [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_text},
        ],
        "temperature": 0.1,
    }
    if cfg.ai_structured_output:
        payload["response_format"] = {"type": "json_object"}

    headers = {
        "Authorization": f"Bearer {cfg.ai_api_key}",
        "Content-Type": "application/json",
    }

    try:
        resp = requests.post(cfg.ai_api_url, headers=headers, json=payload, timeout=cfg.ai_timeout_s)
        resp.raise_for_status()
        data = resp.json()
        parsed = _parse_structured_response(data)
        if parsed is not None:
            return parsed
        LOGGER.warning("Structured intent parse failed, using local fallback.")
    except Exception as exc:
        LOGGER.warning("Cloud intent request failed, using local fallback: %s", exc)
    return fallback_intent(user_text)


def command_json(action: str, obj: str | None, x: int | None = None, y: int | None = None) -> dict:
    return {"cmd": action, "object": obj, "x": x, "y": y}


def print_json(title: str, payload: dict[str, Any]) -> None:
    LOGGER.info("%s: %s", title, json.dumps(payload, ensure_ascii=False))



