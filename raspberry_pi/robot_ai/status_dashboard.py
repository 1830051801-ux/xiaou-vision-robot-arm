"""Safe, presentation-only state model for the XiaoU control-tower UI.

The dashboard intentionally has no serial, CAN, ROS, camera, subprocess, or
motion APIs.  It turns saved read-only reports and simulation snapshots into a
single deterministic view model.  A UI may render this model, but it cannot
use it to enable or execute the arm.
"""

from __future__ import annotations

from copy import deepcopy
import json
import math
from pathlib import Path
from typing import Any, Iterable


DASHBOARD_SCHEMA_VERSION = 1
SUPPORTED_MODES = ("simulation", "read_only_hardware", "hardware")
JOINT_COUNT = 6


def _finite_number(value: object) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _safe_bool(value: object) -> bool:
    return value is True


def _joint_samples(report: dict[str, Any]) -> list[dict[str, Any]]:
    """Read both historical probe layouts without modifying their evidence."""
    samples = report.get("samples")
    if isinstance(samples, list):
        return [item for item in samples if isinstance(item, dict)]
    scans = report.get("scans")
    if not isinstance(scans, list):
        return []
    flattened: list[dict[str, Any]] = []
    for scan in scans:
        if not isinstance(scan, dict):
            continue
        for item in scan.get("joints") or []:
            if isinstance(item, dict):
                flattened.append(item)
    return flattened


def build_joint_cards(
    report: dict[str, Any] | None,
    *,
    now_monotonic_s: float | None = None,
    freshness_threshold_s: float = 0.5,
) -> list[dict[str, Any]]:
    """Summarise one latest read-only record per joint.

    A successful request is intentionally *not* treated as online.  In
    particular, an old reply, a future clock value, or a firmware-declared
    offline joint remains visibly unsafe for a hardware-mode preview.
    """
    if freshness_threshold_s <= 0.0 or not math.isfinite(freshness_threshold_s):
        raise ValueError("freshness_threshold_s must be finite and positive")
    samples = _joint_samples(report or {})
    cards: list[dict[str, Any]] = []
    for joint_id in range(1, JOINT_COUNT + 1):
        joint_rows = [row for row in samples if row.get("joint_id") == joint_id]
        latest = joint_rows[-1] if joint_rows else None
        feedback = dict(latest.get("feedback") or {}) if latest else {}
        observed = _finite_number(latest.get("observed_at_monotonic_s")) if latest else None
        age_s = None if observed is None or now_monotonic_s is None else now_monotonic_s - observed
        if latest is None:
            state, reason = "missing", "未收到该关节的只读样本"
        elif not _safe_bool(latest.get("ok")):
            state, reason = "query_failed", str(latest.get("error") or "只读查询失败")
        elif not _safe_bool(feedback.get("online")):
            state, reason = "offline_reported", "控制器反馈为 offline"
        elif age_s is None:
            state, reason = "freshness_unknown", "历史报告没有单调时钟时间戳"
        elif age_s < 0.0:
            state, reason = "future_timestamp", "报告时间戳晚于当前单调时钟"
        elif age_s > freshness_threshold_s:
            state, reason = "stale", f"反馈年龄超过 {freshness_threshold_s:.2f} s"
        else:
            state, reason = "fresh_online", "在线且反馈新鲜"
        cards.append(
            {
                "joint_id": joint_id,
                "name": f"J{joint_id}",
                "state": state,
                "safe_for_hardware_preview": state == "fresh_online",
                "reason": reason,
                "age_s": age_s,
                "angle_deg": _finite_number(feedback.get("angle_deg")),
                "speed_rpm": _finite_number(feedback.get("speed_rpm")),
                "torque_nm": _finite_number(feedback.get("torque_nm")),
                "online": feedback.get("online") if feedback else None,
                "round_trip_ms": _finite_number(latest.get("round_trip_ms")) if latest else None,
            }
        )
    return cards


def _normalise_mode(value: object) -> str:
    mode = str(value or "simulation").strip().lower()
    if mode not in SUPPORTED_MODES:
        raise ValueError(f"unsupported dashboard mode: {mode}")
    return mode


def _events(snapshot: dict[str, Any]) -> list[dict[str, str]]:
    raw = snapshot.get("events") or snapshot.get("timeline") or []
    result: list[dict[str, str]] = []
    if isinstance(raw, list):
        for item in raw[-20:]:
            if isinstance(item, str):
                result.append({"level": "info", "message": item})
            elif isinstance(item, dict):
                result.append(
                    {
                        "level": str(item.get("level") or "info"),
                        "message": str(item.get("message") or item.get("status") or ""),
                        "time": str(item.get("time") or item.get("timestamp") or ""),
                    }
                )
    return result


def build_dashboard(
    snapshot: dict[str, Any] | None,
    *,
    now_monotonic_s: float | None = None,
) -> dict[str, Any]:
    """Create the only state shape that the XiaoU status UI renders.

    ``hardware_motion`` is a literal invariant.  A future caller must go
    through a separate, explicitly authorised execution boundary; this module
    only explains why a state is safe or blocked.
    """
    source = deepcopy(snapshot or {})
    mode_value = source.get("mode")
    if mode_value is None and source.get("read_only") is True:
        mode_value = "read_only_hardware"
    mode = _normalise_mode(mode_value)
    threshold = _finite_number(source.get("freshness_threshold_s")) or 0.5
    joint_report = source.get("joint_report")
    if not isinstance(joint_report, dict) and (isinstance(source.get("samples"), list) or isinstance(source.get("scans"), list)):
        # A saved motor_feedback_diagnostic.py or joint_status_probe.py report
        # can be opened by the UI directly, without a hand-written wrapper.
        joint_report = source
    joint_report = joint_report if isinstance(joint_report, dict) else {}
    cards = build_joint_cards(
        joint_report,
        now_monotonic_s=now_monotonic_s,
        freshness_threshold_s=threshold,
    )
    safety = dict(source.get("safety") or {})
    blockers: list[str] = []
    if mode == "simulation":
        mode_label = "仿真"
    elif mode == "read_only_hardware":
        mode_label = "只读硬件"
        blockers.append("只读模式：界面不提供执行能力")
    else:
        mode_label = "硬件预览"
        blockers.append("状态面板不提供执行能力")
    if mode != "simulation":
        for card in cards:
            if not card["safe_for_hardware_preview"]:
                blockers.append(f"{card['name']}：{card['reason']}")
        for field in ("motion_enabled", "hardware_ready", "feedback_verified", "collision_free"):
            if safety.get(field) is not True:
                blockers.append(f"安全门未满足：{field}")
    phase = dict(source.get("phase") or {})
    target = dict(source.get("target") or {})
    target_class = str(target.get("class") or (source.get("yolo") or {}).get("class") or "未选择目标")
    status = "blocked" if blockers else "preview_ready"
    return {
        "schema_version": DASHBOARD_SCHEMA_VERSION,
        "presentation_only": True,
        "hardware_motion": False,
        "mode": mode,
        "mode_label": mode_label,
        "overall_status": status,
        "phase": {
            "name": str(phase.get("name") or "observe"),
            "label": str(phase.get("label") or "观察"),
        },
        "target": {
            "class": target_class,
            "confidence": _finite_number((source.get("yolo") or {}).get("confidence")),
            "frame": str(target.get("frame") or "base_link"),
            "strategy": str(source.get("strategy") or "待决策"),
        },
        "safety": {
            "motion_enabled": _safe_bool(safety.get("motion_enabled")),
            "hardware_ready": _safe_bool(safety.get("hardware_ready")),
            "feedback_verified": _safe_bool(safety.get("feedback_verified")),
            "collision_free": _safe_bool(safety.get("collision_free")),
        },
        "freshness_threshold_s": threshold,
        "joints": cards,
        "blockers": blockers,
        "events": _events(source),
    }


def load_snapshot(path: str | Path) -> dict[str, Any]:
    loaded = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(loaded, dict):
        raise ValueError("dashboard snapshot must be a JSON object")
    return loaded


def demo_snapshot(*, now_monotonic_s: float = 100.0) -> dict[str, Any]:
    """A deliberately blocked demo that makes stale J5/J6 visible."""
    samples: list[dict[str, Any]] = []
    for joint_id in range(1, JOINT_COUNT + 1):
        age = 0.08
        online = True
        if joint_id == 5:
            age = 0.82
        if joint_id == 6:
            online = False
        samples.append(
            {
                "joint_id": joint_id,
                "ok": True,
                "observed_at_monotonic_s": now_monotonic_s - age,
                "round_trip_ms": 35.0 + joint_id,
                "feedback": {
                    "joint_id": joint_id,
                    "angle_deg": -5.0 * joint_id,
                    "speed_rpm": 0.0,
                    "torque_nm": 0.0,
                    "online": online,
                },
            }
        )
    return {
        "mode": "read_only_hardware",
        "freshness_threshold_s": 0.5,
        "phase": {"name": "observe", "label": "观察 / 只读诊断"},
        "target": {"class": "bottle", "frame": "base_link"},
        "yolo": {"class": "bottle", "confidence": 0.92},
        "strategy": "侧向包覆（预览）",
        "safety": {"motion_enabled": False, "hardware_ready": False, "feedback_verified": False, "collision_free": True},
        "joint_report": {"samples": samples},
        "events": [
            {"time": "刚刚", "level": "warning", "message": "J5 数据年龄超过阈值，保持阻断"},
            {"time": "刚刚", "level": "warning", "message": "J6 控制器反馈 offline"},
            {"time": "刚刚", "level": "info", "message": "只读诊断完成，未发送控制命令"},
        ],
    }
