"""Presentation-only policy joining XiaoU task status to its existing face.

This policy has no side effects.  The Control Tower can preview its result in
its embedded animation, while the existing full-screen face continues to be
driven by dialog, vision, and voice through ``face_state.json``.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class FaceRecommendation:
    state: str
    text: str
    reason: str


def recommend_face(view: dict[str, Any]) -> FaceRecommendation:
    """Choose an empathetic preview without taking ownership of the real face."""
    mode = str(view.get("mode") or "simulation")
    blockers = [str(item) for item in view.get("blockers") or []]
    lower = " ".join(blockers).lower()
    phase = str((view.get("phase") or {}).get("name") or "observe")
    # Read-only diagnostics deliberately keep every hardware gate false.  That
    # expected state must not visually overpower a useful J5/J6 diagnosis.
    if mode == "hardware" and ("安全门未满足" in lower or "motion_enabled" in lower or "hardware_ready" in lower):
        return FaceRecommendation("stop", "安全门未就绪，先保持暂停", "safety_gate")
    if "stale" in lower or "反馈年龄" in lower:
        return FaceRecommendation("confused", "数据有点慢，小U在等新反馈", "stale_feedback")
    if "offline" in lower or "查询失败" in lower or "missing" in lower:
        return FaceRecommendation("question", "有一个关节状态还不明确", "joint_unavailable")
    if phase in {"approach", "grasp", "lift", "pull"}:
        return FaceRecommendation("searching", "正在专注看规划预览", "planning_phase")
    if mode == "simulation":
        return FaceRecommendation("thinking", "仿真场景已准备好", "simulation")
    if view.get("overall_status") == "preview_ready":
        return FaceRecommendation("happy", "状态检查通过，先预览", "preview_ready")
    return FaceRecommendation("idle", "小U在等待下一步", "idle")
