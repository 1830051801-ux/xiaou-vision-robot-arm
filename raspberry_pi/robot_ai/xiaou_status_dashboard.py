#!/usr/bin/env python3
"""XiaoU Control Tower: an interactive, presentation-only desktop dashboard.

The window never opens a camera, serial port, CAN interface, ROS2 process, or
motion transport.  It imports JSON snapshots made by offline simulators and
read-only diagnostics, so its controls only change the presentation.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time
import tkinter as tk
from tkinter import filedialog, ttk
from typing import Any

try:
    from .status_dashboard import build_dashboard, demo_snapshot, load_snapshot
    from .face_animation import CanvasFaceAnimator
    from .face_presentation import recommend_face
except ImportError:  # Direct execution from robot_ai/ on a Pi desktop.
    from status_dashboard import build_dashboard, demo_snapshot, load_snapshot
    from face_animation import CanvasFaceAnimator
    from face_presentation import recommend_face


MODE_LABELS = {
    "simulation": "仿真",
    "read_only_hardware": "只读硬件",
    "hardware": "硬件预览",
}


class XiaoUStatusDashboard:
    def __init__(self, snapshot: dict[str, Any] | None = None) -> None:
        self.root = tk.Tk()
        self.root.title("小U Control Tower | 安全状态总览")
        self.root.minsize(1080, 720)
        self.root.geometry("1280x780")
        self.snapshot = snapshot or demo_snapshot()
        self.mode = tk.StringVar(value=str(self.snapshot.get("mode") or "simulation"))
        self.face_mode = tk.StringVar(value="live")
        self.source_path: Path | None = None
        self._last_preview_face: tuple[str, str] | None = None
        self._configure_style()
        self._build()
        self.refresh()

    def _configure_style(self) -> None:
        style = ttk.Style(self.root)
        style.theme_use("clam")
        style.configure("Title.TLabel", font=("Microsoft YaHei UI", 18, "bold"))
        style.configure("Section.TLabelframe.Label", font=("Microsoft YaHei UI", 11, "bold"))
        style.configure("Good.TLabel", foreground="#18794e")
        style.configure("Warn.TLabel", foreground="#b54708")
        style.configure("Block.TLabel", foreground="#b42318")

    def _build(self) -> None:
        root = self.root
        root.columnconfigure(0, weight=3)
        root.columnconfigure(1, weight=5)
        root.columnconfigure(2, weight=5)
        root.rowconfigure(1, weight=1)
        root.rowconfigure(2, weight=2)

        top = ttk.Frame(root, padding=(16, 12, 16, 8))
        top.grid(row=0, column=0, columnspan=3, sticky="ew")
        top.columnconfigure(1, weight=1)
        ttk.Label(top, text="小U CONTROL TOWER", style="Title.TLabel").grid(row=0, column=0, sticky="w")
        self.status_label = ttk.Label(top, text="")
        self.status_label.grid(row=0, column=1, sticky="w", padx=(16, 0))
        ttk.Label(top, text="仅呈现 / 无执行入口", style="Block.TLabel").grid(row=0, column=2, sticky="e")

        controls = ttk.Frame(root, padding=(16, 0, 16, 10))
        controls.grid(row=1, column=0, columnspan=3, sticky="new")
        for mode, label in MODE_LABELS.items():
            ttk.Radiobutton(controls, text=label, value=mode, variable=self.mode, command=self._change_mode).pack(side=tk.LEFT, padx=(0, 10))
        ttk.Separator(controls, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=8)
        ttk.Button(controls, text="加载状态 JSON", command=self.load_json).pack(side=tk.LEFT)
        ttk.Button(controls, text="演示阻断状态", command=self.load_demo).pack(side=tk.LEFT, padx=8)
        ttk.Button(controls, text="刷新", command=self.refresh).pack(side=tk.LEFT)
        ttk.Separator(controls, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=8)
        ttk.Radiobutton(controls, text="实时表情", value="live", variable=self.face_mode, command=self.refresh).pack(side=tk.LEFT)
        ttk.Radiobutton(controls, text="任务表情预览", value="preview", variable=self.face_mode, command=self.refresh).pack(side=tk.LEFT, padx=(8, 0))
        ttk.Button(controls, text="导出当前诊断", command=self.export_json).pack(side=tk.RIGHT)

        task = ttk.LabelFrame(root, text="任务与安全门", style="Section.TLabelframe", padding=14)
        task.grid(row=2, column=0, sticky="nsew", padx=(16, 8), pady=(0, 8))
        task.columnconfigure(0, weight=1)
        self.target_label = ttk.Label(task, text="", justify=tk.LEFT, wraplength=270)
        self.target_label.grid(row=0, column=0, sticky="ew")
        ttk.Separator(task, orient=tk.HORIZONTAL).grid(row=1, column=0, sticky="ew", pady=12)
        self.gate_box = tk.Text(task, height=6, wrap=tk.WORD, state=tk.DISABLED, relief=tk.FLAT)
        self.gate_box.grid(row=2, column=0, sticky="nsew")
        self.face_caption = ttk.Label(task, text="", justify=tk.LEFT, wraplength=270)
        self.face_caption.grid(row=3, column=0, sticky="ew", pady=(10, 2))
        self.face_host = tk.Frame(task, bg="#111318", height=150)
        self.face_host.grid(row=4, column=0, sticky="nsew")
        self.face_host.grid_propagate(False)
        self.face_animator = CanvasFaceAnimator(self.face_host, width=270, height=150, show_label=False, background="#111318")
        self.face_animator.start()
        task.rowconfigure(4, weight=1)

        center = ttk.LabelFrame(root, text="阶段与阻断原因", style="Section.TLabelframe", padding=14)
        center.grid(row=2, column=1, sticky="nsew", padx=8, pady=(0, 8))
        center.columnconfigure(0, weight=1)
        self.phase_label = ttk.Label(center, text="", font=("Microsoft YaHei UI", 16, "bold"))
        self.phase_label.grid(row=0, column=0, sticky="w")
        ttk.Label(center, text="规划与执行分离：本窗口只显示策略预览和安全原因。", wraplength=420).grid(row=1, column=0, sticky="ew", pady=(4, 10))
        self.blocker_box = tk.Text(center, height=11, wrap=tk.WORD, state=tk.DISABLED, relief=tk.FLAT)
        self.blocker_box.grid(row=2, column=0, sticky="nsew")
        center.rowconfigure(2, weight=1)

        joints = ttk.LabelFrame(root, text="六关节实时性", style="Section.TLabelframe", padding=8)
        joints.grid(row=2, column=2, sticky="nsew", padx=(8, 16), pady=(0, 8))
        columns = ("state", "angle", "age", "online")
        self.joint_table = ttk.Treeview(joints, columns=columns, show="tree headings", height=8)
        self.joint_table.heading("#0", text="关节")
        self.joint_table.heading("state", text="状态")
        self.joint_table.heading("angle", text="角度")
        self.joint_table.heading("age", text="年龄")
        self.joint_table.heading("online", text="在线")
        self.joint_table.column("#0", width=42, anchor=tk.W)
        self.joint_table.column("state", width=115, anchor=tk.W)
        self.joint_table.column("angle", width=70, anchor=tk.E)
        self.joint_table.column("age", width=70, anchor=tk.E)
        self.joint_table.column("online", width=52, anchor=tk.CENTER)
        self.joint_table.pack(fill=tk.BOTH, expand=True)

        timeline = ttk.LabelFrame(root, text="事件时间线", style="Section.TLabelframe", padding=8)
        timeline.grid(row=3, column=0, columnspan=3, sticky="nsew", padx=16, pady=(0, 16))
        root.rowconfigure(3, weight=1)
        self.events = ttk.Treeview(timeline, columns=("time", "level", "message"), show="headings", height=6)
        self.events.heading("time", text="时间")
        self.events.heading("level", text="级别")
        self.events.heading("message", text="事件")
        self.events.column("time", width=100, anchor=tk.W)
        self.events.column("level", width=90, anchor=tk.W)
        self.events.column("message", width=900, anchor=tk.W)
        self.events.pack(fill=tk.BOTH, expand=True)

    @staticmethod
    def _text(widget: tk.Text, value: str) -> None:
        widget.configure(state=tk.NORMAL)
        widget.delete("1.0", tk.END)
        widget.insert(tk.END, value)
        widget.configure(state=tk.DISABLED)

    @staticmethod
    def _fmt(value: object, suffix: str = "") -> str:
        return "—" if value is None else f"{float(value):.3f}{suffix}"

    def _change_mode(self) -> None:
        self.snapshot["mode"] = self.mode.get()
        self.refresh()

    def load_demo(self) -> None:
        self.source_path = None
        self.snapshot = demo_snapshot(now_monotonic_s=time.monotonic())
        self.mode.set(str(self.snapshot["mode"]))
        self.refresh()

    def load_json(self) -> None:
        selected = filedialog.askopenfilename(title="选择状态 JSON", filetypes=(("JSON", "*.json"), ("所有文件", "*.*")))
        if not selected:
            return
        try:
            self.snapshot = load_snapshot(selected)
            self.source_path = Path(selected)
            self.mode.set(str(self.snapshot.get("mode") or "simulation"))
            self.refresh()
        except Exception as exc:
            self.status_label.configure(text=f"无法加载：{exc}", style="Block.TLabel")

    def export_json(self) -> None:
        selected = filedialog.asksaveasfilename(
            title="导出状态诊断", defaultextension=".json", filetypes=(("JSON", "*.json"),)
        )
        if not selected:
            return
        current = build_dashboard(self.snapshot, now_monotonic_s=time.monotonic())
        Path(selected).write_text(json.dumps(current, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        self.status_label.configure(text=f"诊断已导出：{Path(selected).name}", style="Good.TLabel")

    def refresh(self) -> None:
        self.snapshot["mode"] = self.mode.get()
        view = build_dashboard(self.snapshot, now_monotonic_s=time.monotonic())
        style = "Good.TLabel" if view["overall_status"] == "preview_ready" else "Block.TLabel"
        self.status_label.configure(text=f"{view['mode_label']} · {view['overall_status']}", style=style)
        target = view["target"]
        self.target_label.configure(
            text=(
                f"目标：{target['class']}\n"
                f"置信度：{self._fmt(target['confidence'])}\n"
                f"坐标系：{target['frame']}\n"
                f"策略：{target['strategy']}"
            )
        )
        safety = view["safety"]
        safety_text = "\n".join(f"{'✓' if enabled else '×'} {name}" for name, enabled in safety.items())
        self._text(self.gate_box, f"安全门\n{safety_text}\n\n反馈新鲜度阈值：{view['freshness_threshold_s']:.2f} s")
        self.phase_label.configure(text=f"当前阶段：{view['phase']['label']}")
        blocker_text = "\n".join(f"• {reason}" for reason in view["blockers"]) or "• 仿真预览可继续；仍无真实执行入口。"
        self._text(self.blocker_box, blocker_text)
        self.joint_table.delete(*self.joint_table.get_children())
        for joint in view["joints"]:
            online = "是" if joint["online"] is True else ("否" if joint["online"] is False else "—")
            self.joint_table.insert(
                "",
                tk.END,
                text=joint["name"],
                values=(joint["state"], self._fmt(joint["angle_deg"], "°"), self._fmt(joint["age_s"], " s"), online),
            )
        self.events.delete(*self.events.get_children())
        for event in view["events"]:
            self.events.insert("", tk.END, values=(event.get("time", ""), event.get("level", "info"), event.get("message", "")))
        face = recommend_face(view)
        if self.face_mode.get() == "preview":
            preview_key = (face.state, face.text)
            if preview_key != self._last_preview_face:
                self.face_animator.set_state(face.state, face.text)
                self._last_preview_face = preview_key
            self.face_caption.configure(text=f"任务表情预览：{face.text}")
        else:
            self.face_animator.resume_live_state()
            self._last_preview_face = None
            self.face_caption.configure(text="实时表情：保留语音、视觉和对话对表情屏的原有驱动")

    def run(self) -> None:
        self.root.mainloop()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, help="optional offline/read-only status JSON")
    args = parser.parse_args()
    snapshot = load_snapshot(args.input) if args.input else None
    XiaoUStatusDashboard(snapshot).run()


if __name__ == "__main__":
    main()
