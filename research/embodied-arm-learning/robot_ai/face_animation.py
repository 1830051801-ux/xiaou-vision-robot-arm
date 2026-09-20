from __future__ import annotations

import math
import random
import time
import tkinter as tk
from dataclasses import dataclass

from face_state import get_face_state, set_face_state
from xiaou_runtime import get_logger, get_xiaou_config


CFG = get_xiaou_config()
LOGGER = get_logger(__name__)


@dataclass(frozen=True)
class FaceStyle:
    eye_open: float
    eye_round: float
    eye_slant: float
    eye_cross: float
    brow_left: float
    brow_right: float
    mouth_curve: float
    mouth_open: float
    blush: float
    red: float
    shake: float
    particle: str
    label: str


STYLE_MAP: dict[str, FaceStyle] = {
    "greet": FaceStyle(0.98, 1.0, 0.0, 0.0, 0.04, 0.04, 0.40, 0.10, 0.14, 0.0, 0.0, "", "greet"),
    "idle": FaceStyle(1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.35, 0.08, 0.18, 0.0, 0.0, "", "idle"),
    "listening": FaceStyle(0.72, 0.9, 0.1, 0.0, 0.08, 0.08, 0.0, 0.22, 0.12, 0.0, 0.0, "", "listening"),
    "thinking": FaceStyle(0.78, 0.8, 0.34, 0.0, 0.18, -0.08, -0.15, 0.07, 0.10, 0.0, 0.0, "", "thinking"),
    "curious": FaceStyle(0.86, 0.95, 0.18, 0.0, 0.20, -0.12, 0.05, 0.12, 0.16, 0.0, 0.0, "", "curious"),
    "confused": FaceStyle(0.72, 0.88, 0.22, 0.0, 0.16, -0.18, -0.05, 0.08, 0.12, 0.0, 0.0, "", "confused"),
    "happy": FaceStyle(0.35, 0.25, 0.0, 0.0, -0.06, -0.06, 0.95, 0.12, 0.55, 0.0, 0.0, "heart", "happy"),
    "excited": FaceStyle(0.48, 0.45, 0.0, 0.0, -0.02, -0.02, 0.78, 0.24, 0.45, 0.0, 0.0, "heart", "excited"),
    "sad": FaceStyle(0.28, 0.5, 0.0, 0.0, -0.10, -0.10, -0.90, 0.04, 0.12, 0.0, 0.0, "tear", "sad"),
    "angry": FaceStyle(0.18, 0.16, 0.0, 0.0, 0.35, 0.38, -0.22, 0.05, 0.20, 0.45, 0.7, "spark", "angry"),
    "speaking": FaceStyle(0.72, 0.9, 0.0, 0.0, 0.02, 0.02, 0.18, 0.28, 0.18, 0.0, 0.0, "", "speaking"),
    "error": FaceStyle(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, -0.35, 0.06, 0.15, 0.55, 0.0, "", "error"),
    "searching": FaceStyle(0.68, 0.82, 0.12, 0.0, 0.10, 0.10, -0.05, 0.10, 0.10, 0.0, 0.0, "", "searching"),
    "stop": FaceStyle(0.16, 0.1, 0.0, 0.0, 0.30, 0.30, -0.30, 0.04, 0.16, 0.25, 0.2, "", "stop"),
}


def _clamp(value: float, low: float = 0.0, high: float = 1.0) -> float:
    return max(low, min(high, value))


def _lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def _ease(t: float) -> float:
    t = _clamp(t)
    return t * t * (3.0 - 2.0 * t)


def _mix_style(a: FaceStyle, b: FaceStyle, t: float) -> FaceStyle:
    return FaceStyle(
        eye_open=_lerp(a.eye_open, b.eye_open, t),
        eye_round=_lerp(a.eye_round, b.eye_round, t),
        eye_slant=_lerp(a.eye_slant, b.eye_slant, t),
        eye_cross=_lerp(a.eye_cross, b.eye_cross, t),
        brow_left=_lerp(a.brow_left, b.brow_left, t),
        brow_right=_lerp(a.brow_right, b.brow_right, t),
        mouth_curve=_lerp(a.mouth_curve, b.mouth_curve, t),
        mouth_open=_lerp(a.mouth_open, b.mouth_open, t),
        blush=_lerp(a.blush, b.blush, t),
        red=_lerp(a.red, b.red, t),
        shake=_lerp(a.shake, b.shake, t),
        particle=b.particle if t >= 0.5 else a.particle,
        label=b.label if t >= 0.5 else a.label,
    )


class CanvasFaceAnimator:
    def __init__(
        self,
        parent: tk.Misc,
        width: int | None = None,
        height: int | None = None,
        show_label: bool | None = None,
        background: str = "#111318",
    ) -> None:
        self.width = width or CFG.face_width
        self.height = height or CFG.face_height
        self.show_label = CFG.face_show_label if show_label is None else show_label
        self.background = background
        self.canvas = tk.Canvas(parent, width=self.width, height=self.height, bg=background, highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.canvas.bind("<Configure>", self._on_resize)

        self._current_state = "idle"
        self._current_text = ""
        self._from_style = STYLE_MAP["idle"]
        self._to_style = STYLE_MAP["idle"]
        self._transition_started = time.monotonic()
        self._transition_seconds = 0.42
        self._last_state_poll = 0.0
        self._state_poll_interval = 0.05
        self._next_blink = time.monotonic() + random.uniform(2.0, 4.0)
        self._blink_started = 0.0
        self._blink_seconds = 0.12
        self._particles: list[dict[str, float | str]] = []
        self._last_particle_spawn = 0.0
        self._ticker_running = False
        self._manual_state: str | None = None

    def set_state(self, state: str, text: str = "") -> None:
        self._manual_state = state
        self._current_state = state
        self._current_text = text
        self._from_style = self._sample_current_style()
        self._to_style = STYLE_MAP.get(state, STYLE_MAP["idle"])
        self._transition_started = time.monotonic()

    def _on_resize(self, _event: tk.Event) -> None:
        self.width = max(160, int(self.canvas.winfo_width()))
        self.height = max(120, int(self.canvas.winfo_height()))

    def _poll_state(self) -> None:
        if self._manual_state is not None:
            return
        now = time.monotonic()
        if now - self._last_state_poll < self._state_poll_interval:
            return
        self._last_state_poll = now
        try:
            data = get_face_state()
        except Exception:
            return
        state = str(data.get("state") or "idle").strip().lower()
        text = str(data.get("text") or "")
        if state != self._current_state or text != self._current_text:
            self._current_state = state
            self._current_text = text
            self._from_style = self._sample_current_style()
            self._to_style = STYLE_MAP.get(state, STYLE_MAP["idle"])
            self._transition_started = now

    def _sample_current_style(self) -> FaceStyle:
        now = time.monotonic()
        t = _ease((now - self._transition_started) / self._transition_seconds)
        return _mix_style(self._from_style, self._to_style, t)

    def _current_style(self) -> FaceStyle:
        now = time.monotonic()
        t = _ease((now - self._transition_started) / self._transition_seconds)
        return _mix_style(self._from_style, self._to_style, t)

    def _spawn_particle(self, style: FaceStyle, cx: float, cy: float, eye_dx: float) -> None:
        now = time.monotonic()
        if now - self._last_particle_spawn < 0.15:
            return
        self._last_particle_spawn = now
        if style.particle == "heart":
            self._particles.append(
                {
                    "kind": "heart",
                    "x": cx + eye_dx,
                    "y": cy - self.height * 0.08,
                    "vx": random.uniform(-0.35, 0.25),
                    "vy": random.uniform(-1.15, -0.65),
                    "life": 1.1,
                }
            )
        elif style.particle == "tear":
            self._particles.append(
                {
                    "kind": "tear",
                    "x": cx + eye_dx,
                    "y": cy - self.height * 0.06,
                    "vx": random.uniform(-0.04, 0.04),
                    "vy": random.uniform(0.55, 0.82),
                    "life": 1.2,
                }
            )
        elif style.particle == "spark":
            self._particles.append(
                {
                    "kind": "spark",
                    "x": cx + eye_dx,
                    "y": cy - self.height * 0.12,
                    "vx": random.uniform(-0.5, 0.5),
                    "vy": random.uniform(-0.5, 0.15),
                    "life": 0.7,
                }
            )

    def _update_particles(self, dt: float) -> None:
        alive: list[dict[str, float | str]] = []
        for particle in self._particles:
            particle["life"] = float(particle["life"]) - dt
            particle["x"] = float(particle["x"]) + float(particle["vx"]) * dt * 42.0
            particle["y"] = float(particle["y"]) + float(particle["vy"]) * dt * 42.0
            particle["vy"] = float(particle["vy"]) + 0.03 * dt * 42.0
            if float(particle["life"]) > 0:
                alive.append(particle)
        self._particles = alive

    def _draw_round_rect(self, x1: float, y1: float, x2: float, y2: float, r: float, **kwargs) -> None:
        points = [
            x1 + r,
            y1,
            x2 - r,
            y1,
            x2,
            y1,
            x2,
            y1 + r,
            x2,
            y2 - r,
            x2,
            y2,
            x2 - r,
            y2,
            x1 + r,
            y2,
            x1,
            y2,
            x1,
            y2 - r,
            x1,
            y1 + r,
            x1,
            y1,
        ]
        self.canvas.create_polygon(points, smooth=True, splinesteps=16, **kwargs)

    def _draw_eye(self, x: float, y: float, style: FaceStyle, scale: float, blink: float, side: int) -> None:
        eye_w = self.width * 0.085 * scale
        eye_h = self.height * 0.105 * scale * max(0.12, style.eye_open * blink)
        eye_slant = self.height * 0.016 * style.eye_slant * side
        face_fill = "#0f1218"
        pupil_fill = "#eaf0ff"
        outline = "#1b2331"
        if style.eye_cross >= 0.7:
            size = eye_w * 0.42
            self.canvas.create_line(x - size, y - size, x + size, y + size, fill=outline, width=max(2, int(scale * 2.6)))
            self.canvas.create_line(x - size, y + size, x + size, y - size, fill=outline, width=max(2, int(scale * 2.6)))
            return
        if eye_h < 1.6:
            self.canvas.create_line(x - eye_w * 0.42, y, x + eye_w * 0.42, y, fill=outline, width=max(2, int(scale * 2.5)))
            return
        self.canvas.create_oval(x - eye_w, y - eye_h, x + eye_w, y + eye_h, fill=face_fill, outline=outline, width=max(2, int(scale * 2.5)))
        pupil_w = eye_w * (0.34 + 0.18 * style.eye_round)
        pupil_h = eye_h * (0.34 + 0.18 * style.eye_round)
        self.canvas.create_oval(
            x - pupil_w,
            y - pupil_h + eye_slant,
            x + pupil_w,
            y + pupil_h + eye_slant,
            fill=pupil_fill,
            outline="",
        )
        if style.eye_round > 0.4:
            self.canvas.create_oval(
                x - pupil_w * 0.45,
                y - pupil_h * 0.45 + eye_slant,
                x + pupil_w * 0.2,
                y + pupil_h * 0.2 + eye_slant,
                fill="#ffffff",
                outline="",
            )

    def _draw_mouth(self, cx: float, cy: float, style: FaceStyle, scale: float, speaking_wave: float) -> None:
        mouth_w = self.width * (0.20 + 0.02 * max(0.0, style.mouth_curve)) * scale
        mouth_open = max(0.02, style.mouth_open)
        if self._current_state == "speaking":
            mouth_open = max(mouth_open, 0.08 + 0.08 * speaking_wave)
        if self._current_state == "happy":
            mouth_open = max(mouth_open, 0.06 + 0.04 * speaking_wave)
        mouth_h = self.height * 0.12 * scale * mouth_open
        curve = style.mouth_curve
        y = cy + self.height * 0.15 * scale
        x1 = cx - mouth_w
        x2 = cx + mouth_w
        if curve >= 0.7:
            self.canvas.create_arc(
                x1,
                y - mouth_h,
                x2,
                y + mouth_h * 1.2,
                start=200,
                extent=140,
                style=tk.ARC,
                outline="#161b24",
                width=max(3, int(scale * 3)),
            )
        elif curve <= -0.55:
            self.canvas.create_arc(
                x1,
                y - mouth_h * 1.1,
                x2,
                y + mouth_h,
                start=20,
                extent=140,
                style=tk.ARC,
                outline="#161b24",
                width=max(3, int(scale * 3)),
            )
        elif mouth_open > 0.15:
            self.canvas.create_oval(
                cx - mouth_w * 0.35,
                y - mouth_h * 0.55,
                cx + mouth_w * 0.35,
                y + mouth_h * 0.55,
                fill="#161b24",
                outline="#161b24",
                width=max(2, int(scale * 2)),
            )
        else:
            if abs(curve) < 0.1:
                self.canvas.create_line(x1, y, x2, y, fill="#161b24", width=max(3, int(scale * 3)))
            else:
                self.canvas.create_arc(
                    x1,
                    y - mouth_h,
                    x2,
                    y + mouth_h,
                    start=200 if curve > 0 else 20,
                    extent=140,
                    style=tk.ARC,
                    outline="#161b24",
                    width=max(3, int(scale * 3)),
                )

    def _draw_particles(self, scale: float) -> None:
        for particle in self._particles:
            x = float(particle["x"]) * scale
            y = float(particle["y"]) * scale
            life = float(particle["life"])
            if particle["kind"] == "heart":
                size = max(6.0, 9.0 * life)
                self.canvas.create_text(x, y, text="♥", fill="#ff7da8", font=("Arial", max(8, int(size))))
            elif particle["kind"] == "tear":
                size = max(3.0, 5.0 * life)
                self.canvas.create_oval(x - size, y - size, x + size, y + size * 1.8, fill="#7dd3ff", outline="")
            else:
                size = max(3.0, 6.0 * life)
                self.canvas.create_line(x - size, y, x + size, y, fill="#ff5a6a", width=max(2, int(size / 2)))

    def _draw_frame(self) -> None:
        self.canvas.delete("all")
        now = time.monotonic()
        dt = 1.0 / 30.0
        self._poll_state()
        style = self._current_style()
        if self._current_state == "angry":
            style = FaceStyle(
                style.eye_open,
                style.eye_round,
                style.eye_slant,
                style.eye_cross,
                style.brow_left,
                style.brow_right,
                style.mouth_curve,
                style.mouth_open,
                style.blush,
                style.red,
                style.shake,
                style.particle,
                style.label,
            )

        if now >= self._next_blink:
            self._blink_started = now
            self._next_blink = now + random.uniform(2.0, 4.0)
        blink = 1.0
        if self._blink_started:
            elapsed = now - self._blink_started
            if elapsed < self._blink_seconds:
                blink = 1.0 - _clamp(elapsed / self._blink_seconds)
            else:
                self._blink_started = 0.0

        if self._current_state in {"idle", "listening", "thinking", "speaking", "searching"}:
            breathing = 1.0 + 0.015 * math.sin(now * 2.0 * math.pi / 4.2)
        else:
            breathing = 1.0
        shake_x = 0.0
        shake_y = 0.0
        if style.shake > 0.0:
            amp = style.shake * 3.2
            shake_x = math.sin(now * 48.0) * amp
            shake_y = math.cos(now * 37.0) * amp * 0.25

        cx = self.width * 0.5 + shake_x
        cy = self.height * 0.5 + shake_y - self.height * 0.02
        face_w = self.width * 0.66 * breathing
        face_h = self.height * 0.74 * breathing
        face_fill = "#f2d6cc"
        if self._current_state == "angry":
            face_fill = "#efb0b0"
        elif self._current_state == "sad":
            face_fill = "#f0d0e0"
        elif self._current_state == "happy":
            face_fill = "#f2dfd0"
        elif self._current_state == "error":
            face_fill = "#f0b5c2"
        face_fill = face_fill
        outline = "#232734"
        if style.red > 0:
            face_fill = "#f0b0b6"
        shadow = "#0b0d11"
        self._draw_round_rect(
            cx - face_w * 0.53,
            cy - face_h * 0.46 + 6,
            cx + face_w * 0.53,
            cy + face_h * 0.46 + 6,
            r=min(face_w, face_h) * 0.18,
            fill=shadow,
            outline="",
        )
        self._draw_round_rect(
            cx - face_w * 0.53,
            cy - face_h * 0.46,
            cx + face_w * 0.53,
            cy + face_h * 0.46,
            r=min(face_w, face_h) * 0.18,
            fill=face_fill,
            outline=outline,
            width=max(2, int(self.width * 0.012)),
        )

        eye_sep = self.width * 0.14
        eye_y = cy - self.height * 0.10
        self._draw_eye(cx - eye_sep, eye_y, style, breathing, blink, -1)
        self._draw_eye(cx + eye_sep, eye_y, style, breathing, blink, 1)

        brow_y = eye_y - self.height * 0.08
        brow_len = self.width * 0.10
        brow_thickness = max(2, int(self.width * 0.012))
        self.canvas.create_line(
            cx - eye_sep - brow_len,
            brow_y + self.height * 0.02 * style.brow_left,
            cx - eye_sep + brow_len,
            brow_y - self.height * 0.03 * style.brow_left,
            fill="#232734",
            width=brow_thickness,
            capstyle=tk.ROUND,
        )
        self.canvas.create_line(
            cx + eye_sep - brow_len,
            brow_y - self.height * 0.03 * style.brow_right,
            cx + eye_sep + brow_len,
            brow_y + self.height * 0.02 * style.brow_right,
            fill="#232734",
            width=brow_thickness,
            capstyle=tk.ROUND,
        )

        cheek_w = self.width * 0.06
        cheek_h = self.height * 0.04
        cheek_fill = "#f29cb3" if style.blush >= 0.2 else "#dba7b5"
        self.canvas.create_oval(
            cx - eye_sep - cheek_w,
            cy + self.height * 0.00,
            cx - eye_sep + cheek_w,
            cy + self.height * 0.00 + cheek_h,
            fill=cheek_fill,
            outline="",
        )
        self.canvas.create_oval(
            cx + eye_sep - cheek_w,
            cy + self.height * 0.00,
            cx + eye_sep + cheek_w,
            cy + self.height * 0.00 + cheek_h,
            fill=cheek_fill,
            outline="",
        )

        speaking_wave = 0.5 + 0.5 * math.sin(now * 9.0)
        self._draw_mouth(cx, cy, style, breathing, speaking_wave)

        if self._current_state in {"happy", "sad", "angry"}:
            self._spawn_particle(style, cx, cy, -eye_sep)
            self._spawn_particle(style, cx, cy, eye_sep)
        self._update_particles(dt)
        self._draw_particles(breathing)

        if self.show_label:
            label = self._current_text or style.label or self._current_state
            self.canvas.create_text(
                12,
                12,
                anchor="nw",
                text=label,
                fill="#eef2ff",
                font=("Arial", max(10, int(self.height * 0.06)), "bold"),
            )

    def start(self) -> None:
        if self._ticker_running:
            return
        self._ticker_running = True
        self._loop()

    def _loop(self) -> None:
        if not self._ticker_running:
            return
        try:
            self._draw_frame()
        except Exception as exc:
            LOGGER.warning("face draw failed: %s", exc)
        self.canvas.after(33, self._loop)


class FaceOnlyApp:
    def __init__(self) -> None:
        self.root = tk.Tk()
        self.root.title("XiaoU Face")
        self.root.configure(bg="#111318")
        self.root.geometry(f"{CFG.face_width}x{CFG.face_height}")
        self.root.resizable(False, False)
        if CFG.face_fullscreen:
            self.root.attributes("-fullscreen", True)
        self.root.bind("<Escape>", lambda _event: self.root.destroy())
        self.root.bind("q", lambda _event: self.root.destroy())
        set_face_state("idle", "idle")
        self.animator = CanvasFaceAnimator(self.root, width=CFG.face_width, height=CFG.face_height, show_label=CFG.face_show_label)
        self.animator.start()

    def run(self) -> None:
        self.root.mainloop()


def main() -> None:
    FaceOnlyApp().run()


XiaoUTkEmoteDisplay = FaceOnlyApp


if __name__ == "__main__":
    main()
