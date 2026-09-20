from __future__ import annotations

from pathlib import Path

import tkinter as tk
from PIL import Image, ImageSequence, ImageTk

from common import PROJECT_DIR
from face_state import get_face_state
from xiaou_runtime import get_logger, get_xiaou_config


CFG = get_xiaou_config()
LOGGER = get_logger(__name__)

FACE_ASSET_DIR = PROJECT_DIR / "robot_ai" / "emote_assets" / "gif"
FACE_BG = "#151a22"
FACE_EDGE = "#2a3342"
FACE_STATE_TO_GIF = {
    "greet": "smile.gif",
    "idle": "idle.gif",
    "listening": "investigate.gif",
    "thinking": "ponder.gif",
    "searching": "investigate.gif",
    "curious": "question.gif",
    "question": "question.gif",
    "confused": "question.gif",
    "mock": "mock.gif",
    "happy": "smile.gif",
    "smile": "smile.gif",
    "laugh": "laugh.gif",
    "excited": "laugh.gif",
    "celebrate": "laugh.gif",
    "sad": "sad.gif",
    "shocked": "shocked.gif",
    "error": "shocked.gif",
    "stop": "shocked.gif",
    "angry": "angry.gif",
    "speaking": "smile.gif",
}


def _remove_green_bg(image: Image.Image) -> Image.Image:
    image = image.convert("RGBA")
    if not CFG.face_remove_green_bg:
        return image
    pixels = image.load()
    width, height = image.size
    for y in range(height):
        for x in range(width):
            r, g, b, a = pixels[x, y]
            if g > 180 and r < 80 and b < 80:
                pixels[x, y] = (0, 0, 0, a)
    return image


def _fit_face(image: Image.Image, size: tuple[int, int]) -> Image.Image:
    image = _remove_green_bg(image)
    background = Image.new("RGBA", image.size, (21, 26, 34, 255))
    background.alpha_composite(image)
    image = background.convert("RGB")
    image.thumbnail(size, Image.Resampling.LANCZOS)
    canvas = Image.new("RGB", size, (21, 26, 34))
    canvas.paste(image, ((size[0] - image.width) // 2, (size[1] - image.height) // 2))
    return canvas


class GifFacePanel:
    def __init__(
        self,
        parent: tk.Misc,
        width: int | None = None,
        height: int | None = None,
        *,
        fill: bool = False,
        padx: int = 10,
        pady: tuple[int, int] | int = (8, 6),
    ) -> None:
        self.width = width or CFG.face_width
        self.height = height or CFG.face_height
        self._gif_name = ""
        self._frame_index = 0
        self._frames: list[tk.PhotoImage] = []
        self._durations: list[int] = []

        self.frame = tk.Frame(parent, bg=FACE_BG, highlightthickness=0 if fill else 1, highlightbackground=FACE_EDGE)
        self.frame.pack(fill=tk.BOTH if fill else tk.X, expand=fill, padx=padx, pady=pady)
        self.canvas = tk.Canvas(self.frame, width=self.width, height=self.height, bg=FACE_BG, highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.image_item = self.canvas.create_image(self.width // 2, self.height // 2, anchor=tk.CENTER)
        self.canvas.bind("<Configure>", self._on_configure)
        placeholder = Image.new("RGB", (self.width, self.height), (21, 26, 34))
        self.placeholder = ImageTk.PhotoImage(placeholder)
        self.canvas.itemconfigure(self.image_item, image=self.placeholder)

    def _on_configure(self, event: tk.Event) -> None:
        self.canvas.coords(self.image_item, max(1, event.width) // 2, max(1, event.height) // 2)

    def _load_animation(self, gif_name: str) -> None:
        path = FACE_ASSET_DIR / gif_name
        if not path.exists():
            raise FileNotFoundError(path)
        frames: list[tk.PhotoImage] = []
        durations: list[int] = []
        frame_step = max(1, int(getattr(CFG, "face_frame_step", 1)))
        frame_limit = max(1, int(getattr(CFG, "face_max_frames", 24)))
        with Image.open(path) as gif:
            for source_index, frame in enumerate(ImageSequence.Iterator(gif)):
                if source_index % frame_step != 0:
                    continue
                fitted = _fit_face(frame.copy(), (self.width, self.height))
                frames.append(ImageTk.PhotoImage(fitted))
                source_delay = int(frame.info.get("duration", 70))
                durations.append(max(90, min(source_delay * frame_step, 160)))
                if len(frames) >= frame_limit:
                    break
        if not frames:
            raise RuntimeError(f"No frames in {path}")
        self._frames = frames
        self._durations = durations
        self._frame_index = 0
        self._gif_name = gif_name

    def update(self, root: tk.Tk) -> None:
        state = str(get_face_state().get("state", "idle")).strip().lower()
        gif_name = FACE_STATE_TO_GIF.get(state, "idle.gif")
        if gif_name != self._gif_name:
            try:
                self._load_animation(gif_name)
            except Exception as exc:
                LOGGER.warning("face gif load failed: %s", exc)
                self._frames = [self.placeholder]
                self._durations = [120]
                self._gif_name = gif_name

        if self._frames:
            self.canvas.itemconfigure(self.image_item, image=self._frames[self._frame_index])
            delay = self._durations[self._frame_index] if self._durations else 120
            self._frame_index = (self._frame_index + 1) % len(self._frames)
            root.after(delay, lambda: self.update(root))


class FaceOnlyApp:
    def __init__(self) -> None:
        self.root = tk.Tk()
        self.root.title("XiaoU Face")
        self.root.configure(bg=FACE_BG)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.geometry(f"{max(1, self.root.winfo_screenwidth())}x{max(1, self.root.winfo_screenheight())}+0+0")
        self.root.attributes("-fullscreen", True)
        self.root.bind("<Escape>", self._exit_fullscreen)
        self.root.bind("<F11>", self._toggle_fullscreen)
        self.root.update_idletasks()
        self.face = GifFacePanel(
            self.root,
            width=max(1, self.root.winfo_screenwidth()),
            height=max(1, self.root.winfo_screenheight()),
            fill=True,
            padx=0,
            pady=0,
        )
        self.root.after(60, lambda: self.face.update(self.root))
        self.root.after(200, self._raise_window)

    def _raise_window(self) -> None:
        try:
            self.root.lift()
            self.root.focus_force()
            self.root.attributes("-topmost", True)
            self.root.after(250, lambda: self.root.attributes("-topmost", False))
        except Exception:
            pass

    def _toggle_fullscreen(self, _event: tk.Event | None = None) -> None:
        current = bool(self.root.attributes("-fullscreen"))
        self.root.attributes("-fullscreen", not current)

    def _exit_fullscreen(self, _event: tk.Event | None = None) -> None:
        self.root.attributes("-fullscreen", False)

    def close(self) -> None:
        try:
            self.root.destroy()
        except Exception:
            pass

    def run(self) -> None:
        self.root.mainloop()


def main() -> None:
    FaceOnlyApp().run()


if __name__ == "__main__":
    main()
