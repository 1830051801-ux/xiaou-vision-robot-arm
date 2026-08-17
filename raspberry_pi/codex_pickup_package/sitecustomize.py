from __future__ import annotations

import sys
from pathlib import Path


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
EXTRA_PATHS = [
    ROOT,
    ROOT / "robot_ai",
    ROOT / "codex_deskpet_package",
    HERE,
]

for path in EXTRA_PATHS:
    if path.exists():
        value = str(path)
        if value not in sys.path:
            sys.path.insert(0, value)
