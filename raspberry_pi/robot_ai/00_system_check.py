import os
import platform
import shutil
import subprocess
import sys
from pathlib import Path

from common import PROJECT_DIR


def run(command: list[str]) -> tuple[bool, str]:
    try:
        result = subprocess.run(command, capture_output=True, text=True, timeout=8)
        output = (result.stdout + result.stderr).strip()
        return result.returncode == 0, output
    except Exception as exc:
        return False, str(exc)


def exists(path: str) -> str:
    return "OK" if Path(path).exists() else "MISSING"


def main() -> None:
    print("Raspberry Pi robot AI system check")
    print("=" * 40)
    print(f"Project: {PROJECT_DIR}")
    print(f"Python: {sys.version.split()[0]}")
    print(f"Platform: {platform.platform()}")
    print(f"config.env: {exists(str(PROJECT_DIR / 'config.env'))}")
    print(f"requirements.txt: {exists(str(PROJECT_DIR / 'requirements.txt'))}")
    print("")

    print("Camera devices:")
    video_devices = sorted(Path("/dev").glob("video*"))
    if video_devices:
        for dev in video_devices:
            print(f"  {dev}")
    else:
        print("  No /dev/video* found. Check USB camera or CSI camera.")

    print("")
    print("Audio devices:")
    ok, output = run(["arecord", "-l"])
    print(output if ok or output else "  arecord unavailable")

    print("")
    print("Useful commands:")
    for cmd in ["python3", "pip", "v4l2-ctl", "rpicam-hello"]:
        print(f"  {cmd}: {shutil.which(cmd) or 'not found'}")

    print("")
    print("Environment:")
    for key in ["CAMERA_INDEX", "YOLO_MODEL", "AI_API_URL", "AI_MODEL"]:
        print(f"  {key}={os.getenv(key, '')}")


if __name__ == "__main__":
    main()
