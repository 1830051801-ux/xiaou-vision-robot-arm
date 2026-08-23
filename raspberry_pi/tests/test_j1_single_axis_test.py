import sys
from pathlib import Path

PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR / "robot_ai"))

from arm_control.j1_single_axis_test import build_j1_frame  # noqa: E402
from arm_control.uart_protocol import CMD_TRAJ_POINT  # noqa: E402


def test_j1_payload_is_formal_single_axis_trajectory_point() -> None:
    frame = build_j1_frame(60.0, 2500, 9)
    assert frame.cmd == CMD_TRAJ_POINT
    assert len(frame.payload) == 26
    assert frame.payload.hex(" ") == "00 00 70 42 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 c4 09"


def test_j1_payload_rejects_invalid_duration() -> None:
    try:
        build_j1_frame(60.0, 0, 9)
    except ValueError:
        return
    raise AssertionError("zero speed must be rejected")
