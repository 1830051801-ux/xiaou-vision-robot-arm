from __future__ import annotations

import json
import struct
import time
from typing import Any

from xiaou_runtime import get_xiaou_config


HEAD = 0xAA
TAIL = 0x55
ESC = 0xBB

ESCAPE_BYTES = {
    0xAA: 0x0A,
    0x55: 0x05,
    0xBB: 0x0B,
}

UNESCAPE_BYTES = {value: key for key, value in ESCAPE_BYTES.items()}

CMD_IDS = {
    "ping": 0x01,
    "heartbeat": 0x01,
    "estop": 0x02,
    "stop": 0x13,
    "clear_error": 0x03,
    "move_joints": 0x11,
    "joints": 0x11,
    "move_cart": 0x12,
    "pick": 0x14,
    "grasp_move": 0x14,
    "get_state": 0x20,
    "set_zero": 0x30,
    "calib_start": 0x35,
    "calib_end": 0x36,
    "set_sys_origin": 0x37,
}

GRIP_TYPE_IDS = {
    "top_grip": 0x01,
    "side_grip": 0x02,
    "flat_grip": 0x03,
    "no_pick": 0x00,
}

OBJECT_TYPE_IDS = {
    "pen": 0x00,
    "cup": 0x01,
    "bottle": 0x02,
    "cola": 0x03,
    "earphone": 0x04,
}

LEGACY_CMD_IDS = {
    "heartbeat": 0x00,
    "pick": 0x50,
    "not_found": 0x02,
    "tidy": 0x03,
    "stop": 0x04,
    "home": 0x05,
    "chat": 0x06,
}


def _i16(value: Any) -> int:
    value = int(round(float(value or 0)))
    return max(-32768, min(32767, value))


def _i16_x10(value: Any) -> int:
    value = int(round(float(value or 0) * 10.0))
    return max(-32768, min(32767, value))


def _u16(value: Any) -> int:
    value = int(round(float(value or 0)))
    return max(0, min(65535, value))


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def escape_payload(data: bytes) -> bytes:
    out = bytearray()
    for byte in data:
        escaped = ESCAPE_BYTES.get(byte)
        if escaped is None:
            out.append(byte)
        else:
            out.append(ESC)
            out.append(escaped)
    return bytes(out)


def unescape_payload(data: bytes) -> bytes:
    out = bytearray()
    i = 0
    while i < len(data):
        byte = data[i]
        if byte != ESC:
            out.append(byte)
            i += 1
            continue
        if i + 1 >= len(data):
            raise ValueError("truncated escape sequence")
        mapped = UNESCAPE_BYTES.get(data[i + 1])
        if mapped is None:
            raise ValueError(f"invalid escape sequence: 0x{data[i + 1]:02X}")
        out.append(mapped)
        i += 2
    return bytes(out)


def _grip_id(payload: dict) -> int:
    if "grip_id" in payload:
        return max(0, min(5, int(payload.get("grip_id") or 0)))
    grip_type = str(payload.get("grip_type", "top_grip"))
    return GRIP_TYPE_IDS.get(grip_type, GRIP_TYPE_IDS["top_grip"])


def _object_id(payload: dict) -> int:
    if "object_id" in payload:
        return max(0, min(255, int(payload.get("object_id") or 0)))
    obj = str(payload.get("object", "")).strip().lower()
    return OBJECT_TYPE_IDS.get(obj, 0xFF)


def _build_move_cart_payload(payload: dict) -> bytes:
    x = float(payload.get("x_base_mm", payload.get("x", 0.0)))
    y = float(payload.get("y_base_mm", payload.get("y", 0.0)))
    z = float(payload.get("z_mm", payload.get("z_base_mm", payload.get("z_grab_mm", 0.0))))
    rx = float(payload.get("rx_deg", payload.get("roll_deg", 0.0)))
    ry = float(payload.get("ry_deg", payload.get("pitch_deg", 0.0)))
    rz = float(payload.get("rz_deg", payload.get("yaw_deg", payload.get("theta_deg", 0.0))))
    speed = float(payload.get("speed", payload.get("speed_mm_s", 50.0)))
    accel = float(payload.get("accel", payload.get("accel_mm_s2", 100.0)))
    object_id = _object_id(payload)
    grip_id = _grip_id(payload)
    force = max(0, min(100, int(payload.get("grip_force_pct", 60))))
    return struct.pack(
        "<ffffffffBBBB",
        x,
        y,
        z,
        rx,
        ry,
        rz,
        speed,
        accel,
        object_id,
        grip_id,
        force,
        0,
    )


def _build_grasp_move_payload(payload: dict) -> bytes:
    pick_x = float(payload.get("pick_x_mm", payload.get("x_base_mm", payload.get("x", 0.0))))
    pick_y = float(payload.get("pick_y_mm", payload.get("y_pick_mm", payload.get("y_base_mm", payload.get("y", 0.0)))))
    pick_z = float(payload.get("pick_z_mm", payload.get("z_pick_mm", payload.get("z_mm", 225.0))))
    pick_yaw = float(payload.get("pick_yaw_deg", payload.get("yaw_deg", payload.get("theta_deg", 0.0))))

    drop_x = float(payload.get("drop_x_mm", pick_x))
    drop_y = float(payload.get("drop_y_mm", pick_y))
    drop_z = float(payload.get("drop_z_mm", pick_z))
    drop_yaw = float(payload.get("drop_yaw_deg", pick_yaw))

    z_safe = float(payload.get("grasp_z_safe_mm", payload.get("safe_z_mm", 50.0)))
    grip_id = max(0, min(255, int(payload.get("grip_id", 3))))
    mode = max(0, min(255, int(payload.get("mode", 2))))

    return struct.pack(
        "<hhhhhhhhhBB",
        _i16_x10(pick_x),
        _i16_x10(pick_y),
        _i16_x10(pick_z),
        _i16_x10(pick_yaw),
        _i16_x10(drop_x),
        _i16_x10(drop_y),
        _i16_x10(drop_z),
        _i16_x10(drop_yaw),
        _i16_x10(z_safe),
        grip_id,
        mode,
    )


def _encode_stm_grasp_frame(payload: dict, seq: int = 0) -> bytes:
    body = _build_grasp_move_payload(payload)
    if len(body) != 20:
        raise ValueError(f"grasp payload must be 20 bytes, got {len(body)}")
    return bytes([HEAD, CMD_IDS["pick"], len(body), seq & 0xFF]) + body + b"\xFF\xFF" + bytes([TAIL])


def _build_move_joints_payload(payload: dict) -> bytes:
    joints = payload.get("joints")
    if joints is None:
        values = [
            payload.get("j1_deg", payload.get("j1", 0.0)),
            payload.get("j2_deg", payload.get("j2", 0.0)),
            payload.get("j3_deg", payload.get("j3", 0.0)),
            payload.get("j4_deg", payload.get("j4", 0.0)),
        ]
    else:
        values = list(joints)[:4]
        while len(values) < 4:
            values.append(0.0)
    speed = float(payload.get("speed", payload.get("speed_deg_s", 30.0)))
    accel = float(payload.get("accel", payload.get("accel_deg_s2", 60.0)))
    return struct.pack(
        "<ffffff",
        float(values[0]),
        float(values[1]),
        float(values[2]),
        float(values[3]),
        speed,
        accel,
    )


def _encode_modbus_frame(cmd: int, payload: bytes, seq: int = 0) -> bytes:
    if len(payload) > 248:
        raise ValueError(f"payload too long: {len(payload)} bytes")
    header = bytes([cmd & 0xFF, len(payload) & 0xFF, seq & 0xFF])
    crc = crc16_modbus(header + payload)
    return bytes([HEAD]) + header + escape_payload(payload) + bytes([crc & 0xFF, (crc >> 8) & 0xFF, TAIL])


def _encode_legacy_frame(payload: dict, seq: int = 0) -> bytes:
    cmd = LEGACY_CMD_IDS.get(str(payload.get("cmd", "chat")), LEGACY_CMD_IDS["chat"])
    x = _i16(payload.get("x_base_mm", payload.get("x", 0)))
    y = _i16(payload.get("y_base_mm", payload.get("y", 0)))
    theta10 = _i16(float(payload.get("theta_deg", 0.0)) * 10.0)
    z_safe = _i16(payload.get("z_safe_mm", 0))
    z_grab = _i16(payload.get("z_grab_mm", 0))
    width = _u16(payload.get("width_mm", 0))
    gripper_open = _u16(payload.get("gripper_open_mm", width))
    gripper_close = _u16(payload.get("gripper_close_mm", 0))
    grip_type = GRIP_TYPE_IDS.get(str(payload.get("grip_type", "top_grip")), GRIP_TYPE_IDS["top_grip"])
    force = max(0, min(100, int(payload.get("grip_force_pct", 60))))
    body = struct.pack(
        ">BhhhhhHHHBBB",
        cmd,
        x,
        y,
        theta10,
        z_safe,
        z_grab,
        width,
        gripper_open,
        gripper_close,
        grip_type,
        force,
        seq & 0xFF,
    )
    checksum = sum(body) & 0xFF
    return bytes([HEAD, len(body)]) + body + bytes([checksum, 0xBB])


def encode_motion_frame(payload: dict, seq: int = 0) -> bytes:
    cmd_name = str(payload.get("cmd", "chat")).strip().lower()
    if cmd_name in {"pick", "grasp_move"}:
        return _encode_stm_grasp_frame(payload, seq=seq)
    if cmd_name in {"move_cart", "cart"}:
        return _encode_modbus_frame(CMD_IDS["move_cart"], _build_move_cart_payload(payload), seq=seq)
    if cmd_name in {"move_joints", "joints"}:
        return _encode_modbus_frame(CMD_IDS["move_joints"], _build_move_joints_payload(payload), seq=seq)
    if cmd_name == "estop":
        return _encode_modbus_frame(CMD_IDS["estop"], b"", seq=seq)
    if cmd_name == "stop":
        return _encode_modbus_frame(CMD_IDS["stop"], b"", seq=seq)
    if cmd_name == "clear_error":
        return _encode_modbus_frame(CMD_IDS["clear_error"], b"", seq=seq)
    if cmd_name == "get_state":
        return _encode_modbus_frame(CMD_IDS["get_state"], b"", seq=seq)
    if cmd_name == "calib_end":
        return _encode_modbus_frame(CMD_IDS["calib_end"], b"", seq=seq)
    if cmd_name in {"set_zero", "calib_start"}:
        joint_id = max(1, min(4, int(payload.get("joint_id", 1))))
        return _encode_modbus_frame(CMD_IDS[cmd_name], bytes([joint_id]), seq=seq)
    if cmd_name == "set_sys_origin":
        body = struct.pack(
            "<fff",
            float(payload.get("x", payload.get("x_base_mm", 0.0))),
            float(payload.get("y", payload.get("y_base_mm", 0.0))),
            float(payload.get("z", payload.get("z_base_mm", 0.0))),
        )
        return _encode_modbus_frame(CMD_IDS["set_sys_origin"], body, seq=seq)
    if cmd_name in {"ping", "heartbeat"}:
        return _encode_modbus_frame(CMD_IDS["ping"], b"", seq=seq)
    return _encode_modbus_frame(CMD_IDS["ping"], b"", seq=seq)


def encode_heartbeat(seq: int = 0) -> bytes:
    return _encode_modbus_frame(CMD_IDS["ping"], b"", seq=seq)


def decode_motion_frame(frame: bytes) -> dict[str, Any]:
    if len(frame) < 7 or frame[0] != HEAD or frame[-1] != TAIL:
        raise ValueError("invalid frame boundary")
    cmd = frame[1]
    payload_len = frame[2]
    seq = frame[3]
    payload_raw = unescape_payload(frame[4:-3])
    crc_expected = frame[-3] | (frame[-2] << 8)
    crc_actual = crc16_modbus(frame[1:4] + payload_raw)
    if crc_expected != 0xFFFF and crc_actual != crc_expected:
        raise ValueError(f"crc mismatch: expected 0x{crc_expected:04X}, got 0x{crc_actual:04X}")
    if len(payload_raw) != payload_len:
        raise ValueError(f"payload length mismatch: expected {payload_len}, got {len(payload_raw)}")
    return {"cmd": cmd, "seq": seq, "payload": payload_raw}


def frame_to_hex(frame: bytes) -> str:
    return " ".join(f"{byte:02X}" for byte in frame)


def send_robot_payload(payload: dict, seq: int = 0) -> None:
    import serial

    cfg = get_xiaou_config()
    port = cfg.serial_port
    baud = cfg.serial_baud
    protocol = cfg.serial_protocol.strip().lower()

    with serial.Serial(port, baudrate=baud, timeout=1, write_timeout=1) as ser:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        time.sleep(0.05)
        # STM32 UART 唤醒: 丢一个哑字节吃掉 open 毛刺，确保帧头 0xAA 对齐
        ser.write(b'\x00')
        ser.flush()
        time.sleep(0.1)
        if protocol == "json":
            line = json.dumps(payload, ensure_ascii=False) + "\n"
            ser.write(line.encode("utf-8"))
            ser.flush()
            print(f"Sent JSON: {line.strip()}")
        elif protocol in {"legacy", "old", "compat"}:
            frame = _encode_legacy_frame(payload, seq=seq)
            ser.write(frame)
            ser.flush()
            print(f"Sent legacy frame: {frame_to_hex(frame)}")
        else:
            frame = encode_motion_frame(payload, seq=seq)
            ser.write(frame)
            ser.flush()
            print(f"Sent frame: {frame_to_hex(frame)}")

