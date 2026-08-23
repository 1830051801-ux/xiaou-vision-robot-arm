#!/usr/bin/env python3
"""Simulate 8-frame trajectory batch frame generation for protocol doc."""
import struct

CRC16_TABLE = [
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040,
]

def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc = (crc >> 8) ^ CRC16_TABLE[(crc ^ b) & 0xFF]
    return crc

def escape_payload(payload):
    """Apply byte stuffing to payload."""
    result = bytearray()
    for b in payload:
        if b == 0xAA:
            result.extend([0xBB, 0x0A])
        elif b == 0x55:
            result.extend([0xBB, 0x05])
        elif b == 0xBB:
            result.extend([0xBB, 0x0B])
        else:
            result.append(b)
    return bytes(result)

def build_frame(cmd, seq, payload):
    """Build a complete protocol frame with CRC and byte stuffing."""
    raw_payload = bytes(payload)
    stuffed = escape_payload(raw_payload)
    crc_input = bytes([cmd, len(raw_payload), seq]) + raw_payload
    crc = crc16(crc_input)
    frame = bytes([0xAA, cmd, len(raw_payload), seq]) + stuffed
    frame += bytes([crc & 0xFF, (crc >> 8) & 0xFF, 0x55])
    return frame, crc

# ============================================================
# Scenario: A(0,0,0,0,0,0) -> B(84,-42,56,28,-28,42), 8 frames, 40ms each
# ============================================================
print("=" * 70)
print("轨迹批量传输模拟: A点 → B点, 8帧, 每帧40ms")
print("=" * 70)

frames = []
for i in range(8):
    t = i / 7.0
    frames.append((84.0*t, -42.0*t, 56.0*t, 28.0*t, -28.0*t, 42.0*t, 40))

print("\n运动描述:")
print(f"  起点A: J1=0°  J2=0°   J3=0°  J4=0°  J5=0°   J6=0°")
print(f"  终点B: J1=84° J2=-42° J3=56° J4=28° J5=-28° J6=42°")
print(f"  总时长: 8×40ms = 320ms")
print(f"  传输方式: CMD_TRAJ_BATCH (0x53), 8帧合并发送\n")

print("-" * 70)
print("各轨迹点详情:")
print("-" * 70)
print(f"{'帧':>3}  {'J1(°)':>8}  {'J2(°)':>8}  {'J3(°)':>8}  {'J4(°)':>8}  {'J5(°)':>8}  {'J6(°)':>8}  {'dur(ms)':>8}")
print("-" * 70)

for i, f in enumerate(frames):
    raw = struct.pack('<ffffffH', *f)
    marker = "← 起点A" if i == 0 else ("← 终点B" if i == 7 else "")
    print(f"{i:3d}  {f[0]:8.1f}  {f[1]:8.1f}  {f[2]:8.1f}  {f[3]:8.1f}  {f[4]:8.1f}  {f[5]:8.1f}  {f[6]:8d}     {marker}")

# Build batch payload
batch_payload = b''
for f in frames:
    batch_payload += struct.pack('<ffffffH', *f)

print(f"\n单帧尺寸: 6×float32 + uint16 = 26 字节")
print(f"批量 Payload: 8 × 26 = {len(batch_payload)} 字节")

# Build frame
cmd, seq = 0x53, 0x01
frame, crc = build_frame(cmd, seq, batch_payload)

print(f"\n{'='*70}")
print("实际发送的完整帧 (CMD_TRAJ_BATCH, SEQ=0x01):")
print(f"{'='*70}")
print(f"  帧头:     0xAA")
print(f"  命令:     0x{cmd:02X} (CMD_TRAJ_BATCH)")
print(f"  Payload长: 0x{len(batch_payload):02X} ({len(batch_payload)} 字节)")
print(f"  序列号:   0x{seq:02X}")
print(f"  CRC16:    0x{crc:04X}")
print(f"  帧尾:     0x55")
print(f"  总帧长:   {len(frame)} 字节")
print()

# Hex dump
print("Hex Dump (可直接用于串口发送):")
for i in range(0, len(frame), 16):
    chunk = frame[i:i+16]
    hex_str = ' '.join(f'{b:02X}' for b in chunk)
    ascii_str = ''.join(chr(b) if 32 <= b < 127 else '.' for b in chunk)
    print(f"  {i:04X}:  {hex_str:<48s}  {ascii_str}")

# Show payload byte-by-byte annotation
print(f"\n{'='*70}")
print("Payload 逐字节解析 (26B × 8 = 208B):")
print(f"{'='*70}")
for fi, f in enumerate(frames):
    raw = struct.pack('<ffffffH', *f)
    offset = fi * 26
    print(f"\n  Frame {fi} (offset {offset}~{offset+25}):")
    # J1-J6 floats
    names = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']
    for ji in range(6):
        bo = ji * 4
        fb = raw[bo:bo+4]
        print(f"    [{offset+bo:3d}~{offset+bo+3:3d}] {names[ji]:3s} = {f[ji]:8.2f}°  →  {fb.hex(' ').upper():14s}  (float32 LE)")
    # Duration
    db = raw[24:26]
    print(f"    [{offset+24:3d}~{offset+25:3d}] dur = {f[6]:8d}ms →  {db.hex(' ').upper():14s}  (uint16 LE)")

# ============================================================
# MCU Response
# ============================================================
rsp_cmd, rsp_seq, buf_rem = 0x81, 0x01, 8
rsp_payload = bytes([buf_rem])
rsp_frame, rsp_crc = build_frame(rsp_cmd, rsp_seq, rsp_payload)

print(f"\n{'='*70}")
print("MCU 响应帧 (RSP_TRAJ_ACK):")
print(f"{'='*70}")
print(f"  帧头:     0xAA")
print(f"  命令:     0x{rsp_cmd:02X} (RSP_TRAJ_ACK)")
print(f"  Payload长: 0x01 (1 字节)")
print(f"  序列号:   0x{rsp_seq:02X} (回显)")
print(f"  Payload:  0x{buf_rem:02X} → buf_remaining = {buf_rem} (16槽-8帧=8剩余)")
print(f"  CRC16:    0x{rsp_crc:04X}")
print(f"  帧尾:     0x55")
print(f"\n  Hex: {rsp_frame.hex(' ').upper()}")

# ============================================================
# Byte stuffing demo
# ============================================================
print(f"\n{'='*70}")
print("字节转义说明:")
print(f"{'='*70}")
# Check if any payload bytes need escaping
needs_escape = any(b in (0xAA, 0x55, 0xBB) for b in batch_payload)
if needs_escape:
    print("  [WARN] 本帧 Payload 包含需转义的字节")
else:
    print("  [OK] 本帧 Payload 中不含 0xAA/0x55/0xBB，无需字节转义")
    print("    帧中出现的 AA 仅为帧头，55 仅为帧尾，BB 未出现")

print(f"\n{'='*70}")
print("Pi 侧发送伪代码:")
print(f"{'='*70}")
print("""
# 构建8个轨迹点
points = []
for i in range(8):
    t = i / 7.0
    points.append((84.0*t, -42.0*t, 56.0*t, 28.0*t, -28.0*t, 42.0*t, 40))

# 打包为 CMD_TRAJ_BATCH (0x53)
payload = struct.pack('<ffffffH', *points[0])  # ×8 拼接
for p in points[1:]:
    payload += struct.pack('<ffffffH', *p)

# 发送
frame = build_frame(0x53, seq=1, payload)
ser.write(frame)

# 等待响应
resp = read_response(ser, timeout=1.0)
if resp.cmd == 0x81:
    print(f"MCU 已接收, 缓冲区剩余 {resp.payload[0]} 槽位")
    # buf_remaining=8, 可继续发送
""")

print("\n模拟完成。")
