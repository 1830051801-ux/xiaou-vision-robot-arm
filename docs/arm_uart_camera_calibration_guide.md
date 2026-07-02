# 小U机械臂联调指南

## 1. STM32 串口协议

当前树莓派侧按机械臂工程协议发送：

- 串口：`/dev/serial0`，`115200 8N1`
- 帧格式：`AA CMD LEN SEQ PAYLOAD CRC_LO CRC_HI 55`
- CRC：`CRC-16/MODBUS`，计算范围是 `CMD + LEN + SEQ + PAYLOAD`
- `move_cart` 命令：`CMD=0x12`
- `move_cart` payload：8 个 little-endian float32，共 32 字节

payload 顺序：

```text
x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg, speed, accel
```

默认测试帧示例：

```text
x=200, y=30, z=25, rx=0, ry=0, rz=0, speed=50, accel=100
```

## 2. 每 5 秒发送给 STM32

```bash
cd ~/raspi_robot_ai
source .venv/bin/activate
bash scripts/run_stm_motion_loop.sh
```

自定义坐标：

```bash
python robot_ai/16_stm_motion_send_loop.py \
  --port /dev/serial0 \
  --baud 115200 \
  --interval 5 \
  --x 200 --y 30 --z 25 \
  --rx 0 --ry 0 --rz 0 \
  --speed 50 --accel 100
```

脚本会打印：

- `frame check: cmd=0x12, payload_len=32`
- `sent ...`
- 如果 STM32 有回包，会打印 `rx ...`

## 3. USB 相机棋盘格标定

普通 USB 相机先做内参标定，改善透视和畸变误差。

采集棋盘格图片：

```bash
cd ~/raspi_robot_ai
source .venv/bin/activate
bash scripts/run_calib_capture.sh
```

如果不是默认 0 号相机：

```bash
CAMERA_INDEX=1 bash scripts/run_calib_capture.sh
```

计算内参：

```bash
CHESSBOARD_COLS=7 CHESSBOARD_ROWS=9 CHESSBOARD_SQUARE_M=0.02 \
bash scripts/run_camera_calibrate_chessboard.sh
```

输出文件：

```text
runtime/calibration/camera.yaml
```

## 4. 工作台四点标定

用于把图像像素坐标映射到机械臂底座平面坐标。

```bash
bash scripts/run_workspace_homography.sh \
  u1 v1 u2 v2 u3 v3 u4 v4 \
  x1 y1 x2 y2 x3 y3 x4 y4
```

建议用工作台四角，单位统一用 mm。

输出文件：

```text
codex_pickup_package/workspace_homography.yaml
```

## 5. 旋转角标定

用于修正 YOLO/视觉识别出来的物体角度和机械臂末端 `rz` 的偏差。

```bash
bash scripts/run_rotation_calibration.sh 10 15 -20 -14 35 40
```

含义是：

```text
图像角度10度 -> 机械臂rz 15度
图像角度-20度 -> 机械臂rz -14度
图像角度35度 -> 机械臂rz 40度
```

输出文件：

```text
runtime/calibration/rotation_calibration.yaml
```

视觉定位会自动读取这个文件，并把角度偏移应用到抓取角。

## 6. 关于深度

普通 USB 相机没有真实深度。第一版不要强上深度，推荐路线：

1. YOLO 预训练模型识别目标框。
2. 用框中心点得到像素坐标。
3. 用四点标定的 homography 转成机械臂 `x/y`。
4. `z` 先用固定桌面高度或按物体类别配置固定抓取高度。
5. 后续再考虑单目深度模型，只作为高度估计辅助，不作为第一版闭环核心。

## 7. 推荐联调顺序

1. 先跑 `ping` 或 5 秒 `move_cart`，确认 STM32 能收到帧并 ACK。
2. 用低速、低加速度发一个安全点位，确认机械臂动作方向正确。
3. 做 USB 相机棋盘格标定。
4. 做工作台四点标定。
5. 做旋转角标定。
6. 最后接 YOLO 识别和自动抓取。

