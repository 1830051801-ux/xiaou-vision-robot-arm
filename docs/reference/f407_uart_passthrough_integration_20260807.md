# F407 UART 透传集成说明

本文件是概览。精确帧格式、响应码、接线和测试阶段以
`docs/F407_UART_联调协议_v1.0_20260808.md` 为准。

## 当前边界

```text
语音/VLA 意图 -> YOLO 固定俯视相机中心点 -> Pi 确定性门禁
-> Pi UART 编解码 -> STM32F407 USART1 -> 锁定响应/状态回传
```

当前 F407 是 `transport-only` 映像：不初始化 CAN、不初始化夹爪 PWM、不启动轨迹或标定线程。Pi 不拥有 CAN，总线所有权仍属于未来经过审查的 F407 硬件使能映像。

- 六轴固定为 J1..J6；Pi 发送的轨迹结构为 `<6fH>`，不是旧四轴帧。
- 夹爪结构为 `<Bf>`；当前只校验长度并返回锁定响应。
- VLA 只能选择意图、物品类别和目标，不得生成关节角、PWM 或 CAN 帧。
- 固定相机为 1920x1080；目标中心点、类别高度和单应性必须通过 Pi 的确定性检查。
- `hardware_calibration.json` 中实机动作门必须保持 `false`，直到逐轴完成实测。

## 安全 PING

仅在接线、F407 刷机和安全基线核实后，Pi 上可以运行：

```bash
cd /home/pi/raspi_robot_ai
python3 robot_ai/arm_control/uart_protocol.py --ping --port /dev/serial0 --baud 115200 --sequence 1
```

它只发送 `PING` 并等待 ACK，不发送 CAN、PWM、夹爪或关节命令。完整的 J1..J6 离线帧、锁定响应和明日联调顺序见 `docs/树莓派_F407_分阶段联调清单_20260808.md`。
