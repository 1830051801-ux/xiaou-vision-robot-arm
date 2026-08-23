# F407 / Raspberry Pi UART 联调协议 V1.0

## 1. 当前阶段和边界

本协议用于六轴机械臂的 Pi 到 STM32F407 直接 UART 数据透传。当前固件是
`transport-only` 安全基线：它初始化 USART1 和 USART3，但不初始化 CAN、不初始化夹爪 PWM、不启动轨迹、标定或执行器线程。

因此当前阶段可以验证接线、收发、CRC、帧分片、状态回传和“动作被锁定”的响应；不能据此宣称关节、夹爪或急停已完成实机验证。

- 关节数固定为 J1..J6；不得降级为四轴帧。
- 旧 CAN 源码/记录速率为 `1 Mbps`，尚不是实测总线参数；本阶段不打开 CAN。
- VLA 只给出任务意图、类别和目标选择，不能直接生成关节角、PWM 或 CAN 数据。
- 当前 `hardware_calibration.json` 的所有实机动作门保持锁定。

## 2. 物理 UART

| 项目 | 值 |
|---|---|
| F407 端口 | USART1 |
| F407 引脚 | PA9 = TX，PA10 = RX |
| Pi 端口 | `/dev/serial0` |
| Pi 引脚 | GPIO14/TXD = TX，GPIO15/RXD = RX |
| 连接 | Pi TXD -> F407 PA10；Pi RXD <- F407 PA9；两端 GND 必须共地 |
| 电平 | 3.3 V TTL；禁止向 UART 引脚接入 5 V |
| 参数 | 115200 baud，8 data bits，no parity，1 stop bit，无硬件/软件流控 |

接线、供电、急停和 CAN 均应在断电状态下检查。UART 只传输数据，不为 F407 或电机供电。

## 3. 帧格式

```text
AA CMD LEN SEQ ESCAPED_PAYLOAD CRC_LO CRC_HI 55
```

| 字段 | 长度 | 说明 |
|---|---:|---|
| `AA` | 1 | 帧起始字节 `0xAA` |
| `CMD` | 1 | 命令或响应码 |
| `LEN` | 1 | 解码后 payload 长度，范围 0..248 |
| `SEQ` | 1 | 主机序号；应答回显该序号 |
| `ESCAPED_PAYLOAD` | 0..496 | 仅 payload 需要转义 |
| `CRC_LO CRC_HI` | 2 | CRC-16/MODBUS，小端序，字节不转义 |
| `55` | 1 | 帧结束字节 `0x55` |

CRC 覆盖未转义的 `CMD + LEN + SEQ + PAYLOAD`，初值 `0xFFFF`，反射多项式 `0xA001`。`CMD`、`LEN`、`SEQ` 和 CRC 字节始终原样传输，只有 payload 按下表转义：

| 原始 payload 字节 | 在线字节 |
|---:|---|
| `AA` | `BB 0A` |
| `55` | `BB 05` |
| `BB` | `BB 0B` |

固件流式解析，可接受任意帧分片；CRC 不匹配、错误转义、提前结束或 payload 长度超限的帧会被丢弃，不执行、不回 ACK。

PING 例子，序号为 1：

```text
TX AA 01 00 01 E1 C0 55
RX AA 00 00 01 B0 00 55
```

## 4. 类型和状态数据

多字节无符号数与 `float32` 均为小端序。`float32` 使用 IEEE-754 单精度。

`CMD_GET_STATE` 的 ACK payload 固定为 82 字节：

```text
state:u8 error:u8 estop:u8 motion_busy:u8
repeat 6 times: angle_deg:f32 speed_rpm:f32 torque_nm:f32 online:u8
```

当前安全基线的正常返回值为：

| 字段 | 值 | 含义 |
|---|---:|---|
| `state` | `0x05` | `SYS_STATE_TRANSPORT_ONLY` |
| `error` | `0x0C` | `ERR_MOTION_LOCKED` |
| `online` | `0` | 未初始化 CAN，不能声称关节在线 |
| `motion_busy` | `0` | 没有执行轨迹 |

`CMD_GET_JOINT` 的 ACK payload 固定为 14 字节：

```text
joint_id:u8 angle_deg:f32 speed_rpm:f32 torque_nm:f32 online:u8
```

## 5. 命令和当前响应

| CMD | 名称 | 主机 payload | 当前安全基线行为 |
|---:|---|---|---|
| `01` | `PING` | 空 | `RSP_ACK`，空 payload |
| `02` | `ESTOP` | 空 | 只置软件 estop 标志，`RSP_ACK`；不发送 CAN |
| `03` | `CLEAR_ERROR` | 空 | 只清软件 estop 标志，动作锁仍保留，`RSP_ACK` |
| `13` | `STOP` | 空 | 只清软件 busy 标志，`RSP_ACK` |
| `20` | `GET_STATE` | 空 | `RSP_ACK` + 82 字节状态 |
| `21` | `GET_JOINT` | `<B>`，ID 1..6 | `RSP_ACK` + 14 字节状态 |
| `22` | `STREAM_START` | `<H>`，100..10000 ms | `RSP_ACK` 回显周期，随后 `RSP_TELEMETRY` |
| `23` | `STREAM_STOP` | 空 | `RSP_ACK` |
| `30/31/32/33/34/35/36/39` | 零点、PID、限位、重启、保存、标定 | 见未来硬件版 | `RSP_MOTION_LOCKED` |
| `50` | `TRAJ_POINT` | `<6fH>`，26 字节 | payload 合法时 `RSP_MOTION_LOCKED` |
| `51` | `TRAJ_BUFFER_CLEAR` | 空 | `RSP_MOTION_LOCKED` |
| `52` | `GRIPPER` | `<Bf>`，5 字节 | payload 合法时 `RSP_MOTION_LOCKED` |

轨迹 payload 的六个 `float32` 是 J1..J6 的绝对角度（度），最后一个 `uint16` 为持续时间（ms）。夹爪 payload 的第一个字节是夹爪 ID，第二项是 PWM 角度（度）。它们在当前固件中只用于协议长度验证，绝不会到达 CAN 或 PWM。

## 6. 响应码

| 响应 | 值 | payload |
|---|---:|---|
| `RSP_ACK` | `00` | 对查询命令为定义的数据；普通安全命令为空 |
| `RSP_NACK` | `01` | `<B>` 原始未知 CMD |
| `RSP_BUSY` | `02` | 保留 |
| `RSP_ESTOP_ACTIVE` | `03` | 保留 |
| `RSP_INVALID_PARAM` | `04` | `<B>` 原始 CMD |
| `RSP_QUEUE_FULL` | `05` | 保留 |
| `RSP_MOTION_LOCKED` | `06` | `<B>` 原始 CMD |
| `RSP_TELEMETRY` | `80` | 固定 82 字节状态 payload |

上位机必须把 `RSP_MOTION_LOCKED` 视为预期的安全结果，而不是允许继续执行的 ACK。

## 7. 阶段性使用顺序

1. 在 Windows 完成 Python 单元测试和六轴模型检查；本轮不执行仿真。
2. 使用 Keil 构建并刷入本 `transport-only` F407 映像；确认启动日志明确显示 CAN/PWM 未初始化。
3. 在断电下核查交叉 TX/RX、共地和 3.3 V 电平，确认不使用 CAN/PWM 接口。
4. Pi 上只运行 `uart_protocol.py --ping`，确认 ACK。
5. 读取 `GET_STATE`，必须看到 `state=0x05`、`error=0x0C` 和 6 个离线关节记录。
6. 逐个运行 `joint_uart_staged_test.py --joint 1..6` 的默认 dry-run；默认不打开串口。
7. 已确认安全基线后，逐个使用 `--probe-locked` 验证合法 J1..J6 帧都得到 `RSP_MOTION_LOCKED`，仍不会运动。
8. 只有在独立硬件使能审查完成后，才讨论单轴低速实机测试。此时必须重新确认物理急停、零位、方向、限位、反馈和 CAN 实测速率。

任何失败、未知状态、未测量字段或意外 ACK 都应停止在当前阶段，不得把工作区、速度或限位放宽来“通过测试”。
