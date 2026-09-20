# STM32F407 通信协议适配改动 v1.2

本文件将 `通信协议规范_v4.0_6轴轨迹.md` 适配到当前六轴 Pi/F407 架构。它
不解除 `transport-only` 安全基线，也不授权真实运动；硬件使能必须使用独立
Keil target。

## 1. 结论

v4.0 的物理 UART、基础帧、转义、CRC、`CMD_TRAJ_POINT (0x50)` 和
`CMD_GRIPPER (0x52)` 与当前架构兼容。当前不兼容项如下，必须按本 v1.2
实现后才能使用 v4.0 的批量轨迹能力：

| 项目 | v4.0 | 当前基线 | v1.2 决议 |
|---|---|---|---|
| 批量轨迹 | `CMD_TRAJ_BATCH=0x53` | 未定义，返回 NACK | 定义并校验 1..9 个完整点 |
| 批量响应 | `0x81/0x82` | 未定义 | 硬件 target 实现 `RSP_TRAJ_ACK/OVERFLOW` |
| 状态帧长度 | 文中同时出现 56 和 4+6x13 | 当前为 82 | 固定为 82 字节，56 为文档笔误 |
| CRC `0xFFFF` 跳过 | v4.0 调试建议 | 当前拒绝 | v1.2 生产和调试均禁止跳过 |
| 段时长 | 0..65535 ms | Pi 安全层为 1..10000 ms | 固定 1..10000 ms，0 和超长拒绝 |
| 超限行为 | 文中建议钳位 | 当前未执行 | 硬件 target 必须拒绝，不能静默钳位 |

## 2. 固定线协议

```text
AA CMD LEN SEQ ESCAPED_PAYLOAD CRC16_LO CRC16_HI 55
```

- UART：USART1 PA9 TX / PA10 RX，115200 8N1，无流控。
- `LEN` 是解转义后的 payload 长度，最大 248。
- 只有 payload 转义：`AA->BB 0A`，`55->BB 05`，`BB->BB 0B`。
- CRC-16/MODBUS 覆盖解转义后的 `CMD+LEN+SEQ+PAYLOAD`，初值 `FFFF`，
  多项式 `A001`，低字节先发送。
- CRC 不匹配、错误转义、`CRC=FFFF`、长度不符或非有限 float 一律丢弃，
  不入队、不执行、不回成功 ACK。

## 3. 六轴轨迹

### 3.1 单点 `CMD_TRAJ_POINT = 0x50`

payload 必须为 `<6fH>`，26 字节：J1..J6 六个绝对模型角度（deg）和
`duration_ms`。正的电机命令约定为“从对应电机齿轮侧看逆时针”。该约定不等价于
ROS/编码器符号；硬件 target 必须应用逐轴实测的 `encoder_direction` 和
`zero_offset` 后再发 CAN。

### 3.2 批量 `CMD_TRAJ_BATCH = 0x53`

payload 为 `N * <6fH>`，N 为 1..9，长度只能是 26、52、...、234 字节。
每一点都必须是完整的 J1..J6 绝对目标，禁止只传一个关节或传相对角度。

硬件 target 的入队必须是原子操作：先检查 `free_slots >= N` 和每一点的全部
安全条件，再一次性写入 FIFO。任一验证失败时，禁止写入任意部分。

| 条件 | 响应 |
|---|---|
| 入队成功 | `RSP_TRAJ_ACK=0x81`，payload `<B>` 为入队后的剩余槽位 |
| FIFO 空间不足 | `RSP_TRAJ_OVERFLOW=0x82`，空 payload |
| 急停已锁定 | `RSP_ESTOP_ACTIVE=0x03` |
| 正在清零/标定或不可接受状态 | `RSP_BUSY=0x02` |
| 长度、float、时间、零位、限位或反馈非法 | `RSP_INVALID_PARAM=0x04`，payload 为原 CMD `0x53` |

FIFO 设为 16 个点。`CMD_TRAJ_BUFFER_CLEAR=0x51` 只能在未执行 CAN 指令或已
停止后清空；`CMD_STOP=0x13` 和 `CMD_ESTOP=0x02` 必须立即清空 FIFO 并停止所有
CAN 运动输出。

## 4. 真实状态与零位

`CMD_GET_STATE=0x20` 的 `RSP_ACK` payload 固定 82 字节：

```text
state:u8 error:u8 estop:u8 motion_busy:u8
repeat J1..J6: angle_deg:f32 speed_rpm:f32 torque_nm:f32 online:u8
```

硬件 target 只能报告真实 CAN 反馈。离线、超时或错误的关节 `online=0`，其角度
不能填默认 0 冒充反馈。

`CMD_SET_ZERO=0x30` 的 payload 为 `<B>`，只接受 J1..J6。执行要求：系统不 busy、
急停有效、六轴在线、当前转速低于静止阈值。收到单轴命令后，将当前**原始反馈**记录
为该轴零位，读回验证后回 `RSP_ACK`。Pi 的“一键零位”应顺序发送 J1..J6 六次，
六次都 ACK 后再发 `CMD_SAVE_CONFIG=0x34`；任一失败则 STOP、报告失败、不得保存。

ready 位不是新的 MCU 命令：Pi 记录完成的 `ready_pose_rad` 后，发送一个完整
`CMD_TRAJ_POINT` 或 `CMD_TRAJ_BATCH` 到该六轴姿势。该动作仍必须通过限位、反馈和
急停门。

## 5. Keil 文件改动

| 文件 | 必须改动 |
|---|---|
| `Inc/comm_protocol.h` | 添加 `CMD_TRAJ_BATCH 0x53`、`RSP_TRAJ_ACK 0x81`、`RSP_TRAJ_OVERFLOW 0x82`；保留所有既有编号 |
| `Src/app_threads.c` | transport-only 分支仅校验 `0x53` 长度后回 `RSP_MOTION_LOCKED`；硬件 target 将有效帧交给轨迹 FIFO，不能在 UART 线程调用 CAN |
| `Inc/app_threads.h` | 定义 16 槽 `traj_fifo`、读写索引和 `CTRL_MSG_TRAJ_BATCH`；共享状态必须由互斥量保护 |
| `Src/trajectory.c`（新增） | 校验 6 个 finite float、时间、零位、反馈、限位；实现原子批量入队、10 ms 消费、到位/超时处理 |
| `Src/safety.c`（新增或现有安全线程） | 急停、CAN 超时、离线、过流/过温和限位必须停止并清 FIFO |
| `Src/calibration.c` | 实现 J1..J6 的 `CMD_SET_ZERO` 静止采样、读回、持久化；禁止默认零位 |
| `Src/can.c` / `Device/src/DrEmpower_can.c` | 确认 ID 1..6、CAN 实测速率、反馈时间戳和 bus-off 处理；只有安全线程允许恢复输出 |

`ARM_ACTUATOR_OUTPUTS_ENABLED=0` 的正式 target 只做第 1 行和 `app_threads.c`
锁定分支，绝不初始化 CAN/PWM。硬件 target 必须复制出来，不能直接改正式 target
宏。

## 6. Pi 侧适配

Pi 端已增加 `pack_trajectory_batch_payload()`：只构建 1..9 个点的 payload，仍受
`hardware_calibration.json` 的运动门控制。Pi 必须按 `0x81` 的 `free_slots` 发送，
收到 `0x82` 后等待至少 50 ms 再读取状态/重试；不得用估算值增加空槽，也不得在
未知响应、超时、ESTOP 或反馈异常时重发。

## 7. 验收顺序

1. `transport-only`：PING、GET_STATE、单点/批量合法帧均返回 `0x06`；不接 CAN。
2. 硬件 target 断开电机或使用受控 CAN：验证 `0x53` 的 26/52/234 合法长度和
   非法长度拒绝。
3. 六轴全部在线但不运动：验证 82 字节真实状态、急停、STOP 和 FIFO 清空。
4. 手动摆零位：按 J1..J6 执行静止 `SET_ZERO`，每轴读回，再 SAVE_CONFIG。
5. 填入实测方向、限位、速度、加速度；单轴低速小角度并验证反馈符号。
6. 记录 ready 位；只预览六轴帧；最终经明确实机授权后才发送第一条 ready 轨迹。
