# STM32 六轴机械臂三项架构决策 v1.0

日期：2026-08-08  
适用范围：树莓派 ROS2 <-> STM32F407 UART <-> CAN 六轴电机

本文件只记录当前已经确定的三项决策，作为 Keil 硬件使能 target 的实现边界。

## 决策总表

| 编号 | 议题 | 最终决策 |
|---|---|---|
| A | `encoder_direction` / `zero_offset` 的归属 | **由 STM32F407 保存、加载和应用；Pi 只发送机械模型角度** |
| B | `trajectory.c` / `safety.c` 文件拆分 | **现在做最小拆分；不做全量重构** |
| C | `transport-only` 模式 | **必须实现，并长期保留为独立 Keil target** |

---

## A. 方向和零偏：MCU 负责

### A.1 结论

Pi/ROS2 发送相对于用户设定机械零位的六轴模型目标角度
`q_model_target_deg[6]`。F407 负责把模型角度转换为 CAN 电机的原始命令角度，
并把 CAN 原始反馈转换回模型角度。

```text
q_model_feedback = encoder_direction * (q_raw_feedback - zero_offset_raw)
q_raw_command    = zero_offset_raw + encoder_direction * q_model_target
```

其中 `encoder_direction[i]` 只能为 `+1` 或 `-1`。

### A.2 已知物理约定

J1..J6 的 CAN ID 固定为 1..6。对每一个电机，从电机齿轮侧观察，正的电机命令
对应逆时针旋转。这个约定是电机侧正方向，**不能直接推导** ROS/URDF 的
`encoder_direction`；后者必须通过逐轴小角度命令和真实反馈测量。

### A.3 STM32 必须实现

1. 定义六轴 `zero_offset_raw_deg[6]` 与 `encoder_direction[6]`。
2. `CMD_SET_ZERO (0x30)` 只接受单轴 ID 1..6；仅在六轴在线、机械臂静止、急停
   可用且无轨迹执行时记录当前原始反馈。
3. 每轴设零成功后读回验证；六轴成功后才允许 `CMD_SAVE_CONFIG (0x34)` 写 Flash。
4. Flash 记录：magic、结构版本、六组零偏、六组方向、CRC32；启动时 CRC 或版本
   异常必须将系统保持在不可运动状态。
5. `CMD_GET_STATE (0x20)` 返回已经转换的模型反馈角度，且必须是 CAN 实测值，不能
   用默认 0 填充。
6. MCU 在转换后的模型角度域执行软限位判断；Pi 的限位只是额外检查，不能替代 MCU。

### A.4 Pi 的职责

- Pi 使用 POE/URDF/世界模型求 IK，发送完整 J1..J6 模型角度。
- Pi 备份零位/方向的 F407 配置版本和 CRC，用于核对；版本不一致时不发送轨迹。
- Pi 不直接生成 CAN 帧，不保存唯一的电机零位真值。

### A.5 验收

每轴无负载、低速、1~2 度：正模型角命令与预期机械方向一致，反馈增减方向正确；
重复回到手动零位至少三次，模型反馈误差满足现场定义的容差后才记录为有效零位。

---

## B. 文件拆分：最小拆分现在开始

### B.1 结论

现在做接口级拆分，避免继续向 `app_threads.c` 堆叠轨迹、CAN、Flash 和安全逻辑；
不在此阶段重写现有驱动或改变 UART 线协议。

### B.2 文件边界

```text
app_threads.c
  UART 收字节、协议解析、响应发送、向控制层投递消息。

trajectory.c / trajectory.h
  <6fH> 单点和批量轨迹校验、16 槽 FIFO、原子入队、出队、到位检测、段超时。

safety.c / safety.h
  急停、CAN 反馈超时、在线状态、零位有效性、软限位、执行许可、故障锁存。

calibration.c / calibration.h
  单轴设零、读回校验、Flash 保存/加载、配置版本/CRC。

can.c / DrEmpower_can.c
  原始 CAN 收发、反馈解析、时间戳；不包含轨迹策略。
```

### B.3 线程规则

- UART 线程绝不直接发 CAN。
- CAN IRQ 只收帧、更新时间戳/通知；不得阻塞、写 Flash 或计算轨迹。
- `joint_ctrl` 线程是唯一允许消费轨迹 FIFO 并下发 CAN 目标的位置。
- `safety` 线程可锁存故障、清 FIFO 和请求停止；真实急停仍应有硬件断电路径。
- 批量 `CMD_TRAJ_BATCH (0x53)` 必须先完成所有点校验和 FIFO 空间检查，再原子写入；
  不能部分入队后报错。

### B.4 验收

Keil map 中上述模块各自存在；UART 线程调用图不含 `Can_Send_Msg`/
`motion_aid`；CAN IRQ 调用图不含 `rt_thread_mdelay`、Flash 写入和轨迹函数。

---

## C. transport-only：独立安全 target

### C.1 结论

保留两个独立 Keil target，禁止通过修改同一个正式 target 的宏来临时解锁。

```text
F407_TRANSPORT_ONLY
  ARM_ACTUATOR_OUTPUTS_ENABLED=0
  仅初始化 UART、RT-Thread 基础对象和状态 LED。
  不初始化 CAN、PWM、轨迹、标定或 Flash 写入。
  PING / GET_STATE / GET_JOINT / STREAM 可用。
  TRAJ_POINT / TRAJ_BATCH / GRIPPER / SET_ZERO 等动作与写配置命令一律回 0x06。

F407_HARDWARE_ENABLED
  独立 target，显式初始化 CAN、反馈、急停、标定、轨迹 FIFO 和夹爪 PWM。
  即使已编译，只要零位、方向、限位、反馈或急停不合格，仍回拒绝响应且不发 CAN。
```

### C.2 目的

- `transport-only` 用于 Pi UART 接线、CRC、帧分片、单点/批量编码和锁定回包的
  无运动联调。
- 它也是每次协议升级后的回归版本：可以证明 Pi 不会因解析错误或未知命令使电机运动。
- 它不是硬件 target 的替代，也不代表 CAN/急停/夹爪已经验证。

### C.3 验收

在 transport-only target 下，合法 `CMD_TRAJ_POINT (0x50)`、
`CMD_TRAJ_BATCH (0x53)`、`CMD_GRIPPER (0x52)` 和 `CMD_SET_ZERO (0x30)` 均返回
`RSP_MOTION_LOCKED (0x06)`，并且示波/日志确认 CAN 与 PWM 未初始化。

---

## 实施顺序

1. 在 `transport-only` target 完成 v1.2 批量命令识别和锁定回归。
2. 建立 `F407_HARDWARE_ENABLED` target 及 B 的最小文件拆分。
3. 实现 A 的反馈转换、逐轴设零和 Flash 配置校验。
4. 实测方向、反馈、限位、急停和 CAN 超时。
5. 启用单点轨迹；完成验收后才启用批量 FIFO 和 ready 位执行。

在第 4 步之前，Pi 只能进行 UART/协议联调，不能授权真实运动。
