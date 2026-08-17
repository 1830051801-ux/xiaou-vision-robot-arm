# MCU 固件架构重构报告

> **项目**：BasicSetting_DaRanRobot — 4 轴机械臂控制系统  
> **日期**：2026-07-27  
> **版本**：v2.1 → v3.0 (Executor Mode)  
> **重构原则**：算法层（运动学/轨迹规划/抓取序列）迁移至树莓派 ROS2，MCU 退化为纯执行器 + 安全守护者

---

## 一、变更统计

| 类别 | 数量 | 说明 |
|------|:---:|------|
| **修改的头文件** | 10 | arm_config.h, app_threads.h, comm_protocol.h, calibration.h, calib_defaults.h, monitor.h, telemetry.h, sys_status.h, main.h, servo.h |
| **修改的源文件** | 7 | main.c, app_threads.c, calibration.c, monitor.c, telemetry.c, sys_status.c |
| **删除的文件** | 6 | arm_kinematics.c/h, trajectory_planner.c/h, grasp_sequence.c/h |
| **未改动的文件** | ~30+ | 全部硬件驱动/RTOS/CAN电机库/辅助项目 |
| **代码量变化** | -1,800 行 | main.c: 1220→170, app_threads.c: 1062→580 |

---

## 二、修改详情

### 2.1 `Inc/arm_config.h` — 配置精简

**删除的配置项**（迁移至树莓派 ROS2 URDF/YAML）：

| 原节号 | 内容 | 迁移目标 |
|:---:|------|------|
| §2 | DH 运动学参数 (D1~D4, A1~A4, ALPHA1~4, TOOL_X/Y/Z) | `arm.urdf` / `arm.xacro` |
| §4 | 重力补偿参数 (LINK2~4_MASS, LINK2~4_COM, GRAVITY, PAYLOAD_MASS) | `arm.urdf` inertial tags |
| §6 | 轨迹规划默认参数 (TRAJ_DEFAULT_SPEED/ACCEL/JERK/CART_SPEED/CART_ACCEL) | `arm_controllers.yaml` |
| §7 | 工作空间软限位 (WORKSPACE_X/Y/Z_MIN/MAX) | `arm.urdf` limit tags |
| §8 | 归位点 & 原点 (HOME_J1~J4, ORIGIN_J1~J4) | `arm_controllers.yaml` |

**保留的配置项**（MCU 安全执行必需）：

| 节号 | 内容 | 保留原因 |
|:---:|------|------|
| §1 | CAN ID (CAN_ID_JOINT_1~4) | CAN 总线寻址 |
| §2 | 关节角度/转速/力矩物理限位 (JOINT1~4_*) | safety_thread 安全保护 |
| §3 | 安全阈值 (过流/过热/CAN超时/通信超时/预测限位比例) | 独立安全层 |
| §4 | 插补周期 (TRAJ_INTERPOLATION_MS=10) | 轨迹点执行节拍 |
| §5 | 杂项 (关节在线重试/反馈速率) | 系统运行参数 |

### 2.2 `Inc/comm_protocol.h` — 协议命令重构

**删除的命令**（算法迁移后不再需要）：

| 命令码 | 名称 | 替代方案 |
|:---:|------|------|
| 0x10 | CMD_MOVE_JOINT | Pi 做完 IK 后发 CMD_TRAJ_POINT |
| 0x11 | CMD_MOVE_JOINTS | Pi 做完轨迹规划后发 CMD_TRAJ_POINT |
| 0x12 | CMD_MOVE_CART | Pi 做完 IK+轨迹规划后发 CMD_TRAJ_POINT |
| 0x14 | CMD_GRASP_MOVE | Pi 用 BehaviorTree 编排后逐点下发 |
| 0x37 | CMD_SET_SYS_ORIGIN | Pi TF2 static_transform |
| 0x38 | CMD_GET_SYS_ORIGIN | Pi TF2 |
| 0x40-0x43 | CMD_TEACH_* | Pi 侧实现 |

**新增的命令**（执行器模式）：

| 命令码 | 名称 | Payload | 说明 |
|:---:|------|------|------|
| 0x50 | CMD_TRAJ_POINT | 4×float32 (J1~J4 angle (°)) + uint16 (duration_ms) = **18B** | Pi 逐点下发轨迹 |
| 0x51 | CMD_TRAJ_BUFFER_CLEAR | 空 | 紧急清空 MCU 轨迹缓冲区 |
| 0x52 | CMD_GRIPPER | uint8 (servo_id) + float32 (angle (°)) = **5B** | 夹爪角度控制 |

### 2.3 `Inc/app_threads.h` — 架构简化

**sys_status 结构体变更**：

```c
// 删除的字段:
- volatile float cart_pose[6];        // FK 末端位姿 → Pi 自行计算
- volatile float base_offset[3];      // 系统原点偏移 → Pi TF2 管理

// 新增的字段:
+ volatile uint8_t estop_triggered;   // 急停标志 (从 sys_state 独立)
```

**ctrl_msg_type 枚举变更**：

```c
// 删除:
- CTRL_MSG_MOVE_JOINTS     // 多关节 PTP → Pi 拆为 TRAJ_POINT
- CTRL_MSG_MOVE_CARTESIAN  // 笛卡尔直线 → Pi 拆为 TRAJ_POINT
- CTRL_MSG_TEACH_MODE      // 示教 → Pi 侧
- CTRL_MSG_SET_SYS_ORIGIN  // 原点偏移 → Pi TF2

// 新增:
+ CTRL_MSG_TRAJ_POINT   = 0x50  // 轨迹点执行
+ CTRL_MSG_TRAJ_CLEAR   = 0x51  // 清空轨迹缓冲区
+ CTRL_MSG_GRIPPER      = 0x52  // 夹爪控制
+ CTRL_MSG_CLEAR_ERROR  = 0x05  // 清除错误 (独立出来)
```

### 2.4 `Inc/calibration.h` / `calib_defaults.h` — 标定精简

移除所有 `base_offset` 相关：
- `calib_set_base_offset()`
- `calib_apply_base_offset()`
- `calib_unapply_base_offset()`
- `CALIB_BASE_OFFSET_X/Y/Z_DEFAULT`

保留全部零点校验和助力标定功能（这些需要在 MCU 本地执行）。

### 2.5 `Inc/monitor.h` — 安全监控简化

移除 `MON_HOMING`、`MON_HOME_DONE` 状态（MCU 不再自主生成回零轨迹），移除 `home_joints` 字段。

### 2.6 `Src/main.c` — 主入口大幅精简

| 指标 | 重构前 | 重构后 |
|------|:---:|:---:|
| 行数 | 1220 | 170 |
| 测试线程 | 4 (can_test, servo_test, proto_test, sysmon) | 0 |
| 调试命令 | 25+ (debug_cmd_exec) | 0 |
| 硬编码运动序列 | 10+ (可乐/笔/耳机/挥手/三连笔等) | 0 |
| 手动 CAN 初始化 | 有 (can_self_init, 绕过 HAL) | 无 (使用 HAL MX_CAN1_Init) |

重构后 `main()` 流程：HAL 初始化 → RT-Thread 启动 → 外设初始化 → `app_threads_init()` → 调度器启动。

### 2.7 `Src/app_threads.c` — 核心架构重构

**joint_ctrl_thread 变更**（最关键的改动）：

| 功能 | 重构前 | 重构后 |
|------|------|------|
| 命令分发 | 16 种 ctrl_msg_type | 12 种 (移除笛卡尔/示教,新增轨迹点/夹爪) |
| 轨迹执行 | S曲线规划 + IK + 重力补偿前馈 | 轨迹点环形缓冲区 + set_angles 直发 |
| FK 更新 | 每周期 arm_fk() + calib_unapply_base_offset() | 删除 |
| 重力补偿 | arm_gravity_comp() 前馈力矩 | 删除 (Pi 侧计算后直接下发力矩值到 mode=2) |

**新增轨迹点环形缓冲区**：

```c
#define TRAJ_BUF_SIZE 16
float    traj_buf_angles[16][4];   // 预存 16 个轨迹点
uint16_t traj_buf_duration[16];   // 每个轨迹点的时长
// 生产者: comm_thread → CMD_TRAJ_POINT 入队
// 消费者: joint_ctrl_thread → 逐个出队执行
```

**comm_thread 变更**：
- 移除 CMD_MOVE_CART/CMD_MOVE_JOINTS/CMD_GRASP_MOVE 解析
- 新增 CMD_TRAJ_POINT/CMD_TRAJ_BUFFER_CLEAR/CMD_GRIPPER 处理
- UART 接收从 TODO 注释变为实际调用 `Rpi_Uart_Available()`/`Rpi_Uart_GetChar()`

### 2.8 其他文件

| 文件 | 变更内容 |
|------|------|
| `calibration.c` | 删除 `calib_set_base_offset` / `calib_apply_base_offset` / `calib_unapply_base_offset` 实现 |
| `monitor.c` | 删除 `trajectory_planner.h` / `arm_kinematics.h` 依赖，移除自动回零逻辑。超时后只做急停 |
| `telemetry.c` | 删除 `Telem_Send()` (含末端位姿)，保留 `Telem_SendAngles()` (纯关节角度) |
| `sys_status.c` | 删除 `SYS_MOD_KINEMATICS`/`SYS_MOD_TRAJECTORY` 模块标志，更新 `sys_dump_status()` |
| `main.h` | 移除 `grasp_sequence.h` include |

---

## 三、重构后 MCU 架构

### 3.1 线程架构（保持不变）

```
优先级    线程        栈      周期    职责
  2     watchdog     512B   500ms   IWDG 喂狗 + 6 线程心跳
  5     joint_data   2048B    5ms   CAN 关节状态流
  6     safety       1024B   10ms   预测限位 + 过流/超速/通信丢失 → ESTOP
  8     joint_ctrl   2048B   10ms   轨迹点缓冲区执行 + 零点校验 + 标定
 15     comm         1536B   20ms   协议解析 + 命令分发 + 遥测
 22     led          512B    50ms   LED 灯语
```

### 3.2 数据流

```
              UART 115200
 Pi ──────────────────────────→ MCU
     CMD_TRAJ_POINT (18B/帧)       环形缓冲区 [18B × 16] → 逐点 set_angles()
     CMD_GRIPPER (5B/帧)           Servo_SetAngle()
     CMD_ESTOP / PING / CALIB ...

 Pi ←────────────────────────── MCU
     RSP_ACK / NACK                应答
     TELEMETRY (关节状态 @ 可配 Hz)  角度 + 转速 + 力矩 + 在线 + 错误码

              CAN 1Mbps
 MCU ──────────────────────────→ 关节 1~4
     set_angles() / set_speed()    电机控制命令
     estop()                       急停

 MCU ←────────────────────────── 关节 1~4
     angle_speed_torque_state()    实时状态流 @ 8ms/关节
```

### 3.3 安全机制（多层独立保护）

| 层级 | 触发条件 | 动作 | 延迟 |
|:---:|------|------|:---:|
| 1. Pi 侧 | MoveIt2 碰撞检测/工作空间溢出 | 停止下发轨迹点 | ~50ms |
| 2. MCU safety_thread | 预测限位/过流/超速/通信丢失 | ESTOP + 锁定 | <10ms |
| 3. MCU monitor | Pi 通信超时 (>200ms) | ESTOP 全部关节 | ~200ms |
| 4. MCU watchdog | 关键线程挂死 (>2s) | 硬件复位 (IWDG) | ~4s |

---

## 四、未改动的代码（禁区）

| 层级 | 文件 | 原因 |
|------|------|------|
| CAN 电机驱动 | `Device/src/DrEmpower_can.c`, `Device/inc/DrEmpower_can.h` | 大然关节唯一控制接口，调通后不动 |
| HAL 外设 | `Src/can.c`, `Src/usart.c`, `Src/gpio.c`, `Src/adc.c`, `Src/dma.c`, `Src/spi.c`, `Src/iwdg.c` | 硬件层，CubeMX 生成 |
| RT-Thread 内核 | `RTT_RTOS/` 全部文件 | OS 核心 |
| 舵机 PWM | `Src/servo.c`, `Inc/servo.h` | 夹爪控制 |
| LED 驱动 | `Src/led.c`, `Inc/led.h` | 状态指示 |
| 看门狗 | `Src/watchdog.c`, `Inc/watchdog.h` | IWDG + 心跳监控 |
| 协议解析 | `Src/comm_protocol.c` | CRC/转义/帧解析逻辑未变 |
| CMSIS/HAL | `Drivers/` 全部文件 | STM32F4 标准库 |
| 辅助项目 | `123/` 全部文件 | STM32F103 独立测试 |
| Python 工具 | `Tools/` 全部文件 | PC 端仿真可视化 |

---

## 五、树莓派 ROS2 侧待建内容

MCU 重构完成后的对应端：

| 模块 | ROS2 包 | 推荐方案 |
|------|------|------|
| 运动学 (FK/IK) | `arm_kinematics` | Trac-IK 或 KDL 插件 |
| 轨迹规划 | `arm_controller` | MoveIt2 JointTrajectoryController |
| 重力补偿 | `arm_controller` | KDL 动力学 + `ros2_control` 前馈 |
| 抓取序列 | `arm_behaviors` | BehaviorTree.CPP / Smach |
| 坐标管理 (base_offset) | TF2 | `static_transform_publisher` |
| 硬件接口 (UART) | `arm_hardware` | `ros2_control` HardwareInterface 插件 |
| 机器人模型 | `arm_description` | URDF/XACRO (含 DH 参数/限位/质量) |

**新协议帧 Pi 侧实现参考**（CMD_TRAJ_POINT, 18 字节）：

```python
import struct

def encode_traj_point(j1: float, j2: float, j3: float, j4: float,
                      duration_ms: int) -> bytes:
    """编码轨迹点: 4×float32 + uint16 = 18B"""
    payload = struct.pack('<ffffH', j1, j2, j3, j4, duration_ms)
    return payload  # 加上帧头/CMD/LEN/SEQ/CRC/帧尾后通过 UART 发送
```

---

## 六、重构前后对比

| 指标 | 重构前 (v2.1) | 重构后 (v3.0) |
|------|:---:|:---:|
| MCU 角色 | 全能控制器 | 执行器 + 安全守护者 |
| 运动学 (FK/IK) | MCU C 实现 | 树莓派 Trac-IK |
| 轨迹规划 | MCU S曲线 | 树莓派 MoveIt2 |
| 重力补偿 | MCU 虚功原理 | 树莓派 KDL 动力学 |
| 抓取序列 | MCU 硬编码状态机 | 树莓派 BehaviorTree |
| 坐标系管理 | MCU base_offset | 树莓派 TF2 |
| main.c 行数 | 1220 | 170 |
| app_threads.c 行数 | 1062 | 580 |
| 协议命令数 | 25 | 18 |
| 线程数 | 6 | 6 (不变) |
| 硬实时安全 | ✅ MCU 本地 | ✅ MCU 本地 (不变) |
| 开发语言 | 100% C | MCU: C, Pi: C++/Python |
| 算法调试效率 | 编译→烧录→串口日志 | hot-reload Python/ROS2 launch |

---

## 七、验证建议

1. **编译测试**：在 Keil MDK 中重新编译，确认无缺失符号引用
2. **CAN 通信测试**：上电后确认 `[JointData] streaming enabled for 4 joints` 正常打印
3. **协议帧测试**：从 PC 通过 USART1 发送 `CMD_PING (0x01)`，验证 `[Comm] RX frame: cmd=0x01` 响应
4. **轨迹点测试**：发送 `CMD_TRAJ_POINT`，验证关节实际运动
5. **安全测试**：断开 Pi→MCU 串口线，验证 200ms 内触发急停
6. **零点校验测试**：断电重启，验证上电零点自检流程
