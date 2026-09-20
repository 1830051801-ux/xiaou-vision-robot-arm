# XiaoU CAN V1（六轴控制器与 STM32 的自定义协议草案）

这是一份为当前 ROS2 `ros2_control` 驱动和 STM32 固件约定的**自定义协议**，
不是已从实机抓包确认的厂商协议，也不是旧四轴串口协议。STM32 必须先按此文档实现
仿真/回环测试，再进行单轴、低速、小角度实测。

## 1. 总线与节点

- Classic CAN，标准 11 位 ID；禁止扩展帧、远程帧和错误帧作为应用数据。
- 波特率固定 **500000 bit/s**，数据长度固定 DLC=8。
- 暂定节点映射：`J1=1, J2=2, J3=3, J4=4, J5=5, J6=6`。
- ID 映射必须在实机总线上逐个确认；确认前 `motion_enabled=false`。
- 数值均为小端、有符号定点数。

| 关节 | 节点 ID | 命令 ID | 反馈 ID | 诊断 ID |
|---|---:|---:|---:|---:|
| J1 | 1 | `0x101` | `0x181` | `0x1C1` |
| J2 | 2 | `0x102` | `0x182` | `0x1C2` |
| J3 | 3 | `0x103` | `0x183` | `0x1C3` |
| J4 | 4 | `0x104` | `0x184` | `0x1C4` |
| J5 | 5 | `0x105` | `0x185` | `0x1C5` |
| J6 | 6 | `0x106` | `0x186` | `0x1C6` |

## 2. 位置命令（Pi -> STM32）

ID=`0x100 + node_id`，DLC=8。

| 字节 | 字段 | 编码 |
|---|---|---|
| 0 | opcode | `0x01` |
| 1 | flags | bit0=enable，bit1=clear_fault，bit2=quick_stop，bit3 保留，bit4..7=4 bit sequence |
| 2..5 | target_position | signed int32，`round(physical_rad * 1e6)`，单位微弧度 |
| 6..7 | target_velocity | signed int16，`round(physical_rad_s * 1e3)`，单位毫弧度/秒 |

Pi 在每个控制周期为六个关节各发一帧。`quick_stop=1` 时速度必须为 0；STM32
必须立即停止输出、禁止继续跟踪目标，并在反馈 status 置 `estop` 或 `fault`。

## 3. 反馈（STM32 -> Pi）

ID=`0x180 + node_id`，DLC=8。

| 字节 | 字段 | 编码 |
|---|---|---|
| 0 | opcode | `0x81` |
| 1 | status | bit0=enabled，bit1=fault，bit2=estop，bit3=homed，bit4=heartbeat |
| 2..5 | measured_position | signed int32，编码单位微弧度 |
| 6..7 | measured_velocity | signed int16，编码单位毫弧度/秒 |

Pi 只接受标准帧、DLC=8、opcode=`0x81`、ID 与已配置节点匹配的反馈。出现
`fault` 或 `estop` 时立即关闭硬件接口；任一关节首次激活后 200 ms 内没有反馈，
或后续连续超过 200 ms 没有新反馈，Pi 侧看门狗返回硬件错误并停用控制器。

## 4. 诊断（STM32 -> Pi）

ID=`0x1C0 + node_id`，DLC=8，数据为
`[0xE0, error_code, detail_lo, detail_hi, 0, 0, 0, 0]`。
诊断帧用于记录原因；`feedback.status` 中的 fault/estop 才是运动禁止的权威状态。

## 5. ROS 角度与电机角度换算

对每个关节配置 `direction ∈ {-1,+1}` 和实测 `zero_offset_rad`：

```text
physical_command_rad = direction * (ros_target_rad - zero_offset_rad)
ros_feedback_rad      = direction * encoder_rad + zero_offset_rad
```

这里的方向、零位、限位和速度上限不能从模型猜测；必须按单轴、低速、小角度、
有人急停流程实测后写入 `hardware_calibration.json`。

当前已知的一条独立事实是：从每个电机齿轮侧观察，正的电机位置命令使齿轮逆时针
转动。这记录在 `motor_positive_rotation`，不能直接替代编码器反馈方向；编码器
方向仍必须用 STM32 反馈实测确认。

## 6. STM32 必须实现的安全行为

1. 拒绝未知 opcode、节点 ID 不在 1..63、非标准帧、DLC 非 8 的应用帧。
2. 每个节点独立维护命令看门狗：超过 200 ms 没有有效位置命令，关闭使能/扭矩并置 fault。
3. `quick_stop`、硬件急停或内部故障都必须停止输出，不能自动恢复运动。
4. 清故障必须是显式动作，且重新使能前需要收到新的有效命令。
5. 反馈中的位置/速度必须使用与本协议一致的单位和小端序。

## 7. 验证顺序

1. 先用虚拟 CAN 或回环测试验证六种命令帧、反馈帧、字节序、缩放和错误帧拒绝。
2. 断开执行器，仅接收总线，逐个确认 J1..J6 的实际 ID。
3. 单轴、低速、小角度、有人急停，记录方向、零位、限位、反馈频率和急停响应。
4. 六轴均通过后才填写全部标定字段并解除软件锁；任何字段为 `null` 时禁止真实运动。
