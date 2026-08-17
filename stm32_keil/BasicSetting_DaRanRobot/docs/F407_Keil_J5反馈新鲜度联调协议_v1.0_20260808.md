# F407 Keil J5反馈新鲜度联调协议 v1.0

## 目的与边界

本文件是树莓派项目中的联调副本，记录 Windows 上 F407 源码分支
`D:\机械臂\stm32_firmware_f407_vla_uart_20260807\BasicSetting_DaRanRobot` 的 J5
反馈新鲜度修正。树莓派不编译 Keil，也不保存或下发 F407 二进制；Pi 仅运行只读
UART 验收脚本。

本次允许验证 UART、CAN 反馈和 `online` 状态，禁止轨迹、使能、电机运动、清错、
零位或参数写入。固件中的以下安全宏必须保持为 `0`：

```c
#define ARM_ACTUATOR_COMMANDS_ENABLED  0
#define ARM_HARDWARE_ESTOP_VERIFIED     0
```

## Keil 工程和本次改动

在 Windows Keil 中打开：

```text
D:\机械臂\stm32_firmware_f407_vla_uart_20260807\BasicSetting_DaRanRobot\MDK-ARM\BasicSetting_DaRanRobot.uvprojx
```

目标名为 `BasicSetting_DaRanRobot`，芯片为 `STM32F407VE`。确认打开路径完全一致后，
才允许 Build；不要误用 `123\MDK-ARM\123.uvprojx` 或旧的 baseline/uart_bridge 目录。

| 文件 | 已纳入的改动 | 作用 |
|---|---|---|
| `Inc/arm_config.h` | `ARM_CAN_TIMEOUT_MS: 50 -> 200`；`JOINT_FEEDBACK_RATE_MS: 8 -> 12` | 给六轴反馈轮询留出诊断阶段的时间窗口；不是 Pi UART 超时设置。 |
| `Device/inc/DrEmpower_can.h` | 声明 `angle_speed_torque_state_fresh()` 和两帧上限 | 明确新鲜匹配反馈的返回值。 |
| `Device/src/DrEmpower_can.c` | 新增按 CAN ID 匹配的有界读取；原 `angle_speed_torque_state()` 调用它 | 不把其他轴或旧帧当作当前关节反馈。每次最多等两帧，单次 CAN 等待为 10 ms。 |
| `Src/app_threads.c` | 仅在 `fresh_feedback` 成功时更新 `g_sys.joints[]` 和 `g_sys.joint_online[]`；移除实时流中的 `get_volcur()` | J5 没有新鲜匹配 CAN 帧时必须保持离线，不能用缓存角度冒充在线。 |

`online=true` 的含义是 F407 刚收到且 ID 匹配该关节的 CAN 实时状态帧；它不是 Pi
查询成功，也不是“数据数值非零”。因此延长 Pi 的 `--timeout-s` 不能让 J5 的旧数据变新。

## Keil 编译与烧录

1. 断开机械臂执行器电源或确保执行器物理上不可运动；USB 调试器只连接到 F407。
2. 在 Keil 的 Target 下确认 `BasicSetting_DaRanRobot`，检查上述两个安全宏仍为 `0`。
3. 点击 Rebuild。仅当编译和链接均为 0 error 后，使用当前工程生成的镜像下载到 F407。
4. 不修改 CAN ID、零位、限位、PID、速度，也不将安全宏改为 `1`。
5. 如需回退，恢复这四个文件到 `stm32_firmware_f407_uart_bridge_20260807` 对应版本：
   `ARM_CAN_TIMEOUT_MS=50`、`JOINT_FEEDBACK_RATE_MS=8`，并删除新鲜读取调用；回退后仍不得运动。

若 Keil 报错、目标名不符、源文件路径不符，或 Build 没有明确的 0 error，停止，不下载。

## 烧录后的 Pi 只读验收

在树莓派执行以下命令。它只发送 `PING` 和 `GET_JOINT`，不会使能、运动或写入配置：

```bash
cd /home/pi/raspi_robot_ai_transport_only_20260808
python3 robot_ai/arm_control/joint_status_probe.py --port /dev/serial0 --baud 115200 --ping-count 3 --rounds 2 --timeout-s 5
```

验收判断：

- `ping_fail` 为 `0`，`joint_query_fail` 为 `0`。
- J1-J5 只有收到本轴新鲜匹配 CAN 帧时才显示 `online: true`；无帧时显示 `false`，数值可保留为最近快照但不得据此判定在线。
- J6 当前暂不在线是允许的现场状态；它不应阻塞 J1-J5 查询，也不应令 J5 显示为在线。
- 若首帧偶发超时，先检查 Pi/F407 共地、TX/RX 交叉和 3.3 V TTL；不要以更大 Pi 超时掩盖 CAN 反馈问题。

## 当前现场基线

2026-08-08 的只读结果显示 UART 可恢复应答；J1/J2 曾为 `online=true`，J3-J5 出现过
“有角度/速度/力矩快照但 `online=false`”，J6 暂离线。该现象正是本次修正需要区分的
状态，尚未代表 F407 新固件已经编译、下载或实机验证。
