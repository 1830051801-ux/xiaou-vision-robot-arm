# F407 UART 透传集成说明

## 当前正式边界

本树莓派工程的正式控制入口已经改为：

~~~text
语音或 VLA 意图
    -> YOLO 固定俯视相机中心点
    -> 固定 1920x1080 标定与类别策略检查
    -> Pi UART 帧编码
    -> STM32F407 USART1
    -> F407 内部 CAN 与夹爪 PWM
~~~

Pi 不再把 CAN 当作执行入口。F407 才拥有 CAN 总线；Pi 只通过 /dev/serial0 发送完整二进制协议帧并校验回复。VLA 只能选择 action 和 object class，不能生成关节角、PWM 或 CAN 帧。

## 本次 Pi 端文件

| 文件 | 作用 |
|---|---|
| robot_ai/arm_control/uart_protocol.py | 与 F407 一致的 CRC、转义、流式解析、PING 和安全 UART 传输 |
| robot_ai/arm_control/task_planner.py | pick、tidy_all、stop、home 的确定性预检；只接收 YOLO 中心点 |
| robot_ai/arm_control/safety.py | 动作门改为 UART/F407 验证，不再要求 Pi 直接控制 CAN |
| robot_ai/arm_control/config/hardware_calibration.json | UART 115200 8N1、F407 内部 CAN 所有权与硬件急停未知状态 |
| robot_ai/arm_control/config/object_grasp_profiles.json | 每类高度、PWM 和放置 pose 的空白实测表；未测量即拒绝 |
| robot_ai/vision_targeting.py | 不再回退到经验像素比例；无 1920x1080 合格 homography 时返回标定错误 |
| codex_pickup_package/create_workspace_homography.py | 只接受 1920x1080 标定，输出 robot_base_table、米单位的格式 |

## 安全 PING

F407 刷入本工作副本且 ARM_ACTUATOR_COMMANDS_ENABLED 保持 0 后，树莓派只运行：

~~~bash
cd /home/pi/raspi_robot_ai
python3 robot_ai/arm_control/uart_protocol.py --ping --port /dev/serial0 --baud 115200 --sequence 1
~~~

预期发送和返回：

~~~text
TX  AA 01 00 01 E1 C0 55
RX  AA 00 00 01 B0 00 55
~~~

这一步不会产生 CAN、PWM、夹爪或关节动作。详细接线、所有字段、逃逸规则、响应码和超时规则见 F407 工作副本中的 docs/Pi_F407_UART_透传联调协议_v1.0.md。

## 相机与标定规则

相机是固定高度、固定朝向的外部俯视相机，输入必须为 1920 x 1080。新 workspace_homography.yaml 必须同时含有：

~~~yaml
image_width_px: 1920
image_height_px: 1080
output_frame: robot_base_table
output_unit: m
homography:
  - [ ... ]
  - [ ... ]
  - [ ... ]
~~~

旧文件没有这些元数据，因此会被新任务规划器拒绝，即使旧文件本身的重投影误差看起来较小。不能只补写分辨率字段后继续使用；必须在固定 1080p 画面下重新采集至少 4 点，建议 9 点，并重新实测误差。

## 类别策略与整理全部

每个被执行的类别都必须先填入：

- grasp_height_m
- approach_height_m
- gripper_open_pwm_deg
- gripper_close_pwm_deg
- placement_pose_id

未知类别或任一字段为空时，pick 和 tidy_all 都返回锁定结果。tidy_all 不会把所有物体压成一个泛化动作；它按当前画面中 YOLO 中心点的确定顺序逐个生成类别策略预览。实际循环必须在每件物品后做视觉复检，失败时回安全位并上报，不能声称夹爪具有力反馈。

## 预检命令

普通离线预检允许显示未完成项目：

~~~bash
cd /home/pi/raspi_robot_ai
python3 robot_ai/preflight_check.py
~~~

要求真实动作时才使用：

~~~bash
python3 robot_ai/preflight_check.py --require-motion --camera
~~~

在当前阶段第二条命令失败是正确结果，因为标定、类别参数、物理急停和六轴反馈均未实测。

## 未完成前禁止改动

- 不要把 hardware_calibration.json 中任何布尔门改为 true。
- 不要把 F407 的 ARM_ACTUATOR_COMMANDS_ENABLED 改为 1。
- 不要因为旧 Pi 文档出现 500 kbit/s 就改 F407 内部 CAN；当前 F407 源码为 1 Mbps，物理值仍待确认。
- 不要使用旧 vision_targeting 的经验比例作为动作坐标。
