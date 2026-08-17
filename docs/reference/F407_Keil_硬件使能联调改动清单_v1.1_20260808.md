# STM32F407 / Keil 硬件使能联调改动清单 v1.1

本清单承接 `transport-only` 基线，针对用户已确认的六轴约束：CAN ID 为
J1..J6 = 1..6；从每个电机齿轮侧观察，正角度命令为逆时针。它是实现真实
控制前的文件级要求，不代表当前固件已经允许运动。

## 必须保留的 Pi 线协议

`AA CMD LEN SEQ PAYLOAD CRC16_LO CRC16_HI 55`，CRC-16/MODBUS，小端发送；
payload 之外不转义。`CMD_TRAJ_POINT` 必须为 `<6fH>` 26 字节，角度单位为度，
`CMD_GRIPPER` 必须为 `<Bf>` 5 字节。UART 为 USART1 PA9/PA10，115200 8N1。

`CMD_TRAJ_POINT` 的六个 float 是按 J1..J6 排列的**完整绝对目标**，不是只发送
一个关节，也不是增量角度。Pi 负责基于 `base_link` 的已审核六轴 POE/URDF 模型完成
感知、坐标转换、IK 和轨迹采样；F407 不重新解释相机/世界坐标，只负责将已经校验的
六个关节目标转换为电机命令并返回真实反馈。电机命令的正方向固定为“从该电机齿轮侧
观察的逆时针”；ROS 模型关节符号与反馈符号必须由逐轴反馈测量映射，不能从外观猜测。

F407 对 `CMD_GET_STATE` 的 ACK payload 固定为 82 字节：`state,error,estop,busy`
4 字节，随后每个 J1..J6 为 `<fffB>`（实际角度 deg、转速 rpm、力矩 Nm、online）。
硬件使能 target 必须填写真实值；不能继续返回 transport-only 的占位零值。

## 独立 Keil target

复制 `BasicSetting_DaRanRobot` 为独立的 `F407_HARDWARE_ENABLED` target，禁止
直接把正式 transport-only target 的宏改为 1。新 target 至少改动：

1. `Inc/arm_config.h`: 保留 `MAX_JOINT_COUNT=6`、CAN ID 1..6 和正方向注释；
   将实测的零位、编码器方向、软限位、速度/加速度写入独立硬件配置，不能使用
   当前临时角度宏。
2. `Src/main.c`: 在 UART 初始化后初始化 CAN、急停输入、六轴反馈和夹爪 PWM，
   启动顺序中加入硬件安全线程；启动失败必须保持输出关闭。
3. `Src/app_threads.c`: 对 `CMD_TRAJ_POINT` 解包六个 float 和 duration，执行
   逐轴限位/速度/急停/反馈检查后投递轨迹；对 `CMD_GRIPPER` 解包 `<Bf>` 后
   经过夹爪开度表再投递 PWM。任何检查失败回 `RSP_NACK`/`RSP_INVALID_PARAM`，
   不能回 ACK 假装执行。
4. `Src/can.c` 与 `Device/src/DrEmpower_can.c`: 使用已确认 ID 1..6，完成发送、
   接收过滤、反馈超时和 bus-off 恢复；CAN 速率必须由现场确认后固定。
5. `Src/calibration.c`: 一键零位只保存“当前静止反馈角”为
   `encoder_zero_offset`，写入前要求 E-stop 可用、六轴在线、速度接近零；写入
   后读回校验并持久化。禁止用默认 0 冒充零位。
6. `Src/servo.c`: 夹爪 PWM 开度映射独立配置，必须有输出禁用和急停路径；无夹爪
   反馈时只能按时间/开度策略并报告“无反馈”，不能伪造抓取成功。
7. `Src/stm32f4xx_it.c`: CAN RX、定时器 PWM 和急停 ISR 只置事件标志；耗时动作
   在线程中执行，不能在 ISR 中阻塞。

## 零位和 ready 位流程

树莓派工具 `robot_ai/arm_control/capture_pose.py` 只读 `GET_STATE`。摆好机械
参考姿势后执行 `--write --confirm CAPTURE-ZERO`，将六轴**模型反馈快照**写入
`zero_pose_reference_rad`；F407 才是原始编码器零偏和方向的唯一持久化权威。再摆好
ready 姿势执行 `CAPTURE-READY`，写入 `ready_pose_rad`。当前 transport-only 固件
因无真实反馈会拒绝捕获，不能绕过。

## 解锁前验收

必须完成 J1..J6 单轴、无负载、低速、小角度测试，确认正角度确实为齿轮侧逆时针，
记录编码器符号、零位、软/硬限位、反馈周期、急停断电和 CAN 错误。完成后才可
在独立 target 打开输出，并在 Pi 配置中逐项将安全门改为 true。未完成反馈和限位
前，真实运动入口必须保持锁定。
