# STM32F407 / Keil 工程

`BasicSetting_DaRanRobot/` 是小U六轴控制器的 STM32F407 工程。它包含 RT-Thread、UART 协议、轨迹 FIFO、关节状态、CAN 访问和 Keil MDK 项目文件。

## 打开与构建

1. 安装 Keil MDK-ARM v5 和 STM32F4 Device Family Pack；
2. 打开 `BasicSetting_DaRanRobot/MDK-ARM/BasicSetting_DaRanRobot.uvprojx`；
3. 根据实际硬件选择正确 target，检查时钟、USART、CAN 和节点配置；
4. 先编译，再在现场完成下载和只读 UART 验证。

仓库不提交 `.axf`、`.hex`、Objects、Listings、J-Link 日志或用户 Keil 窗口配置。这些文件与具体电脑、工具版本和现场硬件绑定，不能作为源码版本的依据。

## 当前源码重点

- `Src/comm_protocol.c`、`Inc/comm_protocol.h`：Pi–F407 帧解析、CRC、转义、应答和命令分发；
- `Src/trajectory.c`、`Inc/trajectory.h`：六轴轨迹 FIFO、时长、插补和队列流控；
- `Src/app_threads.c`：RT-Thread 调度、关节状态、轨迹消费和控制循环；
- `Src/safety.c`、`Src/debug_cmd.c`：状态和诊断路径；
- `Device/src/DrEmpower_can.c`：关节 CAN 层。

协议设计参考位于 `BasicSetting_DaRanRobot/docs/`，Pi 侧匹配实现位于 `raspberry_pi/robot_ai/arm_control/uart_protocol.py`。

## 上机前提醒

Keil 编译成功只说明源代码和工程文件能够生成固件。零位、方向、限位、急停、反馈和动作空间属于现场验证，必须单独确认，不能从此仓库的历史配置推断。
