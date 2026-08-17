# STM32 协议适配 v1.2

树莓派侧副本。正式 STM32 Keil 文件级改动清单见同名文件：
`BasicSetting_DaRanRobot/docs/STM32_通信协议适配改动_v1.2_20260808.md`。

本版本与 v4.0 的基础帧和单点六轴轨迹兼容，补充 `CMD_TRAJ_BATCH=0x53` 与
`RSP_TRAJ_ACK=0x81` / `RSP_TRAJ_OVERFLOW=0x82` 的受控适配。状态 payload 固定为
82 字节；生产环境禁止 CRC `0xFFFF` 绕过；批量发送和实际运动都必须通过现有
hardware calibration 安全门。
