# Pi-F407 协议

树莓派与 STM32F407 使用 UART `115200 8N1`、无硬件流控。帧结构为：

```text
[0xAA][CMD][LEN][SEQ][PAYLOAD...][CRC_LO][CRC_HI][0x55]
```

CRC 是覆盖 `CMD`、`LEN`、`SEQ` 和 payload 的 CRC-16/MODBUS，小端序发送。串口实现位于 [`uart_protocol.py`](../raspberry_pi/robot_ai/arm_control/uart_protocol.py)，协议回放不会打开真实接口。

## 约束

- `SEQ` 用于把 ACK/反馈与请求配对。
- 接收端必须先验证帧头、长度、CRC、帧尾和序号，再交给命令处理器。
- 丢帧、CRC 错误、过期反馈和未知节点不得被当作成功反馈。
- 公开仓库默认只读/运动锁定；真实运动必须由站点配置和操作者确认同时开放。

## 离线回放

```powershell
Push-Location raspberry_pi
python tools/protocol_offline_replay.py
python -m unittest discover -s tests -p 'test_uart_protocol.py' -q
Pop-Location
```

STM32 端解析与 ACK/NACK 定义位于
[`comm_protocol.h`](../stm32_keil/BasicSetting_DaRanRobot/Inc/comm_protocol.h) 和
[`comm_protocol.c`](../stm32_keil/BasicSetting_DaRanRobot/Src/comm_protocol.c)。
