# STM32 Keil Setup

Open the Keil project:

```text
stm32_keil/BasicSetting_DaRanRobot/MDK-ARM/BasicSetting_DaRanRobot.uvprojx
```

Main files to inspect:

- `Src/main.c`: UART command loop and coordinate grasp execution
- `Inc/comm_protocol.h`: command IDs and protocol constants
- `Src/comm_protocol.c`: frame parser and CRC check
- `Src/usart.c`: USART configuration
- `Src/can.c`: CAN initialization
- `Device/src/DrEmpower_can.c`: motor command transport
- `Src/servo.c`: gripper PWM output

Hardware assumptions:

- USART1 receives Pi frames on PA9/PA10 at 115200 8N1
- CAN1 drives the arm motor bus on PA11/PA12
- Gripper close/open values are selected by `grip_profile`

Flash procedure:

1. Build the Keil target.
2. Flash the STM32 board.
3. Open the debug UART and send `0F 00` to print available debug commands.
4. Test gripper and single-joint commands before sending a coordinate grasp frame.

