# UART Protocol

The Raspberry Pi sends binary frames to STM32 over USART1 at 115200 8N1.

```text
[HEAD][CMD][LEN][SEQ][PAYLOAD][CRC_LO][CRC_HI][TAIL]
```

- `HEAD`: `0xAA`
- `TAIL`: `0x55`
- `CMD_GRASP_MOVE`: `0x14`
- `CRC`: CRC-16/MODBUS over `CMD`, `LEN`, `SEQ` and the unescaped payload
- Escape byte: `0xBB`

Escaped bytes inside the payload:

| Raw | Encoded |
| --- | --- |
| `0xAA` | `0xBB 0x0A` |
| `0x55` | `0xBB 0x05` |
| `0xBB` | `0xBB 0x0B` |

## Coordinate Grasp Payload

The active grasp command is `CMD_GRASP_MOVE = 0x14`.

Payload layout:

```text
int16 pick_x_mm_x10
int16 pick_y_mm_x10
int16 pick_z_mm_x10
int16 pick_yaw_deg_x10
int16 drop_x_mm_x10
int16 drop_y_mm_x10
int16 drop_z_mm_x10
int16 drop_yaw_deg_x10
int16 safe_z_mm_x10
uint8 grip_id
uint8 grip_profile
```

The STM32 parser treats `grip_profile` as a gripper close/open setting. It is not an object task selector.

