# STM32 Pick 协议对齐文档

## 物理层
USART1, 115200 8N1, 3.3V TTL

## 帧结构
```
┌──────┬──────┬──────┬──────┬────────────────────┬────────┬──────┐
│ 0xAA │ 0x14 │ LEN  │ SEQ  │  PAYLOAD (20 bytes) │ CRC16  │ 0x55 │
│ 帧头  │ CMD  │ 20   │ 0x01 │                    │ MODBUS │ 帧尾  │
└──────┴──────┴──────┴──────┴────────────────────┴────────┴──────┘
```

## 转义规则
载荷中以下字节需转义（仅 payload 部分）：
- 0xAA → 0xBB 0x0A
- 0x55 → 0xBB 0x05
- 0xBB → 0xBB 0x0B

## CRC16 MODBUS
计算范围: [0x14, 0x14, 0x01] + raw_payload (23 bytes)
多项式: 0xA001, 初始值: 0xFFFF
小端序附在帧尾前: [CRC_L, CRC_H, 0x55]

## Payload 格式 (20 bytes, little-endian)
```
struct: <hhhh hhhh h BB   (9 × int16 + 2 × uint8)

Offset  Size   Field        Type     说明
0       2      pick_X       int16    ×10, 小端
2       2      pick_Y       int16    ×10, 小端
4       2      pick_Z       int16    ×10, 小端
6       2      pick_yaw     int16    ×10, 小端
8       2      drop_X       int16    ×10, 小端
10      2      drop_Y       int16    ×10, 小端
12      2      drop_Z       int16    ×10, 小端
14      2      drop_yaw     int16    ×10, 小端
16      2      z_safe       int16    ×10, 小端
18      1      grip_id      uint8    固定 3
19      1      mode         uint8    1=水瓶 2=笔 3=可乐 4=耳机
```

## 当前演示坐标
| 物体   | pick_X | pick_Y | pick_Z | pick_yaw | drop_X | drop_Y | drop_Z | z_safe | mode |
|--------|--------|--------|--------|----------|--------|--------|--------|--------|------|
| 可乐   | 260    | -200   | 225    | 0        | 200    | 0      | 200    | 50     | 3    |
| 水瓶   | 230    | -180   | 225    | 0        | 200    | 0      | 200    | 50     | 1    |
| 笔     | 280    | -240   | 225    | 0        | 200    | 0      | 200    | 50     | 2    |
| 耳机   | 280    | -100   | 225    | 0        | 200    | 0      | 200    | 50     | 4    |

## 命令列表
| CMD  | 名称       | 说明         |
|------|-----------|-------------|
| 0x01 | ping      | 心跳         |
| 0x02 | estop     | 急停         |
| 0x03 | clear_err | 清除错误       |
| 0x11 | joints    | 关节控制       |
| 0x12 | cart      | 笛卡尔移动     |
| 0x13 | stop      | 停止         |
| 0x14 | pick      | 抓取指令     |
| 0x20 | get_state | 查询状态       |
| 0x30 | set_zero  | 设置零点       |
| 0x35 | calib_st  | 标定开始       |
| 0x36 | calib_end | 标定结束       |
| 0x37 | sys_org   | 设置系统原点   |
