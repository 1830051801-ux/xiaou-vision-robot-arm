/**
 ******************************************************************************
 * @file    comm_protocol.h
 * @brief   上位机通信协议 — 二进制帧解析 / 命令定义 / 遥测打包
 * @date    2026-07-27 (架构重构: 算法层迁移至 ROS2, 协议简化)
 ******************************************************************************
 * @attention
 * 物理层: USART1 (PA9/PA10), 115200 8N1
 *
 * 帧格式 (字节填充):
 *   [0xAA] [CMD] [LEN] [SEQ] [PAYLOAD 0~248B] [CRC16_LO] [CRC16_HI] [0x55]
 *
 *   0xAA / 0x55 = 帧定界符
 *   CMD         = 命令/响应码 (1B)
 *   LEN         = Payload 长度 (1B, 0~248)
 *   SEQ         = 序列号 (1B, host递增, arm回显)
 *   PAYLOAD     = 数据负载 (0~248B, 含字节填充)
 *   CRC16       = CRC-16/MODBUS over [CMD+LEN+SEQ+PAYLOAD] (小端序)
 *
 * 字节填充: 负载中出现 0xAA→0xBB 0x0A, 0x55→0xBB 0x05, 0xBB→0xBB 0x0B
 *
 * 架构变更: MCU 不再理解笛卡尔坐标/抓取序列/示教 —
 *          仅执行轨迹点、夹爪、标定、系统控制四类基本指令。
 ******************************************************************************
 */

#ifndef __COMM_PROTOCOL_H__
#define __COMM_PROTOCOL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* -------------------------------------------------------------------------- */
/* 帧常量                                                                    */
/* -------------------------------------------------------------------------- */
#define PROTO_FRAME_START    0xAA
#define PROTO_FRAME_END      0x55
#define PROTO_ESCAPE_BYTE    0xBB
#define PROTO_MAX_PAYLOAD    248
#define PROTO_RX_BUF_SIZE    512     /* 接收环形缓冲区 */
#define PROTO_TX_BUF_SIZE    512     /* 发送缓冲区 */

/* -------------------------------------------------------------------------- */
/* 命令码 (Host → Arm)                                                        */
/* -------------------------------------------------------------------------- */

/* ---- 系统控制 (保留) ---- */
#define CMD_PING             0x01    /* 心跳测试 */
#define CMD_ESTOP            0x02    /* 急停 */
#define CMD_CLEAR_ERROR      0x03    /* 清除错误 */
#define CMD_STOP             0x13    /* 停止运动 */

/* ---- 轨迹执行 (新增) ---- */
#define CMD_TRAJ_POINT       0x50    /* 轨迹点下发: 6×float(J1~J6,°) + uint16(duration_ms) = 26B */
#define CMD_TRAJ_BUFFER_CLEAR 0x51   /* 清空轨迹缓冲区 */

/* ---- 夹爪控制 (新增) ---- */
#define CMD_GRIPPER          0x52    /* 夹爪控制: uint8(id) + float(angle°) = 5B */

/* ---- 批量轨迹 (新增) ---- */
#define CMD_TRAJ_BATCH       0x53    /* 批量轨迹点: N×(6×float + uint16) = N×26B, N=1~9 */

/* ---- 状态查询 (保留, 精简) ---- */
#define CMD_GET_STATE        0x20    /* 查询完整状态 (不再含末端位姿) */
#define CMD_GET_JOINT        0x21    /* 查询单关节状态 */
#define CMD_STREAM_START     0x22    /* 开始遥测流 (关节状态高刷) */
#define CMD_STREAM_STOP      0x23    /* 停止遥测流 */

/* ---- 标定 (保留) ---- */
#define CMD_SET_ZERO         0x30    /* 设置零点 */
#define CMD_CALIB_START      0x35    /* 开始单关节标定 (payload: joint_id) */
#define CMD_CALIB_END        0x36    /* 结束标定模式 */
#define CMD_GET_CALIB_STATUS 0x39    /* 查询标定状态 */

/* ---- 参数设置 (保留) ---- */
#define CMD_SET_PID          0x31    /* 设置 PID */
#define CMD_SET_LIMIT        0x32    /* 设置限位 */
#define CMD_REBOOT_JOINT     0x33    /* 重启关节 */
#define CMD_SAVE_CONFIG      0x34    /* 保存配置 */

/* ---- 已删除的命令 (算法层迁移至 ROS2) ----
 * CMD_MOVE_JOINT     0x10  → Pi 做完 IK 后逐点下发 CMD_TRAJ_POINT
 * CMD_MOVE_JOINTS    0x11  → Pi 做完轨迹规划后逐点下发 CMD_TRAJ_POINT
 * CMD_MOVE_CART      0x12  → Pi 做完 IK+轨迹规划后逐点下发 CMD_TRAJ_POINT
 * CMD_GRASP_MOVE     0x14  → Pi 用 BehaviorTree 编排后逐点下发
 * CMD_SET_SYS_ORIGIN 0x37  → Pi TF2 static_transform 管理
 * CMD_GET_SYS_ORIGIN 0x38  → Pi TF2 管理
 * CMD_TEACH_START    0x40  → Pi 侧实现
 * CMD_TEACH_STOP     0x41
 * CMD_TEACH_RECORD   0x42
 * CMD_PLAY_SEQUENCE  0x43
 */

/* -------------------------------------------------------------------------- */
/* 响应码 (Arm → Host)                                                       */
/* -------------------------------------------------------------------------- */
#define RSP_ACK              0x00    /* 成功响应 */
#define RSP_NACK             0x01    /* 失败 */
#define RSP_BUSY             0x02    /* 忙碌 (正在执行运动) */
#define RSP_ESTOP_ACTIVE     0x03    /* 急停激活中 */
#define RSP_INVALID_PARAM    0x04    /* 参数非法 */
#define RSP_TELEMETRY        0x80    /* 遥测数据帧 (主动推送) */
#define RSP_TRAJ_ACK          0x81    /* 轨迹应答: [buf_remaining(1B)] 缓冲区剩余槽位 */
#define RSP_TRAJ_OVERFLOW     0x82    /* 轨迹溢出: 缓冲区满, 拒绝接收 */
#define RSP_MOTION_LOCKED      0x06    /* 运动锁定: transport-only 模式, CAN/PWM 未初始化 */

/* -------------------------------------------------------------------------- */
/* 协议解析状态机                                                             */
/* -------------------------------------------------------------------------- */
enum proto_rx_state {
    PROTO_RX_WAIT_START = 0,   /* 等待 0xAA */
    PROTO_RX_CMD,              /* 读取 CMD */
    PROTO_RX_LEN,              /* 读取 LEN */
    PROTO_RX_SEQ,              /* 读取 SEQ */
    PROTO_RX_PAYLOAD,          /* 读取 Payload (含解转义) */
    PROTO_RX_CRC1,             /* 读取 CRC 低字节 */
    PROTO_RX_CRC2,             /* 读取 CRC 高字节 */
    PROTO_RX_WAIT_END,         /* 等待 0x55 */
};

/* -------------------------------------------------------------------------- */
/* 解析后的帧数据                                                             */
/* -------------------------------------------------------------------------- */
struct proto_frame {
    uint8_t  cmd;                           /* 命令码 */
    uint8_t  len;                           /* 负载长度 */
    uint8_t  seq;                           /* 序列号 */
    uint8_t  payload[PROTO_MAX_PAYLOAD];    /* 解码后的负载 */
    uint16_t crc;                           /* 接收到的 CRC */
    uint8_t  valid;                         /* 帧有效标志 */
};

/* -------------------------------------------------------------------------- */
/* 协议解析器上下文                                                           */
/* -------------------------------------------------------------------------- */
struct proto_parser {
    enum proto_rx_state state;          /* 当前解析状态 */
    uint8_t  rx_ring[PROTO_RX_BUF_SIZE]; /* 接收环形缓冲区 */
    uint16_t rx_head;                   /* 写指针 (ISR 写入) */
    uint16_t rx_tail;                   /* 读指针 (线程读取) */
    struct proto_frame rx_frame;        /* 当前解码中的帧 */
    uint8_t  payload_index;             /* payload 写入索引 */
    uint8_t  escape_next;               /* 下一字节为转义 */
    uint16_t crc_calc;                  /* CRC 计算值 */
};

/* -------------------------------------------------------------------------- */
/* API                                                                        */
/* -------------------------------------------------------------------------- */

void proto_init(struct proto_parser *pp);

void proto_rx_byte(struct proto_parser *pp, uint8_t ch);

int proto_try_parse(struct proto_parser *pp, struct proto_frame *frame);

void proto_build_response(uint8_t rsp_cmd, uint8_t seq,
                          const uint8_t *payload, uint8_t len,
                          uint8_t *tx_buf, uint16_t *tx_len);

void proto_build_telemetry(uint8_t seq, const uint8_t *payload, uint8_t len,
                           uint8_t *tx_buf, uint16_t *tx_len);

uint16_t proto_crc16(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __COMM_PROTOCOL_H__ */
