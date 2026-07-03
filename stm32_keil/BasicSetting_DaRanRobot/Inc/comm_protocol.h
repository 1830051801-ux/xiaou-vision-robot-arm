/**
 ******************************************************************************
 * @file    comm_protocol.h
 * @brief   上位机通信协议 — 二进制帧解析 / 命令定义 / 遥测打包
 * @author  YaowenLi
 * @date    2026-06-18
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
/* 命令码 (Host → Arm)                                                       */
/* -------------------------------------------------------------------------- */
#define CMD_PING             0x01    /* 心跳测试 */
#define CMD_ESTOP            0x02    /* 急停 */
#define CMD_CLEAR_ERROR      0x03    /* 清除错误 */
#define CMD_MOVE_JOINT       0x10    /* 单关节角度控制 */
#define CMD_MOVE_JOINTS      0x11    /* 多关节 PTP 运动 */
#define CMD_MOVE_CART        0x12    /* 笛卡尔直线运动 */
#define CMD_STOP             0x13    /* 停止运动 */
#define CMD_GRASP_MOVE       0x14    /* 完整抓取: 取物→夹取→放物 (payload见文档) */
#define CMD_GET_STATE        0x20    /* 查询完整状态 */
#define CMD_GET_JOINT        0x21    /* 查询单关节状态 */
#define CMD_STREAM_START     0x22    /* 开始遥测流 */
#define CMD_STREAM_STOP      0x23    /* 停止遥测流 */
#define CMD_SET_ZERO         0x30    /* 设置零点 */
#define CMD_SET_PID          0x31    /* 设置 PID */
#define CMD_SET_LIMIT        0x32    /* 设置限位 */
#define CMD_REBOOT_JOINT     0x33    /* 重启关节 */
#define CMD_SAVE_CONFIG      0x34    /* 保存配置 */
#define CMD_CALIB_START      0x35    /* 开始单关节标定 (payload: joint_id) */
#define CMD_CALIB_END        0x36    /* 结束标定模式 */
#define CMD_SET_SYS_ORIGIN   0x37    /* 设置系统原点 (payload: x/y/z 各4B float) */
#define CMD_GET_SYS_ORIGIN   0x38    /* 查询系统原点 */
#define CMD_GET_CALIB_STATUS 0x39    /* 查询标定状态 (零点有效性+基线) */
#define CMD_TEACH_START      0x40    /* 开始示教 */
#define CMD_TEACH_STOP       0x41    /* 停止示教 */
#define CMD_TEACH_RECORD     0x42    /* 记录示教点 */
#define CMD_PLAY_SEQUENCE    0x43    /* 回放示教序列 */

/* -------------------------------------------------------------------------- */
/* 响应码 (Arm → Host)                                                       */
/* -------------------------------------------------------------------------- */
#define RSP_ACK              0x00    /* 成功响应 */
#define RSP_NACK             0x01    /* 失败 */
#define RSP_BUSY             0x02    /* 忙碌 (正在执行运动) */
#define RSP_ESTOP_ACTIVE     0x03    /* 急停激活中 */
#define RSP_INVALID_PARAM    0x04    /* 参数非法 */
#define RSP_TELEMETRY        0x80    /* 遥测数据帧 (主动推送) */

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

/**
 * @brief 初始化协议解析器
 */
void proto_init(struct proto_parser *pp);

/**
 * @brief 向接收缓冲区喂入一个字节 (由 UART ISR 或轮询调用)
 * @param pp  解析器实例
 * @param ch  接收到的字节
 */
void proto_rx_byte(struct proto_parser *pp, uint8_t ch);

/**
 * @brief 尝试解析一帧 (由 comm 线程周期性调用)
 * @param pp     解析器实例
 * @param frame  输出: 解析完成的有效帧 (frame->valid == 1 表示成功)
 * @return 1=解析到有效帧, 0=无完整帧
 */
int proto_try_parse(struct proto_parser *pp, struct proto_frame *frame);

/**
 * @brief 构建响应帧并写入发送缓冲区
 * @param rsp_cmd  响应码
 * @param seq      序列号 (回显)
 * @param payload  负载数据指针 (可为 NULL)
 * @param len      负载长度
 * @param tx_buf   发送缓冲区
 * @param tx_len   输出: 发送数据长度
 */
void proto_build_response(uint8_t rsp_cmd, uint8_t seq,
                          const uint8_t *payload, uint8_t len,
                          uint8_t *tx_buf, uint16_t *tx_len);

/**
 * @brief 构建遥测帧
 * @param seq      序列号
 * @param payload  遥测负载
 * @param len      负载长度
 * @param tx_buf   发送缓冲区
 * @param tx_len   输出: 发送数据长度
 */
void proto_build_telemetry(uint8_t seq, const uint8_t *payload, uint8_t len,
                           uint8_t *tx_buf, uint16_t *tx_len);

/**
 * @brief CRC-16/MODBUS 计算
 */
uint16_t proto_crc16(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __COMM_PROTOCOL_H__ */
