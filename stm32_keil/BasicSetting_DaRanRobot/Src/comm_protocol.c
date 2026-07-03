/**
 ******************************************************************************
 * @file    comm_protocol.c
 * @brief   上位机通信协议实现 — 帧解析 / CRC / 打包
 * @date    2026-06-18
 ******************************************************************************
 */

#include "comm_protocol.h"
#include <string.h>

/* -------------------------------------------------------------------------- */
/* CRC-16/MODBUS 查找表                                                       */
/* -------------------------------------------------------------------------- */
static const uint16_t crc16_table[256] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040,
};

uint16_t proto_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc = (crc >> 8) ^ crc16_table[(crc ^ data[i]) & 0xFF];
    }
    return crc;
}

/* -------------------------------------------------------------------------- */
/* 协议解析                                                                   */
/* -------------------------------------------------------------------------- */

void proto_init(struct proto_parser *pp)
{
    memset(pp, 0, sizeof(*pp));
    pp->state = PROTO_RX_WAIT_START;
}

void proto_rx_byte(struct proto_parser *pp, uint8_t ch)
{
    /* 写入环形缓冲区 */
    uint16_t next = (pp->rx_head + 1) % PROTO_RX_BUF_SIZE;
    if (next != pp->rx_tail) {  /* 缓冲区未满 */
        pp->rx_ring[pp->rx_head] = ch;
        pp->rx_head = next;
    }
}

/* 从环形缓冲区读取一个字节 */
static int ring_get(struct proto_parser *pp, uint8_t *ch)
{
    if (pp->rx_tail == pp->rx_head) return 0;  /* 空 */
    *ch = pp->rx_ring[pp->rx_tail];
    pp->rx_tail = (pp->rx_tail + 1) % PROTO_RX_BUF_SIZE;
    return 1;
}

int proto_try_parse(struct proto_parser *pp, struct proto_frame *frame)
{
    uint8_t ch;
    memset(frame, 0, sizeof(*frame));

    while (ring_get(pp, &ch)) {
        switch (pp->state) {
        case PROTO_RX_WAIT_START:
            if (ch == PROTO_FRAME_START) {
                pp->state = PROTO_RX_CMD;
                pp->payload_index = 0;
                pp->escape_next = 0;
                pp->crc_calc = 0xFFFF;
            }
            break;

        case PROTO_RX_CMD:
            frame->cmd = ch;
            pp->crc_calc = (pp->crc_calc >> 8) ^ crc16_table[(pp->crc_calc ^ ch) & 0xFF];
            pp->state = PROTO_RX_LEN;
            break;

        case PROTO_RX_LEN:
            frame->len = ch;
            if (frame->len > PROTO_MAX_PAYLOAD) {
                pp->state = PROTO_RX_WAIT_START;  /* 长度非法, 丢弃 */
                break;
            }
            pp->crc_calc = (pp->crc_calc >> 8) ^ crc16_table[(pp->crc_calc ^ ch) & 0xFF];
            pp->state = PROTO_RX_SEQ;
            break;

        case PROTO_RX_SEQ:
            frame->seq = ch;
            pp->crc_calc = (pp->crc_calc >> 8) ^ crc16_table[(pp->crc_calc ^ ch) & 0xFF];
            pp->state = (frame->len > 0) ? PROTO_RX_PAYLOAD : PROTO_RX_CRC1;
            break;

        case PROTO_RX_PAYLOAD:
            /* 字节解转义 */
            if (pp->escape_next) {
                pp->escape_next = 0;
                if (ch == 0x0A) ch = 0xAA;
                else if (ch == 0x05) ch = 0x55;
                else if (ch == 0x0B) ch = 0xBB;
                /* 其他情况保持原值 */
            } else if (ch == PROTO_ESCAPE_BYTE) {
                pp->escape_next = 1;
                break;  /* 不存储转义字节自身 */
            }

            frame->payload[pp->payload_index++] = ch;
            pp->crc_calc = (pp->crc_calc >> 8) ^ crc16_table[(pp->crc_calc ^ ch) & 0xFF];

            if (pp->payload_index >= frame->len) {
                pp->state = PROTO_RX_CRC1;
            }
            break;

        case PROTO_RX_CRC1:
            frame->crc = ch;  /* 低字节 */
            pp->state = PROTO_RX_CRC2;
            break;

        case PROTO_RX_CRC2:
            frame->crc |= ((uint16_t)ch << 8);  /* 高字节 */
            pp->state = PROTO_RX_WAIT_END;
            break;

        case PROTO_RX_WAIT_END:
            if (ch == PROTO_FRAME_END) {
                /* 校验 CRC (0xFFFF 跳过校验, 用于调试) */
                if (frame->crc == 0xFFFF || pp->crc_calc == frame->crc) {
                    frame->valid = 1;
                }
            }
            pp->state = PROTO_RX_WAIT_START;
            if (frame->valid) return 1;  /* 有效帧 */
            /* 帧无效, 清空当前帧继续搜索 */
            memset(frame, 0, sizeof(*frame));
            break;
        }
    }
    return 0;  /* 无完整帧 */
}

/* -------------------------------------------------------------------------- */
/* 响应帧构建                                                                 */
/* -------------------------------------------------------------------------- */

/**
 * @brief 写入一字节到发送缓冲区, 处理字节转义
 */
static void tx_escape_byte(uint8_t ch, uint8_t *buf, uint16_t *idx)
{
    if (ch == PROTO_FRAME_START) {       /* 0xAA → 0xBB 0x0A */
        buf[(*idx)++] = PROTO_ESCAPE_BYTE;
        buf[(*idx)++] = 0x0A;
    } else if (ch == PROTO_FRAME_END) {  /* 0x55 → 0xBB 0x05 */
        buf[(*idx)++] = PROTO_ESCAPE_BYTE;
        buf[(*idx)++] = 0x05;
    } else if (ch == PROTO_ESCAPE_BYTE) {/* 0xBB → 0xBB 0x0B */
        buf[(*idx)++] = PROTO_ESCAPE_BYTE;
        buf[(*idx)++] = 0x0B;
    } else {
        buf[(*idx)++] = ch;
    }
}

void proto_build_response(uint8_t rsp_cmd, uint8_t seq,
                          const uint8_t *payload, uint8_t len,
                          uint8_t *tx_buf, uint16_t *tx_len)
{
    *tx_len = 0;

    /* 帧头 */
    tx_buf[(*tx_len)++] = PROTO_FRAME_START;

    /* CMD + LEN + SEQ (计入 CRC) */
    uint8_t header[3] = { rsp_cmd, len, seq };
    uint16_t crc = proto_crc16(header, 3);

    tx_escape_byte(rsp_cmd, tx_buf, tx_len);
    tx_escape_byte(len, tx_buf, tx_len);
    tx_escape_byte(seq, tx_buf, tx_len);

    /* Payload */
    for (uint8_t i = 0; i < len; i++) {
        tx_escape_byte(payload[i], tx_buf, tx_len);
    }

    /* 更新 CRC (payload 部分) */
    crc = proto_crc16(header, 3);
    if (len > 0)
        crc = proto_crc16(payload, len);  /* 简化: 重算整个 CRC */
    /* 更正: 需要联合计算 header + payload */
    {
        uint8_t crc_buf[256];
        crc_buf[0] = rsp_cmd; crc_buf[1] = len; crc_buf[2] = seq;
        if (len > 0) memcpy(crc_buf + 3, payload, len);
        crc = proto_crc16(crc_buf, 3 + len);
    }

    /* CRC 小端 (CRC 自身不转义, 不参与 CRC 计算) */
    tx_buf[(*tx_len)++] = (uint8_t)(crc & 0xFF);       /* CRC_LO */
    tx_buf[(*tx_len)++] = (uint8_t)((crc >> 8) & 0xFF);/* CRC_HI */

    /* 帧尾 */
    tx_buf[(*tx_len)++] = PROTO_FRAME_END;
}

void proto_build_telemetry(uint8_t seq, const uint8_t *payload, uint8_t len,
                           uint8_t *tx_buf, uint16_t *tx_len)
{
    proto_build_response(RSP_TELEMETRY, seq, payload, len, tx_buf, tx_len);
}
