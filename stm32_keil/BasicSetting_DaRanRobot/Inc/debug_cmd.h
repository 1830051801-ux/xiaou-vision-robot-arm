/**
 ******************************************************************************
 * @file    debug_cmd.h
 * @brief   USART3 调试命令行 — 人机交互接口
 * @date    2026-08-03
 ******************************************************************************
 * @attention
 * 通过 USART3 (115200) 接收 ASCII 文本命令, 用于开发阶段的手动调试。
 * 覆盖: 关节控制 / 标定 / 状态查询 / Pi 透传追踪
 *
 * 命令列表: 输入 h 或 help 查看完整帮助
 ******************************************************************************
 */

#ifndef __DEBUG_CMD_H__
#define __DEBUG_CMD_H__

#ifdef __cplusplus
extern "C" {
#endif

/** Pi 原始字节监控开关 (raw on/off 控制, comm 线程读取) */
extern volatile int g_raw_monitor;

/**
 * @brief 初始化调试命令行 (启动监听线程)
 * @return RT_EOK 成功
 */
int debug_cmd_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __DEBUG_CMD_H__ */
