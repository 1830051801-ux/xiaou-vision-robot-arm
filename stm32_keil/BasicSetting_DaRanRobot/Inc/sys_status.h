/**
 ******************************************************************************
 * @file    sys_status.h
 * @brief   系统运行状态管理 API — 模块就绪标志 / 状态汇总打印
 *
 * struct sys_status 和 g_sys 已在 app_threads.h 中定义
 * 本文件仅提供辅助宏和 API 函数声明
 ******************************************************************************
 */

#ifndef __SYS_STATUS_H__
#define __SYS_STATUS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* -------------------------------------------------------------------------- */
/* 模块就绪标志 (位图, 用于 g_sys.modules_ready)                              */
/* -------------------------------------------------------------------------- */
#define SYS_MOD_USART2      0x0001   /* 调试串口就绪 */
#define SYS_MOD_USART1      0x0002   /* 树莓派串口就绪 */
#define SYS_MOD_CAN         0x0004   /* CAN 总线就绪 */
#define SYS_MOD_SERVO       0x0008   /* 舵机 PWM 就绪 */
#define SYS_MOD_PROTOCOL    0x0010   /* 协议解析器就绪 */
#define SYS_MOD_KINEMATICS  0x0020   /* 运动学就绪 */
#define SYS_MOD_TRAJECTORY  0x0040   /* 轨迹规划就绪 */
#define SYS_MOD_JOINTS      0x0080   /* 关节全部在线 */

/* -------------------------------------------------------------------------- */
/* API                                                                        */
/* -------------------------------------------------------------------------- */

/** 标记模块就绪 */
void sys_mark_ready(uint16_t module_bit);

/** 命令计数+1 */
void sys_cmd_inc(void);

/** 汇总打印系统状态 (sysmonitor 调用) */
void sys_dump_status(void);

#ifdef __cplusplus
}
#endif

#endif /* __SYS_STATUS_H__ */
