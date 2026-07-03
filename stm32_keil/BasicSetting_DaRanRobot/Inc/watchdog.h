/**
 ******************************************************************************
 * @file    watchdog.h
 * @brief   独立看门狗 (IWDG) 驱动 — 硬件喂狗 + 线程心跳监控
 * @author  YaowenLi
 * @date    2026-06-11
 ******************************************************************************
 * @attention
 * IWDG 时钟源: LSI (32kHz 标称, 17~47kHz 实际范围)
 * 预分频: /64 → 500Hz 标称
 * 重载值: 1999 → 超时 ≈ 4 秒 (实际 2.7s ~ 7.5s 取决于 LSI 精度)
 * 喂狗周期: 500ms (看门狗线程) → 超时窗口内约 8 次喂狗
 *
 * 线程心跳监控:
 *   关键线程在其主循环中递增心跳计数器
 *   看门狗线程检查所有心跳是否在超时周期内更新
 *   若某线程心跳超时(>2s)，记录错误并触发系统急停
 ******************************************************************************
 */

#ifndef __WATCHDOG_H__
#define __WATCHDOG_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* IWDG 参数 -----------------------------------------------------------------*/
#define WDT_PRESCALER        IWDG_PRESCALER_64   /* LSI/64 = 500Hz 标称 */
#define WDT_RELOAD           1999                /* (1999+1)/500Hz ≈ 4s */
#define WDT_TIMEOUT_MS       4000                /* 标称超时 (ms)       */
#define WDT_FEED_INTERVAL_MS  500                /* 喂狗周期 (ms)       */

/* 心跳超时: 线程超过此时间未更新心跳视为挂死 */
#define WDT_HB_TIMEOUT_MS    2000                /* 2 秒                */

/* 心跳索引 (对应 thread_heartbeat[] 数组) — 6线程架构 */
#define HB_IDX_WATCHDOG       0   /* 看门狗自身 (自检) */
#define HB_IDX_JOINT_DATA     1   /* 关节数据接收线程   */
#define HB_IDX_SAFETY         2   /* 保护限位线程       */
#define HB_IDX_JOINT_CTRL     3   /* 关节控制线程       */
#define HB_IDX_COMM           4   /* 通讯线程           */
#define HB_IDX_LED            5   /* LED 指示灯线程     */
#define HB_COUNT              6

/* API -----------------------------------------------------------------------*/

void Watchdog_Init(void);                               /* 初始化 IWDG (MX_IWDG_Init 已启动) */
void Watchdog_Feed(void);                               /* 喂狗 (重载计数器) */
void Watchdog_Heartbeat(int thread_idx);                /* 线程心跳登记 (各线程主循环调用) */
int  Watchdog_CheckAll(void);                           /* 检查所有心跳, 返回挂死线程数 */
void Watchdog_PrintInfo(void);                          /* 打印看门狗状态信息 */

#ifdef __cplusplus
}
#endif

#endif /* __WATCHDOG_H__ */
