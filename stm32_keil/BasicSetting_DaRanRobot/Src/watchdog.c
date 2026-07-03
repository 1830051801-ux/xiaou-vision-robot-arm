/**
 ******************************************************************************
 * @file    watchdog.c
 * @brief   独立看门狗驱动实现
 * @author  YaowenLi
 * @date    2026-06-11
 ******************************************************************************
 * @note
 * STM32F4 IWDG 特性:
 *   - 由 LSI 驱动, 独立于系统时钟, 即使主晶振停振仍工作
 *   - 一旦使能, 只能由硬件复位关闭 (软件无法停止)
 *   - 计数器不可直接读取, 只能通过 KR 寄存器刷新
 *
 * 心跳监控策略:
 *   1. 每个关键线程在主循环中调用 Watchdog_Heartbeat(idx) 递增计数器
 *   2. 看门狗线程每 500ms 调用 Watchdog_CheckAll() 快照所有心跳
 *   3. 比较相邻两次快照: 若某心跳值未变化 → 线程可能挂死
 *   4. 挂死超过 WDT_HB_TIMEOUT_MS → 触发急停或记录错误
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "watchdog.h"
#include "iwdg.h"
#include "app_threads.h"
#include <rtthread.h>

/* Private variables ---------------------------------------------------------*/
static rt_uint32_t last_hb[HB_COUNT];    /* 上一次心跳快照 */
static rt_uint32_t hb_stuck_ms[HB_COUNT];/* 各线程挂死累计时间 */
static rt_bool_t   initialized = RT_FALSE;

/* 线程名称 (用于日志输出) */
static const char *thread_names[HB_COUNT] = {
    "watchdog",
    "joint_data",
    "safety",
    "joint_ctrl",
    "attitude",
    "comm"
//    "led"
};

/* -------------------------------------------------------------------------- */
/* 初始化                                                                    */
/* -------------------------------------------------------------------------- */
void Watchdog_Init(void)
{
    int i;

    /* IWDG 硬件已在 MX_IWDG_Init() 中启动, 此处仅初始化软件状态 */
    for (i = 0; i < HB_COUNT; i++) {
        g_sys.thread_heartbeat[i] = 0;
        last_hb[i]      = 0;
        hb_stuck_ms[i]  = 0;
    }

    /* 立即喂一次狗, 确保初始化期间不会复位 */
    HAL_IWDG_Refresh(&hiwdg);

    initialized = RT_TRUE;
    rt_kprintf("[Watchdog] Driver initialized, timeout=%dms\n", WDT_TIMEOUT_MS);
}

/* -------------------------------------------------------------------------- */
/* 硬件喂狗                                                                   */
/* -------------------------------------------------------------------------- */
void Watchdog_Feed(void)
{
    HAL_IWDG_Refresh(&hiwdg);
}

/* -------------------------------------------------------------------------- */
/* 心跳登记 (各线程主循环调用)                                               */
/* -------------------------------------------------------------------------- */
void Watchdog_Heartbeat(int thread_idx)
{
    if (thread_idx >= 0 && thread_idx < HB_COUNT) {
        g_sys.thread_heartbeat[thread_idx]++;
    }
}

/* -------------------------------------------------------------------------- */
/* 心跳检查 (看门狗线程调用, 每 500ms 一次)                                  */
/* -------------------------------------------------------------------------- */
int Watchdog_CheckAll(void)
{
    int i;
    int stuck_count = 0;

    if (!initialized) return 0;

    for (i = 0; i < HB_COUNT; i++) {
        rt_uint32_t current = g_sys.thread_heartbeat[i];

        if (current == last_hb[i]) {
            /* 心跳未更新 → 线程可能挂死或尚未启动 */
            hb_stuck_ms[i] += WDT_FEED_INTERVAL_MS;

            if (hb_stuck_ms[i] >= WDT_HB_TIMEOUT_MS) {
                rt_kprintf("[Watchdog] WARNING: thread '%s' hung! "
                           "(no heartbeat for %dms)\n",
                           thread_names[i], (int)hb_stuck_ms[i]);
                stuck_count++;

                /* 关键线程挂死 → 触发急停 */
                if (i == HB_IDX_JOINT_DATA || i == HB_IDX_SAFETY ||
                    i == HB_IDX_JOINT_CTRL) {
                    g_sys.error_code = ERR_CAN_TIMEOUT;
                    g_sys.state      = SYS_STATE_ESTOP;
                    g_sys.estop_triggered = 1;
                    rt_kprintf("[Watchdog] CRITICAL: thread '%s' hung → E-STOP!\n",
                               thread_names[i]);
                }
            }
        } else {
            /* 心跳正常更新 */
            hb_stuck_ms[i] = 0;
        }

        last_hb[i] = current;
    }

    return stuck_count;
}

/* -------------------------------------------------------------------------- */
/* 状态信息输出 (调试用)                                                      */
/* -------------------------------------------------------------------------- */
void Watchdog_PrintInfo(void)
{
    int i;
    rt_kprintf("[Watchdog] --- Status ---\n");
    rt_kprintf("  Timeout: %dms, Feed: %dms\n", WDT_TIMEOUT_MS, WDT_FEED_INTERVAL_MS);
    rt_kprintf("  Heartbeats:\n");
    for (i = 0; i < HB_COUNT; i++) {
        rt_kprintf("    %-14s: %lu (stuck=%dms)\n",
                   thread_names[i],
                   g_sys.thread_heartbeat[i],
                   (int)hb_stuck_ms[i]);
    }
}
