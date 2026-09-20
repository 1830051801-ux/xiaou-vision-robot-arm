/**
 ******************************************************************************
 * @file    monitor.h
 * @brief   安全监控模块 — 通信超时检测 / 急停处理 / 关节安全
 * @date    2026-07-27 (架构重构: 移除自动回零, MCU 不再自主生成轨迹)
 ******************************************************************************
 * @attention
 * 重构后行为:
 *   - 通信超时 → 急停 + 通知 Pi (不再自主回零)
 *   - 保留独立 ESTOP 和关节级安全监控
 ******************************************************************************
 */

#ifndef __MONITOR_H__
#define __MONITOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "arm_config.h"

/* -------------------------------------------------------------------------- */
/* 监控状态                                                                   */
/* -------------------------------------------------------------------------- */
enum monitor_state {
    MON_NORMAL      = 0,   /* 正常通信中 */
    MON_WARNING     = 1,   /* 通信延迟 (未超时但接近) */
    MON_TIMEOUT     = 2,   /* 通信超时, 已触发急停 */
    MON_ESTOP       = 3,   /* 急停激活 */
};

/* -------------------------------------------------------------------------- */
/* 监控上下文                                                                 */
/* -------------------------------------------------------------------------- */
struct monitor {
    enum monitor_state  state;          /* 当前状态 */

    uint32_t last_cmd_tick;             /* 最后收到有效命令的 tick */
    uint32_t timeout_ms;                /* 超时阈值 (ms) */
    uint32_t warn_ms;                   /* 预警阈值 (ms, < timeout_ms) */

    uint8_t  estop_active;              /* 急停激活标志 */
    uint8_t  feed_count;                /* 喂狗计数 (调试用) */
    uint8_t  timeout_count;             /* 超时次数 */
};

/* -------------------------------------------------------------------------- */
/* API                                                                        */
/* -------------------------------------------------------------------------- */

void monitor_init(struct monitor *mon, uint32_t timeout_ms);

void monitor_feed(struct monitor *mon, uint32_t now_tick);

enum monitor_state monitor_check(struct monitor *mon, uint32_t now_tick);

void monitor_estop(struct monitor *mon);

void monitor_clear_estop(struct monitor *mon);

/* -------------------------------------------------------------------------- */
/* 关节级安全监控 — 过力矩 / 到位超时                                         */
/* -------------------------------------------------------------------------- */

enum joint_safe_state {
    JOINT_SAFE_OK       = 0,   /* 正常 */
    JOINT_SAFE_OVERTORQ = 1,   /* 力矩超限 */
    JOINT_SAFE_TIMEOUT  = 2,   /* 到位超时 */
};

struct joint_safety {
    uint8_t  active;                    /* 监控激活标志 */
    float    target[4];                 /* 目标关节角度 */
    float    torque_limit[4];           /* 力矩上限 (Nm) */
    uint32_t motion_timeout_ms;         /* 运动超时时间 */
    uint32_t motion_start_tick;         /* 运动开始时刻 */
    float    angle_tolerance;           /* 到位角度容差 (°) */
    uint8_t  alarm_joint;              /* 触发报警的关节号 (1~4) */
    enum joint_safe_state alarm_reason; /* 报警原因 */
};

void joint_safety_init(struct joint_safety *js);
void joint_safety_set_target(struct joint_safety *js,
                             const float target[MAX_JOINT_COUNT], uint32_t now_tick);
enum joint_safe_state joint_safety_check(struct joint_safety *js,
                                          uint32_t now_tick,
                                          const float current[MAX_JOINT_COUNT]);

#ifdef __cplusplus
}
#endif

#endif /* __MONITOR_H__ */
