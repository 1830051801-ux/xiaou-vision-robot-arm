/**
 ******************************************************************************
 * @file    monitor.h
 * @brief   安全监控模块 — 通信超时检测 / 自动回零 / 错误处理
 *
 * 每个控制周期调用 monitor_check() 检测超时:
 *   - 超时: 自动触发回零序列, 避免机械臂悬停
 *   - 收到有效帧: 调用 monitor_feed() 喂狗, 重置计时
 *
 * 回零流程: 急停 → 张开夹爪 → PTP 回零点 → DONE
 ******************************************************************************
 */

#ifndef __MONITOR_H__
#define __MONITOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* -------------------------------------------------------------------------- */
/* 监控状态                                                                   */
/* -------------------------------------------------------------------------- */
enum monitor_state {
    MON_NORMAL      = 0,   /* 正常通信中 */
    MON_WARNING     = 1,   /* 通信延迟 (未超时但接近) */
    MON_TIMEOUT     = 2,   /* 通信超时, 触发回零 */
    MON_HOMING      = 3,   /* 回零进行中 */
    MON_HOME_DONE   = 4,   /* 已回到零点, 等待恢复 */
    MON_ESTOP       = 5,   /* 急停激活 */
};

/* -------------------------------------------------------------------------- */
/* 监控上下文                                                                 */
/* -------------------------------------------------------------------------- */
struct monitor {
    enum monitor_state  state;          /* 当前状态 */

    uint32_t last_cmd_tick;             /* 最后收到有效命令的 tick */
    uint32_t timeout_ms;                /* 超时阈值 (ms) */
    uint32_t warn_ms;                   /* 预警阈值 (ms, < timeout_ms) */

    uint32_t homing_start_tick;         /* 回零开始时刻 */
    uint32_t homing_duration_ms;        /* 回零预计时长 */

    float    home_joints[4];            /* 零点关节角度[4] */

    uint8_t  estop_active;              /* 急停激活标志 */
    uint8_t  feed_count;                /* 喂狗计数 (调试用) */
    uint8_t  timeout_count;             /* 超时次数 */
};

/* -------------------------------------------------------------------------- */
/* API                                                                        */
/* -------------------------------------------------------------------------- */

/**
 * @brief 初始化监控器
 * @param mon          监控器实例
 * @param timeout_ms   通信超时阈值 (ms), 默认 200ms
 * @param home_joints  零点关节角度[4] (°)
 */
void monitor_init(struct monitor *mon, uint32_t timeout_ms,
                  const float home_joints[4]);

/**
 * @brief 喂狗 — 收到有效帧时调用, 重置超时计数器
 * @param mon      监控器实例
 * @param now_tick 当前系统 tick (rt_tick_get())
 */
void monitor_feed(struct monitor *mon, uint32_t now_tick);

/**
 * @brief 监控检查 — 每个控制周期调用
 * @param mon            监控器实例
 * @param now_tick       当前系统 tick
 * @param current_joints 当前关节角度[4] (°)
 * @param output_joints  输出: 如果回零中, 返回目标关节角度; 否则 = current
 * @param do_home        输入: 是否允许自动回零 (1=允许, 0=仅报警)
 * @return 当前监控状态
 */
enum monitor_state monitor_check(struct monitor *mon,
                                  uint32_t now_tick,
                                  const float current_joints[4],
                                  float output_joints[4],
                                  int do_home);

/**
 * @brief 手动触发急停
 */
void monitor_estop(struct monitor *mon);

/**
 * @brief 清除急停, 恢复监控
 */
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
                             const float target[4], uint32_t now_tick);
enum joint_safe_state joint_safety_check(struct joint_safety *js,
                                          uint32_t now_tick,
                                          const float current[4]);

#ifdef __cplusplus
}
#endif

#endif /* __MONITOR_H__ */
