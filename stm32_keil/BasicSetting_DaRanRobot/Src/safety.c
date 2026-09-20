/**
 ******************************************************************************
 * @file    safety.c
 * @brief   安全模块实现
 * @date    2026-08-08
 ******************************************************************************
 */

#include "safety.h"
#include "trajectory.h"
#include <rtthread.h>
#include <string.h>

/* ---- 故障锁存 ---- */
static volatile int    fault_latch   = SAFETY_OK;
static volatile int    estop_active  = 0;
static          rt_mutex_t safety_mutex = RT_NULL;

/* 外部引用 — trajectory 限位表 */
extern const float traj_effective_min[MAX_JOINT_COUNT];
extern const float traj_effective_max[MAX_JOINT_COUNT];

/* ---- 初始化 ---- */
void safety_init(void)
{
    if (safety_mutex == RT_NULL)
        safety_mutex = rt_mutex_create("safety", RT_IPC_FLAG_FIFO);
    fault_latch  = SAFETY_OK;
    estop_active = 0;
}

/* ---- 持续监控 ---- */
int safety_poll(const float joint_angles_deg[MAX_JOINT_COUNT],
                const float joint_currents_a[MAX_JOINT_COUNT],
                const uint8_t joint_online[MAX_JOINT_COUNT],
                const uint8_t zero_valid[MAX_JOINT_COUNT],
                uint32_t last_can_tick_ms)
{
    int new_faults = SAFETY_OK;
    uint32_t now = rt_tick_get();

    /* CAN 超时检查 */
    if ((now - last_can_tick_ms) > SAFETY_CAN_TIMEOUT_MS) {
        new_faults |= SAFETY_FAULT_CAN_TIMEOUT;
    }

    /* 逐关节检查 */
    for (int i = 0; i < MAX_JOINT_COUNT; i++) {
        if (!joint_online[i])
            new_faults |= SAFETY_FAULT_OFFLINE;

        if (!zero_valid[i])
            new_faults |= SAFETY_FAULT_ZERO_LOST;

        if (joint_currents_a[i] > SAFETY_OVERCURRENT_A)
            new_faults |= SAFETY_FAULT_OVERCURRENT;

        /* 软限位 */
        if (joint_angles_deg[i] < traj_effective_min[i] ||
            joint_angles_deg[i] > traj_effective_max[i])
            new_faults |= SAFETY_FAULT_LIMIT;
    }

    /* 急停 */
    if (estop_active)
        new_faults |= SAFETY_FAULT_ESTOP;

    /* 锁存 */
    if (new_faults != SAFETY_OK) {
        if (safety_mutex) rt_mutex_take(safety_mutex, RT_WAITING_FOREVER);
        fault_latch |= new_faults;
        if (safety_mutex) rt_mutex_release(safety_mutex);
    }

    return new_faults;
}

/* ---- 运动许可 ---- */
int safety_check_motion(const float target_angles_deg[MAX_JOINT_COUNT])
{
    int faults = safety_get_faults();
    if (faults != SAFETY_OK)
        return faults;

    /* 目标角度软限位 */
    for (int i = 0; i < MAX_JOINT_COUNT; i++) {
        if (target_angles_deg[i] < traj_effective_min[i] ||
            target_angles_deg[i] > traj_effective_max[i])
            return SAFETY_FAULT_LIMIT;
    }

    return SAFETY_OK;
}

/* ---- 软限位检查 ---- */
int safety_check_limits(const float angles_deg[MAX_JOINT_COUNT])
{
    for (int i = 0; i < MAX_JOINT_COUNT; i++) {
        if (angles_deg[i] < traj_effective_min[i] ||
            angles_deg[i] > traj_effective_max[i])
            return 0;
    }
    return 1;
}

/* ---- 获取故障 ---- */
int safety_get_faults(void)
{
    return fault_latch;
}

/* ---- 清除故障 ---- */
void safety_clear_faults(void)
{
    if (safety_mutex) rt_mutex_take(safety_mutex, RT_WAITING_FOREVER);
    fault_latch  = SAFETY_OK;
    estop_active = 0;
    if (safety_mutex) rt_mutex_release(safety_mutex);
}

/* ---- 触发急停 ---- */
void safety_trigger_estop(void)
{
    if (safety_mutex) rt_mutex_take(safety_mutex, RT_WAITING_FOREVER);
    estop_active = 1;
    fault_latch |= SAFETY_FAULT_ESTOP;
    if (safety_mutex) rt_mutex_release(safety_mutex);
}

/* ---- 急停状态 ---- */
int safety_estop_active(void)
{
    return estop_active;
}
