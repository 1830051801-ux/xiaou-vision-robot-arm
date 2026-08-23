/**
 ******************************************************************************
 * @file    safety.h
 * @brief   安全模块 — 软限位 / 急停 / CAN超时 / 零位有效性 / 执行许可
 * @date    2026-08-08 (Pi v1.2: 从 app_threads.c 最小拆分)
 ******************************************************************************
 * @attention
 * 本模块不直接操作 CAN 或电机, 只做策略判断。
 * joint_ctrl 线程调用 safety_check_motion() 决定是否允许执行轨迹。
 * safety 线程每周期调用 safety_poll() 做持续监控和故障锁存。
 ******************************************************************************
 */

#ifndef __SAFETY_H__
#define __SAFETY_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "arm_config.h"

/* ---- 安全阈值 ---- */
#define SAFETY_CAN_TIMEOUT_MS       50       /* CAN 反馈超时 (ms) */
#define SAFETY_OVERCURRENT_A        8.0f     /* 过流阈值 (A) */
#define SAFETY_OVERTEMP_C           80.0f    /* 过热阈值 (°C) */
#define SAFETY_ZERO_LOST_LOCK       1        /* 零位丢失时锁定运动 */

/* ---- 故障码 ---- */
enum safety_fault {
    SAFETY_OK               = 0x00,
    SAFETY_FAULT_ESTOP      = 0x01,   /* 急停 */
    SAFETY_FAULT_CAN_TIMEOUT = 0x02,  /* CAN 超时 */
    SAFETY_FAULT_OVERCURRENT = 0x04,  /* 过流 */
    SAFETY_FAULT_OVERTEMP   = 0x08,   /* 过热 */
    SAFETY_FAULT_OFFLINE    = 0x10,   /* 关节离线 */
    SAFETY_FAULT_ZERO_LOST  = 0x20,   /* 零位丢失 */
    SAFETY_FAULT_LIMIT      = 0x40,   /* 软限位触发 */
};

/* -------------------------------------------------------------------------- */
/* API                                                                        */
/* -------------------------------------------------------------------------- */

/* 初始化 */
void safety_init(void);

/* 持续监控 (safety 线程每周期调用) */
int  safety_poll(const float joint_angles_deg[MAX_JOINT_COUNT],
                 const float joint_currents_a[MAX_JOINT_COUNT],
                 const uint8_t joint_online[MAX_JOINT_COUNT],
                 const uint8_t zero_valid[MAX_JOINT_COUNT],
                 uint32_t last_can_tick_ms);

/* 运动许可: 返回 0=允许, 非0=拒绝原因 (enum safety_fault 组合) */
int  safety_check_motion(const float target_angles_deg[MAX_JOINT_COUNT]);

/* 软限位检查 (模型角度域) */
int  safety_check_limits(const float angles_deg[MAX_JOINT_COUNT]);

/* 获取当前锁存的故障码 */
int  safety_get_faults(void);

/* 清除故障锁存 (CMD_CLEAR_ERROR 后调用) */
void safety_clear_faults(void);

/* 设置急停 */
void safety_trigger_estop(void);

/* 急停是否激活 */
int  safety_estop_active(void);

#ifdef __cplusplus
}
#endif

#endif /* __SAFETY_H__ */
