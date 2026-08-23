/**
 ******************************************************************************
 * @file    monitor.c
 * @brief   安全监控实现 — 通信超时 → 急停 (精简版)
 * @date    2026-07-27 (架构重构: 移除自动回零, MCU 不再自主生成轨迹)
 *
 * 依赖:
 *   - DrEmpower_can: estop / 伺服控制
 *   - arm_config.h: 安全参数 (ARM_COMM_LOSS_TIMEOUT_MS, 关节限位)
 ******************************************************************************
 */

#include "monitor.h"
#include "arm_config.h"
#include "app_threads.h"
#include "DrEmpower_can.h"

#include <string.h>

static const uint8_t joint_ids[MAX_JOINT_COUNT] = JOINT_ID_LIST;

/* ========================================================================== */
/* 公开 API                                                                   */
/* ========================================================================== */

void monitor_init(struct monitor *mon, uint32_t timeout_ms)
{
    memset(mon, 0, sizeof(*mon));
    mon->state      = MON_NORMAL;
    mon->timeout_ms = (timeout_ms > 0) ? timeout_ms : ARM_COMM_LOSS_TIMEOUT_MS;
    mon->warn_ms    = mon->timeout_ms * 2 / 3;
    mon->estop_active = 0;
    mon->feed_count   = 0;
    mon->timeout_count = 0;
}

void monitor_feed(struct monitor *mon, uint32_t now_tick)
{
    if (mon == NULL) return;

    mon->last_cmd_tick = now_tick;
    mon->feed_count++;

    if (mon->state == MON_TIMEOUT || mon->state == MON_WARNING) {
        mon->state = MON_NORMAL;
    }
}

enum monitor_state monitor_check(struct monitor *mon, uint32_t now_tick)
{
    if (mon == NULL) return MON_ESTOP;

    if (mon->estop_active) {
        return MON_ESTOP;
    }

    uint32_t elapsed = now_tick - mon->last_cmd_tick;

    /* ---- 超时检测: 急停所有关节 ---- */
    if (elapsed >= mon->timeout_ms) {
        mon->timeout_count++;
        for (int i = 0; i < 4; i++) {
            estop(joint_ids[i]);
        }
        mon->estop_active = 1;
        mon->state = MON_TIMEOUT;
        return MON_TIMEOUT;
    }

    /* ---- 预警检测 ---- */
    if (elapsed >= mon->warn_ms) {
        mon->state = MON_WARNING;
        return MON_WARNING;
    }

    mon->state = MON_NORMAL;
    return MON_NORMAL;
}

void monitor_estop(struct monitor *mon)
{
    if (mon == NULL) return;

    mon->estop_active = 1;
    mon->state = MON_ESTOP;

    for (int i = 0; i < 4; i++) {
        estop(joint_ids[i]);
    }
}

void monitor_clear_estop(struct monitor *mon)
{
    if (mon == NULL) return;

    for (int i = 0; i < 4; i++) {
        uint8_t clear_cmd[8] = {0x0B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        Can_Send_Msg(0x140 + joint_ids[i], 1, clear_cmd);
    }

    mon->estop_active = 0;
    mon->state = MON_TIMEOUT;
}

/* ========================================================================== */
/* 关节级安全监控 — 过力矩 / 到位超时                                         */
/* ========================================================================== */

void joint_safety_init(struct joint_safety *js)
{
    memset(js, 0, sizeof(*js));
    js->motion_timeout_ms = 5000;
    js->angle_tolerance   = 1.5f;
    js->active            = 0;
    static const float tq_max[MAX_JOINT_COUNT] = JOINT_TORQUE_MAX_LIST;
    memcpy(js->torque_limit, tq_max, 4 * sizeof(float));
}

void joint_safety_set_target(struct joint_safety *js,
                             const float target[MAX_JOINT_COUNT], uint32_t now_tick)
{
    memcpy(js->target, target, 4 * sizeof(float));
    js->motion_start_tick = now_tick;
    js->active = 1;
    js->alarm_joint = 0;
    js->alarm_reason = JOINT_SAFE_OK;
}

enum joint_safe_state joint_safety_check(struct joint_safety *js,
                                          uint32_t now_tick,
                                          const float current[MAX_JOINT_COUNT])
{
    if (!js->active) return JOINT_SAFE_OK;

    for (int i = 0; i < 4; i++) {
        float torq = get_torque(joint_ids[i]);
        if (torq > js->torque_limit[i]) {
            js->active = 0;
            js->alarm_joint = i + 1;
            js->alarm_reason = JOINT_SAFE_OVERTORQ;
            for (int j = 0; j < 4; j++) estop(joint_ids[j]);
            return JOINT_SAFE_OVERTORQ;
        }
    }

    uint8_t all_done = 1;
    for (int i = 0; i < 4; i++) {
        float err = current[i] - js->target[i];
        if (err < 0) err = -err;
        if (err > js->angle_tolerance) { all_done = 0; break; }
    }
    if (all_done) {
        js->active = 0;
        return JOINT_SAFE_OK;
    }

    if ((now_tick - js->motion_start_tick) > js->motion_timeout_ms) {
        js->active = 0;
        js->alarm_reason = JOINT_SAFE_TIMEOUT;
        float max_err = 0;
        for (int i = 0; i < 4; i++) {
            float err = current[i] - js->target[i];
            if (err < 0) err = -err;
            if (err > max_err) { max_err = err; js->alarm_joint = i + 1; }
        }
        for (int j = 0; j < 4; j++) estop(joint_ids[j]);
        return JOINT_SAFE_TIMEOUT;
    }

    return JOINT_SAFE_OK;
}
