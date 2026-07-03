/**
 ******************************************************************************
 * @file    monitor.c
 * @brief   安全监控实现 — 通信超时 → 急停 → 自动回零
 *
 * 依赖:
 *   - traj_planner: 回零轨迹插补
 *   - arm_kinematics: 关节限位检查
 *   - DrEmpower_can: estop / 伺服控制
 *   - arm_config.h: 安全参数 (ARM_COMM_LOSS_TIMEOUT_MS, 关节限位)
 ******************************************************************************
 */

#include "monitor.h"
#include "trajectory_planner.h"
#include "arm_kinematics.h"
#include "arm_config.h"
#include "DrEmpower_can.h"

#include <string.h>

static const uint8_t joint_ids[4] = JOINT_ID_LIST;

/* ========================================================================== */
/* 公开 API                                                                   */
/* ========================================================================== */

void monitor_init(struct monitor *mon, uint32_t timeout_ms,
                  const float home_joints[4])
{
    memset(mon, 0, sizeof(*mon));
    mon->state      = MON_NORMAL;
    mon->timeout_ms = (timeout_ms > 0) ? timeout_ms : ARM_COMM_LOSS_TIMEOUT_MS;
    mon->warn_ms    = mon->timeout_ms * 2 / 3;  /* 预警 = 超时 × 2/3 */
    memcpy(mon->home_joints, home_joints, 4 * sizeof(float));
    mon->estop_active = 0;
    mon->feed_count   = 0;
    mon->timeout_count = 0;
}

void monitor_feed(struct monitor *mon, uint32_t now_tick)
{
    if (mon == NULL) return;

    mon->last_cmd_tick = now_tick;
    mon->feed_count++;

    /* 从超时/回零状态恢复 */
    if (mon->state == MON_TIMEOUT || mon->state == MON_WARNING) {
        mon->state = MON_NORMAL;
    }
}

enum monitor_state monitor_check(struct monitor *mon,
                                  uint32_t now_tick,
                                  const float current_joints[4],
                                  float output_joints[4],
                                  int do_home)
{
    if (mon == NULL || output_joints == NULL) return MON_ESTOP;

    /* 急停激活时不做任何事 */
    if (mon->estop_active) {
        memcpy(output_joints, current_joints, 4 * sizeof(float));
        return MON_ESTOP;
    }

    uint32_t elapsed = now_tick - mon->last_cmd_tick;

    /* ---- 回零进行中 ---- */
    if (mon->state == MON_HOMING) {
        static struct traj_planner tp_home;
        static uint8_t  homing_started = 0;
        static uint32_t home_start_tick = 0;

        if (!homing_started) {
            traj_init(&tp_home);
            traj_set_joint_ptp(&tp_home, mon->home_joints,
                               TRAJ_DEFAULT_SPEED * 0.5f,    /* 50% 速度回零 */
                               TRAJ_DEFAULT_ACCEL * 0.5f,
                               TRAJ_DEFAULT_JERK * 0.5f);
            traj_start(&tp_home, 0, current_joints);
            home_start_tick = now_tick;
            homing_started = 1;
        }

        uint32_t t = now_tick - home_start_tick;
        enum traj_state ts = traj_update(&tp_home, t, output_joints);

        /* 限位保护 */
        for (int i = 0; i < 4; i++) {
            static const float min_lim[4] = JOINT_ANGLE_MIN_LIST;
            static const float max_lim[4] = JOINT_ANGLE_MAX_LIST;
            if (output_joints[i] < min_lim[i]) output_joints[i] = min_lim[i];
            if (output_joints[i] > max_lim[i]) output_joints[i] = max_lim[i];
        }

        if (ts == TRAJ_DONE) {
            homing_started = 0;
            mon->state = MON_HOME_DONE;
        } else if (ts == TRAJ_ERROR) {
            homing_started = 0;
            mon->state = MON_TIMEOUT;
            memcpy(output_joints, current_joints, 4 * sizeof(float));
            return MON_TIMEOUT;
        }
        return MON_HOMING;
    }

    /* ---- 已回零, 保持位置等待恢复 ---- */
    if (mon->state == MON_HOME_DONE) {
        memcpy(output_joints, mon->home_joints, 4 * sizeof(float));
        return MON_HOME_DONE;
    }

    /* ---- 超时检测 ---- */
    if (elapsed >= mon->timeout_ms) {
        mon->timeout_count++;

        if (do_home) {
            /* 急停所有关节 + 夹爪 */
            for (int i = 0; i < 4; i++) {
                estop(joint_ids[i]);
            }
            estop(5);  /* 夹爪 */

            mon->state = MON_HOMING;
            mon->homing_start_tick = now_tick;
            memcpy(output_joints, current_joints, 4 * sizeof(float));
            return MON_HOMING;
        } else {
            mon->state = MON_TIMEOUT;
            memcpy(output_joints, current_joints, 4 * sizeof(float));
            return MON_TIMEOUT;
        }
    }

    /* ---- 预警检测 ---- */
    if (elapsed >= mon->warn_ms) {
        mon->state = MON_WARNING;
        memcpy(output_joints, current_joints, 4 * sizeof(float));
        return MON_WARNING;
    }

    /* ---- 正常 ---- */
    mon->state = MON_NORMAL;
    memcpy(output_joints, current_joints, 4 * sizeof(float));
    return MON_NORMAL;
}

void monitor_estop(struct monitor *mon)
{
    if (mon == NULL) return;

    mon->estop_active = 1;
    mon->state = MON_ESTOP;

    /* 急停所有关节 + 夹爪 */
    for (int i = 0; i < 4; i++) {
        estop(joint_ids[i]);
    }
    estop(5);
}

void monitor_clear_estop(struct monitor *mon)
{
    if (mon == NULL) return;

    /* 清除各关节错误 */
    for (int i = 0; i < 4; i++) {
        /* 发送清除错误命令 (CMD_CLEAR_ERROR 的 CAN 封装) */
        uint8_t clear_cmd[8] = {0x0B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        Can_Send_Msg(0x140 + joint_ids[i], 1, clear_cmd);
    }

    mon->estop_active = 0;
    mon->state = MON_TIMEOUT;  /* 清除后回零 */
}

/* ========================================================================== */
/* 关节级安全监控 — 过力矩 / 到位超时                                         */
/* ========================================================================== */

void joint_safety_init(struct joint_safety *js)
{
    memset(js, 0, sizeof(*js));
    js->motion_timeout_ms = 5000;        /* 默认5秒超时 */
    js->angle_tolerance   = 1.5f;        /* 到位容差1.5° */
    js->active            = 0;
    /* 力矩上限从 arm_config.h 读取 */
    static const float tq_max[4] = JOINT_TORQUE_MAX_LIST;
    memcpy(js->torque_limit, tq_max, 4 * sizeof(float));
}

void joint_safety_set_target(struct joint_safety *js,
                             const float target[4], uint32_t now_tick)
{
    memcpy(js->target, target, 4 * sizeof(float));
    js->motion_start_tick = now_tick;
    js->active = 1;
    js->alarm_joint = 0;
    js->alarm_reason = JOINT_SAFE_OK;
}

enum joint_safe_state joint_safety_check(struct joint_safety *js,
                                          uint32_t now_tick,
                                          const float current[4])
{
    if (!js->active) return JOINT_SAFE_OK;

    for (int i = 0; i < 4; i++) {
        /* 1. 过力矩检测 */
        float torq = get_torque(joint_ids[i]);
        if (torq > js->torque_limit[i]) {
            js->active = 0;
            js->alarm_joint = i + 1;
            js->alarm_reason = JOINT_SAFE_OVERTORQ;
            /* 急停全部 */
            for (int j = 0; j < 4; j++) estop(joint_ids[j]);
            return JOINT_SAFE_OVERTORQ;
        }
    }

    /* 2. 到位检测: 全部关节到位 → 正常完成 */
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

    /* 3. 超时检测 */
    if ((now_tick - js->motion_start_tick) > js->motion_timeout_ms) {
        js->active = 0;
        js->alarm_reason = JOINT_SAFE_TIMEOUT;
        /* 找到偏差最大的关节 */
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
