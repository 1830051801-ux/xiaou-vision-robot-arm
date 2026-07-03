/**
 ******************************************************************************
 * @file    trajectory_planner.c
 * @brief   轨迹规划器实现 — S曲线 PTP + 笛卡尔直线
 * @date    2026-06-18
 ******************************************************************************
 */

#include "trajectory_planner.h"
#include "arm_kinematics.h"
#include "arm_config.h"
#include <math.h>
#include <string.h>

/* -------------------------------------------------------------------------- */
/* S曲线 — 解析分段多项式 (精确、无数值积分)                                  */
/*                                                                           */
/* 速度剖面为对称 S曲线, 7 阶段:                                              */
/*   [0] 加加速 (+J)   [1] 匀加速 (0)   [2] 减加速 (-J)                       */
/*   [3] 匀速 (0)      [4] 加减速 (-J)  [5] 匀减速 (0)   [6] 减减速 (+J)      */
/*                                                                           */
/* 每段内运动学由三次多项式精确描述:                                          */
/*   j(τ)=±Jmax,  a(τ)=a0+jτ,  v(τ)=v0+a0τ+jτ²/2,  s(τ)=s0+v0τ+a0τ²/2+jτ³/6 */
/* -------------------------------------------------------------------------- */

/**
 * @brief 计算 S曲线阶段时长 — 解析解
 *        处理 4 种情况: 全7段 / 无匀速 / 无匀加速段 / 纯三角
 */
static uint32_t scurve_compute(float dist, float vmax, float amax, float jmax,
                               uint32_t T[7])
{
    memset(T, 0, 7 * sizeof(uint32_t));

    if (dist < 1e-6f) return 0;

    float Tj = amax / jmax;  /* jerk ramp time to reach amax (s) */
    float dist_jerk_only = jmax * Tj * Tj * Tj;  /* displacement for one jerk-only ramp */

    /* Case 1: 能否达到 amax? */
    if (amax * amax / jmax <= vmax) {
        /* 可达到 amax → 加速段 = Tj + Ta + Tj */
        float dist_full = 2.0f * amax * Tj * Tj + vmax * vmax / amax
                        + vmax * (vmax / amax);  /* 不是准确公式, 近似检查 */
        /* 更简洁: 比较位移与最小全7段行程 */
        float d_min_7seg = 2.0f * (jmax * Tj * Tj * Tj + amax * Tj * (vmax/amax))
                         + vmax * (vmax / amax);
        /* 实际用: 先算最大加速度段能走多远 */
        float Ta = (vmax - amax * Tj) / amax;
        if (Ta < 0) Ta = 0;
        float d_accel_full = amax * Tj * Tj + amax * Tj * Ta
                           + 0.5f * amax * Ta * Ta
                           + 0.5f * jmax * Tj * Tj * Tj;  /* jerk up + const + jerk down */
        /* 简化为: d_accel_side = vmax * (Tj + Ta/2) = vmax * (Tj + vmax/(2*amax) - Tj/2) */

        if (Ta > 0 && dist >= 2.0f * vmax * Tj + vmax * vmax / amax) {
            /* Full 7-segment */
            T[0] = T[2] = T[4] = T[6] = (uint32_t)(Tj * 1000.0f);
            T[1] = T[5] = (uint32_t)(Ta * 1000.0f);
            float d_ramp = 0.5f * jmax * Tj * Tj * Tj      /* jerk-up displacement */
                         + amax * Tj * Ta                  /* const-accel */
                         + 0.5f * amax * Ta * Ta           /* accel at const */
                         + 0.5f * jmax * Tj * Tj * Tj;     /* jerk-down */
            /* 用对称性: 单边加速位移 = ∫₀^Tj+Tj+Ta v(t) dt */
            float v_end_jerk_up = 0.5f * jmax * Tj * Tj;
            float v_end_const = v_end_jerk_up + amax * Ta;
            float d_side = (jmax * Tj * Tj * Tj) / 6.0f                       /* jerk-up */
                         + v_end_jerk_up * Ta + 0.5f * amax * Ta * Ta         /* const */
                         + v_end_const * Tj - 0.5f * amax * Tj * Tj
                         + jmax * Tj * Tj * Tj / 6.0f;                         /* jerk-down */
            float d_const = dist - 2.0f * d_side;
            if (d_const > 0 && vmax > 1e-6f)
                T[3] = (uint32_t)(d_const / vmax * 1000.0f);
        } else {
            /* 无匀速段 (梯形): T[3]=0, 只加速到某峰值速度 → 减速 */
            /* 简化: 计算实际可达到的峰值速度 */
            float vp = (-amax*amax + sqrtf(amax*amax*amax*amax + 4.0f*jmax*jmax*amax*dist))
                     / (2.0f * jmax);
            if (vp > vmax) vp = vmax;
            Ta = (vp - amax * Tj) / amax;
            if (Ta < 0) { Ta = 0; vp = jmax * Tj * Tj; }
            T[0] = T[2] = T[4] = T[6] = (uint32_t)(Tj * 1000.0f);
            T[1] = T[5] = (uint32_t)(Ta * 1000.0f);
            T[3] = 0;
        }
    } else {
        /* Case 2: 达不到 amax — 三角型剖面 */
        float Tv = sqrtf(vmax / jmax);  /* time to reach vmax with pure jerk ramp */
        float d_tri = jmax * Tv * Tv * Tv;  /* displacement for TWO complete jerk ramps */

        if (dist >= d_tri) {
            /* 达到 vmax, 有匀速段, 无匀加速段 */
            float d_const = dist - d_tri;
            float t_const = d_const / vmax;
            T[0] = T[6] = (uint32_t)(Tv * 1000.0f);
            T[3] = (uint32_t)(t_const * 1000.0f);
        } else {
            /* 纯三角: 未达 vmax */
            float t_half = powf(dist / (2.0f * jmax), 1.0f/3.0f);
            T[0] = T[6] = (uint32_t)(t_half * 1000.0f);
        }
    }

    uint32_t total = 0;
    for (int i = 0; i < 7; i++) total += T[i];
    if (total == 0) { T[0] = 1; total = 1; }  /* 防止除零 */
    return total;
}

/**
 * @brief 计算 S曲线比例 s(t) ∈ [0,1] — 解析分段多项式 (精确)
 */
static float scurve_evaluate_one_side(float t_sec, float Tj, float Ta,
                                      float jmax, float amax)
{
    /* 单边 (加速侧) 位移计算: t ∈ [0, Tj+Ta+Tj] */
    float pos = 0, vel = 0, accel = 0;
    float t = 0;
    float active_time = 2.0f * Tj + Ta;

    if (t_sec <= 0) return 0;
    if (t_sec >= active_time) {
        /* 完整单边位移 */
        float v1 = 0.5f * jmax * Tj * Tj;              /* end of phase 0 */
        float a1 = jmax * Tj;
        float s0 = jmax * Tj * Tj * Tj / 6.0f;         /* phase 0: jt³/6 */
        float s1 = v1 * Ta + 0.5f * amax * Ta * Ta;    /* phase 1: v0t + at²/2 */
        float v2 = v1 + amax * Ta;                     /* end of phase 1 */
        float s2 = v2 * Tj + 0.5f * a1 * Tj * Tj
                  - jmax * Tj * Tj * Tj / 6.0f;         /* phase 2: v0t + a0t²/2 - jt³/6 */
        return s0 + s1 + s2;
    }

    /* Phase 0: jerk = +jmax, t ∈ [0, Tj] */
    if (t_sec <= Tj) {
        return jmax * t_sec * t_sec * t_sec / 6.0f;
    }
    float v0 = 0.5f * jmax * Tj * Tj;
    float a0 = jmax * Tj;
    float s0 = jmax * Tj * Tj * Tj / 6.0f;
    t_sec -= Tj;

    /* Phase 1: jerk = 0, a = amax, t ∈ [0, Ta] */
    if (t_sec <= Ta) {
        return s0 + v0 * t_sec + 0.5f * amax * t_sec * t_sec;
    }
    float v1 = v0 + amax * Ta;
    s0 += v0 * Ta + 0.5f * amax * Ta * Ta;
    t_sec -= Ta;

    /* Phase 2: jerk = -jmax, t ∈ [0, Tj] */
    return s0 + v1 * t_sec + 0.5f * a0 * t_sec * t_sec - jmax * t_sec * t_sec * t_sec / 6.0f;
}

static void scurve_evaluate(const uint32_t T[7], uint32_t t_ms, uint32_t total_ms,
                            float jmax, float amax, float vmax, float *s)
{
    if (total_ms == 0 || t_ms >= total_ms) { *s = 1.0f; return; }
    if (t_ms == 0) { *s = 0.0f; return; }

    float t_sec = t_ms * 0.001f;
    float Tj    = T[0] * 0.001f;     /* jerk ramp time */
    float Ta    = T[1] * 0.001f;     /* const accel time */
    float Tv    = T[3] * 0.001f;     /* const vel time */
    float total_sec = total_ms * 0.001f;

    /* 计算总位移 (解析) */
    float d_accel = scurve_evaluate_one_side(Tj + Ta + Tj, Tj, Ta, jmax, amax);
    float d_const = vmax * Tv;
    float d_total = 2.0f * d_accel + d_const;

    /* 判断当前处于加速侧 / 匀速段 / 减速侧 */
    float t_accel_side = Tj + Ta + Tj;

    if (t_sec <= t_accel_side) {
        /* 加速侧 */
        *s = scurve_evaluate_one_side(t_sec, Tj, Ta, jmax, amax) / d_total;
    } else if (t_sec <= t_accel_side + Tv) {
        /* 匀速段 */
        float pos = d_accel + vmax * (t_sec - t_accel_side);
        *s = pos / d_total;
    } else {
        /* 减速侧 → 利用对称性 */
        float t_decel = t_sec - t_accel_side - Tv;
        float d_decel = scurve_evaluate_one_side(t_decel, Tj, Ta, jmax, amax);
        float pos = d_accel + d_const + d_decel;
        *s = pos / d_total;
    }

    if (*s > 1.0f) *s = 1.0f;
    if (*s < 0.0f) *s = 0.0f;
}

/* -------------------------------------------------------------------------- */
/* API 实现                                                                   */
/* -------------------------------------------------------------------------- */

void traj_init(struct traj_planner *tp)
{
    memset(tp, 0, sizeof(*tp));
    tp->state = TRAJ_IDLE;
    tp->type  = TRAJ_NONE;
}

int traj_set_joint_ptp(struct traj_planner *tp, const float target[4],
                       float max_vel, float max_accel, float max_jerk)
{
    if (tp == NULL || target == NULL) return -1;
    if (max_vel <= 0 || max_accel <= 0 || max_jerk <= 0) return -1;

    traj_init(tp);
    tp->type = TRAJ_JOINT_PTP;

    memcpy(tp->target_joints, target, 4 * sizeof(float));

    /* 安全限幅: 使用 arm_config.h 的默认值作为上限 */
    if (max_vel   > TRAJ_DEFAULT_SPEED * 3) max_vel   = TRAJ_DEFAULT_SPEED * 3;
    if (max_accel > TRAJ_DEFAULT_ACCEL * 3) max_accel = TRAJ_DEFAULT_ACCEL * 3;
    if (max_jerk  > TRAJ_DEFAULT_JERK * 3)  max_jerk  = TRAJ_DEFAULT_JERK * 3;

    tp->max_vel   = max_vel;
    tp->max_accel = max_accel;
    tp->max_jerk  = max_jerk;

    return 0;
}

int traj_set_cart_linear(struct traj_planner *tp, const float target_pose[6],
                         float speed, float accel)
{
    if (tp == NULL || target_pose == NULL) return -1;
    if (speed <= 0 || accel <= 0) return -1;

    traj_init(tp);
    tp->type = TRAJ_CART_LINEAR;

    memcpy(tp->target_pose, target_pose, 6 * sizeof(float));
    tp->cart_speed = speed;
    tp->cart_accel = accel;

    return 0;
}

void traj_start(struct traj_planner *tp, uint32_t start_tick,
                const float current[4])
{
    if (tp == NULL || current == NULL) return;

    tp->start_tick = start_tick;
    memcpy(tp->start_joints, current, 4 * sizeof(float));
    memcpy(tp->last_joints, current, 4 * sizeof(float));

    float max_delta = 0;

    if (tp->type == TRAJ_JOINT_PTP) {
        /* 计算各关节位移, 找出主导关节 */
        for (int i = 0; i < 4; i++) {
            tp->delta_angle[i] = tp->target_joints[i] - tp->start_joints[i];
            float ad = fabsf(tp->delta_angle[i]);
            if (ad > max_delta) {
                max_delta = ad;
                tp->dominant_joint = i;
            }
        }

        /* 对主导关节计算 S 曲线 */
        tp->total_duration_ms = scurve_compute(max_delta,
            tp->max_vel, tp->max_accel, tp->max_jerk, tp->phase_duration);
    }
    else if (tp->type == TRAJ_CART_LINEAR) {
        if (arm_fk(current, tp->start_pose) != KIN_OK) {
            tp->state = TRAJ_ERROR;
            return;
        }

        /* 笛卡尔模式: 距离 = ||target - start|| */
        float dx = tp->target_pose[0] - tp->start_pose[0];
        float dy = tp->target_pose[1] - tp->start_pose[1];
        float dz = tp->target_pose[2] - tp->start_pose[2];
        float dist = sqrtf(dx*dx + dy*dy + dz*dz);
        tp->total_duration_ms = scurve_compute(dist,
            tp->cart_speed, tp->cart_accel, tp->cart_accel * 2.0f,
            tp->phase_duration);
    }

    tp->state = TRAJ_RUNNING;
}

enum traj_state traj_update(struct traj_planner *tp, uint32_t elapsed_ms,
                            float output_joints[4])
{
    if (tp == NULL || output_joints == NULL) return TRAJ_ERROR;
    if (tp->state != TRAJ_RUNNING) return tp->state;

    if (elapsed_ms >= tp->total_duration_ms) {
        /* 轨迹完成 */
        if (tp->type == TRAJ_JOINT_PTP) {
            memcpy(output_joints, tp->target_joints, 4 * sizeof(float));
        } else if (tp->type == TRAJ_CART_LINEAR) {
            if (arm_ik(tp->target_pose, tp->last_joints, output_joints) != KIN_OK) {
                tp->state = TRAJ_ERROR;
                return TRAJ_ERROR;
            }
            memcpy(tp->last_joints, output_joints, 4 * sizeof(float));
        }
        tp->state = TRAJ_DONE;
        return TRAJ_DONE;
    }

    if (tp->type == TRAJ_JOINT_PTP) {
        /* 计算 S 曲线比例 s ∈ [0,1] */
        float s;
        scurve_evaluate(tp->phase_duration, elapsed_ms, tp->total_duration_ms,
                        tp->max_jerk, tp->max_accel, tp->max_vel, &s);

        /* 按比例插补所有关节 */
        for (int i = 0; i < 4; i++) {
            output_joints[i] = tp->start_joints[i] + s * tp->delta_angle[i];
        }
        memcpy(tp->last_joints, output_joints, 4 * sizeof(float));
    }
    else if (tp->type == TRAJ_CART_LINEAR) {
        /* 笛卡尔直线插补 */
        float s;
        scurve_evaluate(tp->phase_duration, elapsed_ms, tp->total_duration_ms,
                        tp->cart_accel * 2.0f, tp->cart_accel, tp->cart_speed, &s);

        float interp_pose[6];
        for (int i = 0; i < 6; i++) {
            interp_pose[i] = tp->start_pose[i] +
                             s * (tp->target_pose[i] - tp->start_pose[i]);
        }

        /* IK 求解 */
        if (arm_ik(interp_pose, tp->last_joints, output_joints) != KIN_OK) {
            tp->state = TRAJ_ERROR;
            return TRAJ_ERROR;
        }
        memcpy(tp->last_joints, output_joints, 4 * sizeof(float));
    }

    return TRAJ_RUNNING;
}

void traj_abort(struct traj_planner *tp)
{
    if (tp) {
        tp->state = TRAJ_IDLE;
        tp->type  = TRAJ_NONE;
    }
}
