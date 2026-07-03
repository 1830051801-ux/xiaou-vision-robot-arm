/**
 ******************************************************************************
 * @file    calibration.c
 * @brief   标定模块实现
 * @date    2026-06-24
 ******************************************************************************
 */

#include "calibration.h"
#include "calib_defaults.h"
#include "DrEmpower_can.h"
#include <rtthread.h>
#include <math.h>
#include <string.h>

/* -------------------------------------------------------------------------- */
/* 外部依赖 — CAN 互斥量 (app_threads.c 中定义)                               */
/* -------------------------------------------------------------------------- */
extern rt_mutex_t mutex_can_tx;
extern rt_mutex_t mutex_can_rx;

/* -------------------------------------------------------------------------- */
/* 公开 API 实现                                                              */
/* -------------------------------------------------------------------------- */

/**
 * @brief 初始化标定模块
 */
void calib_init(struct calib_ctx *ctx, float drift_threshold)
{
    if (ctx == NULL) return;

    memset(ctx, 0, sizeof(*ctx));
    ctx->state          = CALIB_ZERO_UNCHECKED;
    ctx->drift_threshold = (drift_threshold > 0.0f)
                           ? drift_threshold : CALIB_ZERO_DRIFT_DEG;
    ctx->calib_joint_id = 0;
    ctx->check_rounds   = 0;

    /* 系统原点偏移初始化为 0 (未标定) */
    ctx->base_offset[0] = CALIB_BASE_OFFSET_X_DEFAULT;
    ctx->base_offset[1] = CALIB_BASE_OFFSET_Y_DEFAULT;
    ctx->base_offset[2] = CALIB_BASE_OFFSET_Z_DEFAULT;
}

/**
 * @brief 启动上电零点校验
 */
void calib_start_check(struct calib_ctx *ctx)
{
    if (ctx == NULL) return;

    ctx->state            = CALIB_ZERO_CHECKING;
    ctx->check_start_tick = rt_tick_get();
    ctx->check_rounds     = 0;

    for (int i = 0; i < 4; i++) {
        ctx->zero_valid[i] = 0;
    }
}

/**
 * @brief 零点校验更新
 */
int calib_check_update(struct calib_ctx *ctx, const float joint_angles[4],
                       uint32_t now_tick)
{
    if (ctx == NULL || joint_angles == NULL) return -1;
    if (ctx->state != CALIB_ZERO_CHECKING)   return 0;

    /* 等待足够的数据轮次以稳定读数 */
    ctx->check_rounds++;
    if (ctx->check_rounds < CALIB_CHECK_MIN_ROUNDS) {
        return 0;  /* 数据不足, 继续等待 */
    }

    /* 逐一检查各关节 */
    int all_ok = 1;
    for (int i = 0; i < 4; i++) {
        if (fabsf(joint_angles[i]) <= ctx->drift_threshold) {
            ctx->zero_valid[i] = 1;
        } else {
            all_ok = 0;
            ctx->zero_valid[i] = 0;
        }
    }

    /* 全部通过 */
    if (all_ok) {
        ctx->state = CALIB_ZERO_OK;
        return 1;
    }

    /* 超时仍未通过 */
    uint32_t elapsed = now_tick - ctx->check_start_tick;
    if (elapsed > rt_tick_from_millisecond(CALIB_CHECK_TIMEOUT_MS)) {
        ctx->state = CALIB_ZERO_LOST;
        return -1;
    }

    /* 仍在等待: 数据持续到来但尚未全部归零, 继续重试 */
    return 0;
}

/**
 * @brief 进入单关节标定模式 (助力拖拽)
 */
int calib_start_joint(struct calib_ctx *ctx, uint8_t joint_id)
{
    if (ctx == NULL) return -1;
    if (ctx->state != CALIB_ZERO_OK && ctx->state != CALIB_CALIBRATING) {
        return -1;
    }
    if (joint_id < 1 || joint_id > 4) return -1;

    ctx->state          = CALIB_CALIBRATING;
    ctx->calib_joint_id = joint_id;

    /* 对该关节使能助力控制, 便于人手拖动到机械零位
     * angle=0: 助力朝向 0° 方向
     * speed_limit: 防止助力引起持续加速
     * angle_err/speed_err: 灵敏度参数
     * torque: 助力力矩, 应小于人力可克服的范围内
     */
    rt_mutex_take(mutex_can_tx, RT_WAITING_FOREVER);
    rt_mutex_take(mutex_can_rx, RT_WAITING_FOREVER);
    motion_aid(joint_id,
               0.0f,                    /* 助力目标角度 = 0° (零位) */
               CALIB_AID_SPEED_RPM,     /* 限定转速 */
               CALIB_AID_ANGLE_ERR_DEG, /* 角度灵敏度 */
               CALIB_AID_SPEED_ERR_RPM, /* 转速灵敏度 */
               CALIB_AID_TORQUE_NM);    /* 助力力矩 */
    rt_mutex_release(mutex_can_rx);
    rt_mutex_release(mutex_can_tx);

    return 0;
}

/**
 * @brief 确认写入零点并回读校验
 */
int calib_confirm_zero(struct calib_ctx *ctx, uint8_t joint_id)
{
    if (ctx == NULL) return -2;
    if (ctx->state != CALIB_CALIBRATING) return -2;
    if (joint_id < 1 || joint_id > 4) return -2;

    int success = 0;

    for (int retry = 0; retry <= CALIB_SET_ZERO_RETRY; retry++) {
        /* 写入永久零点 */
        rt_mutex_take(mutex_can_tx, RT_WAITING_FOREVER);
        rt_mutex_take(mutex_can_rx, RT_WAITING_FOREVER);
        set_zero_position(joint_id);
        rt_mutex_release(mutex_can_rx);
        rt_mutex_release(mutex_can_tx);

        /* 等待关节内部 Flash 写入完成 + 编码器稳定 */
        rt_thread_mdelay(CALIB_SET_ZERO_SETTLE_MS);

        /* 回读校验 */
        rt_mutex_take(mutex_can_tx, RT_WAITING_FOREVER);
        rt_mutex_take(mutex_can_rx, RT_WAITING_FOREVER);
        float angle = get_angle(joint_id);
        rt_mutex_release(mutex_can_rx);
        rt_mutex_release(mutex_can_tx);

        if (fabsf(angle) <= ctx->drift_threshold) {
            success = 1;
            break;
        }
    }

    if (!success) {
        ctx->zero_valid[joint_id - 1] = 0;
        return -1;  /* 回读校验失败 */
    }

    /* 单个关节零点确认 */
    ctx->zero_valid[joint_id - 1] = 1;

    /* 检查是否全部 4 关节零点有效 */
    int all_valid = 1;
    for (int i = 0; i < 4; i++) {
        if (!ctx->zero_valid[i]) {
            all_valid = 0;
            break;
        }
    }

    return all_valid ? 1 : 0;
}

/**
 * @brief 退出标定模式
 */
void calib_end(struct calib_ctx *ctx)
{
    if (ctx == NULL) return;

    /* 如果全部关节零点有效, 恢复正常 */
    int all_ok = 1;
    for (int i = 0; i < 4; i++) {
        if (!ctx->zero_valid[i]) {
            all_ok = 0;
            break;
        }
    }

    ctx->state          = all_ok ? CALIB_ZERO_OK : CALIB_ZERO_LOST;
    ctx->calib_joint_id = 0;
}

/**
 * @brief 设置系统原点偏移
 */
void calib_set_base_offset(struct calib_ctx *ctx, float x, float y, float z)
{
    if (ctx == NULL) return;

    ctx->base_offset[0] = x;
    ctx->base_offset[1] = y;
    ctx->base_offset[2] = z;
}

/**
 * @brief 应用系统原点偏移: 视觉坐标 → 机械臂坐标
 */
void calib_apply_base_offset(const float base_offset[3], float pose[6])
{
    if (base_offset == NULL || pose == NULL) return;

    for (int i = 0; i < 3; i++) {
        pose[i] -= base_offset[i];
    }
}

/**
 * @brief 取消系统原点偏移: 机械臂坐标 → 视觉坐标
 */
void calib_unapply_base_offset(const float base_offset[3], float pose[6])
{
    if (base_offset == NULL || pose == NULL) return;

    for (int i = 0; i < 3; i++) {
        pose[i] += base_offset[i];
    }
}

/**
 * @brief 记录空载力矩基线
 */
void calib_record_torque_baseline(struct calib_ctx *ctx, const float torques[4])
{
    if (ctx == NULL || torques == NULL) return;

    for (int i = 0; i < 4; i++) {
        ctx->torque_baseline[i] = torques[i];
    }
}
