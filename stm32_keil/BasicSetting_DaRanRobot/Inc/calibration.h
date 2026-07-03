/**
 ******************************************************************************
 * @file    calibration.h
 * @brief   标定模块 — 关节原点校验 / 标定工作流 / 系统原点管理
 * @date    2026-06-24
 ******************************************************************************
 * @attention
 * 本模块管理三个层次的原点逻辑:
 *   1. 上电零点校验 — 启动时检查各关节角度是否在阈值内
 *   2. 单关节标定 — 助力拖拽 → 写入零点 → 回读确认
 *   3. 系统原点偏移 — 标定诊断工具, 记录 FK 与视觉输出的差值供手眼标定参考
 *      (正常运行时 base_offset 应为 {0,0,0}, 坐标对齐由树莓派手眼矩阵完成)
 *
 * 状态机:
 *   ZERO_UNCHECKED → ZERO_CHECKING → ZERO_OK (→ 正常运动)
 *                                   → ZERO_LOST (→ 锁定 + 报错)
 *   ZERO_OK ↔ CALIBRATING (标定模式)
 *
 * 依赖: DrEmpower_can.h (motion_aid, set_zero_position, get_angle)
 *       calib_defaults.h (阈值参数)
 *       RT-Thread (mutex, tick)
 ******************************************************************************
 */

#ifndef __CALIBRATION_H__
#define __CALIBRATION_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* -------------------------------------------------------------------------- */
/* 标定状态枚举                                                               */
/* -------------------------------------------------------------------------- */
enum calib_state {
    CALIB_ZERO_UNCHECKED = 0,   /* 尚未校验 */
    CALIB_ZERO_CHECKING  = 1,   /* 校验进行中 */
    CALIB_ZERO_OK        = 2,   /* 零点正常 */
    CALIB_ZERO_LOST      = 3,   /* 零点丢失 */
    CALIB_CALIBRATING    = 4,   /* 标定模式 */
};

/* -------------------------------------------------------------------------- */
/* 标定上下文                                                                 */
/* -------------------------------------------------------------------------- */
struct calib_ctx {
    enum calib_state state;             /* 当前状态 */
    uint8_t  calib_joint_id;            /* 当前标定关节 ID (1~4, 0=无) */
    float    drift_threshold;           /* 单关节漂移判定阈值 (°) */
    float    base_offset[3];            /* 系统原点补偿 (x, y, z, mm) */
    float    torque_baseline[4];        /* 空载力矩基线 (Nm, 零点校验通过时记录) */
    uint32_t check_start_tick;          /* 零点校验开始时刻 (rt_tick) */
    uint8_t  zero_valid[4];            /* 各关节零点有效标志 (索引 0~3 对应 J1~J4) */
    uint8_t  check_rounds;             /* 已收到的数据轮次计数 */
};

/* -------------------------------------------------------------------------- */
/* API                                                                        */
/* -------------------------------------------------------------------------- */

/**
 * @brief 初始化标定模块
 * @param ctx            标定上下文 (由调用方分配)
 * @param drift_threshold 零点漂移判定阈值 (°), 传 0 使用默认值
 */
void calib_init(struct calib_ctx *ctx, float drift_threshold);

/**
 * @brief 启动上电零点校验
 * @param ctx  标定上下文
 * @note  状态切换: → CALIB_ZERO_CHECKING
 */
void calib_start_check(struct calib_ctx *ctx);

/**
 * @brief 零点校验更新 — 每个控制周期调用一次
 * @param ctx           标定上下文
 * @param joint_angles  当前 4 关节角度 (°)
 * @param now_tick      当前系统 tick
 * @return  1 = 校验通过 (→ ZERO_OK)
 *          0 = 校验中 (继续等待数据)
 *         -1 = 零点丢失 (→ ZERO_LOST)
 */
int  calib_check_update(struct calib_ctx *ctx, const float joint_angles[4],
                        uint32_t now_tick);

/**
 * @brief 进入单关节标定模式 (调 motion_aid 助力)
 * @param ctx       标定上下文
 * @param joint_id  目标关节 ID (1~4)
 * @return  0 = 成功进入标定模式
 *         -1 = 当前状态不允许标定
 */
int  calib_start_joint(struct calib_ctx *ctx, uint8_t joint_id);

/**
 * @brief 确认写入零点并回读校验
 * @param ctx       标定上下文
 * @param joint_id  目标关节 ID (1~4)
 * @return  1 = 写入成功且全部 4 关节零点有效
 *          0 = 写入成功, 本关节有效
 *         -1 = 回读校验失败 (角度偏差超阈值)
 *         -2 = 当前不在标定模式
 */
int  calib_confirm_zero(struct calib_ctx *ctx, uint8_t joint_id);

/**
 * @brief 退出标定模式
 * @param ctx  标定上下文
 * @note  状态切换: CALIB_CALIBRATING → CALIB_ZERO_OK
 */
void calib_end(struct calib_ctx *ctx);

/**
 * @brief 设置系统原点偏移
 * @param ctx  标定上下文
 * @param x    X 方向偏移 (mm)
 * @param y    Y 方向偏移 (mm)
 * @param z    Z 方向偏移 (mm)
 */
void calib_set_base_offset(struct calib_ctx *ctx, float x, float y, float z);

/**
 * @brief 应用系统原点偏移到目标位姿 (视觉坐标 → 机械臂坐标)
 *        IK 输入前调用: 视觉目标坐标减去系统原点偏移
 * @param base_offset  系统原点偏移 [x, y, z]
 * @param pose         位姿 [x, y, z, rx, ry, rz], 原地修改前 3 元素
 */
void calib_apply_base_offset(const float base_offset[3], float pose[6]);

/**
 * @brief 取消系统原点偏移 (机械臂坐标 → 视觉坐标)
 *        FK 输出后调用: 机械臂坐标加上系统原点偏移
 * @param base_offset  系统原点偏移 [x, y, z]
 * @param pose         位姿 [x, y, z, rx, ry, rz], 原地修改前 3 元素
 */
void calib_unapply_base_offset(const float base_offset[3], float pose[6]);

/**
 * @brief 记录空载力矩基线
 * @param ctx      标定上下文
 * @param torques  4 关节力矩 (Nm)
 */
void calib_record_torque_baseline(struct calib_ctx *ctx, const float torques[4]);

#ifdef __cplusplus
}
#endif

#endif /* __CALIBRATION_H__ */
