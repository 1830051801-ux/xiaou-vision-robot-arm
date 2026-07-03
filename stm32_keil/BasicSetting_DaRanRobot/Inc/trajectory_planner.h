/**
 ******************************************************************************
 * @file    trajectory_planner.h
 * @brief   机械臂轨迹规划器 — S曲线 / 梯形 / 笛卡尔直线
 * @author  YaowenLi
 * @date    2026-06-18
 ******************************************************************************
 * @attention
 * 轨迹类型:
 *   - TRAJ_JOINT_PTP:    关节空间点到点 (S曲线速度剖面, 同步到达)
 *   - TRAJ_CART_LINEAR:  笛卡尔空间直线 (IK 逐点插值)
 *
 * 状态机: IDLE → RUNNING → DONE / ERROR
 * 插补周期: TRAJ_INTERPOLATION_MS (在 arm_config.h 中配置)
 ******************************************************************************
 */

#ifndef __TRAJECTORY_PLANNER_H__
#define __TRAJECTORY_PLANNER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* -------------------------------------------------------------------------- */
/* 轨迹状态 / 类型                                                            */
/* -------------------------------------------------------------------------- */
enum traj_state {
    TRAJ_IDLE    = 0,
    TRAJ_RUNNING = 1,
    TRAJ_DONE    = 2,
    TRAJ_ERROR   = 3,
};

enum traj_type {
    TRAJ_NONE          = 0,
    TRAJ_JOINT_PTP     = 1,   /* 关节空间 PTP (S曲线) */
    TRAJ_CART_LINEAR   = 2,   /* 笛卡尔直线 */
};

/* -------------------------------------------------------------------------- */
/* S曲线阶段 (7段)                                                            */
/* -------------------------------------------------------------------------- */
enum scurve_phase {
    PHASE_ACCEL_JERK_UP    = 0,  /* 加加速段  (+jerk) */
    PHASE_ACCEL_CONST      = 1,  /* 匀加速段  */
    PHASE_ACCEL_JERK_DOWN  = 2,  /* 减加速段  (-jerk) */
    PHASE_VEL_CONST        = 3,  /* 匀速段    */
    PHASE_DECEL_JERK_UP    = 4,  /* 加减速段  (-jerk) */
    PHASE_DECEL_CONST      = 5,  /* 匀减速段  */
    PHASE_DECEL_JERK_DOWN  = 6,  /* 减减速段  (+jerk) */
    PHASE_COUNT = 7,
};

/* -------------------------------------------------------------------------- */
/* 轨迹规划器结构体                                                           */
/* -------------------------------------------------------------------------- */
struct traj_planner {
    enum traj_state state;              /* 当前状态 */
    enum traj_type  type;               /* 轨迹类型 */

    uint32_t start_tick;               /* 开始时刻 (rt_tick_get) */
    uint32_t total_duration_ms;        /* 总时长 (ms) */

    float start_joints[4];             /* 起始关节角度 (°) */
    float target_joints[4];            /* 目标关节角度 (°) */
    float delta_angle[4];              /* 角度差 (°) = target - start */

    /* 同步参数: 所有关节以最慢关节的时长统一插补 */
    uint8_t  dominant_joint;           /* 主导关节 (行程最大的) */

    /* S曲线参数 (对主导关节) */
    float    max_vel;                  /* 最大速度 (°/s) */
    float    max_accel;               /* 最大加速度 (°/s²) */
    float    max_jerk;                /* 最大加加速度 (°/s³) */
    uint32_t phase_duration[7];       /* 每个 S 曲线阶段的时长 (ms) */

    /* 笛卡尔模式 */
    float start_pose[6];               /* 起始位姿 */
    float target_pose[6];              /* 目标位姿 */
    float last_joints[4];              /* CART模式最近一次有效IK解 */
    float cart_speed;                  /* 线速度 (mm/s) */
    float cart_accel;                  /* 线加速度 (mm/s²) */
};

/* -------------------------------------------------------------------------- */
/* API                                                                        */
/* -------------------------------------------------------------------------- */

/**
 * @brief 初始化轨迹规划器 (空闲状态)
 */
void traj_init(struct traj_planner *tp);

/**
 * @brief 配置关节空间 PTP 轨迹 (S曲线同步)
 * @param tp        轨迹规划器实例
 * @param target    目标关节角度[4] (°)
 * @param max_vel   最大转速 (°/s) — 主导关节
 * @param max_accel 最大角加速度 (°/s²)
 * @param max_jerk  最大加加速度 (°/s³)
 * @return 0=成功, -1=参数非法
 */
int traj_set_joint_ptp(struct traj_planner *tp, const float target[4],
                       float max_vel, float max_accel, float max_jerk);

/**
 * @brief 配置笛卡尔直线轨迹
 * @param tp        轨迹规划器实例
 * @param target_pose  目标位姿[6]
 * @param speed     线速度 (mm/s)
 * @param accel     线加速度 (mm/s²)
 * @return 0=成功, -1=参数非法
 */
int traj_set_cart_linear(struct traj_planner *tp, const float target_pose[6],
                         float speed, float accel);

/**
 * @brief 开始执行轨迹 (设置时间基准)
 * @param tp         轨迹规划器实例
 * @param start_tick 起始 tick (通常传入 rt_tick_get())
 * @param current    当前关节角度[4] (°)
 */
void traj_start(struct traj_planner *tp, uint32_t start_tick,
                const float current[4]);

/**
 * @brief 轨迹更新 (每插补周期调用一次)
 * @param tp            轨迹规划器实例
 * @param elapsed_ms    从 traj_start 起经过的时间 (ms)
 * @param output_joints 输出: 当前插补关节角度[4] (°)
 * @return 当前状态 (TRAJ_RUNNING / TRAJ_DONE / TRAJ_ERROR)
 */
enum traj_state traj_update(struct traj_planner *tp, uint32_t elapsed_ms,
                            float output_joints[4]);

/**
 * @brief 中止轨迹
 */
void traj_abort(struct traj_planner *tp);

/**
 * @brief 轨迹是否已完成
 */
static inline int traj_is_done(struct traj_planner *tp) {
    return tp->state == TRAJ_DONE || tp->state == TRAJ_ERROR;
}

#ifdef __cplusplus
}
#endif

#endif /* __TRAJECTORY_PLANNER_H__ */
