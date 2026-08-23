/**
 ******************************************************************************
 * @file    trajectory.h
 * @brief   轨迹模块 — 6轴轨迹点校验 / 16槽FIFO / 原子入队 / 到位检测
 * @date    2026-08-08 (Pi v1.2 架构决策: 从 app_threads.c 最小拆分)
 ******************************************************************************
 * @attention
 * 本模块与具体 CAN 驱动/线程解耦:
 *   - 校验函数为纯函数，UART 线程、控制线程均可调用
 *   - FIFO 操作为临界区，内部加锁
 *   - 到位检测为纯函数，由 joint_ctrl 线程每周期调用
 *
 * 线程规则 (Pi v1.2):
 *   - UART 线程只校验+入队，不发 CAN
 *   - joint_ctrl 线程是唯一出队并下发 CAN 的位置
 ******************************************************************************
 */

#ifndef __TRAJECTORY_H__
#define __TRAJECTORY_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "arm_config.h"

/* ---- FIFO 容量 ---- */
#define TRAJ_FIFO_CAPACITY   TRAJ_FIFO_SIZE   /* arm_config.h: 16 */

/* ---- 到位容差 ---- */
#define TRAJ_TOLERANCE_DEG   TRAJ_ARRIVAL_TOLERANCE  /* arm_config.h: 0.5° */

/* ---- 段时长限制 ---- */
#define TRAJ_DUR_MIN_MS      TRAJ_DURATION_MIN_MS
#define TRAJ_DUR_MAX_MS      TRAJ_DURATION_MAX_MS

/* ---- 批量最大点数 ---- */
#define TRAJ_BATCH_MAX       TRAJ_BATCH_MAX_POINTS

/* -------------------------------------------------------------------------- */
/* 校验结果                                                                   */
/* -------------------------------------------------------------------------- */
enum traj_err {
    TRAJ_OK             =  0,   /* 通过 */
    TRAJ_ERR_LEN        = -1,   /* 载荷长度非法 */
    TRAJ_ERR_NAN_INF    = -2,   /* 角度含 NaN/Inf */
    TRAJ_ERR_LIMIT      = -3,   /* 角度超限 */
    TRAJ_ERR_DURATION   = -4,   /* 时长非法 (0 或 >10s) */
    TRAJ_ERR_FIFO_FULL  = -5,   /* FIFO 空间不足 */
    TRAJ_ERR_ESTOP      = -6,   /* 急停激活 */
    TRAJ_ERR_BUSY       = -7,   /* 系统繁忙 (标定/校验中) */
};

/* -------------------------------------------------------------------------- */
/* 6 关节角度限位 (有效限位 = min(物理, 操作), 来自 arm_config.h)              */
/* -------------------------------------------------------------------------- */
extern const float traj_effective_min[MAX_JOINT_COUNT];
extern const float traj_effective_max[MAX_JOINT_COUNT];

/* -------------------------------------------------------------------------- */
/* v2: Quintic 插值状态 (10ms 每周期)                                           */
/* -------------------------------------------------------------------------- */
struct traj_interp_state {
    float    q_start[MAX_JOINT_COUNT];    /* 段起始关节角度 (模型域) */
    float    q_target[MAX_JOINT_COUNT];   /* 段目标关节角度 (模型域) */
    float    q_raw[MAX_JOINT_COUNT];      /* 当前插值结果 (原始域, 待发送) */
    uint16_t duration_ms;                 /* 段总时长 */
    uint16_t elapsed_ms;                  /* 已过时间 */
    uint8_t  active;                      /* 插值进行中 */
    uint8_t  joint_skip[MAX_JOINT_COUNT]; /* 本段无需运动的关节 */
};

/* -------------------------------------------------------------------------- */
/* FIFO 操作 (临界区, 内部加锁)                                               */
/* -------------------------------------------------------------------------- */

void     traj_fifo_init(void);
void     traj_fifo_clear(void);
uint8_t  traj_fifo_free(void);
uint8_t  traj_fifo_count(void);

/* 单点入队: 校验 + 原子写入, 返回 traj_err */
int      traj_enqueue_single(const uint8_t payload[26], int check_estop);

/* 批量入队: 先校验全部 N 点, 全通过后原子写入, 返回入队数量或错误码 */
int      traj_enqueue_batch(const uint8_t *payload, uint8_t n, int check_estop);

/* 出队: joint_ctrl 调用, 取出队首轨迹点. 返回 0=成功, -1=空 */
int      traj_dequeue(float angles_out[MAX_JOINT_COUNT], uint16_t *duration_ms_out);

/* -------------------------------------------------------------------------- */
/* 纯校验函数 (无副作用, 任何线程可调用)                                      */
/* -------------------------------------------------------------------------- */

/* 校验单个轨迹点 payload (26B), 返回 traj_err */
int      traj_validate(const uint8_t payload[26]);

/* 检查所有在线关节是否已到达目标角度. 返回 1=全部到位, 0=未到位 */
int      traj_all_arrived(const float current_deg[MAX_JOINT_COUNT],
                          const float target_deg[MAX_JOINT_COUNT],
                          const uint8_t online[MAX_JOINT_COUNT]);

/* 检查 float 是否有限 (非 NaN 非 Inf), 返回 1=有限, 0=非法 */
int      traj_is_finite(float v);

/* v2: 启动单个轨迹点 — 计算每关节独立转速, 下发 CAN 指令
 * target_model_deg: 6 关节模型域目标角度
 * duration_ms:     段时长 (ms)
 * joint_online:    6 关节在线标志
 * 返回 0=成功, <0=错误码 (TRAJ_ERR_*) */
int      traj_start_point(const float target_model_deg[MAX_JOINT_COUNT],
                          uint16_t duration_ms,
                          const uint8_t joint_online[MAX_JOINT_COUNT]);

/* v2: Quintic 插值 — 每 10ms 计算一次插值点, 发送 CAN set_angle(mode=0)
 * 返回 0=进行中, 1=段到位, <0=错误 (中止) */
int      traj_interp_tick(struct traj_interp_state *state);

#ifdef __cplusplus
}
#endif

#endif /* __TRAJECTORY_H__ */
