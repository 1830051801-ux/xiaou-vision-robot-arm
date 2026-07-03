/**
 ******************************************************************************
 * @file    grasp_sequence.h
 * @brief   三段式抓取序列执行器
 *          APPROACH → DESCEND → GRIP → LIFT → DONE
 *
 * 依赖: traj_planner (轨迹), arm_kinematics (逆运动学), DrEmpower_can (伺服)
 *
 * 用法:
 *   1. grasp_seq_set_target(&gs, target_pose, current_joints);
 *   2. 每个插补周期调用 grasp_seq_update(&gs, elapsed_ms, current, output);
 *   3. 检查 output 状态: DONE = 完成, ERROR = 失败, RUNNING = 继续
 ******************************************************************************
 */

#ifndef __GRASP_SEQUENCE_H__
#define __GRASP_SEQUENCE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* -------------------------------------------------------------------------- */
/* 抓取序列阶段                                                               */
/* -------------------------------------------------------------------------- */
enum grasp_phase {
    GRASP_IDLE      = 0,   /* 空闲, 等待目标 */
    GRASP_APPROACH  = 1,   /* 接近: 快速移动到目标上方安全高度 */
    GRASP_DESCEND   = 2,   /* 下探: 慢速垂直下降到抓取高度 */
    GRASP_GRIP      = 3,   /* 夹取: 闭合夹爪 */
    GRASP_LIFT      = 4,   /* 抬升: 垂直上升到安全高度 */
    GRASP_DONE      = 5,   /* 完成 */
    GRASP_ERROR     = 6,   /* 异常 (不可达/超限) */
};

/* -------------------------------------------------------------------------- */
/* 抓取序列上下文                                                             */
/* -------------------------------------------------------------------------- */
struct grasp_seq {
    enum grasp_phase  phase;            /* 当前阶段 */

    /* 目标位姿 (世界坐标 mm + 旋转角°) */
    float target_pose[6];               /* {x, y, z_grab, roll, pitch, yaw} */

    /* 高度参数 (世界坐标 mm) */
    float z_safe;                       /* 安全高度 (接近/抬升至此处) */
    float z_grab;                       /* 抓取高度 (下探至此) */

    /* 速度配置 */
    float speed_approach;               /* 接近速度 (mm/s) */
    float speed_descend;                /* 下探速度 (mm/s, 较慢) */
    float speed_lift;                   /* 抬升速度 (mm/s) */

    /* 夹爪参数 */
    uint8_t gripper_id;                 /* 夹爪舵机 CAN ID */
    float    gripper_open;              /* 夹爪张开角度 (°) */
    float    gripper_close;             /* 夹爪闭合角度 (°) */
    uint32_t grip_delay_ms;             /* 夹取稳定延时 (ms) */
    uint32_t grip_start_tick;           /* 夹取开始时刻 */

    /* 状态检查 */
    uint8_t target_set;                 /* 目标已设置标志 */
    uint8_t grasp_done;                 /* 外部查询: 抓取是否完成 */
    uint8_t last_error;                 /* 最后一次错误码 */
};

/* -------------------------------------------------------------------------- */
/* API                                                                        */
/* -------------------------------------------------------------------------- */

/**
 * @brief 初始化抓取序列 (回到 IDLE)
 */
void grasp_seq_init(struct grasp_seq *gs);

/**
 * @brief 设置抓取目标
 * @param gs           抓取序列实例
 * @param target_pose  目标位姿 {x,y,z_grab,roll,pitch,yaw} (世界坐标 mm/°)
 * @param z_safe       安全接近高度 (mm)
 * @param speed_fast   快速段速度 (mm/s, 接近+抬升)
 * @param speed_slow   慢速段速度 (mm/s, 下探)
 * @param grip_open    夹爪张开角度 (°)
 * @param grip_close   夹爪闭合角度 (°)
 * @param grip_delay   夹取后稳定延时 (ms)
 * @return 0=成功, -1=参数非法
 */
int grasp_seq_set_target(struct grasp_seq *gs,
                         const float target_pose[6],
                         float z_safe,
                         float speed_fast, float speed_slow,
                         float grip_open, float grip_close, uint32_t grip_delay);

/**
 * @brief 抓取序列更新 (每个插补周期调用)
 * @param gs             抓取序列实例
 * @param elapsed_ms     从序列开始起的累计时间 (ms)
 * @param current_joints 当前关节角度[4] (°)
 * @param output_joints  输出: 本周期目标关节角度[4] (°)
 * @return 当前阶段 (GRASP_DONE/GRASP_ERROR/GRASP_RUNNING/GRASP_IDLE)
 */
enum grasp_phase grasp_seq_update(struct grasp_seq *gs,
                                   uint32_t elapsed_ms,
                                   const float current_joints[4],
                                   float output_joints[4]);

/**
 * @brief 中止抓取 (回到 IDLE)
 */
void grasp_seq_abort(struct grasp_seq *gs);

#ifdef __cplusplus
}
#endif

#endif /* __GRASP_SEQUENCE_H__ */
