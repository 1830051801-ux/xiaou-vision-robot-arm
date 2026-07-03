/**
 ******************************************************************************
 * @file    grasp_sequence.c
 * @brief   三段式抓取序列实现
 *
 * 状态流程:
 *   IDLE → [设置目标] → APPROACH(快速接近安全高度)
 *     → DESCEND(慢速下探抓取高度) → GRIP(闭合夹爪+延时)
 *     → LIFT(抬升至安全高度) → DONE
 *
 * 依赖:
 *   - traj_planner: 每段运动的轨迹插补
 *   - arm_kinematics: FK/IK 解算 + 可达性检查
 *   - DrEmpower_can: 伺服电机 CAN 控制 (set_angle, etc.)
 ******************************************************************************
 */

#include "grasp_sequence.h"
#include "trajectory_planner.h"
#include "arm_kinematics.h"
#include "arm_config.h"
#include "DrEmpower_can.h"
#include <string.h>
#include <math.h>

/* 四关节 CAN ID 列表 */
static const uint8_t joint_ids[4] = JOINT_ID_LIST;

/* ========================================================================== */
/* 内部辅助                                                                   */
/* ========================================================================== */

/** 设置四关节角度 (通过 CAN 伺服) */
static void set_joints_can(const float joints[4])
{
    static const float spd_max[4] = JOINT_SPEED_MAX_LIST;
    for (int i = 0; i < 4; i++) {
        set_angle(joint_ids[i], joints[i],
                  spd_max[i] * 0.6f,  /* 60% 速度 */
                  0.0f, 0);  /* 同步模式 */
    }
}

/** 将关节角裁剪到限位内 */
static void clamp_joints(float joints[4])
{
    static const float min_lim[4] = JOINT_ANGLE_MIN_LIST;
    static const float max_lim[4] = JOINT_ANGLE_MAX_LIST;

    for (int i = 0; i < 4; i++) {
        if (joints[i] < min_lim[i]) joints[i] = min_lim[i];
        if (joints[i] > max_lim[i]) joints[i] = max_lim[i];
    }
}

/* ========================================================================== */
/* 公开 API                                                                   */
/* ========================================================================== */

void grasp_seq_init(struct grasp_seq *gs)
{
    memset(gs, 0, sizeof(*gs));
    gs->phase = GRASP_IDLE;
    gs->gripper_id = 5;           /* 默认夹爪 CAN ID=5 */
    gs->grip_delay_ms = 300;      /* 默认夹取后延时 */
}

int grasp_seq_set_target(struct grasp_seq *gs,
                         const float target_pose[6],
                         float z_safe,
                         float speed_fast, float speed_slow,
                         float grip_open, float grip_close, uint32_t grip_delay)
{
    if (gs == NULL || target_pose == NULL) return -1;

    /* 工作空间检查 */
    if (!arm_check_pose(target_pose)) {
        gs->last_error = KIN_ERR_UNREACH;
        return -1;
    }

    /* 存储目标 */
    memcpy(gs->target_pose, target_pose, sizeof(gs->target_pose));
    gs->z_safe          = z_safe;
    gs->z_grab          = target_pose[POSE_Z];  /* Z 分量即为抓取高度 */
    gs->speed_approach  = speed_fast;
    gs->speed_descend   = speed_slow;
    gs->speed_lift      = speed_fast;
    gs->gripper_open    = grip_open;
    gs->gripper_close   = grip_close;
    gs->grip_delay_ms   = grip_delay;
    gs->target_set      = 1;
    gs->grasp_done      = 0;
    gs->last_error      = 0;

    /* 先张开夹爪 (在接近过程中) */
    preset_angle(gs->gripper_id, grip_open, 0.3f, 0.2f, 0);

    return 0;
}

enum grasp_phase grasp_seq_update(struct grasp_seq *gs,
                                   uint32_t elapsed_ms,
                                   const float current_joints[4],
                                   float output_joints[4])
{
    float  pose_safe[6];       /* 安全高度位姿 */
    float  pose_grab[6];       /* 抓取高度位姿 */
    float  current_pose[6];    /* 当前末端位姿 (FK) */
    static struct traj_planner tp_segment;  /* 每段轨迹规划器 */
    static uint8_t  segment_active = 0;
    static uint32_t segment_start_ms = 0;
    static float    segment_start_pose[6];

    if (gs == NULL || output_joints == NULL) return GRASP_ERROR;

    switch (gs->phase) {

    /* ------------------------------------------------------------------ */
    case GRASP_IDLE:
    /* ------------------------------------------------------------------ */
        if (gs->target_set) {
            gs->target_set = 0;
            gs->grasp_done = 0;
            segment_active = 0;

            /* 构建安全高度位姿 */
            memcpy(pose_safe, gs->target_pose, sizeof(pose_safe));
            pose_safe[POSE_Z] = gs->z_safe;

            /* IK 验证 */
            float ik_test[4];
            if (arm_ik(pose_safe, current_joints, ik_test) != KIN_OK) {
                gs->last_error = KIN_ERR_UNREACH;
                gs->phase = GRASP_ERROR;
                memcpy(output_joints, current_joints, 4 * sizeof(float));
                return GRASP_ERROR;
            }

            gs->phase = GRASP_APPROACH;
        }
        memcpy(output_joints, current_joints, 4 * sizeof(float));
        break;

    /* ------------------------------------------------------------------ */
    case GRASP_APPROACH:
    /* ------------------------------------------------------------------ */
        /* 构建安全高度位姿 */
        memcpy(pose_safe, gs->target_pose, sizeof(pose_safe));
        pose_safe[POSE_Z] = gs->z_safe;

        if (!segment_active) {
            /* 开始新轨迹段 */
            traj_init(&tp_segment);
            traj_set_cart_linear(&tp_segment, pose_safe,
                                 gs->speed_approach,
                                 TRAJ_DEFAULT_CART_ACCEL);

            /* 获取当前位姿 (FK) 作为起点 */
            arm_fk(current_joints, current_pose);
            memcpy(segment_start_pose, current_pose, sizeof(segment_start_pose));

            traj_start(&tp_segment, 0, current_joints);
            segment_start_ms = elapsed_ms;
            segment_active = 1;
        }

        {
            uint32_t t = elapsed_ms - segment_start_ms;
            enum traj_state ts = traj_update(&tp_segment, t, output_joints);
            clamp_joints(output_joints);

            if (ts == TRAJ_DONE) {
                segment_active = 0;
                gs->phase = GRASP_DESCEND;
            } else if (ts == TRAJ_ERROR) {
                segment_active = 0;
                gs->last_error = -1;
                gs->phase = GRASP_ERROR;
                memcpy(output_joints, current_joints, 4 * sizeof(float));
                return GRASP_ERROR;
            }
        }
        break;

    /* ------------------------------------------------------------------ */
    case GRASP_DESCEND:
    /* ------------------------------------------------------------------ */
        /* 构建抓取高度位姿 */
        memcpy(pose_grab, gs->target_pose, sizeof(pose_grab));
        pose_grab[POSE_Z] = gs->z_grab;

        if (!segment_active) {
            traj_init(&tp_segment);
            traj_set_cart_linear(&tp_segment, pose_grab,
                                 gs->speed_descend,
                                 TRAJ_DEFAULT_CART_ACCEL * 0.5f);

            arm_fk(current_joints, current_pose);
            memcpy(segment_start_pose, current_pose, sizeof(segment_start_pose));

            traj_start(&tp_segment, 0, current_joints);
            segment_start_ms = elapsed_ms;
            segment_active = 1;
        }

        {
            uint32_t t = elapsed_ms - segment_start_ms;
            enum traj_state ts = traj_update(&tp_segment, t, output_joints);
            clamp_joints(output_joints);

            if (ts == TRAJ_DONE) {
                segment_active = 0;
                gs->phase = GRASP_GRIP;
                gs->grip_start_tick = elapsed_ms;
            } else if (ts == TRAJ_ERROR) {
                segment_active = 0;
                gs->last_error = -1;
                gs->phase = GRASP_ERROR;
                memcpy(output_joints, current_joints, 4 * sizeof(float));
                return GRASP_ERROR;
            }
        }
        break;

    /* ------------------------------------------------------------------ */
    case GRASP_GRIP:
    /* ------------------------------------------------------------------ */
        /* 闭合夹爪 (仅首次进入时发送指令) */
        if ((elapsed_ms - gs->grip_start_tick) < 10) {
            set_angle(gs->gripper_id, gs->gripper_close, 30.0f, 0.3f, 0);
        }

        /* 等待延时 */
        if ((elapsed_ms - gs->grip_start_tick) >= gs->grip_delay_ms) {
            gs->phase = GRASP_LIFT;
        }
        memcpy(output_joints, current_joints, 4 * sizeof(float));
        break;

    /* ------------------------------------------------------------------ */
    case GRASP_LIFT:
    /* ------------------------------------------------------------------ */
        /* 构建安全高度位姿 (抬升) */
        memcpy(pose_safe, gs->target_pose, sizeof(pose_safe));
        pose_safe[POSE_Z] = gs->z_safe;

        if (!segment_active) {
            traj_init(&tp_segment);
            traj_set_cart_linear(&tp_segment, pose_safe,
                                 gs->speed_lift,
                                 TRAJ_DEFAULT_CART_ACCEL);

            arm_fk(current_joints, current_pose);
            memcpy(segment_start_pose, current_pose, sizeof(segment_start_pose));

            traj_start(&tp_segment, 0, current_joints);
            segment_start_ms = elapsed_ms;
            segment_active = 1;
        }

        {
            uint32_t t = elapsed_ms - segment_start_ms;
            enum traj_state ts = traj_update(&tp_segment, t, output_joints);
            clamp_joints(output_joints);

            if (ts == TRAJ_DONE) {
                segment_active = 0;
                gs->grasp_done = 1;
                gs->phase = GRASP_DONE;
            } else if (ts == TRAJ_ERROR) {
                segment_active = 0;
                gs->last_error = -1;
                gs->phase = GRASP_ERROR;
                memcpy(output_joints, current_joints, 4 * sizeof(float));
                return GRASP_ERROR;
            }
        }
        break;

    /* ------------------------------------------------------------------ */
    case GRASP_DONE:
    case GRASP_ERROR:
    /* ------------------------------------------------------------------ */
        memcpy(output_joints, current_joints, 4 * sizeof(float));
        break;
    }

    return gs->phase;
}

void grasp_seq_abort(struct grasp_seq *gs)
{
    if (gs == NULL) return;

    /* 急停所有关节 */
    for (int i = 0; i < 4; i++) {
        estop(joint_ids[i]);
    }
    /* 夹爪也急停 */
    estop(gs->gripper_id);

    gs->phase = GRASP_IDLE;
    gs->target_set = 0;
    gs->grasp_done = 0;
}
