/**
 ******************************************************************************
 * @file    arm_config.h
 * @brief   4轴机械臂 MCU 侧配置参数 — 精简版
 * @author  YaowenLi
 * @date    2026-07-27 (架构重构: 运动学/轨迹参数迁移至树莓派 ROS2)
 ******************************************************************************
 * @attention
 * 本文件仅保留 MCU 执行层所需的配置参数:
 *   1. 关节 CAN ID 分配
 *   2. 关节角度 / 转速 / 力矩限位 (安全保护用)
 *   3. 安全保护阈值
 *   4. 系统杂项参数
 *
 * DH 参数、工作空间、轨迹默认值、重力参数等已迁移至树莓派 ROS2 的
 * URDF 模型文件和 YAML 配置文件。
 ******************************************************************************
 */

#ifndef __ARM_CONFIG_H__
#define __ARM_CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* 1. 关节 CAN ID 分配                                                        */
/* -------------------------------------------------------------------------- */
#define CAN_ID_JOINT_1      1   /* 底座旋转关节 */
#define CAN_ID_JOINT_2      2   /* 大臂（肩关节）*/
#define CAN_ID_JOINT_3      3   /* 小臂（肘关节）*/
#define CAN_ID_JOINT_4      4   /* 手腕旋转关节 */
#define CAN_ID_JOINT_5      5   /* 手腕俯仰关节 */
#define CAN_ID_JOINT_6      6   /* 末端旋转关节 */

/* 关节 ID 列表（遍历用） */
#define JOINT_ID_LIST  { CAN_ID_JOINT_1, CAN_ID_JOINT_2, CAN_ID_JOINT_3, CAN_ID_JOINT_4, CAN_ID_JOINT_5, CAN_ID_JOINT_6 }
/* 关节数量 (六轴) */
#define MAX_JOINT_COUNT     6

/* 运动输出使能: 1=硬件 target (CAN/PWM使能), 0=transport-only (锁定运动, 仅响应状态查询) */
#define ARM_ACTUATOR_OUTPUTS_ENABLED  1

/* -------------------------------------------------------------------------- */
/* 2. 关节物理限位 (角度单位: °) — MCU 安全保护的最后防线                       */
/* -------------------------------------------------------------------------- */

/* 关节 1（底座旋转） */
#define JOINT1_ANGLE_MIN    -165.0f
#define JOINT1_ANGLE_MAX     165.0f
#define JOINT1_SPEED_MAX     120.0f   /* r/min */
#define JOINT1_TORQUE_MAX     5.0f    /* Nm   */

/* 关节 2（大臂） */
#define JOINT2_ANGLE_MIN    -125.0f
#define JOINT2_ANGLE_MAX     125.0f
#define JOINT2_SPEED_MAX      90.0f
#define JOINT2_TORQUE_MAX    10.0f

/* 关节 3（小臂） */
#define JOINT3_ANGLE_MIN    -135.0f
#define JOINT3_ANGLE_MAX     135.0f
#define JOINT3_SPEED_MAX      90.0f
#define JOINT3_TORQUE_MAX     5.0f

/* 关节 4（手腕旋转） */
#define JOINT4_ANGLE_MIN    -175.0f
#define JOINT4_ANGLE_MAX     175.0f
#define JOINT4_SPEED_MAX     180.0f
#define JOINT4_TORQUE_MAX     3.0f

/* 关节 5（手腕俯仰） */
#define JOINT5_ANGLE_MIN     -85.0f
#define JOINT5_ANGLE_MAX     115.0f
#define JOINT5_SPEED_MAX     120.0f
#define JOINT5_TORQUE_MAX      2.0f

/* 关节 6（末端旋转） */
#define JOINT6_ANGLE_MIN    -175.0f
#define JOINT6_ANGLE_MAX     175.0f
#define JOINT6_SPEED_MAX     180.0f
#define JOINT6_TORQUE_MAX      2.0f

/* 汇总数组（用于代码中遍历） */
#define JOINT_ANGLE_MIN_LIST  { JOINT1_ANGLE_MIN, JOINT2_ANGLE_MIN, JOINT3_ANGLE_MIN, JOINT4_ANGLE_MIN, JOINT5_ANGLE_MIN, JOINT6_ANGLE_MIN }
#define JOINT_ANGLE_MAX_LIST  { JOINT1_ANGLE_MAX, JOINT2_ANGLE_MAX, JOINT3_ANGLE_MAX, JOINT4_ANGLE_MAX, JOINT5_ANGLE_MAX, JOINT6_ANGLE_MAX }
#define JOINT_SPEED_MAX_LIST  { JOINT1_SPEED_MAX, JOINT2_SPEED_MAX, JOINT3_SPEED_MAX, JOINT4_SPEED_MAX, JOINT5_SPEED_MAX, JOINT6_SPEED_MAX }
#define JOINT_TORQUE_MAX_LIST { JOINT1_TORQUE_MAX, JOINT2_TORQUE_MAX, JOINT3_TORQUE_MAX, JOINT4_TORQUE_MAX, JOINT5_TORQUE_MAX, JOINT6_TORQUE_MAX }

/* 2.1 双层限位体系 (物理机械限位 + 操作限位 = 有效限位)                         */
/*     物理限位: 机械硬限位, 不可逾越, 来自关节数据手册                           */
/*     操作限位: 每端留 5° 的真实运动工作范围；Pi 必须使用同一组数值              */
/*     有效限位: 小于物理极限的当前软件实际使用值                                 */
/* -------------------------------------------------------------------------- */
#define JOINT_PHYSICAL_MIN_LIST { -170.0f, -130.0f, -140.0f, -180.0f, -90.0f, -180.0f }
#define JOINT_PHYSICAL_MAX_LIST {  170.0f,  130.0f,  140.0f,  180.0f, 120.0f,  180.0f }
#define JOINT_OPERATIONAL_MIN_LIST JOINT_ANGLE_MIN_LIST
#define JOINT_OPERATIONAL_MAX_LIST JOINT_ANGLE_MAX_LIST

/* effective[i] = min(physical_max[i], operational_max[i]) — 编译期常量 */
#define JOINT_EFFECTIVE_MIN_LIST { -165.0f, -125.0f, -135.0f, -175.0f, -85.0f, -175.0f }
#define JOINT_EFFECTIVE_MAX_LIST {  165.0f,  125.0f,  135.0f,  175.0f, 115.0f,  175.0f }

/* 关节最小连续转速 (r/min) — 低于此转速使用 mode=0 轨迹跟踪 + MCU 10ms 插值 */
#define JOINT_MIN_CONTINUOUS_RPM      3.0f

/* -------------------------------------------------------------------------- */
/* 3. 安全保护阈值                                                             */
/* -------------------------------------------------------------------------- */
#define ARM_OVERCURRENT_THRESHOLD   8.0f    /* 过流阈值 (A)     */
#define ARM_OVERTEMP_THRESHOLD      80.0f   /* 过热阈值 (°C)    — 需关节支持温度回读 */
#define ARM_CAN_TIMEOUT_MS          500     /* CAN 通信超时 (ms) */
#define ARM_COMM_LOSS_TIMEOUT_MS    200     /* 上位机通信超时 (ms) */
#define ARM_PREDICT_LIMIT_RATIO     0.85f   /* 预测限位触发比例: 预测角度达限位85%时减速 */

/* -------------------------------------------------------------------------- */
/* 4. 轨迹执行参数 (MCU 侧仅保留插补周期)                                      */
/* -------------------------------------------------------------------------- */
#define TRAJ_INTERPOLATION_MS   10          /* 轨迹插补周期 (ms), 与 joint_ctrl 线程同步 */
#define TRAJ_ARRIVAL_TOLERANCE   0.5f       /* 轨迹点到位容差 (°), 关节角度误差小于此值视为到位 */
#define TRAJ_DURATION_MIN_MS     1          /* 段时长下限 (ms), 0=非法 */
#define TRAJ_DURATION_MAX_MS     10000      /* 段时长上限 (ms), Pi安全层设定 */
#define TRAJ_BATCH_MAX_POINTS    9          /* 单批最大轨迹点数: floor(248/26) = 9 */
#define TRAJ_FIFO_SIZE           16         /* 轨迹环形缓冲区槽位数 */

/* -------------------------------------------------------------------------- */
/* 5. 系统杂项参数                                                             */
/* -------------------------------------------------------------------------- */
#define JOINT_ONLINE_CHECK_RETRY     3       /* 关节在线检测重试次数 */
#define JOINT_FEEDBACK_RATE_MS       12      /* 实时状态反馈间隔 (6关节, 12ms/关节) */

#ifdef __cplusplus
}
#endif

#endif /* __ARM_CONFIG_H__ */
