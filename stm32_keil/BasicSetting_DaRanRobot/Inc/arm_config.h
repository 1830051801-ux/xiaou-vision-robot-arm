/**
 ******************************************************************************
 * @file    arm_config.h
 * @brief   4轴机械臂配置参数（DH参数、关节限位、速度限制、CAN ID）
 * @author  YaowenLi
 * @date    2026-06-18
 ******************************************************************************
 * @attention
 * 本文件集中管理机械臂的全部可配置参数：
 *   1. 关节 CAN ID 分配
 *   2. DH 运动学参数（Denavit-Hartenberg）
 *   3. 关节角度 / 转速 / 力矩限位
 *   4. 轨迹规划默认参数
 *   5. 工作空间软限位
 *
 * 修改参数后须重新编译下载，硬件不变时仅需修改此文件。
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

/* 关节 ID 列表（遍历用） */
#define JOINT_ID_LIST  { CAN_ID_JOINT_1, CAN_ID_JOINT_2, CAN_ID_JOINT_3, CAN_ID_JOINT_4 }

/* -------------------------------------------------------------------------- */
/* 2. DH 运动学参数 （单位：mm, °）— 占位默认值, 需按实际机械臂填入         */
/* -------------------------------------------------------------------------- */

/*
 * ========================================================================
 *  坐标系约定 (Base 系统原点) — 树莓派下发的坐标必须遵循此约定
 * ========================================================================
 *
 *   原点: 底座安装面中心
 *   X+:  臂杆向前伸出方向 (J1 θ1=0 时臂杆指向)
 *   Y+:  机械臂左侧 (X 轴绕 Z 逆时针转 90°)
 *   Z+:  垂直向上, 远离台面
 *   旋转: DH 标准 — 从各关节 Z 轴正向俯视, CCW (逆时针) 为正
 *
 *   视觉对接:
 *     - 树莓派通过手眼标定矩阵 T_cam2base 将相机坐标映射到上述 Base 坐标系
 *     - 下发的目标坐标: 单位 mm, 角度 °, 轴方向与本节约定的 Base 坐标系一致
 *     - MCU 侧不做坐标系翻转/正负修正, 只执行逆解和运动控制
 *     - g_sys.base_offset 仅用于标定诊断 (记录 FK 与视觉输出的差值),
 *       正常生产运行时应保持 {0, 0, 0}
 *
 * ========================================================================
 *
 * 标准 DH 参数表 (基于上述坐标系约定):
 *   Joint | theta(°) | d(mm)  | a(mm) | alpha(°)
 *     1   | theta1   | D1     | 0     |  90
 *     2   | theta2   | 0      | L2    |  0
 *     3   | theta3   | 0      | L3    |  0
 *     4   | theta4   | 0      | 0     |  0
 */

/* 连杆偏置 (mm) */
#define ARM_DH_D1           300.0f   /* 底座→J2转轴高度 */
#define ARM_DH_D2           0.0f
#define ARM_DH_D3           0.0f
#define ARM_DH_D4           0.0f

/* 连杆长度 (mm) */
#define ARM_DH_A1           0.0f
#define ARM_DH_A2           180.0f   /* 大臂长度 */
#define ARM_DH_A3           100.0f   /* 小臂长度 */
#define ARM_DH_A4           0.0f

/* DH alpha (°) — 存储为 float 角度 */
#define ARM_DH_ALPHA1       90.0f   /* J1(Z)→J2(X): 正交 */
#define ARM_DH_ALPHA2       0.0f    /* J2(X)→J3(X): 平行 */
#define ARM_DH_ALPHA3        90.0f  /* J3(X)→J4(Z): 正交 */
#define ARM_DH_ALPHA4       0.0f    /* J4(Z)→工具: 平行 */

/* 工具末端偏移 (mm) — 腕→夹爪指尖 */
#define ARM_TOOL_X          0.0f
#define ARM_TOOL_Y          0.0f
#define ARM_TOOL_Z          230.0f

/* -------------------------------------------------------------------------- */
/* 3. 关节物理限位 (角度单位: °)                                            */
/* -------------------------------------------------------------------------- */

/* 关节 1（底座旋转） */
#define JOINT1_ANGLE_MIN    -170.0f
#define JOINT1_ANGLE_MAX     170.0f
#define JOINT1_SPEED_MAX     120.0f   /* r/min */
#define JOINT1_TORQUE_MAX     5.0f    /* Nm   */

/* 关节 2（大臂） */
#define JOINT2_ANGLE_MIN    -130.0f
#define JOINT2_ANGLE_MAX     130.0f
#define JOINT2_SPEED_MAX      90.0f
#define JOINT2_TORQUE_MAX    10.0f

/* 关节 3（小臂） */
#define JOINT3_ANGLE_MIN    -140.0f
#define JOINT3_ANGLE_MAX     140.0f
#define JOINT3_SPEED_MAX      90.0f
#define JOINT3_TORQUE_MAX     5.0f

/* 关节 4（手腕旋转） */
#define JOINT4_ANGLE_MIN    -180.0f
#define JOINT4_ANGLE_MAX     180.0f
#define JOINT4_SPEED_MAX     180.0f
#define JOINT4_TORQUE_MAX     3.0f

/* 汇总数组（用于代码中遍历） */
#define JOINT_ANGLE_MIN_LIST  { JOINT1_ANGLE_MIN, JOINT2_ANGLE_MIN, JOINT3_ANGLE_MIN, JOINT4_ANGLE_MIN }
#define JOINT_ANGLE_MAX_LIST  { JOINT1_ANGLE_MAX, JOINT2_ANGLE_MAX, JOINT3_ANGLE_MAX, JOINT4_ANGLE_MAX }
#define JOINT_SPEED_MAX_LIST  { JOINT1_SPEED_MAX, JOINT2_SPEED_MAX, JOINT3_SPEED_MAX, JOINT4_SPEED_MAX }
#define JOINT_TORQUE_MAX_LIST { JOINT1_TORQUE_MAX, JOINT2_TORQUE_MAX, JOINT3_TORQUE_MAX, JOINT4_TORQUE_MAX }

/* -------------------------------------------------------------------------- */
/* 4. 重力补偿 — 连杆质量与质心参数 (需根据实际机械臂标定)                    */
/* -------------------------------------------------------------------------- */
#define ARM_LINK2_MASS      2.5f        /* 大臂质量 (kg) */
#define ARM_LINK3_MASS      1.5f        /* 小臂质量 (kg) */
#define ARM_LINK4_MASS      0.5f        /* 手腕+工具质量 (kg) */
#define ARM_PAYLOAD_MASS    0.5f        /* 末端负载质量 (kg) — 可动态修改 */

/* 连杆质心位置 (相对于各自连杆坐标系原点, 单位 mm) */
#define ARM_LINK2_COM_X     125.0f      /* 大臂质心 X (约在连杆中点) */
#define ARM_LINK2_COM_Z     0.0f
#define ARM_LINK3_COM_X     100.0f      /* 小臂质心 X */
#define ARM_LINK3_COM_Z     0.0f
#define ARM_LINK4_COM_X     0.0f        /* 手腕质心 X */
#define ARM_LINK4_COM_Z     0.0f

/* 重力加速度 (mm/s² = m/s² × 1000, 用于与 DH 参数 mm 单位统一) */
#define ARM_GRAVITY         9800.0f

/* -------------------------------------------------------------------------- */
/* 5. 安全保护阈值                                                           */
/* -------------------------------------------------------------------------- */
#define ARM_OVERCURRENT_THRESHOLD   8.0f    /* 过流阈值 (A)     */
#define ARM_OVERTEMP_THRESHOLD      80.0f   /* 过热阈值 (°C)    — 需关节支持温度回读 */
#define ARM_CAN_TIMEOUT_MS          50      /* CAN 通信超时 (ms) */
#define ARM_COMM_LOSS_TIMEOUT_MS    200     /* 上位机通信超时 (ms) */
#define ARM_PREDICT_LIMIT_RATIO     0.85f   /* 预测限位触发比例: 预测角度达限位85%时减速 */

/* -------------------------------------------------------------------------- */
/* 6. 轨迹规划默认参数                                                       */
/* -------------------------------------------------------------------------- */
#define TRAJ_DEFAULT_SPEED      60.0f       /* 默认轨迹转速 (r/min)          */
#define TRAJ_DEFAULT_ACCEL      100.0f      /* 默认轨迹加速度 (r/min/s)      */
#define TRAJ_DEFAULT_JERK       500.0f      /* 默认轨迹加加速度 (r/min/s²)   */
#define TRAJ_DEFAULT_CART_SPEED  100.0f     /* 默认笛卡尔线速度 (mm/s)       */
#define TRAJ_DEFAULT_CART_ACCEL  200.0f     /* 默认笛卡尔线加速度 (mm/s²)    */
#define TRAJ_INTERPOLATION_MS   10          /* 轨迹插补周期 (ms), 与 joint_ctrl 线程同步 */

/* -------------------------------------------------------------------------- */
/* 7. 工作空间软限位 (笛卡尔, 单位 mm) — 由 FK 计算得出                      */
/* -------------------------------------------------------------------------- */
#define WORKSPACE_X_MIN     -300.0f
#define WORKSPACE_X_MAX      300.0f
#define WORKSPACE_Y_MIN     -300.0f
#define WORKSPACE_Y_MAX      300.0f
#define WORKSPACE_Z_MIN     -100.0f
#define WORKSPACE_Z_MAX      500.0f

/* -------------------------------------------------------------------------- */
/* 8. 归位点 & 原点 (关节角度 °)                                             */
/* -------------------------------------------------------------------------- */
/* 归位点 (折叠态, 上电/空闲保持): J1=底座12点, J2=大臂4点, J3=小臂12点, J4=腕6点 */
#define HOME_J1             0.0f
#define HOME_J2            -60.0f
#define HOME_J3             90.0f
#define HOME_J4             0.0f

/* 原点 (最高点, 抓取前先到此处): J1=不动, J2=大臂12点, J3=小臂6点, J4=腕向下 */
#define ORIGIN_J1           0.0f
#define ORIGIN_J2           0.0f
#define ORIGIN_J3           0.0f
#define ORIGIN_J4           0.0f

#define HOME_JOINTS   { HOME_J1, HOME_J2, HOME_J3, HOME_J4 }
#define ORIGIN_JOINTS { ORIGIN_J1, ORIGIN_J2, ORIGIN_J3, ORIGIN_J4 }

/* -------------------------------------------------------------------------- */
/* 9. 系统杂项参数                                                           */
/* -------------------------------------------------------------------------- */
#define JOINT_ONLINE_CHECK_RETRY     3       /* 关节在线检测重试次数 */
#define JOINT_FEEDBACK_RATE_MS       8       /* 实时状态反馈间隔 (4关节 × 2ms) */

#ifdef __cplusplus
}
#endif

#endif /* __ARM_CONFIG_H__ */
