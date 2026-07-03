/**
 ******************************************************************************
 * @file    arm_kinematics.c
 * @brief   4轴机械臂正/逆运动学实现 (DH 参数法)
 * @date    2026-06-18
 ******************************************************************************
 */

#include "arm_kinematics.h"
#include "arm_config.h"
#include <math.h>
#include <string.h>

/* -------------------------------------------------------------------------- */
/* 内部常量 (弧度)                                                            */
/* -------------------------------------------------------------------------- */
#ifndef M_PI
#define M_PI        3.14159265358979323846f
#endif
#define DEG2RAD     (M_PI / 180.0f)
#define RAD2DEG     (180.0f / M_PI)

/* DH 参数 (从 arm_config.h 宏展开为运行时 float) */
static const float dh_d[4]     = { ARM_DH_D1, ARM_DH_D2, ARM_DH_D3, ARM_DH_D4 };
static const float dh_a[4]     = { ARM_DH_A1, ARM_DH_A2, ARM_DH_A3, ARM_DH_A4 };
static const float dh_alpha[4] = { ARM_DH_ALPHA1 * DEG2RAD, ARM_DH_ALPHA2 * DEG2RAD,
                                   ARM_DH_ALPHA3 * DEG2RAD, ARM_DH_ALPHA4 * DEG2RAD };

/* 关节限位 (°) */
static const float joint_min[4] = JOINT_ANGLE_MIN_LIST;
static const float joint_max[4] = JOINT_ANGLE_MAX_LIST;

/* -------------------------------------------------------------------------- */
/* 4×4 齐次变换矩阵乘法: C = A * B                                          */
/* -------------------------------------------------------------------------- */
static void mat4_mul(const float A[16], const float B[16], float C[16])
{
    for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 4; col++) {
            float sum = 0.0f;
            for (int k = 0; k < 4; k++)
                sum += A[row * 4 + k] * B[k * 4 + col];
            C[row * 4 + col] = sum;
        }
    }
}

/* -------------------------------------------------------------------------- */
/* 单关节 DH 变换矩阵                                                         */
/*   T = RotZ(theta) * TransZ(d) * TransX(a) * RotX(alpha)                    */
/* -------------------------------------------------------------------------- */
static void dh_transform(float theta_rad, float d, float a, float alpha_rad,
                         float T[16])
{
    float ct = cosf(theta_rad), st = sinf(theta_rad);
    float ca = cosf(alpha_rad), sa = sinf(alpha_rad);

    T[0] = ct;   T[1] = -st * ca;  T[2] =  st * sa;  T[3] = a * ct;
    T[4] = st;   T[5] =  ct * ca;  T[6] = -ct * sa;  T[7] = a * st;
    T[8] = 0;    T[9] =  sa;       T[10] = ca;       T[11] = d;
    T[12]= 0;    T[13]= 0;         T[14] = 0;        T[15] = 1;
}

/* -------------------------------------------------------------------------- */
/* 从旋转矩阵提取 RPY 欧拉角 (ZYX 顺序, 即固定轴 XYZ)                        */
/* -------------------------------------------------------------------------- */
static void rotmat_to_rpy(const float R[16], float *roll, float *pitch, float *yaw)
{
    /* R = [r11 r12 r13; r21 r22 r23; r31 r32 r33] in 4x4 storage */
    float r11 = R[0], r12 = R[1], r13 = R[2];
    float r21 = R[4], r22 = R[5], r23 = R[6];
    float r31 = R[8], r32 = R[9], r33 = R[10];

    *pitch = asinf(-r31);
    if (cosf(*pitch) > 1e-6f) {
        *roll = atan2f(r32, r33);
        *yaw  = atan2f(r21, r11);
    } else {
        /* 万向节锁 */
        *roll = 0.0f;
        *yaw  = atan2f(-r12, r22);
    }
}

/* -------------------------------------------------------------------------- */
/* API: 正运动学                                                              */
/* -------------------------------------------------------------------------- */
int arm_fk(const float joints[4], float pose[6])
{
    float T[16], T_i[16], T_tmp[16];

    /* 初始化单位矩阵 */
    memset(T, 0, sizeof(T));
    T[0] = T[5] = T[10] = T[15] = 1.0f;

    for (int i = 0; i < 4; i++) {
        dh_transform(joints[i] * DEG2RAD, dh_d[i], dh_a[i], dh_alpha[i], T_i);
        mat4_mul(T, T_i, T_tmp);
        memcpy(T, T_tmp, sizeof(T));
    }

    /* 位置 */
    pose[POSE_X] = T[3];
    pose[POSE_Y] = T[7];
    pose[POSE_Z] = T[11];

    /* 姿态 (RPY °) */
    float roll, pitch, yaw;
    rotmat_to_rpy(T, &roll, &pitch, &yaw);
    pose[POSE_ROLL]  = roll  * RAD2DEG;
    pose[POSE_PITCH] = pitch * RAD2DEG;
    pose[POSE_YAW]   = yaw   * RAD2DEG;

    return KIN_OK;
}

/* -------------------------------------------------------------------------- */
/* API: 逆运动学 (解析解, 4-DOF 机械臂)                                       */
/*   假设: 4轴臂 (基座旋转 + 大臂 + 小臂 + 手腕旋转)                          */
/*         腕部旋转轴平行于小臂旋转轴 (即手腕控制工具 Z 轴旋转)                */
/* -------------------------------------------------------------------------- */
int arm_ik(const float pose[6], const float current[4], float solution[4])
{
    float x = pose[POSE_X];
    float y = pose[POSE_Y];
    float z = pose[POSE_Z];
    float yaw_des = pose[POSE_YAW] * DEG2RAD;  /* 目标 Yaw → 弧度 */

    float L2 = dh_a[1];  /* 大臂长度 */
    float L3 = dh_a[2];  /* 小臂长度 */
    float d1 = dh_d[0];  /* 底座高度 */

    /* ---- θ1: 基座旋转 ---- */
    float theta1 = atan2f(y, x);

    /* 验证 θ1 是否在限位内 */
    float t1_deg = theta1 * RAD2DEG;
    if (t1_deg < joint_min[0] || t1_deg > joint_max[0]) {
        /* 尝试另一个解: θ1 + π */
        float alt = theta1 + M_PI;
        if (alt > M_PI) alt -= 2.0f * M_PI;
        float alt_deg = alt * RAD2DEG;
        if (alt_deg >= joint_min[0] && alt_deg <= joint_max[0]) {
            theta1 = alt;
        } else {
            return KIN_ERR_UNREACH;
        }
    }

    /* ---- 平面 2R 求解 (关节 2, 3) ---- */
    /* 将目标点投影到由 θ1 确定的平面内 */
    float r  = sqrtf(x * x + y * y);   /* 水平面投影距离 */
    float z_adj = z - d1;              /* 减去底座高度 */

    /* 余弦定理求 θ3 */
    float c3 = (r * r + z_adj * z_adj - L2 * L2 - L3 * L3) / (2.0f * L2 * L3);
    if (c3 < -1.0f || c3 > 1.0f) {
        return KIN_ERR_UNREACH;  /* 工作空间外 */
    }

    /* θ3 两个解: elbow-up (负) / elbow-down (正) */
    float theta3_up   = -acosf(c3);   /* elbow-up */
    float theta3_down =  acosf(c3);   /* elbow-down */

    /* 选择最接近 current[2] 的解 */
    float cur3_rad = current[2] * DEG2RAD;
    float theta3 = (fabsf(theta3_up - cur3_rad) < fabsf(theta3_down - cur3_rad))
                   ? theta3_up : theta3_down;

    /* 验证 θ3 限位 */
    float t3_deg = theta3 * RAD2DEG;
    if (t3_deg < joint_min[2] || t3_deg > joint_max[2]) {
        /* 尝试另一个解 */
        float alt3 = (theta3 == theta3_up) ? theta3_down : theta3_up;
        float alt3_deg = alt3 * RAD2DEG;
        if (alt3_deg >= joint_min[2] && alt3_deg <= joint_max[2]) {
            theta3 = alt3;
        } else {
            return KIN_ERR_UNREACH;
        }
    }

    /* 求 θ2 */
    float s3 = sinf(theta3);
    float c3_val = cosf(theta3);
    float theta2 = atan2f(z_adj, r) - atan2f(L3 * s3, L2 + L3 * c3_val);

    /* 验证 θ2 限位 */
    float t2_deg = theta2 * RAD2DEG;
    if (t2_deg < joint_min[1] || t2_deg > joint_max[1]) {
        return KIN_ERR_UNREACH;
    }

    /* ---- θ4: 手腕旋转 ---- */
    /* yaw = θ2 + θ3 + θ4 → θ4 = yaw - θ2 - θ3 */
    float theta4 = yaw_des - theta2 - theta3;

    /* 归一化到 [-π, π] */
    while (theta4 >  M_PI) theta4 -= 2.0f * M_PI;
    while (theta4 < -M_PI) theta4 += 2.0f * M_PI;

    /* 验证 θ4 限位 */
    float t4_deg = theta4 * RAD2DEG;
    if (t4_deg < joint_min[3] || t4_deg > joint_max[3]) {
        return KIN_ERR_UNREACH;
    }

    /* 输出 */
    solution[0] = theta1 * RAD2DEG;
    solution[1] = theta2 * RAD2DEG;
    solution[2] = theta3 * RAD2DEG;
    solution[3] = theta4 * RAD2DEG;

    return KIN_OK;
}

/* -------------------------------------------------------------------------- */
/* API: 雅可比矩阵 (数值法)                                                   */
/*   J[6][4]: J[row][col], 行=末端速度分量, 列=关节速度                       */
/*   末端速度 = J * 关节速度                                                   */
/* -------------------------------------------------------------------------- */
void arm_jacobian(const float joints[4], float J[6][4])
{
    float pose0[6], pose_i[6];
    float delta = 0.001f;  /* 0.001° 微小扰动 */

    arm_fk(joints, pose0);

    for (int j = 0; j < 4; j++) {
        float jt[4];
        memcpy(jt, joints, sizeof(jt));
        jt[j] += delta;

        arm_fk(jt, pose_i);

        /* 有限差分近似 */
        J[0][j] = (pose_i[0] - pose0[0]) / delta;  /* dx/dθⱼ */
        J[1][j] = (pose_i[1] - pose0[1]) / delta;  /* dy/dθⱼ */
        J[2][j] = (pose_i[2] - pose0[2]) / delta;  /* dz/dθⱼ */
        J[3][j] = (pose_i[3] - pose0[3]) / delta;  /* droll/dθⱼ */
        J[4][j] = (pose_i[4] - pose0[4]) / delta;  /* dpitch/dθⱼ */
        J[5][j] = (pose_i[5] - pose0[5]) / delta;  /* dyaw/dθⱼ */
    }
}

/* -------------------------------------------------------------------------- */
/* API: 工作空间可达性检查                                                    */
/* -------------------------------------------------------------------------- */
int arm_check_pose(const float pose[6])
{
    /* 先检查笛卡尔边界 */
    if (pose[POSE_X] < WORKSPACE_X_MIN || pose[POSE_X] > WORKSPACE_X_MAX ||
        pose[POSE_Y] < WORKSPACE_Y_MIN || pose[POSE_Y] > WORKSPACE_Y_MAX ||
        pose[POSE_Z] < WORKSPACE_Z_MIN || pose[POSE_Z] > WORKSPACE_Z_MAX) {
        return 0;
    }

    /* 尝试 IK, 用零位作为参考 */
    float current_zero[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float solution[4];
    if (arm_ik(pose, current_zero, solution) == KIN_OK) {
        /* 验证解在限位内 */
        return arm_check_joints(solution);
    }
    return 0;
}

/* -------------------------------------------------------------------------- */
/* API: 关节角度限位检查                                                      */
/* -------------------------------------------------------------------------- */
int arm_check_joints(const float joints[4])
{
    for (int i = 0; i < 4; i++) {
        if (joints[i] < joint_min[i] || joints[i] > joint_max[i]) {
            return 0;
        }
    }
    return 1;
}

/* -------------------------------------------------------------------------- */
/* 内部: 计算指定关节数量 (1~4) 的 FK, 返回末端变换矩阵                      */
/* -------------------------------------------------------------------------- */
static void fk_n(const float joints[4], int n, float T[16])
{
    float T_i[16], T_tmp[16];
    memset(T, 0, 16 * sizeof(float));
    T[0] = T[5] = T[10] = T[15] = 1.0f;

    for (int i = 0; i < n && i < 4; i++) {
        dh_transform(joints[i] * DEG2RAD, dh_d[i], dh_a[i], dh_alpha[i], T_i);
        mat4_mul(T, T_i, T_tmp);
        memcpy(T, T_tmp, 16 * sizeof(float));
    }
}

/* -------------------------------------------------------------------------- */
/* API: 重力补偿力矩计算                                                      */
/*                                                                           */
/* 原理: 虚功原理 — τ_gj = Σ m_k · g^T · (∂p_k/∂θ_j)                        */
/*       其中 p_k 为连杆 k 的质心位置, g 为重力加速度向量                      */
/*                                                                           */
/* 对 4 轴机械臂:                                                            */
/*   - 关节 1 (底座旋转): 旋转轴平行于重力方向 → τ_g1 ≈ 0                     */
/*   - 关节 2 (大臂) + 关节 3 (小臂): 主要受重力影响                          */
/*   - 关节 4 (手腕旋转): 旋转轴垂直于重力, 但力臂短 → τ_g4 较小              */
/*                                                                           */
/* 使用数值法: FK 计算各连杆质心位置 → 有限差分求 ∂p/∂θ                       */
/* -------------------------------------------------------------------------- */
void arm_gravity_comp(const float joints[4], float payload_mass, float torques[4])
{
    /* 连杆质量 (kg) — 从 arm_config.h 获取 */
    const float m[4] = { 0, ARM_LINK2_MASS, ARM_LINK3_MASS,
                         ARM_LINK4_MASS + payload_mass };

    /* 连杆质心在各自连杆坐标系中的位置 (mm) */
    const float com_local[4][3] = {
        { 0,              0, 0 },                        /* 连杆1: 质心在转轴上, 重力矩=0 */
        { ARM_LINK2_COM_X, 0, ARM_LINK2_COM_Z },          /* 连杆2 质心 */
        { ARM_LINK3_COM_X, 0, ARM_LINK3_COM_Z },          /* 连杆3 质心 */
        { ARM_LINK4_COM_X, 0, ARM_LINK4_COM_Z },          /* 连杆4 + 负载 质心 */
    };

    /* 重力向量在世界坐标系 (基座) 中: 沿 -Z 方向 */
    float g_vec[3] = { 0, 0, -ARM_GRAVITY };  /* mm/s² */

    float delta = 0.001f;  /* 有限差分扰动 (°) */
    float T_base[16];      /* 到当前连杆的变换矩阵 */

    memset(torques, 0, 4 * sizeof(float));

    for (int j = 0; j < 4; j++) {
        /* 对关节 j 的每个关联连杆 (k >= j) 累加重力矩 */
        for (int k = j; k < 4; k++) {
            if (m[k] < 0.001f) continue;  /* 质量为零, 跳过 */

            /* 计算连杆 k 质心在当前关节角度的世界位置 */
            fk_n(joints, k + 1, T_base);
            float com_world[3];
            com_world[0] = T_base[3]  + T_base[0] * com_local[k][0]
                                       + T_base[1] * com_local[k][1]
                                       + T_base[2] * com_local[k][2];
            com_world[1] = T_base[7]  + T_base[4] * com_local[k][0]
                                       + T_base[5] * com_local[k][1]
                                       + T_base[6] * com_local[k][2];
            com_world[2] = T_base[11] + T_base[8] * com_local[k][0]
                                       + T_base[9] * com_local[k][1]
                                       + T_base[10] * com_local[k][2];

            /* 扰动关节 j → 计算 ∂com_world/∂θ_j */
            float jt[4];
            memcpy(jt, joints, 4 * sizeof(float));
            jt[j] += delta;

            fk_n(jt, k + 1, T_base);
            float com_pert[3];
            com_pert[0] = T_base[3]  + T_base[0] * com_local[k][0]
                                       + T_base[1] * com_local[k][1]
                                       + T_base[2] * com_local[k][2];
            com_pert[1] = T_base[7]  + T_base[4] * com_local[k][0]
                                       + T_base[5] * com_local[k][1]
                                       + T_base[6] * com_local[k][2];
            com_pert[2] = T_base[11] + T_base[8] * com_local[k][0]
                                       + T_base[9] * com_local[k][1]
                                       + T_base[10] * com_local[k][2];

            /* ∂p/∂θ_j ≈ (com_pert - com_world) / delta_rad */
            float dpx = (com_pert[0] - com_world[0]) / (delta * DEG2RAD);
            float dpy = (com_pert[1] - com_world[1]) / (delta * DEG2RAD);
            float dpz = (com_pert[2] - com_world[2]) / (delta * DEG2RAD);

            /* τ_gj = m_k · g^T · (∂p_k/∂θ_j)  →  N·mm */
            float tau_nmm = m[k] * (g_vec[0] * dpx + g_vec[1] * dpy + g_vec[2] * dpz);

            /* 转换为 Nm (÷1000) */
            torques[j] += tau_nmm / 1000.0f;
        }
    }
}
