/**
 ******************************************************************************
 * @file    arm_kinematics.h
 * @brief   4轴机械臂正/逆运动学 — DH 参数法
 * @author  YaowenLi
 * @date    2026-06-18
 ******************************************************************************
 * @attention
 * 支持:
 *   - 正运动学 (FK): 关节角度[4] → 末端位姿[6]
 *   - 逆运动学 (IK): 末端位姿[6] → 关节角度[4] (解析解, 多解选择)
 *   - 雅可比矩阵: 速度级运动学
 *   - 工作空间检查: 笛卡尔点是否可达
 *
 * DH 参数由 arm_config.h 提供 (#define 宏)
 ******************************************************************************
 */

#ifndef __ARM_KINEMATICS_H__
#define __ARM_KINEMATICS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 姿态/位姿数组索引 */
#define POSE_X      0
#define POSE_Y      1
#define POSE_Z      2
#define POSE_ROLL   3
#define POSE_PITCH  4
#define POSE_YAW    5

/* FK / IK 返回值 */
#define KIN_OK            0   /* 成功 */
#define KIN_ERR_UNREACH   -1  /* 不可达 */
#define KIN_ERR_SINGULAR  -2  /* 奇异点 */
#define KIN_ERR_INVALID   -3  /* 参数非法 */

/* -------------------------------------------------------------------------- */
/* API                                                                        */
/* -------------------------------------------------------------------------- */

/**
 * @brief 正运动学: 关节角度 → 末端位姿
 * @param joints   输入: 关节角度[4] (°)
 * @param pose     输出: 末端位姿 {x,y,z(mm), roll,pitch,yaw(°)}
 * @return KIN_OK 成功
 */
int arm_fk(const float joints[4], float pose[6]);

/**
 * @brief 逆运动学: 末端位姿 → 关节角度
 * @param pose      输入: 目标末端位姿 {x,y,z(mm), roll,pitch,yaw(°)}
 * @param current   输入: 当前关节角度[4] (°), 用于多解选择 (最接近解)
 * @param solution  输出: 关节角度解[4] (°)
 * @return KIN_OK / KIN_ERR_UNREACH / KIN_ERR_SINGULAR
 */
int arm_ik(const float pose[6], const float current[4], float solution[4]);

/**
 * @brief 雅可比矩阵计算 (末端速度 ← 关节速度)
 * @param joints  输入: 当前关节角度[4] (°)
 * @param J       输出: 6×4 雅可比矩阵 [6][4] (按行主序, J[row][col])
 */
void arm_jacobian(const float joints[4], float J[6][4]);

/**
 * @brief 工作空间可达性检查
 * @param pose  输入: 待检查位姿 {x,y,z(mm), roll,pitch,yaw(°)}
 * @return 1=可达, 0=不可达
 */
int arm_check_pose(const float pose[6]);

/**
 * @brief 关节角度检查 (是否在限位内)
 * @param joints  输入: 关节角度[4] (°)
 * @return 1=在限位内, 0=超出限位
 */
int arm_check_joints(const float joints[4]);

/**
 * @brief 重力补偿力矩计算
 *        基于当前关节角度和各连杆质量/质心, 计算克服重力所需的关节力矩
 * @param joints      输入: 当前关节角度[4] (°)
 * @param payload_mass 输入: 末端负载质量 (kg), 0 = 无负载
 * @param torques     输出: 重力补偿力矩[4] (Nm)
 */
void arm_gravity_comp(const float joints[4], float payload_mass, float torques[4]);

#ifdef __cplusplus
}
#endif

#endif /* __ARM_KINEMATICS_H__ */
