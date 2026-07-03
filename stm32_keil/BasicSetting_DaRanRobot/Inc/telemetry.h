/**
 ******************************************************************************
 * @file    telemetry.h
 * @brief   机械臂遥测 — USART2 发送关节状态给 PC 3D 可视化
 * @note    MCU 侧完全解耦：不依赖 CAN/线程/外设
 *          启用: 调用 Telem_Init()  禁用: 不调用即可
 ******************************************************************************
 */

#ifndef __TELEMETRY_H__
#define __TELEMETRY_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 遥测帧格式: "J:j1,j2,j3,j4\n"  最长 ~64 字节 */
#define TELEM_MAX_LEN   64

/**
 * @brief  初始化遥测（无需额外外设，仅记录 USART2 句柄）
 */
void Telem_Init(void);

/**
 * @brief  发送一帧关节状态
 * @param  j1..j4  4个关节角度（°）
 * @param  x,y,z   末端位置（mm）
 */
void Telem_Send(float j1, float j2, float j3, float j4,
                float x, float y, float z);

/**
 * @brief  快速发送关节角度（末端位置填 0）
 */
void Telem_SendAngles(float j1, float j2, float j3, float j4);

#ifdef __cplusplus
}
#endif

#endif /* __TELEMETRY_H__ */
