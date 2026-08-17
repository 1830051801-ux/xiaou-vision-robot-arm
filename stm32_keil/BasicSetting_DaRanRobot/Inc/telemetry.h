/**
 ******************************************************************************
 * @file    telemetry.h
 * @brief   机械臂遥测 — USART3 发送关节状态给 PC 3D 可视化
 * @date    2026-07-27 (架构重构: 精简为纯关节状态, 移除 FK 末端位姿)
 ******************************************************************************
 */

#ifndef __TELEMETRY_H__
#define __TELEMETRY_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 遥测帧格式: "J:j1,j2,j3,j4\n"  最长 ~48 字节 */
#define TELEM_MAX_LEN   48

void Telem_Init(void);

/** 发送关节角度 (精简: 不含末端位姿, Pi 自行计算 FK) */
void Telem_SendAngles(float j1, float j2, float j3, float j4);

#ifdef __cplusplus
}
#endif

#endif /* __TELEMETRY_H__ */
