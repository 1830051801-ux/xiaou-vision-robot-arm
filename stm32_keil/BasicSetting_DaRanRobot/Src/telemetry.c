/**
 ******************************************************************************
 * @file    telemetry.c
 * @brief   机械臂遥测实现 — USART3 文本帧发送 (精简版)
 * @date    2026-07-27 (架构重构: 精简为纯关节角度, 移除 FK 末端位姿)
 ******************************************************************************
 */

#include "telemetry.h"
#include "usart.h"
#include <stdio.h>

static uint8_t telem_ready = 0;

void Telem_Init(void)
{
    telem_ready = 1;
}

void Telem_SendAngles(float j1, float j2, float j3, float j4)
{
    if (!telem_ready) return;

    char buf[TELEM_MAX_LEN];
    int len = snprintf(buf, sizeof(buf),
                       "J:%.1f,%.1f,%.1f,%.1f\n",
                       j1, j2, j3, j4);
    if (len > 0 && len < TELEM_MAX_LEN)
    {
        HAL_UART_Transmit(&huart3, (uint8_t *)buf, len, 10);
    }
}
