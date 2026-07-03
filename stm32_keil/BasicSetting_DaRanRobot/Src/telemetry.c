/**
 ******************************************************************************
 * @file    telemetry.c
 * @brief   机械臂遥测实现 — USART2 文本帧发送
 *          帧格式: "J:j1,j2,j3,j4,X,Y,Z\n"
 *          完全解耦：仅依赖 USART2 已初始化的 huart2
 ******************************************************************************
 */

#include "telemetry.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

static uint8_t telem_ready = 0;

/**
 * @brief  初始化遥测
 */
void Telem_Init(void)
{
    telem_ready = 1;
}

/**
 * @brief  发送一帧完整遥测数据
 */
void Telem_Send(float j1, float j2, float j3, float j4,
                float x, float y, float z)
{
    if (!telem_ready) return;

    char buf[TELEM_MAX_LEN];
    int len = snprintf(buf, sizeof(buf),
                       "J:%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\n",
                       j1, j2, j3, j4, x, y, z);
    if (len > 0 && len < TELEM_MAX_LEN)
    {
        HAL_UART_Transmit(&huart2, (uint8_t *)buf, len, 10);
    }
}

/**
 * @brief  快速发送（末端填 0）
 */
void Telem_SendAngles(float j1, float j2, float j3, float j4)
{
    Telem_Send(j1, j2, j3, j4, 0, 0, 0);
}
