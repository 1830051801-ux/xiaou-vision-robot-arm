/**
 ******************************************************************************
 * @file    servo.h
 * @brief   舵机 PWM 驱动 — 5路舵机控制
 *          TIM3 CH1/CH2/CH3 → PC6/PC7/PC8
 *          TIM4 CH1/CH2      → PD12/PD13
 *
 * 参数:
 *   PWM 频率: 50Hz (周期 20ms)
 *   脉冲范围: 500us ~ 2500us (对应 0° ~ 180°)
 *   分辨率:   1us (定时器时钟 1MHz)
 ******************************************************************************
 */

#ifndef __SERVO_H__
#define __SERVO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* 舵机编号 ----------------------------------------------------------------*/
typedef enum {
    SERVO_1 = 0,    /* PD12 — TIM4_CH1 */
    SERVO_2,        /* PD13 — TIM4_CH2 */
    SERVO_3,        /* PC6  — TIM3_CH1 */
    SERVO_4,        /* PC7  — TIM3_CH2 */
    SERVO_5         /* PC8  — TIM3_CH3 */
} ServoID;

/* 硬件通道映射 ------------------------------------------------------------*/
#define SERVO_COUNT             5

/* TIM4 通道 (SERVO_1, SERVO_2) */
#define SERVO1_TIM              TIM4
#define SERVO1_CHANNEL          TIM_CHANNEL_1
#define SERVO1_PORT             GPIOD
#define SERVO1_PIN              GPIO_PIN_12

#define SERVO2_TIM              TIM4
#define SERVO2_CHANNEL          TIM_CHANNEL_2
#define SERVO2_PORT             GPIOD
#define SERVO2_PIN              GPIO_PIN_13

/* TIM3 通道 (SERVO_3, SERVO_4, SERVO_5) */
#define SERVO3_TIM              TIM3
#define SERVO3_CHANNEL          TIM_CHANNEL_1
#define SERVO3_PORT             GPIOC
#define SERVO3_PIN              GPIO_PIN_6

#define SERVO4_TIM              TIM3
#define SERVO4_CHANNEL          TIM_CHANNEL_2
#define SERVO4_PORT             GPIOC
#define SERVO4_PIN              GPIO_PIN_7

#define SERVO5_TIM              TIM3
#define SERVO5_CHANNEL          TIM_CHANNEL_3
#define SERVO5_PORT             GPIOC
#define SERVO5_PIN              GPIO_PIN_8

/* PWM 参数 ----------------------------------------------------------------*/
#define SERVO_PWM_FREQ          50U         /* 50Hz                        */
#define SERVO_PERIOD_US         20000U      /* 20ms 周期                   */
#define SERVO_PULSE_MIN_US      500U        /* 脉宽 0.5ms (对应 0°)        */
#define SERVO_PULSE_MID_US      1500U       /* 脉宽 1.5ms (对应 90°)       */
#define SERVO_PULSE_MAX_US      2500U       /* 脉宽 2.5ms (对应 180°)      */
#define SERVO_ANGLE_MIN          0.0f       /* 最小角度 (夹爪闭合/张开)     */
#define SERVO_ANGLE_MAX          180.0f     /* 最大角度 (夹爪张开/闭合)     */

/* Gripper profiles (0 deg open side ~ 180 deg closed side) ----------------*/
#define GRIP_MODE1_CLOSE         115.0f      /* profile 1 close */
#define GRIP_MODE1_RELEASE        48.0f      /* profile 1 release */
#define GRIP_MODE2_CLOSE         140.0f      /* profile 2 close */
#define GRIP_MODE2_RELEASE        50.0f      /* profile 2 release */
#define GRIP_MODE3_CLOSE          110.0f     /* profile 3 close */
#define GRIP_MODE3_RELEASE         35.0f     /* profile 3 release */
#define GRIP_MODE4_CLOSE          130.0f     /* profile 4 close */
#define GRIP_MODE4_RELEASE         50.0f     /* profile 4 release */

/* 全局定时器句柄 (供 MSP/IT 使用) ----------------------------------------*/
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

/* 函数声明 ----------------------------------------------------------------*/
void Servo_Init(void);                      /* 初始化全部5路舵机             */
void Servo_SetAngle(ServoID id, float angle_deg);  /* 按角度设置 (0°~180°) */
void Servo_SetPulseUs(ServoID id, uint16_t pulse_us); /* 按脉宽设置 (us)  */
void Servo_Stop(ServoID id);                /* 停止某路 PWM 输出            */
void Servo_StopAll(void);                   /* 停止全部 PWM 输出            */

#ifdef __cplusplus
}
#endif

#endif /* __SERVO_H__ */
