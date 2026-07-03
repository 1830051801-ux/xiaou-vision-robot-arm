/**
 ******************************************************************************
 * @file    led.h
 * @brief   LED 运行指示灯驱动 — LED1(PA6) / LED2(PA7)
 * @author  YaowenLi
 * @date    2026-06-11
 ******************************************************************************
 * @attention
 * 两个 LED 用于指示系统运行状态：
 *   LED1 (PA6) — 主状态灯
 *   LED2 (PA7) — 辅助状态灯 / 故障指示
 *
 * 状态指示模式（见 led.c 中 led_show_status）
 ******************************************************************************
 */

#ifndef __LED_H__
#define __LED_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Pin 定义 ------------------------------------------------------------------*/
#define LED1_PIN        GPIO_PIN_6
#define LED1_PORT       GPIOA
#define LED2_PIN        GPIO_PIN_7
#define LED2_PORT       GPIOA

/* 函数声明 ------------------------------------------------------------------*/

void LED_Init(void);                    /* 初始化 PA6/PA7 为推挽输出 */
void LED1_On(void);                     /* LED1 亮 */
void LED1_Off(void);                    /* LED1 灭 */
void LED1_Toggle(void);                 /* LED1 翻转 */
void LED2_On(void);                     /* LED2 亮 */
void LED2_Off(void);                    /* LED2 灭 */
void LED2_Toggle(void);                 /* LED2 翻转 */
void LED_Both_Off(void);                /* 两灯全灭 */
void led_show_status(int state);        /* 根据系统状态显示对应灯语 */

#ifdef __cplusplus
}
#endif

#endif /* __LED_H__ */
