/**
 ******************************************************************************
 * @file    led.h
 * @brief   LED 运行指示灯驱动 — 单灯 (PC0)
 * @author  YaowenLi
 * @date    2026-06-11
 ******************************************************************************
 * @attention
 * 单颗 LED 用于指示系统运行状态：
 *   LED (PC0) — 状态指示灯（共阳极，低电平点亮）
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
#define LED_PIN         GPIO_PIN_0
#define LED_PORT        GPIOC

/* 函数声明 ------------------------------------------------------------------*/

void LED_Init(void);                    /* 初始化 PC0 为推挽输出 */
void LED_On(void);                      /* LED 亮 */
void LED_Off(void);                     /* LED 灭 */
void LED_Toggle(void);                  /* LED 翻转 */
void led_show_status(int state);        /* 根据系统状态显示对应灯语 */

#ifdef __cplusplus
}
#endif

#endif /* __LED_H__ */
