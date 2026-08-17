/**
 ******************************************************************************
 * @file    led.c
 * @brief   LED 运行指示灯驱动实现 — 单灯 (PC0)
 * @author  YaowenLi
 * @date    2026-06-11
 ******************************************************************************
 * @attention
 * LED (PC0): 单颗状态指示灯
 *
 * 接法：共阳极 — RESET (低电平) 点亮, SET (高电平) 熄灭
 *
 * 灯语定义：
 *   INIT     → 慢闪 (500ms 亮 / 500ms 灭)     系统初始化中
 *   READY    → 常亮                             系统就绪
 *   RUNNING  → 心跳 (50ms 亮 / 950ms 灭)        正常运行
 *   ESTOP    → 快闪 (100ms 亮 / 100ms 灭)       急停
 *   ERROR    → 中速闪烁 (250ms 亮 / 250ms 灭)   故障
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "led.h"
#include "app_threads.h"

/* Private variables ---------------------------------------------------------*/
static rt_uint32_t led_tick  = 0;   /* LED 线程累计 tick 计数 (ms) */

/* -------------------------------------------------------------------------- */
/* 底层 GPIO 操作                                                             */
/* -------------------------------------------------------------------------- */

/**
 * @brief  初始化 LED (PC0) 为推挽输出
 * @note   放在 MX_GPIO_Init() 之后调用，或在 LED 线程首次运行时调用
 */
void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 时钟已在 MX_GPIO_Init 中使能，此处仅配置引脚 */
    GPIO_InitStruct.Pin   = LED_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);

    /* 初始点亮 */
    LED_On();
}

void LED_On(void)
{
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);  /* 共阳极: 低电平点亮 */
}

void LED_Off(void)
{
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);    /* 共阳极: 高电平熄灭 */
}

void LED_Toggle(void)
{
    HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
}

/* -------------------------------------------------------------------------- */
/* 状态灯语（由 LED 线程每 50ms 调用一次）                                    */
/* -------------------------------------------------------------------------- */

/**
 * @brief  根据系统状态更新 LED 显示
 * @param  state  当前系统状态 (enum sys_state)
 * @note   每 50ms 调用一次，内部用静态计数器维持闪烁节拍
 */
void led_show_status(int state)
{
    led_tick += 50;  /* 每 50ms 调用一次 */

    switch (state)
    {
    case SYS_STATE_INIT:
        /* 慢闪: 500ms 亮 / 500ms 灭 */
        if (led_tick % 1000 < 500) {
            LED_On();
        } else {
            LED_Off();
        }
        break;

    case SYS_STATE_READY:
        /* 常亮 */
        LED_On();
        break;

    case SYS_STATE_RUNNING:
        /* 心跳: 50ms 亮 / 950ms 灭 */
        if (led_tick % 1000 < 50) {
            LED_On();
        } else {
            LED_Off();
        }
        break;

    case SYS_STATE_ESTOP:
        /* 快闪: 100ms 亮 / 100ms 灭 */
        if (led_tick % 200 < 100) {
            LED_On();
        } else {
            LED_Off();
        }
        break;

    case SYS_STATE_ERROR:
        /* 中速闪烁: 250ms 亮 / 250ms 灭 */
        if (led_tick % 500 < 250) {
            LED_On();
        } else {
            LED_Off();
        }
        break;

    default:
        LED_Off();
        break;
    }
}
