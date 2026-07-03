/**
 ******************************************************************************
 * @file    led.c
 * @brief   LED 运行指示灯驱动实现
 * @author  YaowenLi
 * @date    2026-06-11
 ******************************************************************************
 * @attention
 * LED1 (PA6): 主状态灯 — 心跳 / 常亮
 * LED2 (PA7): 辅助状态灯 — 故障 / 急停指示
 *
 * 接法：共阳极 — RESET (低电平) 点亮, SET (高电平) 熄灭
 *
 * 灯语定义：
 *   INIT     → 两灯同步慢闪 (500ms)      系统初始化中
 *   READY    → LED1 常亮, LED2 灭         系统就绪
 *   RUNNING  → LED1 心跳 (50ms亮/950ms灭)  正常运行
 *   ESTOP    → LED1 灭, LED2 快闪 (100ms)  急停
 *   ERROR    → 两灯交替闪烁 (250ms)        故障
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "led.h"
#include "app_threads.h"

/* Private variables ---------------------------------------------------------*/
static rt_uint32_t led_tick  = 0;   /* LED 线程累计 tick 计数 (ms) */
static rt_uint8_t  led_phase = 0;   /* 当前亮灭相位 */

/* -------------------------------------------------------------------------- */
/* 底层 GPIO 操作                                                             */
/* -------------------------------------------------------------------------- */

/**
 * @brief  初始化 LED1(PA6) 和 LED2(PA7) 为推挽输出
 * @note   放在 MX_GPIO_Init() 之后调用，或在 LED 线程首次运行时调用
 */
void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 时钟已在 MX_GPIO_Init 中使能，此处仅配置引脚 */
    GPIO_InitStruct.Pin   = LED1_PIN | LED2_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED1_PORT, &GPIO_InitStruct);

    /* 初始全灭 */
    LED_Both_Off();
}

void LED1_On(void)
{
    HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_RESET);  /* 共阳极: 低电平点亮 */
}

void LED1_Off(void)
{
    HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_SET);    /* 共阳极: 高电平熄灭 */
}

void LED1_Toggle(void)
{
    HAL_GPIO_TogglePin(LED1_PORT, LED1_PIN);
}

void LED2_On(void)
{
    HAL_GPIO_WritePin(LED2_PORT, LED2_PIN, GPIO_PIN_RESET);  /* 共阳极: 低电平点亮 */
}

void LED2_Off(void)
{
    HAL_GPIO_WritePin(LED2_PORT, LED2_PIN, GPIO_PIN_SET);    /* 共阳极: 高电平熄灭 */
}

void LED2_Toggle(void)
{
    HAL_GPIO_TogglePin(LED2_PORT, LED2_PIN);
}

void LED_Both_Off(void)
{
    HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_SET);    /* 共阳极: 高电平全灭 */
    HAL_GPIO_WritePin(LED2_PORT, LED2_PIN, GPIO_PIN_SET);
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
    led_tick += 50;  /* 假设每 50ms 调用一次 */

    switch (state)
    {
    case SYS_STATE_INIT:
        /* 两灯同步慢闪: 500ms 亮 / 500ms 灭 */
        if (led_tick % 1000 < 500) {
            LED1_On();
            LED2_On();
        } else {
            LED1_Off();
            LED2_Off();
        }
        break;

    case SYS_STATE_READY:
        /* LED1 常亮, LED2 灭 */
        LED1_On();
        LED2_Off();
        break;

    case SYS_STATE_RUNNING:
        /* LED1 心跳: 50ms 亮 / 950ms 灭, LED2 灭 */
        if (led_tick % 1000 < 50) {
            LED1_On();
        } else {
            LED1_Off();
        }
        LED2_Off();
        break;

    case SYS_STATE_ESTOP:
        /* LED1 灭, LED2 快闪: 100ms 亮 / 100ms 灭 */
        LED1_Off();
        if (led_tick % 200 < 100) {
            LED2_On();
        } else {
            LED2_Off();
        }
        break;

    case SYS_STATE_ERROR:
        /* 两灯交替闪烁: 250ms 亮 / 250ms 灭, 相位差 250ms */
        led_phase = (led_tick / 250) % 4;
        switch (led_phase) {
        case 0: LED1_On();  LED2_Off(); break;  /* LED1 亮 */
        case 1: LED1_Off(); LED2_Off(); break;  /* 全灭   */
        case 2: LED1_Off(); LED2_On();  break;  /* LED2 亮 */
        case 3: LED1_Off(); LED2_Off(); break;  /* 全灭   */
        }
        break;

    default:
        LED_Both_Off();
        break;
    }
}
