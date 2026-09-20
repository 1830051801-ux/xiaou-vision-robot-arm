/**
 ******************************************************************************
 * @file    main.c — 关节单步调试模式 (无看门狗)
 ******************************************************************************
 */
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "iwdg.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include <rtthread.h>
#include "app_threads.h"
#include "led.h"
#include "telemetry.h"
#include "debug_cmd.h"

extern void rt_hw_board_init(void);
extern UART_HandleTypeDef huart1;
void SystemClock_Config(void);

static int can_hw_init(void)
{
    MX_CAN1_Init();
    {
        CAN_FilterTypeDef f = {0};
        f.FilterBank = 0;
        f.FilterMode = CAN_FILTERMODE_IDMASK;
        f.FilterScale = CAN_FILTERSCALE_32BIT;
        f.FilterFIFOAssignment = CAN_RX_FIFO0;
        f.FilterActivation = ENABLE;
        f.SlaveStartFilterBank = 14;
        HAL_CAN_ConfigFilter(&hcan1, &f);
    }
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    sys_mark_ready(SYS_MOD_CAN);
    rt_kprintf("[HW] CAN1 init OK (1Mbps)\n");
    return 0;
}

static void gripper_hw_init(void)
{
    Servo_Init();
    Servo_Stop(SERVO_1); Servo_Stop(SERVO_2);
    Servo_Stop(SERVO_4); Servo_Stop(SERVO_5);
    TIM4->CR1 &= ~TIM_CR1_CEN;
    TIM3->CCR2 = 0; TIM3->CCR3 = 0;
    {   GPIO_InitTypeDef g = {0};
        g.Pin = GPIO_PIN_7 | GPIO_PIN_8;
        g.Mode = GPIO_MODE_ANALOG;
        g.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOC, &g);
    }
    {   GPIO_InitTypeDef g = {0};
        g.Pin = GPIO_PIN_12 | GPIO_PIN_13;
        g.Mode = GPIO_MODE_ANALOG;
        g.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOD, &g);
    }
    Servo_SetAngle(SERVO_3, GRIP_MODE1_RELEASE);
    sys_mark_ready(SYS_MOD_SERVO);
    rt_kprintf("[HW] Gripper init OK (SERVO_3 on PC6)\n");
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    rt_hw_board_init();
    NVIC_SetPriority(PendSV_IRQn, 0x0F);

    MX_USART3_UART_Init();
    Dbg_Printf_Init();
    Dbg_Uart_StartRx();

    rt_show_version();
    rt_system_timer_init();
    rt_system_scheduler_init();
    rt_system_timer_thread_init();
    rt_thread_idle_init();

    MX_GPIO_Init();
#if ARM_ACTUATOR_OUTPUTS_ENABLED == 1
    can_hw_init();
    gripper_hw_init();
#else
    rt_kprintf("[Main] ARM_ACTUATOR_OUTPUTS_ENABLED=0 — skipping CAN/Gripper init\n");
#endif
    MX_USART1_UART_Init();
    Rpi_Uart_StartRx();
    Telem_Init();

    rt_kprintf("\n===== BasicSetting_DaRanRobot v3.0 (Joint Debug Mode) =====\n");
#if ARM_ACTUATOR_OUTPUTS_ENABLED == 1
    rt_kprintf("[Main] CAN=%s GRIP=%s USART1=%s\n",
               (g_sys.modules_ready & SYS_MOD_CAN)   ? "OK" : "--",
               (g_sys.modules_ready & SYS_MOD_SERVO) ? "OK" : "--",
               (g_sys.modules_ready & SYS_MOD_USART1)? "OK" : "--");
#else
    rt_kprintf("[Main] TRANSPORT-ONLY mode  USART1=%s\n",
               (g_sys.modules_ready & SYS_MOD_USART1)? "OK" : "--");
#endif

    debug_cmd_init();
    if (app_threads_init() != RT_EOK) {
        rt_kprintf("[Main] FATAL: app_threads_init failed!\n");
        Error_Handler();
    }
    rt_system_scheduler_start();
    while (1) {}
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef o = {0};
    RCC_ClkInitTypeDef c = {0};
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    o.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSI;
    o.HSEState = RCC_HSE_ON; o.LSIState = RCC_LSI_ON;
    o.PLL.PLLState = RCC_PLL_ON; o.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    o.PLL.PLLM = 8; o.PLL.PLLN = 336; o.PLL.PLLP = RCC_PLLP_DIV2; o.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&o) != HAL_OK) Error_Handler();
    c.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    c.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    c.AHBCLKDivider = RCC_SYSCLK_DIV1;
    c.APB1CLKDivider = RCC_HCLK_DIV4;
    c.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&c, FLASH_LATENCY_5) != HAL_OK) Error_Handler();
}

void Error_Handler(void) { __disable_irq(); while (1) {} }
