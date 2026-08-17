/**
 ******************************************************************************
 * @file    can_verify.c
 * @brief   CAN 板级驱动验证程序 — 周期性发送测试帧
 * @date    2026-08-02
 ******************************************************************************
 * @attention
 * 用途:
 *   验证 STM32F407 CAN1 硬件和 HAL 驱动是否正常工作。
 *   用 CAN 盒子在总线上抓包，确认帧 ID 和数据内容与预期一致。
 *
 * 使用方法:
 *   1. 将本文件添加到 MDK 工程 (替换 main.c 或条件编译)
 *   2. 编译烧录
 *   3. CAN 盒子连接到 CANH/CANL (PA12=TX, PA11=RX)
 *   4. 串口工具连接 USART3 (PA2/PA3, 115200) 查看日志
 *
 * 预期结果:
 *   - 串口打印 "CAN Verify Start" 和每包的发送日志
 *   - CAN 盒子上看到:
 *       帧 1: ID=0x100, DLC=8, Data=00 01 02 03 04 05 06 07, 周期 1000ms
 *       帧 2: ID=0x200, DLC=4, Data=DE AD BE EF,               周期 1000ms
 *       帧 3: ID=0x300, DLC=8, Data=AA 55 AA 55 AA 55 AA 55,   周期 500ms
 *
 * CAN 配置:
 *   波特率: 1Mbps (42MHz PCLK1 / prescaler=7 / 6TQ)
 *   引脚:   PA11=CAN_RX, PA12=CAN_TX
 *   模式:   Normal, Auto-Retransmit
 ******************************************************************************
 */

#include "main.h"
#include "can.h"
#include "usart.h"
#include "gpio.h"
#include <rtthread.h>
#include <string.h>

extern void rt_hw_board_init(void);
void SystemClock_Config(void);

/* ======================================================================== */
/*   测试帧定义                                                              */
/* ======================================================================== */

/* 测试帧 1: 递增序列, 便于在 CAN 盒子上确认数据完整性 */
static uint8_t test_data_1[8] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07 };

/* 测试帧 2: 固定魔术字 */
static uint8_t test_data_2[4] = { 0xDE, 0xAD, 0xBE, 0xEF };

/* 测试帧 3: 交替模式 */
static uint8_t test_data_3[8] = { 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55 };

/* ======================================================================== */
/*   测试线程                                                                */
/* ======================================================================== */
static rt_uint8_t  can_verify_stack[1024];
static struct rt_thread can_verify_tcb;

static void can_verify_thread(void *parameter)
{
    uint32_t packet_count = 0;
    uint8_t  seq = 0;

    rt_thread_mdelay(500);  /* 等待 CAN 初始化完成 */

    rt_kprintf("\n");
    rt_kprintf("========================================\n");
    rt_kprintf("  CAN Board-Level Verification Test\n");
    rt_kprintf("  Baudrate: 1Mbps\n");
    rt_kprintf("  TX Pin:   PA12\n");
    rt_kprintf("  RX Pin:   PA11\n");
    rt_kprintf("========================================\n");
    rt_kprintf("\n");
    rt_kprintf("Connect CAN analyzer to CANH/CANL.\n");
    rt_kprintf("Expected frames:\n");
    rt_kprintf("  ID=0x100  DLC=8  周期 1000ms  [00 01 02 ... 07]\n");
    rt_kprintf("  ID=0x200  DLC=4  周期 1000ms  [DE AD BE EF]\n");
    rt_kprintf("  ID=0x300  DLC=8  周期  500ms  [AA 55 AA 55 AA 55 AA 55]\n");
    rt_kprintf("\n");

    while (1)
    {
        rt_kprintf("[%4lu] --- Cycle %u ---\n", rt_tick_get(), (unsigned int)packet_count);

        /* ---- 帧 1: ID=0x100, DLC=8, 递增序列 ---- */
        test_data_1[0] = seq;  /* 首字节递增, 验证帧顺序 */
        {
            uint8_t ret = Can_Send_Msg(0x100, 8, test_data_1);
            rt_kprintf("  TX ID=0x100 DLC=8 [%02X %02X %02X %02X %02X %02X %02X %02X]  ret=%d\n",
                       test_data_1[0], test_data_1[1], test_data_1[2], test_data_1[3],
                       test_data_1[4], test_data_1[5], test_data_1[6], test_data_1[7], ret);
        }

        /* ---- 帧 2: ID=0x200, DLC=4, 固定魔术字 ---- */
        {
            uint8_t ret = Can_Send_Msg(0x200, 4, test_data_2);
            rt_kprintf("  TX ID=0x200 DLC=4 [%02X %02X %02X %02X]                    ret=%d\n",
                       test_data_2[0], test_data_2[1], test_data_2[2], test_data_2[3], ret);
        }

        /* ---- 帧 3: ID=0x300, DLC=8, 交替模式 (每 500ms发一次, 所以 1秒内发2次) ---- */
        {
            uint8_t ret = Can_Send_Msg(0x300, 8, test_data_3);
            rt_kprintf("  TX ID=0x300 DLC=8 [%02X %02X %02X %02X %02X %02X %02X %02X]  ret=%d\n",
                       test_data_3[0], test_data_3[1], test_data_3[2], test_data_3[3],
                       test_data_3[4], test_data_3[5], test_data_3[6], test_data_3[7], ret);
        }

        rt_thread_mdelay(500);  /* 帧3 每隔 500ms 再发一次 */

        /* ---- 帧 3: 第二次 ---- */
        {
            /* 翻转交替模式 */
            for (int i = 0; i < 8; i++) test_data_3[i] = ~test_data_3[i];
            uint8_t ret = Can_Send_Msg(0x300, 8, test_data_3);
            rt_kprintf("  TX ID=0x300 DLC=8 [%02X %02X %02X %02X %02X %02X %02X %02X]  ret=%d\n",
                       test_data_3[0], test_data_3[1], test_data_3[2], test_data_3[3],
                       test_data_3[4], test_data_3[5], test_data_3[6], test_data_3[7], ret);
            for (int i = 0; i < 8; i++) test_data_3[i] = ~test_data_3[i];  /* 恢复 */
        }

        seq++;
        packet_count++;
        rt_thread_mdelay(500);  /* 总计 1000ms 一个完整周期 */
    }
}

/* ======================================================================== */
/*   硬件初始化                                                              */
/* ======================================================================== */
static int can_hw_init(void)
{
    rt_kprintf("[HW] Initializing CAN1...\n");

    /* 使用 HAL 标准初始化流程 (与生产代码一致) */
    MX_CAN1_Init();
    Can_Config();

    /* 启动 CAN 外设 */
    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        rt_kprintf("[HW] CAN1 Start FAILED!\n");
        return -1;
    }

    /* 使能接收中断 (可选, 测试不需要但保持完整) */
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        rt_kprintf("[HW] CAN1 Interrupt enable FAILED!\n");
        return -1;
    }

    /* 打印诊断信息 */
    rt_kprintf("[HW] CAN1 Init OK\n");
    rt_kprintf("[HW]   Instance: 0x%08X\n", (unsigned int)hcan1.Instance);
    rt_kprintf("[HW]   Prescaler: %lu\n", (unsigned long)hcan1.Init.Prescaler);
    rt_kprintf("[HW]   Mode: Normal\n");
    rt_kprintf("[HW]   MCR=0x%08lX  MSR=0x%08lX  TSR=0x%08lX\n",
               CAN1->MCR, CAN1->MSR, CAN1->TSR);
    rt_kprintf("[HW]   ESR=0x%08lX\n", CAN1->ESR);

    /* 检查错误状态 */
    if (CAN1->ESR & CAN_ESR_BOFF) {
        rt_kprintf("[HW] *** WARNING: CAN Bus-Off detected! Check wiring. ***\n");
    }
    if (CAN1->ESR & CAN_ESR_EWGF) {
        rt_kprintf("[HW] *** WARNING: CAN Error Warning. Check termination/bus. ***\n");
    }

    return 0;
}

/* ======================================================================== */
/*   主入口                                                                  */
/* ======================================================================== */
int main(void)
{
    /* ---- HAL 初始化 ---- */
    HAL_Init();
    SystemClock_Config();

    /* ---- RT-Thread 内核初始化 ---- */
    rt_hw_board_init();
    NVIC_SetPriority(PendSV_IRQn, 0x0F);

    /* ---- 调试串口 (USART3) ---- */
    MX_USART3_UART_Init();
    Dbg_Printf_Init();

    rt_show_version();
    rt_system_timer_init();
    rt_system_scheduler_init();
    rt_system_timer_thread_init();
    rt_thread_idle_init();

    /* ---- GPIO 基础初始化 ---- */
    MX_GPIO_Init();

    /* ---- CAN 硬件初始化 ---- */
    if (can_hw_init() != 0) {
        rt_kprintf("[FATAL] CAN init failed, system halted.\n");
        while (1) {}
    }

    rt_kprintf("\n[HW] All hardware initialized.\n\n");

    /* ---- 创建 CAN 验证线程 ---- */
    rt_err_t result = rt_thread_init(&can_verify_tcb, "can_vfy",
                                      can_verify_thread,
                                      RT_NULL,
                                      can_verify_stack,
                                      sizeof(can_verify_stack),
                                      15,     /* priority 15 (低于系统关键线程) */
                                      5);
    if (result == RT_EOK) {
        rt_thread_startup(&can_verify_tcb);
        rt_kprintf("[Main] CAN verify thread created, starting scheduler...\n");
    } else {
        rt_kprintf("[Main] ERROR: Failed to create CAN verify thread!\n");
        while (1) {}
    }

    /* ---- 启动 RT-Thread 调度器 (永不返回) ---- */
    rt_system_scheduler_start();

    while (1) {}
}

/**
 * @brief System Clock Configuration (HSE 8MHz → PLL 168MHz)
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSI;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.LSIState = RCC_LSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        Error_Handler();

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
        Error_Handler();
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}
