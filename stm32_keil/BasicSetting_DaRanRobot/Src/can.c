/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "can.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_it.h"
#include <rtthread.h>
#include <string.h>
extern uint8_t rx_buffer[8];
extern int8_t READ_FLAG;
extern uint16_t can_id;

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 7;  /* 42MHz / (7×6TQ) = 1Mbps */
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_4TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;   /* bus-off 自动恢复 */
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PB8      ------> CAN1_RX
    PB9      ------> CAN1_TX
    */
    /* PB8=CAN1_RX, PB9=CAN1_TX — 必须都是 AF 模式! */
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


    /* CAN1 interrupt Init — 最低优先级, 不得抢占 PendSV */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0x0F, 0x00);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}
void Can_Config(void) {
    CAN_FilterTypeDef  CAN_FilterType;
    CAN_FilterType.FilterBank=0;
    CAN_FilterType.FilterIdHigh=0x0000;
    CAN_FilterType.FilterIdLow=0x0000;
    CAN_FilterType.FilterMaskIdHigh=0x0000;
    CAN_FilterType.FilterMaskIdLow=0x0000;
    CAN_FilterType.FilterFIFOAssignment=CAN_RX_FIFO0;
    CAN_FilterType.FilterMode=CAN_FILTERMODE_IDMASK;
    CAN_FilterType.FilterScale=CAN_FILTERSCALE_32BIT;
    CAN_FilterType.FilterActivation=ENABLE;
    CAN_FilterType.SlaveStartFilterBank=14;
    if(HAL_CAN_ConfigFilter(&SERVO_CAN,&CAN_FilterType)!=HAL_OK)
    {
        Error_Handler();
    }
    /* 先 Start, 再激活中断 — 避免 Start 瞬间假中断挂死调度器 */
    if(HAL_CAN_Start(&SERVO_CAN)!=HAL_OK)
    {
        Error_Handler();
    }
    if(HAL_CAN_ActivateNotification(&SERVO_CAN,CAN_IT_RX_FIFO0_MSG_PENDING)!=HAL_OK)
    {
        Error_Handler();
    }
}

uint8_t Can_Send_Msg(uint32_t id,uint8_t len,uint8_t *data) {
    uint32_t i=0;
    static  uint32_t  TxMailbox;
    CAN_TxHeaderTypeDef  CAN_TxHeader;
    HAL_StatusTypeDef  HAL_RetVal;
    if (id > 0x7FF) {
        CAN_TxHeader.IDE = CAN_ID_EXT;
        CAN_TxHeader.ExtId = id;
    } else {
        CAN_TxHeader.IDE = CAN_ID_STD;
        CAN_TxHeader.StdId = id;
    }
    CAN_TxHeader.DLC = len;
    CAN_TxHeader.RTR = CAN_RTR_DATA;
    CAN_TxHeader.TransmitGlobalTime = DISABLE;

    /* 等待空闲发送邮箱 — 让出 CPU 给其他线程 */
    while(HAL_CAN_GetTxMailboxesFreeLevel(&SERVO_CAN) == 0)
    {
       i++;
       if(i > 1000)  /* 超时 ~1s (1000 * rt_thread_mdelay(1)) */
            return 1;
       rt_thread_mdelay(1);  /* 让出 CPU，不再 busy-wait */
    }
    HAL_RetVal = HAL_CAN_AddTxMessage(&SERVO_CAN,&CAN_TxHeader,data,&TxMailbox);

    if(HAL_RetVal != HAL_OK)
        return  1;
    return  0;
}
void  HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *SERVO_CAN)
{
    CAN_RxHeaderTypeDef  hCAN1_RxHeader;

    if(HAL_CAN_GetRxMessage(SERVO_CAN, CAN_RX_FIFO0, &hCAN1_RxHeader, rx_buffer) == HAL_OK)
    {
    can_id = hCAN1_RxHeader.StdId;
    READ_FLAG = 1;

    /* Pi v1.2: ISR 驱动反馈数据路由 — DLC==8 过滤反馈帧, 跳过 ACK
     * 将 8 字节原始数据直接存入 per-joint 缓冲区, 线程无需 CAN 读取 */
    {
        uint8_t servo_id = (uint8_t)((can_id & 0x07E0) >> 5);
        if (servo_id >= 1 && servo_id <= 6) {
            int idx = servo_id - 1;
            g_last_feedback_tick[idx] = rt_tick_get();
            if (hCAN1_RxHeader.DLC == 8) {
                memcpy(g_raw_fb[idx].data, rx_buffer, 8);
                g_raw_fb[idx].fresh = 1;
            }
        }
    }

    /* ---- 回环测试: ISR 只存数据到全局缓冲区, 不调 rt_kprintf! ---- */
    if (g_can_loopback_active) {
        g_lb_rx_id = (hCAN1_RxHeader.IDE == CAN_ID_EXT)
                     ? hCAN1_RxHeader.ExtId : hCAN1_RxHeader.StdId;
        g_lb_rx_ide = (hCAN1_RxHeader.IDE == CAN_ID_EXT) ? 1 : 0;
        g_lb_rx_dlc = hCAN1_RxHeader.DLC;
        if (g_lb_rx_dlc > 8) g_lb_rx_dlc = 8;
        for (int i = 0; i < g_lb_rx_dlc; i++)
            g_lb_rx_data[i] = rx_buffer[i];
        g_lb_rx_flag = 1;
    }

    /* ---- CAN 总线监听 (canmon on) ---- */
    if (g_can_monitor) {
        g_cm_rx_id  = (hCAN1_RxHeader.IDE == CAN_ID_EXT)
                      ? hCAN1_RxHeader.ExtId : hCAN1_RxHeader.StdId;
        g_cm_rx_ide = (hCAN1_RxHeader.IDE == CAN_ID_EXT) ? 1 : 0;
        g_cm_rx_dlc = hCAN1_RxHeader.DLC;
        if (g_cm_rx_dlc > 8) g_cm_rx_dlc = 8;
        for (int i = 0; i < g_cm_rx_dlc; i++)
            g_cm_rx_data[i] = rx_buffer[i];
        g_cm_rx_flag = 1;
    }

    /* 释放信号量通知等待线程 (ISR 上下文安全) */
    if (sem_can_rx_done != RT_NULL)
        rt_sem_release(sem_can_rx_done);
    }
}
/* USER CODE BEGIN 1 */

/* CAN 回环测试 — ISR→线程 通信缓冲区 */
int      g_can_loopback_active = 0;
uint32_t g_lb_rx_id   = 0;
uint8_t  g_lb_rx_ide  = 0;
uint8_t  g_lb_rx_dlc  = 0;
uint8_t  g_lb_rx_data[8];
volatile int g_lb_rx_flag = 0;  /* 1 = 有新数据待打印 */

/* CAN 总线监听 — 原始帧打印 */
int      g_can_monitor = 0;
uint32_t g_cm_rx_id   = 0;
uint8_t  g_cm_rx_ide  = 0;
uint8_t  g_cm_rx_dlc  = 0;
uint8_t  g_cm_rx_data[8];
volatile int g_cm_rx_flag = 0;  /* 1 = 有新帧待打印 */

/* Pi v1.2: ISR 驱动的关节在线检测 — 每个关节最后收到有效 CAN 反馈的 tick */
rt_tick_t g_last_feedback_tick[6] = {0};

/* Pi v1.2: ISR 驱动的反馈数据缓冲区 — DLC==8 的反馈帧由 ISR 直接路由至此
 * 结构体定义在 can.h, 此处仅定义数组实例 */
struct raw_fb_buf g_raw_fb[6];

void can_loopback_enable(void)
{
    /* 中止所有待发帧 (释放被 joint_data 占满的邮箱) */
    if (!(hcan1.Instance->TSR & CAN_TSR_TME0)) hcan1.Instance->TSR |= CAN_TSR_ABRQ0;
    if (!(hcan1.Instance->TSR & CAN_TSR_TME1)) hcan1.Instance->TSR |= CAN_TSR_ABRQ1;
    if (!(hcan1.Instance->TSR & CAN_TSR_TME2)) hcan1.Instance->TSR |= CAN_TSR_ABRQ2;
    rt_thread_mdelay(5);  /* 等待中止完成 */

    /* 请求 INIT 模式 */
    hcan1.Instance->MCR |= CAN_MCR_INRQ;
    uint32_t timeout = 1000000;
    while (!(hcan1.Instance->MSR & CAN_MSR_INAK) && --timeout);

    /* NART=1: 单次发送不重试，邮箱立即释放 */
    hcan1.Instance->MCR |= CAN_MCR_NART;
    /* 回环模式: 内部 TX→RX */
    hcan1.Instance->BTR |= CAN_BTR_LBKM;

    /* 退出 INIT */
    hcan1.Instance->MCR &= ~CAN_MCR_INRQ;
    timeout = 1000000;
    while ((hcan1.Instance->MSR & CAN_MSR_INAK) && --timeout);

    g_can_loopback_active = 1;
    g_lb_rx_flag = 0;
    rt_kprintf("[CAN] Loopback mode enabled\n");
}

void can_loopback_disable(void)
{
    g_can_loopback_active = 0;

    hcan1.Instance->MCR |= CAN_MCR_INRQ;
    uint32_t timeout = 1000000;
    while (!(hcan1.Instance->MSR & CAN_MSR_INAK) && --timeout);

    hcan1.Instance->BTR &= ~CAN_BTR_LBKM;

    hcan1.Instance->MCR &= ~CAN_MCR_INRQ;
    timeout = 1000000;
    while ((hcan1.Instance->MSR & CAN_MSR_INAK) && --timeout);

    rt_kprintf("[CAN] Loopback mode disabled\n");
}

/**
 * @brief CAN 同步初始化 (创建信号量后调用)
 *        可在此添加 CAN 相关的 RTOS 同步资源初始化
 */
void can_sync_init(void)
{
    /* CAN RX 信号量由 app_threads_init() 中的 IPC 创建 */
    /* 此函数预留用于未来扩展 (如 TX 完成信号量) */
}

/* USER CODE END 1 */
