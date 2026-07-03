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
  hcan1.Init.AutoBusOff = DISABLE;
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

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;     /* 上拉: 无收发器时也能退出init */
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
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
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

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
if(HAL_CAN_ActivateNotification(&SERVO_CAN,CAN_IT_RX_FIFO0_MSG_PENDING)!=HAL_OK)
    {
        Error_Handler();
    }
    if(HAL_CAN_Start(&SERVO_CAN)!=HAL_OK)
    {
        Error_Handler();
    }
}

uint8_t Can_Send_Msg(uint32_t id,uint8_t len,uint8_t *data) {
    uint32_t i=0;
    static  uint32_t  TxMailbox;
    CAN_TxHeaderTypeDef  CAN_TxHeader;
    HAL_StatusTypeDef  HAL_RetVal;
    CAN_TxHeader.IDE = CAN_ID_STD;
    CAN_TxHeader.StdId = id;
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
    /* 释放信号量通知等待线程 (ISR 上下文安全) */
    if (sem_can_rx_done != RT_NULL)
        rt_sem_release(sem_can_rx_done);
    }
}
/* USER CODE BEGIN 1 */

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
