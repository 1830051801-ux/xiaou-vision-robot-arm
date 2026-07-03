/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <rtthread.h>
/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */
#define SERVO_CAN hcan1

/* CAN RTOS 同步对象 (在 app_threads.c 中定义, app_threads_init() 中创建) */
extern rt_sem_t   sem_can_rx_done;  /* CAN RX 完成信号量, ISR 释放 */
extern rt_mutex_t mutex_can_tx;     /* CAN TX 互斥锁, 串行化多线程发送 */
extern rt_mutex_t mutex_can_rx;     /* CAN RX 互斥锁, 串行化多线程接收 */

void MX_CAN1_Init(void);
void Can_Config(void);
uint8_t Can_Send_Msg(uint32_t id,uint8_t len,uint8_t *data);
void can_sync_init(void);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *SERVO_CAN);
/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

