/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* USART1: 树莓派通讯 (PA9-TX, PA10-RX), 接收物体坐标 */
extern UART_HandleTypeDef huart1;

/* USART3: 上位机调试 printf (PB10-TX, PB11-RX) */
extern UART_HandleTypeDef huart3;

/* USER CODE BEGIN Private defines */

/* 树莓派接收缓冲区大小 */
#define RPI_RX_BUF_SIZE   256

/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);
void MX_USART3_UART_Init(void);

/* 树莓派 UART 接收接口 */
void Rpi_Uart_StartRx(void);
int  Rpi_Uart_GetChar(void);
int  Rpi_Uart_Available(void);
void Rpi_Uart_Send(const uint8_t *data, uint16_t len);   /* v1.2: USART1 发送 */

/* 调试 printf 重定向到 USART3 */
int  Dbg_Printf_Init(void);
void rt_hw_console_output(const char *str);

/* USER CODE BEGIN Prototypes */
void Dbg_Uart_StartRx(void);
int  Dbg_Uart_GetChar(void);
int  Dbg_Uart_Available(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

