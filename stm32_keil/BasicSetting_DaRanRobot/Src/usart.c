/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   USART1: 树莓派通讯 (接收物体坐标)
  *          USART2: 上位机调试 printf 重定向
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
#include "usart.h"

/* USER CODE BEGIN 0 */

/* ---- 树莓派 USART1 接收缓冲区 ---- */
static uint8_t  rpi_rx_buf[RPI_RX_BUF_SIZE];
static volatile uint16_t rpi_rx_head = 0;
static volatile uint16_t rpi_rx_tail = 0;

/* ---- 调试 USART2 接收缓冲区 (协议测试) ---- */
#define DBG_RX_BUF_SIZE   1024
static uint8_t  dbg_rx_buf[DBG_RX_BUF_SIZE];
static volatile uint16_t dbg_rx_head = 0;
static volatile uint16_t dbg_rx_tail = 0;

/* ---- 调试 USART2 发送状态 ---- */
static volatile uint8_t dbg_tx_done = 1;

/* USER CODE END 0 */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}
/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* ========================================================================== */
/*  树莓派通讯 (USART1) — 中断接收物体坐标                                    */
/* ========================================================================== */

/**
 * @brief  启动 USART1 中断接收 (单字节循环)
 */
void Rpi_Uart_StartRx(void)
{
    HAL_UART_Receive_IT(&huart1, rpi_rx_buf, 1);
}

/**
 * @brief  检查环形缓冲区中是否有数据
 * @retval 1=有数据, 0=空
 */
int Rpi_Uart_Available(void)
{
    return (rpi_rx_head != rpi_rx_tail);
}

/**
 * @brief  从环形缓冲区读取一个字节 (阻塞)
 * @retval 读取的字节
 */
int Rpi_Uart_GetChar(void)
{
    while (rpi_rx_head == rpi_rx_tail);  /* 等待数据 */
    uint8_t c = rpi_rx_buf[rpi_rx_tail];
    rpi_rx_tail = (rpi_rx_tail + 1) % RPI_RX_BUF_SIZE;
    return c;
}

/**
 * @brief  USARTx RX 中断回调 — 存入对应环形缓冲区
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        uint16_t next = (rpi_rx_head + 1) % RPI_RX_BUF_SIZE;
        if (next != rpi_rx_tail)
        {
            rpi_rx_buf[rpi_rx_head] = rpi_rx_buf[0];  /* HAL已存入[0], 搬到环形区 */
            rpi_rx_head = next;
        }
        HAL_UART_Receive_IT(&huart1, rpi_rx_buf, 1);
    }
    else if (huart->Instance == USART2)
    {
        uint16_t next = (dbg_rx_head + 1) % DBG_RX_BUF_SIZE;
        if (next != dbg_rx_tail)
        {
            dbg_rx_buf[dbg_rx_head] = dbg_rx_buf[0];  /* HAL已存入[0], 搬到环形区 */
            dbg_rx_head = next;
        }
        HAL_UART_Receive_IT(&huart2, dbg_rx_buf, 1);
    }
}

/* ========================================================================== */
/*  USART2 协议测试接收 (调试串口 RX)                                         */
/* ========================================================================== */

/** @brief 启动 USART2 中断接收 */
void Dbg_Uart_StartRx(void)
{
    HAL_UART_Receive_IT(&huart2, dbg_rx_buf, 1);
}

/** @return 1=有数据, 0=空 */
int Dbg_Uart_Available(void)
{
    return (dbg_rx_head != dbg_rx_tail);
}

/** @brief 从 USART2 环形缓冲区读取一个字节 (阻塞) */
int Dbg_Uart_GetChar(void)
{
    while (dbg_rx_head == dbg_rx_tail);
    uint8_t c = dbg_rx_buf[dbg_rx_tail];
    dbg_rx_tail = (dbg_rx_tail + 1) % DBG_RX_BUF_SIZE;
    return c;
}

/* ========================================================================== */
/*  调试 printf (USART2) — 重定向 stdout 到 PA2-TX                           */
/* ========================================================================== */

/**
 * @brief  初始化调试串口 (已由 MX_USART2_UART_Init 完成, 此处可加额外配置)
 */
int Dbg_Printf_Init(void)
{
    dbg_tx_done = 1;
    return 0;
}

/**
 * @brief  fputc 重定向 — 标准 printf 输出到 USART2
 * @note   Keil ARMCC 编译器自动调用此函数
 *         HAL_UART_Transmit 内部已处理 TXE 等待，无需额外忙等
 */
int fputc(int ch, FILE *f)
{
    (void)f;
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

/**
 * @brief  RT-Thread 控制台输出 — rt_kprintf 会调用此函数
 */
void rt_hw_console_output(const char *str)
{
    while (*str)
    {
        fputc(*str++, NULL);
    }
}

/* USER CODE END 1 */
