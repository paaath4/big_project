/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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

/* USER CODE END 0 */

UART_HandleTypeDef huart1;

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
    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
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
}

/* USER CODE BEGIN 1 */
#include "cmsis_os.h"
#include "app_proto.h"
#include <string.h>

extern osMessageQueueId_t vofaCmdQ;

static uint8_t vofa_rx_byte;
static uint8_t vofa_state = 0;   
static uint8_t vofa_buf[3];

/* 启动 单字节接收中断 */
void vofa_uart_start_rx(void)
{
  HAL_UART_Receive_IT(&huart1, &vofa_rx_byte, 1);
}

/* 每收一字节进来, 进行sitch判断; 完整帧 A5 XX XX XX 5A 入队 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART1) return;//若是串口1，进行下一步swtich

  uint8_t b = vofa_rx_byte;
  switch (vofa_state) //状态机进行选择
  {
    case 0:                          /* 等帧头 A5 */
      if (b == VOFA_FRAME_HEAD) vofa_state = 1;
      break;
    case 1:  vofa_buf[0] = b; vofa_state = 2; break;   /* 开/关 */
    case 2:  vofa_buf[1] = b; vofa_state = 3; break;   /* 周期低 */
    case 3:  vofa_buf[2] = b; vofa_state = 4; break;   /* 周期高 */
    case 4:                          /* 等帧尾 5A */
      vofa_state = 0;
      if (b == VOFA_FRAME_TAIL && vofaCmdQ != NULL)
      {
        osMessageQueuePut(vofaCmdQ, vofa_buf, 0, 0); //校验通过入队
      }
      break;
    default: vofa_state = 0; break;
  }
  vofa_uart_start_rx();              /* 重新启动，收下一个字节 */
}

/* JustFloat打波: n 个 float + 帧尾 00 00 80 7F */
void vofa_send_floats(float *data, uint8_t n)
{
  if (n > 3) n = 3; //最多n个
  uint8_t buf[16];
  for (uint8_t i = 0; i < n; i++)
  {
    memcpy(&buf[i * 4], &data[i], 4);   /* float -> 4 字节小端 */
  }
  buf[n*4 + 0] = 0x00;
  buf[n*4 + 1] = 0x00;
  buf[n*4 + 2] = 0x80;
  buf[n*4 + 3] = 0x7F;//justfloat协议尾 00 00 80 7F
  HAL_UART_Transmit(&huart1, buf, n*4 + 4, 10);
}
/* USER CODE END 1 */

