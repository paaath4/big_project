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
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
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
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_TX_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
    HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
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
    HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_SCE_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
#include "cmsis_os.h"
#include "app_proto.h"

extern osMessageQueueId_t canRxQ;

/* 主板硬件过滤: 只收从板反馈0x002 / 从板状态0x012 / 蜂鸣指令0x003,
 * 500Hz 噪声等其它 ID 在硬件层直接被丢弃 */
static void can_config_filter(void)
{
  const uint32_t ids[3] = { ID_SLAVE_FEEDBACK, ID_SLAVE_STATUS, ID_BEEP_CMD };

  for (int i = 0; i < 3; i++)
  {
    CAN_FilterTypeDef f = {0};
    f.FilterActivation    = CAN_FILTER_ENABLE;
    f.FilterBank          = i;                     /* 用前 3 个过滤器 */
    f.FilterMode          = CAN_FILTERMODE_IDMASK;
    f.FilterScale         = CAN_FILTERSCALE_32BIT;
    f.FilterIdHigh        = (uint16_t)(ids[i] << 5);   /* 标准帧 32bit 过滤器格式 */
    f.FilterIdLow         = 0;
    f.FilterMaskIdHigh    = (uint16_t)(0x7FFu << 5);   /* 掩码全 1 = 精确匹配 */
    f.FilterMaskIdLow     = 0;
    f.FilterFIFOAssignment = CAN_RX_FIFO0;
    if (HAL_CAN_ConfigFilter(&hcan1, &f) != HAL_OK)
    {
      Error_Handler();
    }
  }
}

/* 在任务里调用: 配置过滤 -> 启动 CAN -> 使能 RX0 中断 */
void can_start(void)
{
  can_config_filter();
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/* 标准帧/扩展帧自适应发送 */
void can_send_frame(const can_frame_t *f)
{
  CAN_TxHeaderTypeDef tx = {0};
  uint32_t mailbox;

  if (f->id > 0x7FF)               /* 0x02010101 为扩展帧 */
  {
    tx.IDE   = CAN_ID_EXT;
    tx.ExtId = f->id;
  }
  else
  {
    tx.IDE   = CAN_ID_STD;
    tx.StdId = f->id;
  }
  tx.RTR = CAN_RTR_DATA;
  tx.DLC = f->dlc;
  HAL_CAN_AddTxMessage(&hcan1, &tx, (uint8_t*)f->data, &mailbox);
}

/* RX0 中断回调(ISR): 取报文入队, 任务里再处理 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance != CAN1) return;

  can_frame_t f;
  CAN_RxHeaderTypeDef rx;
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx, f.data) == HAL_OK)
  {
    f.id  = (rx.IDE == CAN_ID_EXT) ? rx.ExtId : rx.StdId;
    f.dlc = rx.DLC;
    osMessageQueuePut(canRxQ, &f, 0, 0);
  }
}
/* USER CODE END 1 */

