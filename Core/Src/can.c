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
#include "FreeRTOS.h"
#include "queue.h"
#include <string.h>
#include "app_proto.h"

/* USER CODE BEGIN 0 */


// 从板状态变量
volatile uint8_t g_breath_enable = 0;
volatile uint16_t g_breath_period = 1000;
volatile uint8_t g_beep_times = 0;
volatile uint8_t g_master_alive = 0; // 收到主板0x02010101时置1

// 外部队列句柄
extern QueueHandle_t xBreathQueue;
extern QueueHandle_t xBeepQueue;
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
  hcan1.Init.Prescaler = 7;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
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

void HAL_CAN_MspInit(CAN_HandleTypeDef *canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (canHandle->Instance == CAN1)
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
    GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
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

void HAL_CAN_MspDeInit(CAN_HandleTypeDef *canHandle)
{

  if (canHandle->Instance == CAN1)
  {
    /* USER CODE BEGIN CAN1_MspDeInit 0 */

    /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);

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
  CAN_FilterTypeDef f = {0};

  // ----- 过滤器0：接收标准帧 ID_BREATH_CTRL (0x001) 和 ID_BEEP_CMD (0x003) -----
  f.FilterBank = 0;
  f.FilterMode = CAN_FILTERMODE_IDLIST; // 列表模式，精确匹配
  f.FilterScale = CAN_FILTERSCALE_32BIT;
  // 32位列表：低16位存第一个ID（左移5位），高16位存第二个ID
  f.FilterIdHigh = (ID_BREATH_CTRL << 5) & 0xFFFF;
  f.FilterIdLow = (ID_BEEP_CMD << 5) & 0xFFFF;
  f.FilterMaskIdHigh = 0xFFFF; // 掩码全1，精确匹配
  f.FilterMaskIdLow = 0xFFFF;
  f.FilterFIFOAssignment = CAN_RX_FIFO0;
  f.FilterActivation = ENABLE;
  HAL_CAN_ConfigFilter(&hcan1, &f);

  // ----- 过滤器1：接收扩展帧 ID_MASTER_STATUS (0x02010101) -----
  f.FilterBank = 1;
  f.FilterMode = CAN_FILTERMODE_IDMASK; // 掩码模式
  f.FilterScale = CAN_FILTERSCALE_32BIT;
  // 扩展帧ID存储：高16位放ID>>13，低16位放(ID<<3)|CAN_ID_EXT
  uint32_t ext_id = ID_MASTER_STATUS;
  f.FilterIdHigh = (uint16_t)(ext_id >> 13);
  f.FilterIdLow = (uint16_t)((ext_id << 3) | CAN_ID_EXT);
  f.FilterMaskIdHigh = 0xFFFF; // 精确匹配
  f.FilterMaskIdLow = 0xFFFF;
  f.FilterFIFOAssignment = CAN_RX_FIFO0;
  f.FilterActivation = ENABLE;
  HAL_CAN_ConfigFilter(&hcan1, &f);
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

  if (f->id > 0x7FF) /* 0x02010101 为扩展帧 */
  {
    tx.IDE = CAN_ID_EXT;
    tx.ExtId = f->id;
  }
  else
  {
    tx.IDE = CAN_ID_STD;
    tx.StdId = f->id;
  }
  tx.RTR = CAN_RTR_DATA;
  tx.DLC = f->dlc;
  HAL_CAN_AddTxMessage(&hcan1, &tx, (uint8_t *)f->data, &mailbox);
}

/* RX0 中断回调(ISR): 取报文入队, 任务里再处理 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance != CAN1)
    return;

  CAN_RxHeaderTypeDef rxHeader;
  uint8_t rxData[8];
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData);

  // 处理标准帧
  if (rxHeader.IDE == CAN_ID_STD)
  {
    switch (rxHeader.StdId)
    {
    case ID_BREATH_CTRL: // 0x001
      g_breath_enable = rxData[0];
      g_breath_period = (rxData[1] << 8) | rxData[2];
      if (xBreathQueue != NULL)
      {
        uint32_t cmd = (g_breath_enable << 16) | g_breath_period;
        xQueueSendFromISR(xBreathQueue, &cmd, NULL);
      }
      break;

    case ID_BEEP_CMD: // 0x003
      g_beep_times = rxData[0];
      if (xBeepQueue != NULL)
      {
        xQueueSendFromISR(xBeepQueue, &g_beep_times, NULL);
      }
      break;

      // 0x012 是从板自己发送的状态帧，从板无需接收，忽略
      // 0x002 也是从板发送的反馈，忽略
    }
  }
  else // 扩展帧
  {
    if (rxHeader.ExtId == ID_MASTER_STATUS) // 0x02010101
    {
      g_master_alive = 1;
      // 可以翻转LED指示（例如用板上的某个LED）
      // HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    }
  }
}
/* USER CODE END 1 */
