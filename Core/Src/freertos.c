/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <string.h>
#include "app_proto.h"
#include "tim.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* 队列句柄(在 MX_FREERTOS_Init 里创建, can.c/usart.c 里 extern 使用) */
osMessageQueueId_t vofaCmdQ;
osMessageQueueId_t canTxQ;
osMessageQueueId_t canRxQ;

/* 共享状态 */
volatile uint8_t g_breath_on = 0;
volatile uint16_t g_period_ms = 2000; /* 呼吸周期, 可被 VOFA 指令修改 */
volatile float g_breath_val = 0.0f;   /* 主板自身呼吸亮度 0~1 */
volatile float g_slave_val = 0.0f;    /* 从板 100Hz 反馈的 float */
volatile uint8_t g_slave_valid = 0;
volatile uint8_t g_slave_status = 0;

extern void can_start(void);
extern void can_send_frame(const can_frame_t *f);
extern void vofa_uart_start_rx(void);
extern void vofa_send_floats(float *data, uint8_t n);

void beep_times(uint8_t n); /* 定义在文件底部, 先声明供 StartTask05 调用 */
/* USER CODE END Variables */
/* Definitions for BreathLed__Task */
osThreadId_t BreathLed__TaskHandle;
const osThreadAttr_t BreathLed__Task_attributes = {
  .name = "BreathLed__Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for StatusLed_Task */
osThreadId_t StatusLed_TaskHandle;
const osThreadAttr_t StatusLed_Task_attributes = {
  .name = "StatusLed_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Can_Rx_Task */
osThreadId_t Can_Rx_TaskHandle;
const osThreadAttr_t Can_Rx_Task_attributes = {
  .name = "Can_Rx_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Feedback_Task */
osThreadId_t Feedback_TaskHandle;
const osThreadAttr_t Feedback_Task_attributes = {
  .name = "Feedback_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Status_Send_Tsa */
osThreadId_t Status_Send_TsaHandle;
const osThreadAttr_t Status_Send_Tsa_attributes = {
  .name = "Status_Send_Tsa",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Breath_Led(void *argument);
void Status_Led(void *argument);
void Can_Rx(void *argument);
void Feedback(void *argument);
void Status_Send(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* 中断(串口/CAN) -> 任务 传数据用 */
  vofaCmdQ = osMessageQueueNew(4, 3, NULL);                  /* 3字节 VOFA 指令 */
  canTxQ = osMessageQueueNew(8, sizeof(can_frame_t), NULL);  /* 待发送 CAN 帧 */
  canRxQ = osMessageQueueNew(16, sizeof(can_frame_t), NULL); /* 收到的 CAN 帧 */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of BreathLed__Task */
  BreathLed__TaskHandle = osThreadNew(Breath_Led, NULL, &BreathLed__Task_attributes);

  /* creation of StatusLed_Task */
  StatusLed_TaskHandle = osThreadNew(Status_Led, NULL, &StatusLed_Task_attributes);

  /* creation of Can_Rx_Task */
  Can_Rx_TaskHandle = osThreadNew(Can_Rx, NULL, &Can_Rx_Task_attributes);

  /* creation of Feedback_Task */
  Feedback_TaskHandle = osThreadNew(Feedback, NULL, &Feedback_Task_attributes);

  /* creation of Status_Send_Tsa */
  Status_Send_TsaHandle = osThreadNew(Status_Send, NULL, &Status_Send_Tsa_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Breath_Led */
/**
 * @brief  Function implementing the BreathLed__Task thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_Breath_Led */
void Breath_Led(void *argument)
{
  /* USER CODE BEGIN Breath_Led */
  /* 软件 PWM 控制 LED1 (PA4) */
  uint16_t phase = 0; // 0~100 对应占空比步进
  while (1)
  {
    if (g_breath_on) // 全局变量由 CAN 命令更新
    {
      /* 正弦变化，周期为 g_period_ms (单位 ms) */
      float rad = 2.0f * 3.14159f * (float)phase / 100.0f;
      uint8_t duty = (uint8_t)(50 * (sinf(rad) + 1)); // 0~100
      uint16_t on_time = g_period_ms * duty / 100;
      uint16_t off_time = g_period_ms - on_time;

      HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
      osDelay(on_time);
      HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);
      osDelay(off_time);

      phase = (phase + 1) % 100;
    }
    else
    {
      HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);
      osDelay(100);
    }
  }
}
  /* USER CODE END Breath_Led */


/* USER CODE BEGIN Header_Status_Led */
/**
 * @brief Function implementing the StatusLed_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Status_Led */
void Status_Led(void *argument)
{
  /* USER CODE BEGIN Status_Led */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Status_Led */
}

/* USER CODE BEGIN Header_Can_Rx */
/**
 * @brief Function implementing the Can_Rx_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Can_Rx */
void Can_Rx(void *argument)
{
  /* USER CODE BEGIN Can_Rx */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Can_Rx */
}

/* USER CODE BEGIN Header_Feedback */
/**
 * @brief Function implementing the Feedback_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Feedback */
void Feedback(void *argument)
{
  /* USER CODE BEGIN Feedback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Feedback */
}

/* USER CODE BEGIN Header_Status_Send */
/**
 * @brief Function implementing the Status_Send_Tsa thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Status_Send */
void Status_Send(void *argument)
{
  /* USER CODE BEGIN Status_Send */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Status_Send */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* 蜂鸣器按次数响应 (定次数, 上限保护避免长时间阻塞任务) */
void beep_times(uint8_t n)
{
  if (n > 10)
    n = 10;
  for (uint8_t i = 0; i < n; i++)
  {
    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
    osDelay(100);
    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
    osDelay(100);
  }
}
/* USER CODE END Application */

