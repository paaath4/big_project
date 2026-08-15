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
/* 队列句柄(在 MX_FREERTOS_Init 里创建, can.c 里 extern 使用) */
osMessageQueueId_t canRxQ;

/* 共享状态(Can_Rx 任务更新, 其它任务读取) */
volatile uint8_t  g_breath_on    = 0;
volatile uint16_t g_period_ms    = 2000;   /* 呼吸周期 ms, 由主板 0x001 指令设置 */
volatile float    g_breath_val   = 0.0f;   /* 当前呼吸亮度 0~1, 也是 100Hz 反馈的 float */
volatile uint8_t  g_master_alive = 0;      /* 收到主板 0x02010101 置1 */

extern void can_start(void);
extern void can_send_frame(const can_frame_t *f);
void beep_times(uint8_t n);   /* 定义在文件底部, 先声明供 Can_Rx 调用 */
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
  /* CAN 中断 -> Can_Rx 任务 传数据用 */
  canRxQ = osMessageQueueNew(16, sizeof(can_frame_t), NULL);  /* 收到的 CAN 帧 */
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
  /* 呼吸灯: 按周期更新 PC6 上 TIM3_CH1 的 PWM 占空比 */
  uint32_t t = 0;
  for (;;)
  {
    if (g_breath_on)
    {
      float phase = 2.0f * 3.14159f * (float)t / (float)g_period_ms;
      g_breath_val = 0.5f + 0.5f * sinf(phase);              /* 0~1 正弦 */
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,
                            (uint32_t)(g_breath_val * 999.0f));
      t = (t + 10) % g_period_ms;
    }
    else
    {
      g_breath_val = 0.0f;
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    }
    osDelay(10);
  }
  /* USER CODE END Breath_Led */
}

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
  /* 状态灯: 两颗灯不同频率闪烁, 指示系统运行 */
  uint32_t cnt = 0;
  for (;;)
  {
    cnt++;
    if (cnt % 5 == 0)  HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin);  /* 每 500ms */
    if (cnt % 10 == 0) HAL_GPIO_TogglePin(LED_2_GPIO_Port, LED_2_Pin);  /* 每 1000ms */
    osDelay(100);
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
  /* CAN 接收: 处理主板指令/蜂鸣/主板状态 */
  can_start();                       /* 过滤+启动+使能接收中断 */

  can_frame_t f;
  for (;;)
  {
    if (osMessageQueueGet(canRxQ, &f, NULL, osWaitForever) == osOK)
    {
      switch (f.id)
      {
        case ID_BREATH_CTRL:         /* 主板呼吸灯指令 0x001 */
          if (f.dlc >= 3)
          {
            g_breath_on = (f.data[0] == CTRL_BYTE_ON) ? 1 : 0;
            g_period_ms = (uint16_t)(f.data[1] | (f.data[2] << 8));
            if (g_period_ms < 100) g_period_ms = 100;
          }
          break;
        case ID_BEEP_CMD:            /* CANable 蜂鸣 0x003 */
          if (f.dlc >= 1) beep_times(f.data[0]);
          break;
        case ID_MASTER_STATUS:       /* 主板状态 0x02010101 (证明过滤放行) */
          g_master_alive = 1;
          break;
        default:
          break;
      }
    }
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
  /* 100Hz 反馈: 每 10ms 发一次 0x002, 数据为当前呼吸亮度 float */
  uint32_t last = osKernelGetTickCount();
  for (;;)
  {
    float v = g_breath_val;              /* 先拷到非 volatile 局部变量 */
    can_frame_t f;
    f.id  = ID_SLAVE_FEEDBACK;
    f.dlc = 4;
    memcpy(f.data, &v, 4);               /* float -> 4 字节小端, 和主板解析一致 */

    can_send_frame(&f);
    last += 10;
    osDelayUntil(last);                  /* 精确 10ms = 100Hz */
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
  /* 周期发送从板状态 0x012 */
  uint32_t last = osKernelGetTickCount();
  for (;;)
  {
    can_frame_t f;
    f.id      = ID_SLAVE_STATUS;
    f.dlc     = 1;
    f.data[0] = (uint8_t)g_breath_on;    /* 状态字节: 当前呼吸开关 */

    can_send_frame(&f);
    last += 500;
    osDelayUntil(last);                  /* 每 500ms */
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

