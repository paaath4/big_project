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
volatile uint8_t  g_breath_on    = 0;
volatile uint16_t g_period_ms    = 2000;   /* 呼吸周期, 可被 VOFA 指令修改 */
volatile float    g_breath_val   = 0.0f;   /* 主板自身呼吸亮度 0~1 */
volatile float    g_slave_val    = 0.0f;   /* 从板 100Hz 反馈的 float */
volatile uint8_t  g_slave_valid  = 0;
volatile uint8_t  g_slave_status = 0;

extern void can_start(void);
extern void can_send_frame(const can_frame_t *f);
extern void vofa_uart_start_rx(void);
extern void vofa_send_floats(float *data, uint8_t n);

void beep_times(uint8_t n);   /* 定义在文件底部, 先声明供 StartTask05 调用 */
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
/* Definitions for VOFA_Rx_Task */
osThreadId_t VOFA_Rx_TaskHandle;
const osThreadAttr_t VOFA_Rx_Task_attributes = {
  .name = "VOFA_Rx_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Can_Send_Task */
osThreadId_t Can_Send_TaskHandle;
const osThreadAttr_t Can_Send_Task_attributes = {
  .name = "Can_Send_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Can_Rx_Task */
osThreadId_t Can_Rx_TaskHandle;
const osThreadAttr_t Can_Rx_Task_attributes = {
  .name = "Can_Rx_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for VOFA_Send_Task */
osThreadId_t VOFA_Send_TaskHandle;
const osThreadAttr_t VOFA_Send_Task_attributes = {
  .name = "VOFA_Send_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Breath_Led(void *argument);
void Status_Led(void *argument);
void VOFA_Rx(void *argument);
void Can_Send(void *argument);
void Can_Rx(void *argument);
void VOFA_Send(void *argument);

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
  vofaCmdQ = osMessageQueueNew(4, 3, NULL);               /* 3字节 VOFA 指令 */
  canTxQ   = osMessageQueueNew(8, sizeof(can_frame_t), NULL);  /* 待发送 CAN 帧 */
  canRxQ   = osMessageQueueNew(16, sizeof(can_frame_t), NULL); /* 收到的 CAN 帧 */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of BreathLed__Task */
  BreathLed__TaskHandle = osThreadNew(Breath_Led, NULL, &BreathLed__Task_attributes);

  /* creation of StatusLed_Task */
  StatusLed_TaskHandle = osThreadNew(Status_Led, NULL, &StatusLed_Task_attributes);

  /* creation of VOFA_Rx_Task */
  VOFA_Rx_TaskHandle = osThreadNew(VOFA_Rx, NULL, &VOFA_Rx_Task_attributes);

  /* creation of Can_Send_Task */
  Can_Send_TaskHandle = osThreadNew(Can_Send, NULL, &Can_Send_Task_attributes);

  /* creation of Can_Rx_Task */
  Can_Rx_TaskHandle = osThreadNew(Can_Rx, NULL, &Can_Rx_Task_attributes);

  /* creation of VOFA_Send_Task */
  VOFA_Send_TaskHandle = osThreadNew(VOFA_Send, NULL, &VOFA_Send_Task_attributes);

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
  /* 呼吸灯: 线性渐变 0↔1000, 100 步走完一个周期 */
  int pwm = 0, dir = 20;
  for (;;)
  {
    if (g_breath_on)
    {
      pwm += dir;                                  /* 每步 ±20 */
      if (pwm <= 0 || pwm >= 1000) dir = -dir;     /* 到顶/到底折返 */
      g_breath_val = (float)pwm / 1000.0f;         /* 0~1, 供反馈用 */
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)pwm);
      osDelay(g_period_ms / 100);                  /* 每步延时 = 周期/100 */
    }
    else
    {
      g_breath_val = 0.0f;
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
      osDelay(100);
    }
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
  for(;;)
  {
    cnt++;
    if (cnt % 5 == 0)  HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin);  /* 每 500ms */
    if (cnt % 10 == 0) HAL_GPIO_TogglePin(LED_2_GPIO_Port, LED_2_Pin);  /* 每 1000ms */
    osDelay(100);
  }
  /* USER CODE END Status_Led */
}

/* USER CODE BEGIN Header_VOFA_Rx */
/**
* @brief Function implementing the VOFA_Rx_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_VOFA_Rx */
void VOFA_Rx(void *argument)
{
  /* USER CODE BEGIN VOFA_Rx */
  /* VOFA 指令: 等串口解析出的 A5..5A 指令, 本地生效 + 转发从板 */
  vofa_uart_start_rx();

  uint8_t cmd[3];
  for(;;)
  {
    if (osMessageQueueGet(vofaCmdQ, cmd, NULL, osWaitForever) == osOK)
    {
      uint16_t per = (uint16_t)(cmd[1] | (cmd[2] << 8));   /* 周期(小端) */
      if (per < 100) per = 100;                            /* 周期下限保护 */

      g_breath_on = (cmd[0] == CTRL_BYTE_ON) ? 1 : 0;
      g_period_ms = per;

      /* 组 CAN 帧 0x001 发给从板 */
      can_frame_t f;
      f.id       = ID_BREATH_CTRL;
      f.dlc      = 3;
      f.data[0]  = cmd[0];
      f.data[1]  = (uint8_t)(per & 0xFF);
      f.data[2]  = (uint8_t)(per >> 8);
      osMessageQueuePut(canTxQ, &f, 0, 0);
    }
  }
  /* USER CODE END VOFA_Rx */
}

/* USER CODE BEGIN Header_Can_Send */
/**
* @brief Function implementing the Can_Send_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Can_Send */
void Can_Send(void *argument)
{
  /* USER CODE BEGIN Can_Send */
  /* CAN 发送: 队列里来指令就发; 每 500ms 顺带发主板状态 0x02010101 */
  can_start();                       /* 过滤+启动+使能接收中断 */

  can_frame_t f;
  uint32_t last = 0;
  for(;;)
  {
    if (osMessageQueueGet(canTxQ, &f, NULL, pdMS_TO_TICKS(500)) == osOK)
    {
      can_send_frame(&f);
    }
    if (HAL_GetTick() - last >= 500)
    {
      last = HAL_GetTick();
      can_frame_t s;
      s.id      = ID_MASTER_STATUS;  /* 扩展帧 */
      s.dlc     = 1;
      s.data[0] = (uint8_t)g_breath_on;
      can_send_frame(&s);
    }
  }
  /* USER CODE END Can_Send */
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
  /* CAN 接收: 处理从板反馈/状态、CANable 蜂鸣指令 */
  can_frame_t f;
  for(;;)
  {
    if (osMessageQueueGet(canRxQ, &f, NULL, osWaitForever) == osOK)
    {
      switch (f.id)
      {
        case ID_SLAVE_FEEDBACK:            /* 从板 100Hz float */
          if (f.dlc >= 4)
          {
            memcpy(&g_slave_val, f.data, 4);
            g_slave_valid = 1;
          }
          break;
        case ID_SLAVE_STATUS:              /* 从板状态报文 */
          g_slave_status = f.data[0];
          break;
        case ID_BEEP_CMD:                  /* CANable 蜂鸣定次数 */
          if (f.dlc >= 1) beep_times(f.data[0]);
          break;
        default:
          break;
      }
    }
  }
  /* USER CODE END Can_Rx */
}

/* USER CODE BEGIN Header_VOFA_Send */
/**
* @brief Function implementing the VOFA_Send_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_VOFA_Send */
void VOFA_Send(void *argument)
{
  /* USER CODE BEGIN VOFA_Send */
  /* VOFA 上报: JustFloat 协议打波, ch0=从板亮度 ch1=周期 ch2=开关 */
  for(;;)
  {
    float v[3];
    v[0] = g_slave_valid ? g_slave_val : g_breath_val;  /* 从板没数据时用自己亮度 */
    v[1] = (float)g_period_ms;
    v[2] = (float)g_breath_on;
    vofa_send_floats(v, 3);
    osDelay(10);
  }
  /* USER CODE END VOFA_Send */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* 蜂鸣器按次数响应 (定次数, 上限保护避免长时间阻塞任务) */
void beep_times(uint8_t n)
{
  if (n > 10) n = 10;
  for (uint8_t i = 0; i < n; i++)
  {
    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
    osDelay(100);
    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
    osDelay(100);
  }
}
/* USER CODE END Application */

