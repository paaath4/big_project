/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "tim.h"
#include "gpio.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "app_proto.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE BEGIN PV */
QueueHandle_t xBreathQueue; // 呼吸灯控制队列
QueueHandle_t xBeepQueue;   // 蜂鸣器控制队列
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
void vFeedbackTask(void *pvParameters); // 100Hz 反馈
void vControlTask(void *pvParameters);  // 呼吸灯 + 蜂鸣器
void vStatusTask(void *pvParameters);   // 500ms 状态公告
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);         /* 启动呼吸灯 PWM */
  xBreathQueue = xQueueCreate(5, sizeof(uint32_t)); // 呼吸灯控制：存开关+周期
  xBeepQueue = xQueueCreate(5, sizeof(uint8_t));    // 蜂鸣器：存鸣响次数
  can_start();

  // 创建 FreeRTOS 任务
  xTaskCreate(vFeedbackTask, "Feedback", 256, NULL, 3, NULL);
  xTaskCreate(vControlTask, "Control", 256, NULL, 2, NULL);
  xTaskCreate(vStatusTask, "Status", 256, NULL, 1, NULL);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize(); /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// ========== 任务1：100Hz 反馈 ==========
void vFeedbackTask(void *pvParameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  float feedback_val = 0.0f;
  uint8_t tx_data[4];
  CAN_TxHeaderTypeDef txHeader;

  txHeader.IDE = CAN_ID_STD;
  txHeader.StdId = ID_SLAVE_FEEDBACK; // 0x002
  txHeader.DLC = 4;
  txHeader.TransmitGlobalTime = DISABLE;

  while (1)
  {
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10)); // 精确10ms

    // 生成变化数据（0.0 ~ 1.0 循环）
    feedback_val += 0.01f;
    if (feedback_val > 1.0f)
      feedback_val = 0.0f;
    memcpy(tx_data, &feedback_val, sizeof(float));

    uint32_t mailbox;
    HAL_CAN_AddTxMessage(&hcan1, &txHeader, tx_data, &mailbox);
  }
}

// ========== 任务2：命令执行（呼吸灯 + 蜂鸣器） ==========
void vControlTask(void *pvParameters)
{
  uint8_t beep_count = 0;
  uint32_t breath_cmd = 0;
  uint8_t enable = 0;
  uint16_t period = 1000;

  // 呼吸灯渐变变量（假设 PWM 周期为 1000）
  uint16_t pwm_value = 0;
  int8_t direction = 1;

  while (1)
  {
    // 1. 处理蜂鸣器指令（非阻塞）
    if (xQueueReceive(xBeepQueue, &beep_count, 0) == pdTRUE)
    {
      for (int i = 0; i < beep_count; i++)
      {
        HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
        vTaskDelay(pdMS_TO_TICKS(100)); // 鸣 100ms
        HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
        vTaskDelay(pdMS_TO_TICKS(50)); // 停 50ms
      }
    }

    // 2. 接收呼吸灯控制指令（非阻塞）
    if (xQueueReceive(xBreathQueue, &breath_cmd, 0) == pdTRUE)
    {
      enable = (breath_cmd >> 16) & 0xFF;
      period = breath_cmd & 0xFFFF;
    }

    // 3. 执行呼吸灯 PWM 渐变
    if (enable)
    {
      // 根据 period 计算步进延时（period 越大，呼吸越慢）
      uint16_t step_delay = period / 50;
      if (step_delay < 1)
        step_delay = 1;

      // 改变占空比（假设 ARR = 1000）
      pwm_value += direction;
      if (pwm_value >= 1000)
      {
        pwm_value = 1000;
        direction = -1;
      }
      if (pwm_value <= 0)
      {
        pwm_value = 0;
        direction = 1;
      }
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm_value);

      vTaskDelay(pdMS_TO_TICKS(step_delay));
    }
    else
    {
      // 关闭呼吸灯
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }
}

// ========== 任务3：500ms 状态公告 ==========
void vStatusTask(void *pvParameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint8_t status_data[1] = {0x55}; // 0x55 = 从板在线
  CAN_TxHeaderTypeDef txHeader;

  txHeader.IDE = CAN_ID_STD;
  txHeader.StdId = ID_SLAVE_STATUS; // 0x012
  txHeader.DLC = 1;
  txHeader.TransmitGlobalTime = DISABLE;

  while (1)
  {
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(500));

    uint32_t mailbox;
    HAL_CAN_AddTxMessage(&hcan1, &txHeader, status_data, &mailbox);
  }
}

/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM1 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
