/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : FreeRTOS applicative file
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
#include "app_freertos.h"

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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for uartDebugTask */
osThreadId_t uartDebugTaskHandle;
const osThreadAttr_t uartDebugTask_attributes = {
  .name = "uartDebugTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for testTask */
osThreadId_t testTaskHandle;
const osThreadAttr_t testTask_attributes = {
  .name = "testTask",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 1024 * 4
};
/* Definitions for frameAssemblerTask */
osThreadId_t frameAssemblerTaskHandle;
const osThreadAttr_t frameAssemblerTask_attributes = {
  .name = "frameAssemblerTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 512 * 4
};
/* Definitions for systemManagerTask */
osThreadId_t systemManagerTaskHandle;
const osThreadAttr_t systemManagerTask_attributes = {
  .name = "systemManagerTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 512 * 4
};
/* Definitions for timeSyncTask */
osThreadId_t timeSyncTaskHandle;
const osThreadAttr_t timeSyncTask_attributes = {
  .name = "timeSyncTask",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 256 * 4
};
/* Definitions for imuCanTask */
osThreadId_t imuCanTaskHandle;
const osThreadAttr_t imuCanTask_attributes = {
  .name = "imuCanTask",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 1024 * 4
};
/* Definitions for touchAdcTask */
osThreadId_t touchAdcTaskHandle;
const osThreadAttr_t touchAdcTask_attributes = {
  .name = "touchAdcTask",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 1024 * 4
};
/* Definitions for dataProcessTask */
osThreadId_t dataProcessTaskHandle;
const osThreadAttr_t dataProcessTask_attributes = {
  .name = "dataProcessTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 1024 * 4
};
/* Definitions for rs485Task */
osThreadId_t rs485TaskHandle;
const osThreadAttr_t rs485Task_attributes = {
  .name = "rs485Task",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 512 * 4
};
/* Definitions for storageTask */
osThreadId_t storageTaskHandle;
const osThreadAttr_t storageTask_attributes = {
  .name = "storageTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 512 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

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
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of uartDebugTask */
  uartDebugTaskHandle = osThreadNew(UartDebugTask, NULL, &uartDebugTask_attributes);

  /* creation of testTask */
  testTaskHandle = osThreadNew(StartTestTask, NULL, &testTask_attributes);

  /* creation of frameAssemblerTask */
  frameAssemblerTaskHandle = osThreadNew(FrameAssemblerTask, NULL, &frameAssemblerTask_attributes);

  /* creation of systemManagerTask */
  systemManagerTaskHandle = osThreadNew(SystemManagerTask, NULL, &systemManagerTask_attributes);

  /* creation of timeSyncTask */
  timeSyncTaskHandle = osThreadNew(TimeSyncTask, NULL, &timeSyncTask_attributes);

  /* creation of imuCanTask */
  imuCanTaskHandle = osThreadNew(ImuCanTask, NULL, &imuCanTask_attributes);

  /* creation of touchAdcTask */
  touchAdcTaskHandle = osThreadNew(TouchAdcTask, NULL, &touchAdcTask_attributes);

  /* creation of dataProcessTask */
  dataProcessTaskHandle = osThreadNew(DataProcessTask, NULL, &dataProcessTask_attributes);

  /* creation of rs485Task */
  rs485TaskHandle = osThreadNew(Rs485Task, NULL, &rs485Task_attributes);

  /* creation of storageTask */
  storageTaskHandle = osThreadNew(StorageTask, NULL, &storageTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}
/* USER CODE BEGIN Header_StartDefaultTask */
/**
* @brief Function implementing the defaultTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN defaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END defaultTask */
}

/* USER CODE BEGIN Header_UartDebugTask */
/**
* @brief Function implementing the uartDebugTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UartDebugTask */
__weak void UartDebugTask(void *argument)
{
  /* USER CODE BEGIN uartDebugTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END uartDebugTask */
}

/* USER CODE BEGIN Header_StartTestTask */
/**
* @brief Function implementing the testTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTestTask */
__weak void StartTestTask(void *argument)
{
  /* USER CODE BEGIN testTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END testTask */
}

/* USER CODE BEGIN Header_FrameAssemblerTask */
/**
* @brief Function implementing the frameAssemblerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_FrameAssemblerTask */
__weak void FrameAssemblerTask(void *argument)
{
  /* USER CODE BEGIN frameAssemblerTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END frameAssemblerTask */
}

/* USER CODE BEGIN Header_SystemManagerTask */
/**
* @brief Function implementing the systemManagerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SystemManagerTask */
__weak void SystemManagerTask(void *argument)
{
  /* USER CODE BEGIN systemManagerTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END systemManagerTask */
}

/* USER CODE BEGIN Header_TimeSyncTask */
/**
* @brief Function implementing the timeSyncTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TimeSyncTask */
__weak void TimeSyncTask(void *argument)
{
  /* USER CODE BEGIN timeSyncTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END timeSyncTask */
}

/* USER CODE BEGIN Header_ImuCanTask */
/**
* @brief Function implementing the imuCanTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ImuCanTask */
__weak void ImuCanTask(void *argument)
{
  /* USER CODE BEGIN imuCanTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END imuCanTask */
}

/* USER CODE BEGIN Header_TouchAdcTask */
/**
* @brief Function implementing the touchAdcTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TouchAdcTask */
__weak void TouchAdcTask(void *argument)
{
  /* USER CODE BEGIN touchAdcTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END touchAdcTask */
}

/* USER CODE BEGIN Header_DataProcessTask */
/**
* @brief Function implementing the dataProcessTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DataProcessTask */
__weak void DataProcessTask(void *argument)
{
  /* USER CODE BEGIN dataProcessTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END dataProcessTask */
}

/* USER CODE BEGIN Header_Rs485Task */
/**
* @brief Function implementing the rs485Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Rs485Task */
__weak void Rs485Task(void *argument)
{
  /* USER CODE BEGIN rs485Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END rs485Task */
}

/* USER CODE BEGIN Header_StorageTask */
/**
* @brief Function implementing the storageTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StorageTask */
__weak void StorageTask(void *argument)
{
  /* USER CODE BEGIN storageTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END storageTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

