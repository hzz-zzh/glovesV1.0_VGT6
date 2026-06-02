/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.h
  * Description        : FreeRTOS applicative header file
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
#ifndef __APP_FREERTOS_H
#define __APP_FREERTOS_H

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Exported macro -------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */
extern osThreadId_t defaultTaskHandle;
extern osThreadId_t uartDebugTaskHandle;
extern osThreadId_t testTaskHandle;
extern osThreadId_t frameAssemblerTaskHandle;
extern osThreadId_t systemManagerTaskHandle;
extern osThreadId_t timeSyncTaskHandle;
extern osThreadId_t imuCanTaskHandle;
extern osThreadId_t touchAdcTaskHandle;
extern osThreadId_t dataProcessTaskHandle;
extern osThreadId_t rs485TaskHandle;
extern osThreadId_t storageTaskHandle;

/* Exported function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void UartDebugTask(void *argument);
void StartTestTask(void *argument);
void FrameAssemblerTask(void *argument);
void SystemManagerTask(void *argument);
void TimeSyncTask(void *argument);
void ImuCanTask(void *argument);
void TouchAdcTask(void *argument);
void DataProcessTask(void *argument);
void Rs485Task(void *argument);
void StorageTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

#ifdef __cplusplus
}
#endif
#endif /* __APP_FREERTOS_H */
