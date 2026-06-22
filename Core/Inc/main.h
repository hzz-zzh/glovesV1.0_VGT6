/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h5xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
extern ADC_HandleTypeDef hadc1;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern XSPI_HandleTypeDef hxspi1;
extern TIM_HandleTypeDef htim5;
extern SD_HandleTypeDef hsd1;
/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IMU_RST_Pin GPIO_PIN_13
#define IMU_RST_GPIO_Port GPIOC
#define ADIN_CH10_Pin GPIO_PIN_0
#define ADIN_CH10_GPIO_Port GPIOC
#define ADIN_CH13_Pin GPIO_PIN_2
#define ADIN_CH13_GPIO_Port GPIOC
#define ADIN_CH12_Pin GPIO_PIN_3
#define ADIN_CH12_GPIO_Port GPIOC
#define IMU_SYNC_Pin GPIO_PIN_1
#define IMU_SYNC_GPIO_Port GPIOA
#define POWER_ON_OFF_Pin GPIO_PIN_2
#define POWER_ON_OFF_GPIO_Port GPIOA
#define POWER_ON_OFF_EXTI_IRQn EXTI2_IRQn
#define ADIN_CH15_Pin GPIO_PIN_3
#define ADIN_CH15_GPIO_Port GPIOA
#define ADIN_CH7_Pin GPIO_PIN_7
#define ADIN_CH7_GPIO_Port GPIOA
#define ADIN_CH4_Pin GPIO_PIN_4
#define ADIN_CH4_GPIO_Port GPIOC
#define TOUCH_ROW_SEL0_Pin GPIO_PIN_5
#define TOUCH_ROW_SEL0_GPIO_Port GPIOC
#define ADIN_CH9_Pin GPIO_PIN_0
#define ADIN_CH9_GPIO_Port GPIOB
#define ADIN_CH5_Pin GPIO_PIN_1
#define ADIN_CH5_GPIO_Port GPIOB
#define L_R_HAND_FLAG_Pin GPIO_PIN_12
#define L_R_HAND_FLAG_GPIO_Port GPIOE
#define EEPROM_SCL_Pin GPIO_PIN_10
#define EEPROM_SCL_GPIO_Port GPIOB
#define EEPROM_SDA_Pin GPIO_PIN_12
#define EEPROM_SDA_GPIO_Port GPIOB
#define TOUCH_COL_SEL0_Pin GPIO_PIN_13
#define TOUCH_COL_SEL0_GPIO_Port GPIOB
#define RS485_EN_Pin GPIO_PIN_6
#define RS485_EN_GPIO_Port GPIOC
#define TOUCH_COL_SEL4_Pin GPIO_PIN_7
#define TOUCH_COL_SEL4_GPIO_Port GPIOC
#define TOUCH_COL_SEL3_Pin GPIO_PIN_8
#define TOUCH_COL_SEL3_GPIO_Port GPIOC
#define DISABLE_CHARGE_Pin GPIO_PIN_8
#define DISABLE_CHARGE_GPIO_Port GPIOA
#define TOUCH_COL_SEL2_Pin GPIO_PIN_9
#define TOUCH_COL_SEL2_GPIO_Port GPIOA
#define PPS_IN_Pin GPIO_PIN_15
#define PPS_IN_GPIO_Port GPIOA
#define PPS_IN_EXTI_IRQn EXTI15_IRQn
#define USER_LED_Pin GPIO_PIN_0
#define USER_LED_GPIO_Port GPIOD
#define INT_GAUGE_BQ_Pin GPIO_PIN_3
#define INT_GAUGE_BQ_GPIO_Port GPIOB
#define USER_KEY_Pin GPIO_PIN_4
#define USER_KEY_GPIO_Port GPIOB
#define PERIPH_PWR_EN_Pin GPIO_PIN_5
#define PERIPH_PWR_EN_GPIO_Port GPIOB
#define TOUCH_COL_SEL1_Pin GPIO_PIN_9
#define TOUCH_COL_SEL1_GPIO_Port GPIOB
#define STATUS_CHARGE_Pin GPIO_PIN_0
#define STATUS_CHARGE_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
