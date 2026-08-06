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
#include "stm32f4xx_hal.h"

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

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void set_loop_rate(uint32_t loop_rate_hz);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_BUILTIN_Pin GPIO_PIN_1
#define LED_BUILTIN_GPIO_Port GPIOC
#define UNUSED_Pin GPIO_PIN_11
#define UNUSED_GPIO_Port GPIOB
#define UNUSEDA9_Pin GPIO_PIN_9
#define UNUSEDA9_GPIO_Port GPIOA
#define UNUSEDA10_Pin GPIO_PIN_10
#define UNUSEDA10_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
#define LOOP_TIM_TICK_RATE_HZ 1000000
#define MAIN_LOOP_TIM_CLK 84000000
#define MAIN_LOOP_TIM (&htim2)
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
