/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define COL0_Pin GPIO_PIN_5
#define COL0_GPIO_Port GPIOA
#define COL1_Pin GPIO_PIN_6
#define COL1_GPIO_Port GPIOA
#define COL2_Pin GPIO_PIN_7
#define COL2_GPIO_Port GPIOA
#define G0_Pin GPIO_PIN_10
#define G0_GPIO_Port GPIOB
#define G1_Pin GPIO_PIN_11
#define G1_GPIO_Port GPIOB
#define G2_Pin GPIO_PIN_12
#define G2_GPIO_Port GPIOB
#define R0_Pin GPIO_PIN_13
#define R0_GPIO_Port GPIOB
#define R1_Pin GPIO_PIN_14
#define R1_GPIO_Port GPIOB
#define R2_Pin GPIO_PIN_15
#define R2_GPIO_Port GPIOB
#define ROW0_Pin GPIO_PIN_9
#define ROW0_GPIO_Port GPIOA
#define ROW1_Pin GPIO_PIN_10
#define ROW1_GPIO_Port GPIOA
#define ROW2_Pin GPIO_PIN_11
#define ROW2_GPIO_Port GPIOA
#define ROW3_Pin GPIO_PIN_12
#define ROW3_GPIO_Port GPIOA
#define B0_Pin GPIO_PIN_8
#define B0_GPIO_Port GPIOB
#define B1_Pin GPIO_PIN_9
#define B1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
