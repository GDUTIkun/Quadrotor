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
#include "stm32h7xx_hal.h"

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SCL_Pin GPIO_PIN_10
#define SCL_GPIO_Port GPIOB
#define SDA_Pin GPIO_PIN_11
#define SDA_GPIO_Port GPIOB
#define E4A_Pin GPIO_PIN_12
#define E4A_GPIO_Port GPIOD
#define E4B_Pin GPIO_PIN_13
#define E4B_GPIO_Port GPIOD
#define PWM1_Pin GPIO_PIN_6
#define PWM1_GPIO_Port GPIOC
#define PWM4_Pin GPIO_PIN_9
#define PWM4_GPIO_Port GPIOC
#define debug_tx_Pin GPIO_PIN_9
#define debug_tx_GPIO_Port GPIOA
#define debug_rx_Pin GPIO_PIN_10
#define debug_rx_GPIO_Port GPIOA
#define E1A_Pin GPIO_PIN_15
#define E1A_GPIO_Port GPIOA
#define M4_IN2_Pin GPIO_PIN_0
#define M4_IN2_GPIO_Port GPIOD
#define M4_IN1_Pin GPIO_PIN_1
#define M4_IN1_GPIO_Port GPIOD
#define M1_IN2_Pin GPIO_PIN_6
#define M1_IN2_GPIO_Port GPIOD
#define M1_IN1_Pin GPIO_PIN_7
#define M1_IN1_GPIO_Port GPIOD
#define E1B_Pin GPIO_PIN_3
#define E1B_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
