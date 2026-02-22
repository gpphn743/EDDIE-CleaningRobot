/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#define IN3_Pin GPIO_PIN_14
#define IN3_GPIO_Port GPIOC
#define IN4_Pin GPIO_PIN_15
#define IN4_GPIO_Port GPIOC
#define IN1_Pin GPIO_PIN_0
#define IN1_GPIO_Port GPIOA
#define IN2_Pin GPIO_PIN_1
#define IN2_GPIO_Port GPIOA
#define CS_Pin GPIO_PIN_4
#define CS_GPIO_Port GPIOA
#define DC_Pin GPIO_PIN_0
#define DC_GPIO_Port GPIOB
#define RESET_Pin GPIO_PIN_1
#define RESET_GPIO_Port GPIOB
#define BUZZER_Pin GPIO_PIN_12
#define BUZZER_GPIO_Port GPIOB
#define TRIG1_Pin GPIO_PIN_13
#define TRIG1_GPIO_Port GPIOB
#define TRIG2_Pin GPIO_PIN_14
#define TRIG2_GPIO_Port GPIOB
#define TRIG3_Pin GPIO_PIN_15
#define TRIG3_GPIO_Port GPIOB
#define ENA_Pin GPIO_PIN_8
#define ENA_GPIO_Port GPIOA
#define ENB_Pin GPIO_PIN_11
#define ENB_GPIO_Port GPIOA
#define RELAY1_Pin GPIO_PIN_5
#define RELAY1_GPIO_Port GPIOB
#define ECHO1_Pin GPIO_PIN_6
#define ECHO1_GPIO_Port GPIOB
#define ECHO2_Pin GPIO_PIN_7
#define ECHO2_GPIO_Port GPIOB
#define ECHO3_Pin GPIO_PIN_8
#define ECHO3_GPIO_Port GPIOB
#define RELAY2_Pin GPIO_PIN_9
#define RELAY2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
