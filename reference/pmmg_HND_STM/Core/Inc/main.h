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
#include "stm32g4xx_hal.h"

#include "stm32g4xx_nucleo.h"
#include <stdio.h>

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
#define RCC_OSC32_IN_Pin GPIO_PIN_14
#define RCC_OSC32_IN_GPIO_Port GPIOC
#define RCC_OSC32_OUT_Pin GPIO_PIN_15
#define RCC_OSC32_OUT_GPIO_Port GPIOC
#define RCC_OSC_IN_Pin GPIO_PIN_0
#define RCC_OSC_IN_GPIO_Port GPIOF
#define RCC_OSC_OUT_Pin GPIO_PIN_1
#define RCC_OSC_OUT_GPIO_Port GPIOF
#define CS3_J3_Pin GPIO_PIN_5
#define CS3_J3_GPIO_Port GPIOC
#define CS9_J9_Pin GPIO_PIN_1
#define CS9_J9_GPIO_Port GPIOB
#define CS8_J8_Pin GPIO_PIN_2
#define CS8_J8_GPIO_Port GPIOB
#define CS12_J12_Pin GPIO_PIN_10
#define CS12_J12_GPIO_Port GPIOB
#define CS7_J7_Pin GPIO_PIN_11
#define CS7_J7_GPIO_Port GPIOB
#define CS6_J6_Pin GPIO_PIN_12
#define CS6_J6_GPIO_Port GPIOB
#define CS2_J2_Pin GPIO_PIN_6
#define CS2_J2_GPIO_Port GPIOC
#define CS1_J1_Pin GPIO_PIN_8
#define CS1_J1_GPIO_Port GPIOC
#define CS11_J11_Pin GPIO_PIN_8
#define CS11_J11_GPIO_Port GPIOA
#define CS10_J10_Pin GPIO_PIN_9
#define CS10_J10_GPIO_Port GPIOA
#define CS5_J5_Pin GPIO_PIN_11
#define CS5_J5_GPIO_Port GPIOA
#define CS4_J4_Pin GPIO_PIN_12
#define CS4_J4_GPIO_Port GPIOA
#define T_SWDIO_Pin GPIO_PIN_13
#define T_SWDIO_GPIO_Port GPIOA
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
