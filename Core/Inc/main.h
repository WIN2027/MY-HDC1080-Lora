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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define SX1280NSS_Pin GPIO_PIN_4
#define SX1280NSS_GPIO_Port GPIOA
#define DIO3_Pin GPIO_PIN_0
#define DIO3_GPIO_Port GPIOB
#define DIO3_EXTI_IRQn EXTI0_IRQn
#define DIO2_Pin GPIO_PIN_1
#define DIO2_GPIO_Port GPIOB
#define DIO2_EXTI_IRQn EXTI1_IRQn
#define DIO1_Pin GPIO_PIN_2
#define DIO1_GPIO_Port GPIOB
#define DIO1_EXTI_IRQn EXTI2_IRQn
#define DX1820_BUSY_Pin GPIO_PIN_10
#define DX1820_BUSY_GPIO_Port GPIOB
#define DX1820_BUSY_EXTI_IRQn EXTI15_10_IRQn
#define SX1280NRESET_Pin GPIO_PIN_12
#define SX1280NRESET_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
