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
void Error_Handler (void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define S2_Pin GPIO_PIN_0
#define S2_GPIO_Port GPIOA
#define S1_Pin GPIO_PIN_1
#define S1_GPIO_Port GPIOA
#define KEYB_Pin GPIO_PIN_2
#define KEYB_GPIO_Port GPIOA
#define QSE_EN_Pin GPIO_PIN_3
#define QSE_EN_GPIO_Port GPIOA
#define QSD_EN_Pin GPIO_PIN_4
#define QSD_EN_GPIO_Port GPIOA
#define ILI9341_DC_Pin GPIO_PIN_0
#define ILI9341_DC_GPIO_Port GPIOB
#define ILI9341_CS_Pin GPIO_PIN_1
#define ILI9341_CS_GPIO_Port GPIOB
#define ILI9341_RES_Pin GPIO_PIN_2
#define ILI9341_RES_GPIO_Port GPIOB
#define ILI9341_LED_Pin GPIO_PIN_10
#define ILI9341_LED_GPIO_Port GPIOB
#define KEY_DAH_Pin GPIO_PIN_9
#define KEY_DAH_GPIO_Port GPIOA
#define KEY_DAH_EXTI_IRQn EXTI9_5_IRQn
#define KEY_DIT_Pin GPIO_PIN_10
#define KEY_DIT_GPIO_Port GPIOA
#define KEY_DIT_EXTI_IRQn EXTI15_10_IRQn
#define I2S2_RES_Pin GPIO_PIN_5
#define I2S2_RES_GPIO_Port GPIOB
#define TX_Pin GPIO_PIN_6
#define TX_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
