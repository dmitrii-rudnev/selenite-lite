
/**
  *******************************************************************************
  *
  * @file    codec_if.h
  * @brief   Header for codec_if.c file
  * @version v2.1
  * @date    28.10.2022
  * @author  Dmitrii Rudnev
  *
  *******************************************************************************
  * Copyrigh &copy; 2022 Selenite Project. All rights reserved.
  *
  * This software component is licensed under [BSD 3-Clause license]
  * (http://opensource.org/licenses/BSD-3-Clause/), the "License".<br>
  * You may not use this file except in compliance with the License.
  *******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CODEC_IF_H_
#define CODEC_IF_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "stm32f4xx_hal.h"

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

void Codec_Init (uint32_t);
void Codec_Set_RX (void);
void Codec_Set_TX (void);
uint8_t Codec_AF_Vol (uint8_t);

/* Private defines -----------------------------------------------------------*/

#ifndef CODEC_I2C_PORT
#define CODEC_I2C_PORT             hi2c3
#endif /* CODEC_I2C_PORT */

extern I2C_HandleTypeDef CODEC_I2C_PORT;

#ifndef CODEC_I2C_TIMEOUT
#define CODEC_I2C_TIMEOUT          10
#endif /* CODEC_I2C_TIMEOUT */

#ifndef CODEC_BUS_BASE_ADDR
#define CODEC_BUS_BASE_ADDR        0x18
#endif /* CODEC_BUS_BASE_ADDR */

typedef struct Output_Level
{
  uint8_t percent;
  uint8_t reg_val;
} Output_Level;

#ifdef __cplusplus
}
#endif

#endif /* CODEC_IF_H_ */
