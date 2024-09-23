/**
  *******************************************************************************
  *
  * @file    i2c_if.h
  * @brief   Header for i2c_if.c file
  * @version v1.0
  * @date    22.09.2024
  * @author  Dmitrii Rudnev
  *
  *******************************************************************************
  * Copyrigh &copy; 2024 Selenite Project. All rights reserved.
  *
  * This software component is licensed under [BSD 3-Clause license]
  * (http://opensource.org/licenses/BSD-3-Clause/), the "License".<br>
  * You may not use this file except in compliance with the License.
  *******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef INC_I2C_IF_H_
#define INC_I2C_IF_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

void I2C_Receive (I2C_HandleTypeDef*, uint16_t, uint8_t, uint8_t*);
void I2C_Transmit (I2C_HandleTypeDef*, uint16_t, uint8_t, uint8_t);
void I2C_Transmit_Bulk (I2C_HandleTypeDef*, uint16_t, uint8_t, uint8_t*, uint8_t);

/* Private defines -----------------------------------------------------------*/

#ifndef I2C_TIMEOUT
#define I2C_TIMEOUT          10
#endif /* I2C_TIMEOUT */

#ifdef __cplusplus
}
#endif

#endif /* INC_I2C_IF_H_ */
