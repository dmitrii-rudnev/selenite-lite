/**
  *******************************************************************************
  *
  * @file    cat_if.h
  * @brief   Header for cat_if.c file
  * @version v2.0
  * @date    18.04.2022
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
#ifndef CAT_IF_H_
#define CAT_IF_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void CAT_Handler (uint8_t);
void CAT_Buff_Write_Byte (uint8_t);

/* Private defines -----------------------------------------------------------*/
#define CAT_TIMEOUT   30U // 300 ms
#define CAT_BUFF_SIZE 64U

#ifdef __cplusplus
}
#endif

#endif /* CAT_IF_H_ */
