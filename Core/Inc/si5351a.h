/**
  *******************************************************************************
  *
  * @file    si5351a.h
  * @brief   Header for si5351a.c file
  * @version v1.0
  * @date    18.10.2022
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

#ifndef INC_SI5351A_H_
#define INC_SI5351A_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* Exported functions prototypes ---------------------------------------------*/

void Si5351a_Init (void);
void Si5351a_Set_Freq (uint32_t, uint16_t, uint16_t);

/* Private defines -----------------------------------------------------------*/

#define F_XTAL                    25000000

#ifndef SI5351_I2C_PORT
#define SI5351_I2C_PORT           hi2c3
#endif /* SI5351_I2C_PORT */

#ifndef SI5351_BUS_BASE_ADDR
#define SI5351_BUS_BASE_ADDR      0x60
#endif /* SI5351_BUS_BASE_ADDR */

enum ms_t {
  PLLA = 0, PLLB = 1,
  MSNA =-2, MSNB =-1,
  MS0  = 0, MS1  = 1, MS2 = 2 };

#endif /* INC_SI5351A_H_ */
