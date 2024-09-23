/**
  *******************************************************************************
  *
  * @file    ptt_if.h
  * @brief   Header for ptt_if.c file
  * @version v2.0
  * @date    18.09.2024
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
#ifndef RXTX_IF_H_
#define RXTX_IF_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Exported types ------------------------------------------------------------*/

typedef enum
{
  MODE_LSB = 0x00,
  MODE_USB = 0x01,
  MODE_CW  = 0x02,  /* CW-USB */
  MODE_CWR = 0x03,  /* CW-LSB */
  MODE_AM  = 0x04,
  MODE_FM  = 0x08,
  MODE_DIG = 0x0A,  /* DIG-U */
  MODE_PKT = 0x0C   /* DIG-L */
} Mode;

typedef struct
{
  Mode     mode;
  uint32_t vfoa_bcd;
  uint32_t vfob_bcd;
  uint32_t vfoa;
  uint32_t vfob;
  uint8_t  vfo;
  uint8_t  split;
  uint8_t  is_tx;
  uint32_t sysclock;
  uint8_t  systicks;
  uint32_t displayed;
} TRX_TypeDef;

typedef struct
{
  uint8_t  is_enable;
  uint8_t  cat_is_on;
  uint8_t  dtr_is_on;
  uint8_t  rts_is_on;
  uint8_t  button_is_on;
  uint8_t  key_dah_is_on;
  uint8_t  key_dit_is_on;
  uint32_t key_off_time;
} PTT_TypeDef;

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

void CAT_Buff_Write (uint8_t*, uint32_t);

void RXTX_Init (void);
void RXTX_Handler (void);

void PTT_Set_Mode (uint8_t);

void PTT_Key_RX (void);
void PTT_DTR_TX (uint8_t);
void PTT_RTS_TX (uint8_t);

void VFO_Init (void);
void VFO_Toggle_VFO (void);
void VFO_Set_Tune (uint32_t);
void VFO_Set_Split (uint8_t);

uint32_t VFO_Get_Tune (void);
uint32_t VFO_Get_Tune_BCD (void);

uint32_t HEX_to_BCD (uint32_t);
uint32_t BCD_to_CAT (uint32_t);

/* Private defines -----------------------------------------------------------*/

#ifndef hi2c_tx
#define hi2c_tx                                 hi2c1 //hi2c2
extern I2C_HandleTypeDef hi2c_tx;
#endif

#ifndef I2CTIMEOUT 
#define I2CTIMEOUT                              10
#endif

#ifndef PCA9554_BUS_BASE_ADDR
#define PCA9554_BUS_BASE_ADDR                   0x20
#endif

#define KEY_TIMEOUT                            30 // 300 ms

#ifdef __cplusplus
}
#endif

#endif /* RXTX_IF_H_ */
