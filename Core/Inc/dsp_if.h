/**
  *******************************************************************************
  *
  * @file    dsp_if.h
  * @brief   Header for dsp_if.c file
  * @version v2.2
  * @date    02.11.2022
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
#ifndef INC_DSP_IF_H_
#define INC_DSP_IF_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "usbd_audio.h"
#include "stm32f4xx_hal.h"

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

void DSP_Out_Buff_Write (uint8_t*,  uint32_t);
void DSP_Out_Buff_Mute  (void);
void DSP_In_Buff_Read   (uint8_t*,  uint32_t);
void DSP_Init (void);

void DSP_Set_RX (void);
void DSP_Set_TX (void);
void DSP_Set_Mode (uint8_t);
void DSP_Out_Buff_Read (uint16_t*, uint16_t);
void DSP_In_Buff_Write (uint16_t*, uint16_t);

/* Private defines -----------------------------------------------------------*/

#ifndef USBD_AUDIO_FREQ
#define USBD_AUDIO_FREQ          48000U
#endif /* USBD_AUDIO_FREQ */

#ifndef AUDIO_OUT_PACKET_NUM
#define AUDIO_OUT_PACKET_NUM     2U
#endif /* AUDIO_OUT_PACKET_NUM */

#ifndef I2S_IF
#define I2S_IF                   hi2s2
#endif /* I2S_IF */

extern  I2S_HandleTypeDef        I2S_IF;

#define I2S_BUFF_PACKET_SIZE     (uint16_t)((USBD_AUDIO_FREQ * 2U) / 1000U)                /* I2S buffer packet size in samples */

#define I2S_BUFF_PACKET_NUM      (uint16_t)(AUDIO_OUT_PACKET_NUM)
#define I2S_BUFF_SIZE            (uint16_t)((I2S_BUFF_PACKET_SIZE * I2S_BUFF_PACKET_NUM))  /* I2S buffer size in samples */
#define I2S_BUFF_HALF_SIZE       (uint16_t)((I2S_BUFF_SIZE / 2U))                          /* I2S buffer half size */

typedef struct
{
  uint16_t rx [I2S_BUFF_SIZE];
  uint16_t tx [I2S_BUFF_SIZE];
} I2S_Buff_TypeDef;

#define DSP_BUFF_PACKET_SIZE     (uint16_t)(USBD_AUDIO_FREQ / 1000U)                       /* DSP buffer packet size in samples */

#define DSP_BUFF_PACKET_NUM      (uint16_t)(AUDIO_OUT_PACKET_NUM * 4U)
#define DSP_BUFF_SIZE            (uint16_t)((DSP_BUFF_PACKET_SIZE * DSP_BUFF_PACKET_NUM))  /* DSP buffer size in samples */
#define DSP_BUFF_HALF_SIZE       (uint16_t)((DSP_BUFF_SIZE / 2U))                          /* DSP buffer half size */

typedef struct
{
  int16_t  i [DSP_BUFF_SIZE];
  int16_t  q [DSP_BUFF_SIZE];
  uint8_t  buff_enable;
  uint16_t rd_ptr;
  uint16_t wr_ptr;
} DSP_Buff_TypeDef;

#endif /* INC_DSP_IF_H_ */
