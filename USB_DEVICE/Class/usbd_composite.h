/**
  *******************************************************************************
  *
  * @file    usbd_comp.h
  * @brief   header file for the usbd_comp.c file
  * @version v1.0
  * @date    18.07.2020
  * @author  Dmitrii Rudnev
  *
  *******************************************************************************
  * Copyrigh &copy; 2020 Selenite Project. All rights reserved.
  *
  * This software component is licensed under [BSD 3-Clause license]
  * (http://opensource.org/licenses/BSD-3-Clause/), the "License".<br>
  * You may not use this file except in compliance with the License.
  *******************************************************************************
  */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ST_STM32_USB_DEVICE_LIBRARY_CLASS_COMP_INC_USBD_COMP_H_
#define ST_STM32_USB_DEVICE_LIBRARY_CLASS_COMP_INC_USBD_COMP_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbd_ioreq.h"
#include "usbd_desc.h"
#include "stm32f4xx_hal.h"

/* Exported types ------------------------------------------------------------*/
typedef struct
{
} USBD_COMP_HandleTypeDef;

typedef struct
{
} USBD_COMP_ItfTypeDef;

typedef struct
{
  USBD_ClassTypeDef *class;
  void *classData;
  void *userData;
  uint16_t ctrlIf;
  uint16_t minIf;
  uint16_t maxIf;
} USBD_ClassCompInfo;


/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported variables---------------------------------------------------------*/
extern USBD_ClassTypeDef    USBD_COMP;
extern USBD_COMP_ItfTypeDef USBD_COMP_fops_FS;

/* Exported functions prototypes ---------------------------------------------*/
uint8_t USBD_COMP_RegisterInterface (USBD_HandleTypeDef *pdev, USBD_COMP_ItfTypeDef *fops);
uint8_t COMP_CDC_Transmit_FS (uint8_t* Buf, uint16_t  Len);


/* Private defines -----------------------------------------------------------*/
#define CLASS_NUM                    2U
#define VCP                          0U
#define UAC                          1U
#define USBD_COMP_CLASS              &USBD_COMP
#define USB_COMP_CONFIG_DESC_SIZ     9U + 8U + 58U + 8U + 183U
#define CDC_CTRL_IF                  0x00U
#define CDC_DATA_IF                  0x01U


#endif /* ST_STM32_USB_DEVICE_LIBRARY_CLASS_COMP_INC_USBD_COMP_H_ */
