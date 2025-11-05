/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usb_device.c
  * @version        : v2.0
  * @brief          : This file implements the USB Device
  ******************************************************************************
  * @attention  *
  * Copyrigh &copy; 2024 Selenite Project. All rights reserved.
  *
  * This software component is licensed under [BSD 3-Clause license]
  * (http://opensource.org/licenses/BSD-3-Clause/), the "License".<br>
  * You may not use this file except in compliance with the License.  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

/* USER CODE BEGIN Includes */

#include "usbd_composite.h"
#include "usbd_conf.h"

/* USER CODE END Includes */

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE END PV */

/* USB Device Core handle declaration. */
USBD_HandleTypeDef hUsbDeviceFS;

/**
  * Init USB device Library, add supported class and start the library
  * @retval None
  */

void MX_USB_DEVICE_Init (void)
{
  /* USER CODE BEGIN USB_DEVICE_Init_PreTreatment */

  if (USBD_Init (&hUsbDeviceFS, &FS_Desc, DEVICE_FS) != USBD_OK)
  {
    Error_Handler ();
  }

  HAL_PCDEx_SetRxFiFo (&hpcd_USB_OTG_FS, 0x80);
  HAL_PCDEx_SetTxFiFo (&hpcd_USB_OTG_FS, 0, 0x40);
  HAL_PCDEx_SetTxFiFo (&hpcd_USB_OTG_FS, 1, 0x10);
  HAL_PCDEx_SetTxFiFo (&hpcd_USB_OTG_FS, 2, 0x10);
  HAL_PCDEx_SetTxFiFo (&hpcd_USB_OTG_FS, 3, 0xC0);

  if (USBD_RegisterClass (&hUsbDeviceFS, &USBD_COMP) != USBD_OK)
  {
    Error_Handler ();
  }

  if (USBD_COMP_RegisterInterface (&hUsbDeviceFS, &USBD_COMP_fops_FS)
      != USBD_OK)
  {
    Error_Handler ();
  }

  if (USBD_Start (&hUsbDeviceFS) != USBD_OK)
  {
    Error_Handler ();
  }

  return;

  /* USER CODE END USB_DEVICE_Init_PreTreatment */
}

/**
  * @}
  */

/**
  * @}
  */

