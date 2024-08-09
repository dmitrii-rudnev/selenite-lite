/**
  ******************************************************************************
  * @file    usbd_audio.c
  * @author  MCD Application Team
  * @brief   This file provides the Audio core functions.
  *
  * @verbatim
  *
  *          ===================================================================
  *                                AUDIO Class  Description
  *          ===================================================================
 *           This driver manages the Audio Class 1.0 following the "USB Device Class Definition for
  *           Audio Devices V1.0 Mar 18, 98".
  *           This driver implements the following aspects of the specification:
  *             - Device descriptor management
  *             - Configuration descriptor management
  *             - Standard AC Interface Descriptor management
  *             - 1 Audio Streaming Interface (with single channel, PCM, Stereo mode)
  *             - 1 Audio Streaming Endpoint
  *             - 1 Audio Terminal Input (1 channel)
  *             - Audio Class-Specific AC Interfaces
  *             - Audio Class-Specific AS Interfaces
  *             - AudioControl Requests: only SET_CUR and GET_CUR requests are supported (for Mute)
  *             - Audio Feature Unit (limited to Mute control)
  *             - Audio Synchronization type: Asynchronous
  *             - Single fixed audio sampling rate (configurable in usbd_conf.h file)
  *          The current audio class version supports the following audio features:
  *             - Pulse Coded Modulation (PCM) format
  *             - sampling rate: 48KHz.
  *             - Bit resolution: 16
  *             - Number of channels: 2
  *             - No volume control
  *             - Mute/Unmute capability
  *             - Asynchronous Endpoints
  *
  * @note     In HS mode and when the DMA is used, all variables and data structures
  *           dealing with the DMA during the transaction process should be 32-bit aligned.
  *
  *
  *  @endverbatim
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2015 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                      http://www.st.com/SLA0044
  *
  ******************************************************************************
  */

  /* BSPDependencies
  - "stm32xxxxx_{eval}{discovery}.c"
  - "stm32xxxxx_{eval}{discovery}_io.c"
  - "stm32xxxxx_{eval}{discovery}_audio.c"
  EndBSPDependencies */

/* Includes ------------------------------------------------------------------*/
#include "usbd_audio.h"

#include "usbd_ctlreq.h"

#include <string.h>

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */


/** @defgroup USBD_AUDIO
  * @brief usbd core module
  * @{
  */

/** @defgroup USBD_AUDIO_Private_TypesDefinitions
  * @{
  */
/**
  * @}
  */


/** @defgroup USBD_AUDIO_Private_Defines
  * @{
  */
/**
  * @}
  */


/** @defgroup USBD_AUDIO_Private_Macros
  * @{
  */
#define AUDIO_SAMPLE_FREQ(frq)  (uint8_t)(frq), (uint8_t)((frq >> 8)), (uint8_t)((frq >> 16))

#define AUDIO_PACKET_SZE(frq)   (uint8_t)(((frq * 2U * 2U)/1000U) & 0xFFU), \
                                (uint8_t)((((frq * 2U * 2U)/1000U) >> 8) & 0xFFU)

/**
  * @}
  */

/** @defgroup USBD_AUDIO_Private_FunctionPrototypes
  * @{
  */


static uint8_t  USBD_AUDIO_Init (USBD_HandleTypeDef *pdev,
                               uint8_t cfgidx);

static uint8_t  USBD_AUDIO_DeInit (USBD_HandleTypeDef *pdev,
                                 uint8_t cfgidx);

static uint8_t  USBD_AUDIO_Setup (USBD_HandleTypeDef *pdev,
                                USBD_SetupReqTypedef *req);

static uint8_t  *USBD_AUDIO_GetCfgDesc (uint16_t *length);

static uint8_t  *USBD_AUDIO_GetDeviceQualifierDesc (uint16_t *length);

static uint8_t  USBD_AUDIO_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t  USBD_AUDIO_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t  USBD_AUDIO_EP0_RxReady (USBD_HandleTypeDef *pdev);

static uint8_t  USBD_AUDIO_EP0_TxReady (USBD_HandleTypeDef *pdev);

static uint8_t  USBD_AUDIO_SOF (USBD_HandleTypeDef *pdev);

static uint8_t  USBD_AUDIO_IsoINIncomplete (USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t  USBD_AUDIO_IsoOutIncomplete (USBD_HandleTypeDef *pdev, uint8_t epnum);

static void AUDIO_REQ_GetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);

static void AUDIO_REQ_SetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);

/**
  * @}
  */

/** @defgroup USBD_AUDIO_Private_Variables
  * @{
  */

USBD_AUDIO_HandleTypeDef USBD_AUDIO_Handle;

USBD_ClassTypeDef  USBD_AUDIO =
{
  USBD_AUDIO_Init,
  USBD_AUDIO_DeInit,
  USBD_AUDIO_Setup,
  USBD_AUDIO_EP0_TxReady,
  USBD_AUDIO_EP0_RxReady,
  USBD_AUDIO_DataIn,
  USBD_AUDIO_DataOut,
  USBD_AUDIO_SOF,
  USBD_AUDIO_IsoINIncomplete,
  USBD_AUDIO_IsoOutIncomplete,
  USBD_AUDIO_GetCfgDesc,
  USBD_AUDIO_GetCfgDesc,
  USBD_AUDIO_GetCfgDesc,
  USBD_AUDIO_GetDeviceQualifierDesc,
};

/* USB AUDIO device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_AUDIO_CfgDesc[USB_AUDIO_CONFIG_DESC_SIZ] __ALIGN_END =
{
    /* Configuration 1 */
    0x09, /* bLength = 9 bytes */
    0x02, /* bDescriptorType = USB_DESC_TYPE_CONFIGURATION = 0x02 */
    LOBYTE(USB_AUDIO_CONFIG_DESC_SIZ), /* wTotalLength = 192 bytes */
    HIBYTE(USB_AUDIO_CONFIG_DESC_SIZ),
    0x03, /* bNumInterfaces = 3 interfaces = Control (AC) + OUT (Speaker AS) + IN (Mic AS) */
    0x01, /* bConfigurationValue */
    0x00, /* iConfiguration */
    0xC0, /* bmAttributes = BUS Powered*/
    0xFA, /* bMaxPower = 250 * 2 mA = 500 mA*/
    /* 09 byte*/
    /*---------------------------------------------------------------------------*/
    /* USB Audio Device Standard AC interface descriptor */
    0x09, /* bLength = AUDIO_INTERFACE_DESC_SIZE = 09 bytes */
    0x04, /* bDescriptorType = USB_DESC_TYPE_INTERFACE = 0x04 */
    AUDIO_CTRL_IF, /* bInterfaceNumber = Number of AC Interface */
    0x00, /* bAlternateSetting = Default Alternate Setting */
    0x00, /* bNumEndpoints = No Endpoint Is Used */
    0x01, /* bInterfaceClass = USB_DEVICE_CLASS_AUDIO = 0x01*/
    0x01, /* bInterfaceSubClass = AUDIO_SUBCLASS_AUDIOCONTROL = 0x01 */
    0x00, /* bInterfaceProtocol = AUDIO_PROTOCOL_UNDEFINED = Non-Basic Audio Device*/
    0x00, /* iInterface */
    /* 09 bytes */

    /* USB Audio Device Class-specific AC Interface Descriptor */
    0x0A, /* bLength = (8 + bInCollection) bytes */
    0x24, /* bDescriptorType = AUDIO_INTERFACE_DESCRIPTOR_TYPE = 0x24 */
    0x01, /* bDescriptorSubtype = AUDIO_CONTROL_HEADER = 0x01*/
    0x00, /* bcdADC = 0x0100 = Audio 1.0*/
    0x01,
    0x46, /* wTotalLength = 70*/
    0x00,
    0x02, /* bInCollection */
    AUDIO_OUT_IF, /* baInterfaceNr1 AUDIO_OUT_IF = 0x01U (Speaker AS) */
    AUDIO_IN_IF,  /* baInterfaceNr2 AUDIO_IN_IF  = 0x02U (Mic AS) */
    /* 08 + 02 bytes */

    /* USB Speaker Input Terminal ID1 Descriptor */
    0x0C, /* bLength = AUDIO_INPUT_TERMINAL_DESC_SIZE = 12 bytes*/
    0x24, /* bDescriptorType = AUDIO_INTERFACE_DESCRIPTOR_TYPE = 0x24 */
    0x02, /* bDescriptorSubtype = AUDIO_CONTROL_INPUT_TERMINAL = 0x02 */
    0x01, /* bTerminalID = ID1 */
    0x01, /* wTerminalType = 0x0101 = USB Streaming Audio Terminal */
    0x01,
    0x00, /* bAssocTerminal = No Associated Terminal*/
    0x02, /* bNrChannels = 0x02 = Stereo */
    0x03, /* wChannelConfig = 0x0003 = Left Front & Right Front Channels */
    0x00,
    0x00, /* iChannelNames */
    0x00, /* iTerminal */
    /* 12 bytes */

    /* USB Speaker Audio Feature Unit ID2 Descriptor */
    0x09, /* bLength = 09 bytes */
    0x24, /* bDescriptorType = AUDIO_INTERFACE_DESCRIPTOR_TYPE = 0x24 */
    0x06, /* bDescriptorSubtype = AUDIO_CONTROL_FEATURE_UNIT = 0x06 */
    0x02, /* bUnitID = AUDIO_OUT_STREAMING_CTRL = 0x02 */
    0x01, /* bSourceID = ID1 Output*/
    0x01, /* bControlSize = bmaControls() is 01 byte */
    0x01, /* bmaControls(0) = AUDIO_CONTROL_MUTE = 0x0001 */
    0,    /* bmaControls(1) = ??? */
    0x00, /* iTerminal */
    /* 09 bytes */

    /* USB Speaker Output Terminal ID3 Descriptor */
    0x09, /* bLength = 09 bytes */
    0x24, /* bDescriptorType = AUDIO_INTERFACE_DESCRIPTOR_TYPE = 0x24 */
    0x03, /* bDescriptorSubtype = AUDIO_CONTROL_OUTPUT_TERMINAL = 0x03 */
    0x03, /* bTerminalID = ID3 */
    0x01, /* wTerminalType = 0x0301 = Speaker Output Terminal Type */
    0x03,
    0x00, /* bAssocTerminal = No Associated Terminal*/
    0x02, /* bSourceID = ID2 Output*/
    0x00, /* iTerminal */
    /* 09 bytes */

    /* USB Mic Input Terminal ID4 Descriptor */
    0x0C, /* bLength = AUDIO_INPUT_TERMINAL_DESC_SIZE = 12 bytes*/
    0x24, /* bDescriptorType = AUDIO_INTERFACE_DESCRIPTOR_TYPE = 0x24 */
    0x02, /* bDescriptorSubtype = AUDIO_CONTROL_INPUT_TERMINAL = 0x02 */
    0x04, /* bTerminalID = ID4 */
    0x00, /* wTerminalType = 0x0200 = Undefined Input Terminal Type */
    0x02,
    0x00, /* bAssocTerminal = No Associated Terminal*/
    0x02, /* bNrChannels = 0x02 = Stereo */
    0x03, /* wChannelConfig = 0x0003 = Left Front & Right Front Channels */
    0x00,
    0x00, /* iChannelNames */
    0x00, /* iTerminal */
    /* 12 bytes */

    /* USB Mic Audio Feature Unit ID5 Descriptor */
    0x09, /* bLength = 09 bytes */
    0x24, /* bDescriptorType = AUDIO_INTERFACE_DESCRIPTOR_TYPE = 0x24 */
    0x06, /* bDescriptorSubtype = AUDIO_CONTROL_FEATURE_UNIT = 0x06 */
    0x05, /* bUnitID = ID5 */
    0x04, /* bSourceID = ID4 Output*/
    0x01, /* bControlSize = bmaControls() is 01 byte */
    0x01, /* bmaControls(0) = AUDIO_CONTROL_MUTE = 0x0001 */
    0, /* bmaControls(1) = ??? */
    0x00, /* iTerminal */
    /* 09 bytes */

    /* USB Mic Output Terminal ID6 Descriptor */
    0x09, /* bLength = 09 bytes */
    0x24, /* bDescriptorType = AUDIO_INTERFACE_DESCRIPTOR_TYPE = 0x24 */
    0x03, /* bDescriptorSubtype = AUDIO_CONTROL_OUTPUT_TERMINAL = 0x03 */
    0x06, /* bUnitID = ID6 */
    0x01, /* wTerminalType = 0x0101 = USB Streaming Audio Terminal */
    0x01,
    0x00, /* bAssocTerminal = No Associated Terminal*/
    0x05, /* bSourceID = ID5 Output*/
    0x00, /* iTerminal */
    /* 09 bytes */
    /*---------------------------------------------------------------------------*/
    /* USB Speaker Standard AS Interface Descriptor - Audio Streaming Zero Bandwidth */
    /* Interface AUDIO_OUT, Alternate Setting 0 */
    0x09, /* bLength = AUDIO_INTERFACE_DESC_SIZE = 0x09U */
    0x04, /* bDescriptorType = USB_DESC_TYPE_INTERFACE = 0x04U */
    AUDIO_OUT_IF, /* bInterfaceNumber = AUDIO_OUT_IF */
    0x00, /* bAlternateSetting */
    0x00, /* bNumEndpoints */
    0x01, /* bInterfaceClass = USB_DEVICE_CLASS_AUDIO = 0x01U */
    0x02, /* bInterfaceSubClass = AUDIO_SUBCLASS_AUDIOSTREAMING = 0x02U */
    0x00, /* bInterfaceProtocol = AUDIO_PROTOCOL_UNDEFINED = 0x00U */
    0x00, /* iInterface */
    /* 09 bytes */

    /* USB Speaker Standard AS Interface Descriptor - Audio Streaming Operational */
    /* Interface AUDIO_OUT, Alternate Setting 1 */
    0x09, /* bLength = AUDIO_INTERFACE_DESC_SIZE = 0x09U */
    0x04, /* bDescriptorType = USB_DESC_TYPE_INTERFACE = 0x04U */
    AUDIO_OUT_IF, /* bInterfaceNumber = AUDIO_OUT_IF */
    0x01, /* bAlternateSetting */
    0x01, /* bNumEndpoints */
    0x01, /* bInterfaceClass = USB_DEVICE_CLASS_AUDIO = 0x01U */
    0x02, /* bInterfaceSubClass = AUDIO_SUBCLASS_AUDIOSTREAMING = 0x02U */
    0x00, /* bInterfaceProtocol = AUDIO_PROTOCOL_UNDEFINED = 0x00U */
    0x00, /* iInterface */
    /* 09 bytes */

    /* USB Speaker Audio Streaming Interface Descriptor */
    0x07, /* bLength = AUDIO_STREAMING_INTERFACE_DESC_SIZE = 0x07U */
    0x24, /* bDescriptorType = AUDIO_INTERFACE_DESCRIPTOR_TYPE = 0x24U */
    0x01, /* bDescriptorSubtype = AUDIO_STREAMING_GENERAL = 0x01U */
    0x01, /* bTerminalLink ID1 */
    0x01, /* bDelay */
    0x01, /* wFormatTag AUDIO_FORMAT_PCM  0x0001*/
    0x00,
    /* 07 bytes */

    /* USB Speaker Audio Type I Format Interface Descriptor */
    0x0B, /* bLength = 11 bytes*/
    0x24, /* bDescriptorType = AUDIO_INTERFACE_DESCRIPTOR_TYPE = 0x24U */
    0x02, /* bDescriptorSubtype = AUDIO_STREAMING_FORMAT_TYPE = 0x02U */
    0x01, /* bFormatType = AUDIO_FORMAT_TYPE_I = 0x01U */
    0x02, /* bNrChannels    = 2 channels */
    0x02, /* bSubFrameSize  = 2 Bytes per frame (16bits) */
    0x10, /* bBitResolution = 16-bits per sample */
    0x01, /* bSamFreqType only one frequency supported */
    AUDIO_SAMPLE_FREQ (USBD_AUDIO_FREQ), /* Audio sampling frequency coded on 3 bytes */
    /* 11 bytes */

    /* Endpoint AUDIO_OUT Standard Descriptor */
    0x09, /* bLength = AUDIO_STANDARD_ENDPOINT_DESC_SIZE = 0x09U */
    0x05, /* bDescriptorType = USB_DESC_TYPE_ENDPOINT = 0x05U */
    AUDIO_OUT_EP, /* bEndpointAddress = AUDIO_OUT_EP */
    0x01, /* bmAttributes = USBD_EP_TYPE_ISOC = 0x01U */
    AUDIO_PACKET_SZE (USBD_AUDIO_FREQ), /* wMaxPacketSize in Bytes (Freq(Samples)*2(Stereo)*2(HalfWord)) */
    0x01, /* bInterval */
    0x00, /* bRefresh */
    0x00, /* bSynchAddress */
    /* 09 bytes */

    /* Endpoint AUDIO_OUT Audio Streaming Descriptor*/
    0x07, /* bLength = AUDIO_STREAMING_ENDPOINT_DESC_SIZE = 0x07U */
    0x25, /* bDescriptorType = AUDIO_ENDPOINT_DESCRIPTOR_TYPE = 0x25U */
    0x01, /* bDescriptor = AUDIO_ENDPOINT_GENERAL = 0x01U */
    0x00, /* bmAttributes */
    0x00, /* bLockDelayUnits */
    0x00, /* wLockDelay */
    0x00,
    /* 07 bytes */
    /*---------------------------------------------------------------------------*/
    /* USB Mic Standard AS Interface Descriptor - Audio Streaming Zero Bandwidth */
    /* Interface AUDIO_IN, Alternate Setting 0 */
    0x09, /* bLength = AUDIO_INTERFACE_DESC_SIZE = 0x09U */
    0x04, /* bDescriptorType = USB_DESC_TYPE_INTERFACE = 0x04U */
    AUDIO_IN_IF, /* bInterfaceNumber = AUDIO_IN_IF */
    0x00, /* bAlternateSetting */
    0x00, /* bNumEndpoints */
    0x01, /* bInterfaceClass = USB_DEVICE_CLASS_AUDIO = 0x01U */
    0x02, /* bInterfaceSubClass = AUDIO_SUBCLASS_AUDIOSTREAMING = 0x02U */
    0x00, /* bInterfaceProtocol = AUDIO_PROTOCOL_UNDEFINED = 0x00U */
    0x00, /* iInterface */
    /* 09 bytes */

    /* USB Mic Standard AS Interface Descriptor - Audio Streaming Operational */
    /* Interface AUDIO_IN, Alternate Setting 1 */
    0x09, /* bLength = AUDIO_INTERFACE_DESC_SIZE = 0x09U */
    0x04, /* bDescriptorType = USB_DESC_TYPE_INTERFACE = 0x04U */
    AUDIO_IN_IF, /* bInterfaceNumber = AUDIO_IN_IF */
    0x01, /* bAlternateSetting */
    0x01, /* bNumEndpoints */
    0x01, /* bInterfaceClass = USB_DEVICE_CLASS_AUDIO = 0x01U */
    0x02, /* bInterfaceSubClass = AUDIO_SUBCLASS_AUDIOSTREAMING = 0x02U */
    0x00, /* bInterfaceProtocol = AUDIO_PROTOCOL_UNDEFINED = 0x00U */
    0x00, /* iInterface */
    /* 09 bytes */

    /* USB Mic Audio Streaming Interface Descriptor */
    0x07, /* bLength = AUDIO_STREAMING_INTERFACE_DESC_SIZE = 0x07U */
    0x24, /* bDescriptorType = AUDIO_INTERFACE_DESCRIPTOR_TYPE = 0x24U */
    0x01, /* bDescriptorSubtype = AUDIO_STREAMING_GENERAL = 0x01U */
    0x06, /* bTerminalLink ID6 */
    0x01, /* bDelay */
    0x01, /* wFormatTag AUDIO_FORMAT_PCM  0x0001*/
    0x00,
    /* 07 bytes */

    /* USB Mic Audio Type I Format Interface Descriptor */
    0x0B, /* bLength = 11 bytes*/
    0x24, /* bDescriptorType = AUDIO_INTERFACE_DESCRIPTOR_TYPE = 0x24U */
    0x02, /* bDescriptorSubtype = AUDIO_STREAMING_FORMAT_TYPE = 0x02U */
    0x01, /* bFormatType = AUDIO_FORMAT_TYPE_I = 0x01U */
    0x02, /* bNrChannels    = 2 channels */
    0x02, /* bSubFrameSize  = 2 Bytes per frame (16bits) */
    0x10, /* bBitResolution = 16 bits per sample */
    0x01, /* bSamFreqType only one frequency supported */
    AUDIO_SAMPLE_FREQ (USBD_AUDIO_FREQ), /* Audio sampling frequency coded on 3 bytes */
    /* 11 bytes */

    /* Endpoint AUDIO_IN Standard Descriptor */
    0x09, /* bLength = AUDIO_STANDARD_ENDPOINT_DESC_SIZE = 0x09U */
    0x05, /* bDescriptorType = USB_DESC_TYPE_ENDPOINT = 0x05U */
    AUDIO_IN_EP, /* bEndpointAddress = AUDIO_IN_EP */
    0x01, /* bmAttributes = USBD_EP_TYPE_ISOC = 0x01U */
    AUDIO_PACKET_SZE (USBD_AUDIO_FREQ), /* wMaxPacketSize in Bytes (Freq(Samples)*2(Stereo)*2(HalfWord)) */
    0x01, /* bInterval */
    0x00, /* bRefresh */
    0x00, /* bSynchAddress */
    /* 09 bytes */

    /* Endpoint AUDIO_IN Audio Streaming Descriptor */
    0x07, /* bLength = AUDIO_STREAMING_ENDPOINT_DESC_SIZE = 0x07U */
    0x25, /* bDescriptorType = AUDIO_ENDPOINT_DESCRIPTOR_TYPE = 0x25U */
    0x01, /* bDescriptor = AUDIO_ENDPOINT_GENERAL = 0x01U */
    0x00, /* bmAttributes */
    0x00, /* bLockDelayUnits */
    0x00, /* wLockDelay */
    0x00
    /* 07 bytes */
    /* 192 bytes total */
};

/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_AUDIO_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END=
{
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00,
  0x02,
  0x00,
  0x00,
  0x00,
  0x40,
  0x01,
  0x00,
};

/**
  * @}
  */

/** @defgroup USBD_AUDIO_Private_Functions
  * @{
  */

/**
  * @brief  USBD_AUDIO_Init
  *         Initialize the AUDIO interface
  * @param  pdev:   device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */

static uint8_t USBD_AUDIO_Init (USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  USBD_AUDIO_HandleTypeDef *haudio;

  /* Open EP OUT */
  USBD_LL_OpenEP (pdev, AUDIO_OUT_EP, USBD_EP_TYPE_ISOC, AUDIO_OUT_PACKET);
  pdev->ep_out[AUDIO_OUT_EP & 0xFU].is_used = 1U;

  /* Open EP IN */
  USBD_LL_OpenEP (pdev, AUDIO_IN_EP, USBD_EP_TYPE_ISOC, AUDIO_OUT_PACKET);
  pdev->ep_in[AUDIO_IN_EP & 0xFU].is_used = 1U;

  /* Allocate Audio structure */
  pdev->pClassData = (void*) &USBD_AUDIO_Handle;

  if (pdev->pClassData == NULL)
  {
    return USBD_FAIL;
  }
  else
  {
    haudio = (USBD_AUDIO_HandleTypeDef*) pdev->pClassData;

    for (uint8_t i = 0; i <= USBD_MAX_NUM_INTERFACES; i++)
    {
      haudio->alt_setting [i] = 0U;
    }

    /* Initialize the Audio Out Buffer */
    haudio->out.wr_ptr = 0U;
    haudio->out.rd_ptr = 0U;
    haudio->out.buff_enable = 0U;
    memset ((uint8_t*) haudio->out.buff, 0, AUDIO_TOTAL_BUF_SIZE);

    /* Initialize the Audio In Buffer */
    haudio->in.wr_ptr = 0U;
    haudio->in.rd_ptr = 0U;
    haudio->in.buff_enable = 0U;
    memset ((uint8_t*) haudio->in.buff, 0, AUDIO_TOTAL_BUF_SIZE);

    /* Initialize the Audio output Hardware layer */
    if (((USBD_AUDIO_ItfTypeDef*) pdev->pUserData)->Init (USBD_AUDIO_FREQ,
                                                          AUDIO_DEFAULT_VOLUME,
                                                          0U) != 0)
    {
      return USBD_FAIL;
    }

    /* Prepare Out endpoint to receive 1st packet */
    USBD_LL_PrepareReceive (pdev, AUDIO_OUT_EP, haudio->out.buff, AUDIO_OUT_PACKET);
  }
  return USBD_OK;
}

/**
  * @brief  USBD_AUDIO_DeInit
  *         DeInitialize the AUDIO layer
  * @param  pdev:   device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */

static uint8_t  USBD_AUDIO_DeInit (USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  /* Close EP OUT */
  USBD_LL_CloseEP (pdev, AUDIO_OUT_EP);
  pdev->ep_out[AUDIO_OUT_EP & 0xFU].is_used = 0U;

  /* Close EP IN */
  USBD_LL_CloseEP (pdev, AUDIO_IN_EP);
  pdev->ep_in[AUDIO_IN_EP & 0xFU].is_used = 0U;

  /* DeInit  physical Interface components */
  if (pdev->pClassData != NULL)
  {
    ((USBD_AUDIO_ItfTypeDef*) pdev->pUserData)->DeInit (0U);
    pdev->pClassData = NULL;
  }

  return USBD_OK;
}

/**
  * @brief  USBD_AUDIO_Setup
  *         Handle the AUDIO specific requests
  * @param  pdev: instance
  * @param  req:  usb requests
  * @retval status
  */

static uint8_t  USBD_AUDIO_Setup (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  USBD_AUDIO_HandleTypeDef *haudio;
  uint16_t len;
  uint8_t  *pbuf;
  uint16_t status_info = 0U;
  uint8_t  ret = USBD_OK;

  haudio = (USBD_AUDIO_HandleTypeDef*) pdev->pClassData;

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
  case USB_REQ_TYPE_CLASS:
    switch (req->bRequest)
    {
    case AUDIO_REQ_GET_CUR:
      AUDIO_REQ_GetCurrent (pdev, req);
      break;

    case AUDIO_REQ_SET_CUR:
      AUDIO_REQ_SetCurrent (pdev, req);
      break;

    default:
      USBD_CtlError (pdev, req);
      ret = USBD_FAIL;
      break;
    }
    break;

  case USB_REQ_TYPE_STANDARD:
    switch (req->bRequest)
    {
    case USB_REQ_GET_STATUS:
      if (pdev->dev_state == USBD_STATE_CONFIGURED)
      {
        USBD_CtlSendData (pdev, (uint8_t *) (void *) &status_info, 2U);
      }
      else
      {
        USBD_CtlError (pdev, req);
        ret = USBD_FAIL;
      }
      break;

    case USB_REQ_GET_DESCRIPTOR:
      if( (req->wValue >> 8) == AUDIO_DESCRIPTOR_TYPE)
      {
        pbuf = USBD_AUDIO_CfgDesc + 18;
        len  = MIN (pbuf[0], req->wLength);

        USBD_CtlSendData (pdev, pbuf, len);
      }
      break;

    case USB_REQ_GET_INTERFACE :
      if (pdev->dev_state == USBD_STATE_CONFIGURED)
      {
        USBD_CtlSendData (pdev, (uint8_t*) (void*) &haudio->alt_setting[req->wIndex], 1U);
      }
      else
      {
        USBD_CtlError (pdev, req);
        ret = USBD_FAIL;
      }
      break;

    case USB_REQ_SET_INTERFACE :
      if (pdev->dev_state == USBD_STATE_CONFIGURED)
      {
         if ((uint8_t) (req->wValue) <= USBD_MAX_NUM_INTERFACES)
         {
           haudio->alt_setting [req->wIndex] = (uint8_t) (req->wValue);

           /* Handles Alternate Settings 0 of Audio OUT interface */
           if (haudio->alt_setting[AUDIO_OUT_IF] == 0)
           {
             ((USBD_AUDIO_ItfTypeDef *) pdev->pUserData)->AudioCmd (&haudio->out.buff[0],
                                                                    AUDIO_TOTAL_BUF_SIZE / 2U,
                                                                    AUDIO_CMD_STOP);
           }

           /* Handles Alternate Settings 1 of Audio IN interface */
           if (haudio->alt_setting[AUDIO_IN_IF] == 1)
           {
             if (!haudio->in.buff_enable)
             {
               /* Prepare IN endpoint to send 1st packet */
               ((USBD_AUDIO_ItfTypeDef *) pdev->pUserData)->AudioCmd (&haudio->in.buff[0],
                                                                      AUDIO_TOTAL_BUF_SIZE / 2U,
                                                                      AUDIO_CMD_RECORD);
               haudio->in.wr_ptr = AUDIO_TOTAL_BUF_SIZE / 2U;
               haudio->in.buff_enable = 1U;

               USBD_LL_FlushEP (pdev, AUDIO_IN_EP);

               USBD_LL_Transmit (pdev, AUDIO_IN_EP, haudio->in.buff, AUDIO_OUT_PACKET);

               haudio->in.rd_ptr += AUDIO_OUT_PACKET;
             }
           }
           else
           {
             haudio->in.buff_enable = 0U;
             USBD_LL_FlushEP (pdev, AUDIO_IN_EP);
           }
         }
         else
         {
           /* Call the error management function (command will be nacked */
           USBD_CtlError (pdev, req);
           ret = USBD_FAIL;
         }
      }
      else
      {
        USBD_CtlError (pdev, req);
        ret = USBD_FAIL;
      }
      break;

    default:
      USBD_CtlError (pdev, req);
      ret = USBD_FAIL;
      break;
    }
    break;
  default:
    USBD_CtlError (pdev, req);
    ret = USBD_FAIL;
    break;
  }

  return ret;
}

/**
  * @brief  USBD_AUDIO_GetCfgDesc
  *         return configuration descriptor
  * @param  length: pointer data length
  * @retval pointer to descriptor buffer
  */

static uint8_t *USBD_AUDIO_GetCfgDesc (uint16_t *length)
{
  *length = sizeof (USBD_AUDIO_CfgDesc);
  return USBD_AUDIO_CfgDesc;
}

/**
  * @brief  USBD_AUDIO_DataIn
  *         handle data IN Stage
  * @param  pdev:  device instance
  * @param  epnum: endpoint index
  * @retval status
  */

static uint8_t USBD_AUDIO_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  USBD_AUDIO_HandleTypeDef *haudio;
  uint8_t retval = USBD_OK;

  haudio = (USBD_AUDIO_HandleTypeDef*) pdev->pClassData;

  if (epnum == (AUDIO_IN_EP & 0x7F))
  {
    if (haudio->in.rd_ptr == AUDIO_TOTAL_BUF_SIZE / 2U)
    {
      ((USBD_AUDIO_ItfTypeDef *) pdev->pUserData)->AudioCmd (&haudio->in.buff[0],
                                                             AUDIO_TOTAL_BUF_SIZE / 2U,
                                                             AUDIO_CMD_RECORD);
      haudio->in.wr_ptr = AUDIO_TOTAL_BUF_SIZE / 2U;
    }

    if (haudio->in.rd_ptr == 0U)
    {
      ((USBD_AUDIO_ItfTypeDef *) pdev->pUserData)->AudioCmd (&haudio->in.buff[AUDIO_TOTAL_BUF_SIZE / 2U],
                                                             AUDIO_TOTAL_BUF_SIZE / 2U,
                                                             AUDIO_CMD_RECORD);
      haudio->in.wr_ptr = 0U;
    }

    USBD_LL_FlushEP (pdev, AUDIO_IN_EP);

    USBD_LL_Transmit (pdev, AUDIO_IN_EP, &haudio->in.buff[haudio->in.rd_ptr], AUDIO_OUT_PACKET);

    haudio->in.rd_ptr += AUDIO_OUT_PACKET;

    if (haudio->in.rd_ptr == AUDIO_TOTAL_BUF_SIZE)
    {
      haudio->in.rd_ptr = 0U;
    }
  }

  return retval;
}

/**
  * @brief  USBD_AUDIO_EP0_RxReady
  *         handle EP0 Rx Ready event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t USBD_AUDIO_EP0_RxReady (USBD_HandleTypeDef *pdev)
{
  USBD_AUDIO_HandleTypeDef *haudio;
  haudio = (USBD_AUDIO_HandleTypeDef*) pdev->pClassData;

  if (haudio->control.cmd == AUDIO_REQ_SET_CUR)
  {/* In this driver, to simplify code, only SET_CUR request is managed */

    if (haudio->control.unit == AUDIO_OUT_STREAMING_CTRL)
    {
      ((USBD_AUDIO_ItfTypeDef *) pdev->pUserData)->MuteCtl (haudio->control.data[0]);
      haudio->control.cmd = 0U;
      haudio->control.len = 0U;
    }
  }

  return USBD_OK;
}

/**
  * @brief  USBD_AUDIO_EP0_TxReady
  *         handle EP0 TRx Ready event
  * @param  pdev: device instance
  * @retval status
  */

static uint8_t USBD_AUDIO_EP0_TxReady (USBD_HandleTypeDef *pdev)
{
  /* Only OUT control data are processed */
  return USBD_OK;
}

/**
  * @brief  USBD_AUDIO_SOF
  *         handle SOF event
  * @param  pdev: device instance
  * @retval status
  */

static uint8_t USBD_AUDIO_SOF (USBD_HandleTypeDef *pdev)
{
  return USBD_OK;
}

/**
  * @brief  USBD_AUDIO_Sync
  *         handle Sync event
  * @param  pdev:   device instance
  * @param  offset: buffer offset
  * @retval status
  */

void USBD_AUDIO_Sync (USBD_HandleTypeDef *pdev, AUDIO_OffsetTypeDef offset)
{

}

/**
  * @brief  USBD_AUDIO_IsoINIncomplete
  *         handle data ISO IN Incomplete event
  * @param  pdev:  device instance
  * @param  epnum: endpoint index
  * @retval status
  */

static uint8_t USBD_AUDIO_IsoINIncomplete (USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  return USBD_OK;
}

/**
  * @brief  USBD_AUDIO_IsoOutIncomplete
  *         handle data ISO OUT Incomplete event
  * @param  pdev:  device instance
  * @param  epnum: endpoint index
  * @retval status
  */

static uint8_t USBD_AUDIO_IsoOutIncomplete (USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  return USBD_OK;
}

/**
  * @brief  USBD_AUDIO_DataOut
  *         handle data OUT Stage
  * @param  pdev:  device instance
  * @param  epnum: endpoint index
  * @retval status
  */

static uint8_t USBD_AUDIO_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  USBD_AUDIO_HandleTypeDef *haudio;
  haudio = (USBD_AUDIO_HandleTypeDef*) pdev->pClassData;

  if (epnum == AUDIO_OUT_EP)
  {
    haudio->out.wr_ptr += AUDIO_OUT_PACKET;

    if (haudio->out.wr_ptr == AUDIO_TOTAL_BUF_SIZE / 2U)
    {
      if (haudio->out.buff_enable == 0U)
      {
        haudio->out.buff_enable = 1U;
      }

      ((USBD_AUDIO_ItfTypeDef*) pdev->pUserData)->AudioCmd (&haudio->out.buff[0],
                                                            AUDIO_TOTAL_BUF_SIZE / 2U,
                                                            AUDIO_CMD_PLAY);
    }

    if (haudio->out.wr_ptr == AUDIO_TOTAL_BUF_SIZE)
    {
      /* All buffers are full: roll back */
      haudio->out.wr_ptr = 0U;

      ((USBD_AUDIO_ItfTypeDef*) pdev->pUserData)->AudioCmd (&haudio->out.buff[AUDIO_TOTAL_BUF_SIZE / 2U],
                                                            AUDIO_TOTAL_BUF_SIZE / 2U,
                                                            AUDIO_CMD_PLAY);
    }
    /* Prepare Out endpoint to receive next audio packet */
    USBD_LL_PrepareReceive (pdev, AUDIO_OUT_EP, &haudio->out.buff[haudio->out.wr_ptr], AUDIO_OUT_PACKET);
  }

  return USBD_OK;
}

/**
  * @brief  AUDIO_Req_GetCurrent
  *         Handles the GET_CUR Audio control request.
  * @param  pdev: instance
  * @param  req:  setup class request
  * @retval status
  */

static void AUDIO_REQ_GetCurrent (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  USBD_AUDIO_HandleTypeDef *haudio;
  haudio = (USBD_AUDIO_HandleTypeDef*) pdev->pClassData;

  memset (haudio->control.data, 0, 64U);

  /* Send the current mute state */
  USBD_CtlSendData (pdev, haudio->control.data, req->wLength);
}

/**
  * @brief  AUDIO_Req_SetCurrent
  *         Handles the SET_CUR Audio control request.
  * @param  pdev: instance
  * @param  req:  setup class request
  * @retval status
  */

static void AUDIO_REQ_SetCurrent (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  USBD_AUDIO_HandleTypeDef *haudio;
  haudio = (USBD_AUDIO_HandleTypeDef*) pdev->pClassData;

  if (req->wLength)
  {
    /* Prepare the reception of the buffer over EP0 */
    USBD_CtlPrepareRx (pdev, haudio->control.data, req->wLength);

    haudio->control.cmd  = AUDIO_REQ_SET_CUR;      /* Set the request value */
    haudio->control.len  = (uint8_t) req->wLength; /* Set the request data length */
    haudio->control.unit = HIBYTE (req->wIndex);   /* Set the request target unit */
  }
}

/**
  * @brief  DeviceQualifierDescriptor
  *         return Device Qualifier descriptor
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */

static uint8_t *USBD_AUDIO_GetDeviceQualifierDesc (uint16_t *length)
{
  *length = sizeof (USBD_AUDIO_DeviceQualifierDesc);
  return USBD_AUDIO_DeviceQualifierDesc;
}

/**
  * @brief  USBD_AUDIO_RegisterInterface
  * @param  pdev: instance
  * @param  fops: Audio interface callback
  * @retval status
  */

uint8_t USBD_AUDIO_RegisterInterface (USBD_HandleTypeDef *pdev, USBD_AUDIO_ItfTypeDef *fops)
{
  if(fops != NULL)
  {
    pdev->pUserData= fops;
  }
  return USBD_OK;
}

/**
  * @}
  */


/**
  * @}
  */


/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
