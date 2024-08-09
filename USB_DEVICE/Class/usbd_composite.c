/**
  *******************************************************************************
  *
  * @file    usbd_comp.c
  * @brief   USB composite device driver
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


/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "usbd_ctlreq.h"

#include <stdio.h>
#include "usbd_audio_if.h"
#include "usbd_composite.h"
#include "main.h"
#include "usbd_audio.h"


/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define AUDIO_SAMPLE_FREQ(frq)  (uint8_t)(frq), (uint8_t)((frq >> 8)), (uint8_t)((frq >> 16))

#define AUDIO_PACKET_SZE(frq)   (uint8_t)(((frq * 2U * 2U)/1000U) & 0xFFU), \
                                (uint8_t)((((frq * 2U * 2U)/1000U) >> 8) & 0xFFU)

/* Private macro -------------------------------------------------------------*/

/* Private function prototypes------------------------------------------------*/
static uint8_t USBD_COMP_Init (USBD_HandleTypeDef *pdev, uint8_t cfgidx);

static uint8_t USBD_COMP_DeInit (USBD_HandleTypeDef *pdev, uint8_t cfgidx);

static uint8_t USBD_COMP_Setup (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);

static uint8_t* USBD_COMP_GetCfgDesc (uint16_t *length);

static uint8_t* USBD_COMP_GetDeviceQualifierDesc (uint16_t *length);

static uint8_t USBD_COMP_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t USBD_COMP_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t USBD_COMP_EP0_RxReady (USBD_HandleTypeDef *pdev);

static uint8_t USBD_COMP_EP0_TxReady (USBD_HandleTypeDef *pdev);

static uint8_t USBD_COMP_SOF (USBD_HandleTypeDef *pdev);

static uint8_t USBD_COMP_IsoINIncomplete (USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t USBD_COMP_IsoOutIncomplete (USBD_HandleTypeDef *pdev, uint8_t epnum);

/* Private variables ---------------------------------------------------------*/

USBD_ClassTypeDef  USBD_COMP =
{
  USBD_COMP_Init,
  USBD_COMP_DeInit,
  USBD_COMP_Setup,
  USBD_COMP_EP0_TxReady,
  USBD_COMP_EP0_RxReady,
  USBD_COMP_DataIn,
  USBD_COMP_DataOut,
  USBD_COMP_SOF,
  USBD_COMP_IsoINIncomplete,
  USBD_COMP_IsoOutIncomplete,
  USBD_COMP_GetCfgDesc,
  USBD_COMP_GetCfgDesc,
  USBD_COMP_GetCfgDesc,
  USBD_COMP_GetDeviceQualifierDesc,
};

extern USBD_HandleTypeDef hUsbDeviceFS;

USBD_COMP_ItfTypeDef USBD_COMP_fops_FS;
USBD_ClassCompInfo comp_dev [CLASS_NUM];


/* USB COMP device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_COMP_CfgDesc[USB_COMP_CONFIG_DESC_SIZ] __ALIGN_END =
{
    /* Configuration 1 */
    0x09, /* bLength = 9 bytes */
    0x02, /* bDescriptorType = USB_DESC_TYPE_CONFIGURATION = 0x02 */
    LOBYTE(USB_COMP_CONFIG_DESC_SIZ), /* wTotalLength = 266 bytes */
    HIBYTE(USB_COMP_CONFIG_DESC_SIZ),
    0x05, /* bNumInterfaces = 5 interfaces = Control (AC) + OUT (Speaker AS) + IN (Mic AS) + CDC_CTRL + CDC_DATA */
    0x01, /* bConfigurationValue */
    0x00, /* iConfiguration */
    0xC0, /* bmAttributes = BUS Powered*/
    0x32, /* bMaxPower = 50 * 2 mA = 100 mA*/
    /* 09 bytes */
    /*---------------------------------------------------------------------------*/
    /* CDC IAD */
    0x08, /* bLength: Interface Descriptor size */
    0x0B, /* bDescriptorType: IAD */
    CDC_CTRL_IF,  /* bFirstInterface */
    0x02, /* bInterfaceCount */
    0x02, /* bFunctionClass: CDC */
    0x02, /* bFunctionSubClass */
    0x01, /* bFunctionProtocol */
    0x02, /* iFunction */
    /* 08 bytes */
    /*---------------------------------------------------------------------------*/
    /*Interface Descriptor */
    0x09, /* bLength: Interface Descriptor size */
    0x04, /* bDescriptorType: Interface = USB_DESC_TYPE_INTERFACE = 0x04 */
    /* Interface descriptor type */
    CDC_CTRL_IF, /* bInterfaceNumber: Communication class interface number */
    0x00, /* bAlternateSetting: Alternate setting */
    0x01, /* bNumEndpoints: One endpoint used */
    0x02, /* bInterfaceClass: Communication Interface Class */
    0x02, /* bInterfaceSubClass: Abstract Control Model */
    0x01, /* bInterfaceProtocol: Common AT commands */
    0x00, /* iInterface: */
    /* 09 bytes */

    /*Header Functional Descriptor*/
    0x05, /* bLength: Endpoint Descriptor size */
    0x24, /* bDescriptorType: CS_INTERFACE */
    0x00, /* bDescriptorSubtype: Header Func Desc */
    0x10, /* bcdCDC: 0x0110 spec release number */
    0x01,
    /* 05 bytes */

    /*Call Management Functional Descriptor*/
    0x05, /* bFunctionLength */
    0x24, /* bDescriptorType: CS_INTERFACE */
    0x01, /* bDescriptorSubtype: Call Management Func Desc */
    0x00, /* bmCapabilities: D0+D1 */
    CDC_DATA_IF, /* bDataInterface: Data Class Interface number */
    /* 05 bytes */

    /*ACM Functional Descriptor*/
    0x04, /* bFunctionLength */
    0x24, /* bDescriptorType: CS_INTERFACE */
    0x02, /* bDescriptorSubtype: Abstract Control Management desc */
    0x02, /* bmCapabilities */
    /* 04 bytes */

    /*Union Functional Descriptor*/
    0x05, /* bFunctionLength */
    0x24, /* bDescriptorType: CS_INTERFACE */
    0x06, /* bDescriptorSubtype: Union func desc */
    CDC_CTRL_IF, /* bMasterInterface: Communication class interface number */
    CDC_DATA_IF, /* bSlaveInterface0: Data Class Interface number */
    /* 05 bytes */

    /*Endpoint CMD Descriptor*/
    0x07, /* bLength: Endpoint Descriptor size */
    0x05, /* bDescriptorType: Endpoint = USB_DESC_TYPE_ENDPOINT = 0x05 */
    CDC_CMD_EP, /* bEndpointAddress */
    0x03, /* bmAttributes: Interrupt */
    LOBYTE(CDC_CMD_PACKET_SIZE), /* wMaxPacketSize: */
    HIBYTE(CDC_CMD_PACKET_SIZE),
    CDC_FS_BINTERVAL, /* bInterval: */
    /* 07 bytes */
    /*---------------------------------------------------------------------------*/
    /*Data class interface descriptor*/
    0x09, /* bLength: Endpoint Descriptor size */
    0x04, /* bDescriptorType: Interface = USB_DESC_TYPE_INTERFACE = 0x04 */
    CDC_DATA_IF, /* bInterfaceNumber: Number of Interface */
    0x00, /* bAlternateSetting: Alternate setting */
    0x02, /* bNumEndpoints: Two endpoints used */
    0x0A, /* bInterfaceClass: CDC */
    0x00, /* bInterfaceSubClass: */
    0x00, /* bInterfaceProtocol: */
    0x00, /* iInterface: */
    /* 09 bytes */

    /*Endpoint OUT Descriptor*/
    0x07, /* bLength: Endpoint Descriptor size */
    0x05, /* bDescriptorType: Endpoint = USB_DESC_TYPE_ENDPOINT = 0x05 */
    CDC_OUT_EP, /* bEndpointAddress: CDC_OUT_EP */
    0x02, /* bmAttributes: Bulk */
    LOBYTE (CDC_DATA_FS_MAX_PACKET_SIZE), /* wMaxPacketSize: */
    HIBYTE (CDC_DATA_FS_MAX_PACKET_SIZE),
    0x00, /* bInterval: ignore for Bulk transfer */
    /* 07 bytes */

    /*Endpoint IN Descriptor*/
    0x07,   /* bLength: Endpoint Descriptor size */
    0x05,   /* bDescriptorType: Endpoint = USB_DESC_TYPE_ENDPOINT = 0x05 */
    CDC_IN_EP, /* bEndpointAddress: CDC_IN_EP */
    0x02,   /* bmAttributes: Bulk */
    LOBYTE (CDC_DATA_FS_MAX_PACKET_SIZE), /* wMaxPacketSize: */
    HIBYTE (CDC_DATA_FS_MAX_PACKET_SIZE),
    0x00,    /* bInterval: ignore for Bulk transfer */
    /* 07 bytes */
    /* 58 bytes total */
    /*---------------------------------------------------------------------------*/
    /* AUDIO IAD */
    0x08, /* bLength: Interface Descriptor size */
    0x0B, /* bDescriptorType: IAD */
    AUDIO_CTRL_IF, /* bFirstInterface */
    0x03, /* bInterfaceCount */
    0x01, /* bFunctionClass  = USB_DEVICE_CLASS_AUDIO = 0x01 */
    0x01, /* bFunctionSubClass = AUDIO_SUBCLASS_AUDIOCONTROL = 0x01 */
    0x00, /* bFunctionProtocol = AUDIO_PROTOCOL_UNDEFINED = Non-Basic Audio Device */
    0x00, /* iFunction */
    /* 08 bytes */
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
    AUDIO_OUT_IF, /* baInterfaceNr1 AUDIO_OUT_IF = 0x03U (Speaker AS) */
    AUDIO_IN_IF,  /* baInterfaceNr2 AUDIO_IN_IF  = 0x04U (Mic AS) */
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
    0x03, /* +++ wTerminalType = 0x0603 = Line Connector Terminal Type */
    0x06,
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
    /* 183 bytes total */
};

/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_COMP_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END=
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

/* Private function prototypes -----------------------------------------------*/


/* Private user code ---------------------------------------------------------*/


/* External variables --------------------------------------------------------*/


/* Private functions ---------------------------------------------------------*/

static inline void switchToClass (USBD_HandleTypeDef *pdev, USBD_ClassCompInfo *class)
{
  pdev->pClassData = class->classData;
  pdev->pUserData  = class->userData;
}

static inline void saveClass (USBD_HandleTypeDef *pdev, USBD_ClassCompInfo *class)
{
  class->classData = pdev->pClassData;
  class->userData  = pdev->pUserData;
}

/**
 * @brief  USBD_COMP_Init
 *         Initialize the COMP interface
 * @param  pdev:   device instance
 * @param  cfgidx: configuration index
 * @retval status
 */

static uint8_t USBD_COMP_Init (USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  uint8_t retval = USBD_OK;

  USBD_CDC_LineCodingTypeDef line_coding =
  {
    /* 9600 8n1 */
    .bitrate    = 9600U, /* Data terminal rate, in bits per second */
    .format     = 0U,    /* Stop bits: 0 - 1 Stop bit */
    .paritytype = 0U,    /* Parity:    0 - None */
    .datatype   = 8U,    /* Data bits */
  };

  comp_dev[VCP].class    = &USBD_CDC;
  comp_dev[VCP].userData = &USBD_Interface_fops_FS;
  comp_dev[VCP].ctrlIf   = CDC_CTRL_IF;
  comp_dev[VCP].minIf    = CDC_CTRL_IF;
  comp_dev[VCP].maxIf    = CDC_DATA_IF;

  switchToClass (pdev, &comp_dev[VCP]);
  retval = comp_dev[VCP].class->Init (pdev, cfgidx);
  saveClass (pdev, &comp_dev[VCP]);
  memcpy ((uint8_t*) comp_dev[VCP].classData, &line_coding, sizeof (line_coding));

  comp_dev[UAC].class    = &USBD_AUDIO;
  comp_dev[UAC].userData = &USBD_AUDIO_fops_FS;
  comp_dev[UAC].ctrlIf   = AUDIO_CTRL_IF;
  comp_dev[UAC].minIf    = AUDIO_CTRL_IF;
  comp_dev[UAC].maxIf    = AUDIO_IN_IF;

  switchToClass (pdev, &comp_dev[UAC]);
  retval = comp_dev[UAC].class->Init (pdev, cfgidx);
  saveClass (pdev, &comp_dev[UAC]);

  return retval;
}

/**
 * @brief  USBD_COMP_DeInit
 *         DeInitialize the COMP layer
 * @param  pdev:   device instance
 * @param  cfgidx: configuration index
 * @retval status
 */

static uint8_t USBD_COMP_DeInit (USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  uint8_t retval = USBD_OK;

  for (uint8_t i = 0U; i < CLASS_NUM; i++)
  {
    switchToClass (pdev, &comp_dev[i]);
    retval = comp_dev[i].class->DeInit (pdev, cfgidx);
    saveClass (pdev, &comp_dev[i]);
  }
  return retval;
}

/**
 * @brief  USBD_COMP_Setup
 *         Handle the AUDIO specific requests
 * @param  pdev: instance
 * @param  req: usb requests
 * @retval status
 */

static uint8_t USBD_COMP_Setup (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  uint16_t len;
  uint8_t  *pbuf;
  uint8_t  retval = USBD_OK;
  uint8_t  done = 0U;
  uint8_t  i;

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
    case USB_REQ_TYPE_CLASS:
      switch (req->bRequest)
      {
        case AUDIO_REQ_GET_CUR:
        case AUDIO_REQ_SET_CUR:

          switchToClass (pdev, &comp_dev[UAC]);
          retval = comp_dev[UAC].class->Setup (pdev, req);
          if (!retval) done = 1U;
          break;

        case CDC_SET_LINE_CODING:
        case CDC_GET_LINE_CODING:
        case CDC_SET_CONTROL_LINE_STATE:

          switchToClass (pdev, &comp_dev[VCP]);
          retval = comp_dev[VCP].class->Setup (pdev, req);
          if (!retval) done = 1U;
          break;
      }

      if (done != 1U)
      {
        USBD_CtlError (pdev, req);
        retval = USBD_FAIL;
      }

      break;

    case USB_REQ_TYPE_STANDARD:

      switch (req->bRequest)
      {
        case USB_REQ_GET_DESCRIPTOR:

          if( (req->wValue >> 8) == AUDIO_DESCRIPTOR_TYPE)
          {
            pbuf = USBD_COMP_CfgDesc + 92;
            len  = MIN (pbuf[0] , req->wLength);

            USBD_CtlSendData (pdev, pbuf, len);
          }
          break;

        case USB_REQ_GET_INTERFACE:
        case USB_REQ_SET_INTERFACE:

          if ((uint8_t) (req->wIndex) <= USBD_MAX_NUM_INTERFACES)
          {
            for (i = 0U; i < CLASS_NUM; i++)
            {
              if ((req->wIndex >= comp_dev[i].minIf) && (req->wIndex <= comp_dev[i].maxIf))
              {
                switchToClass (pdev, &comp_dev[i]);
                retval = comp_dev[i].class->Setup (pdev, req);
                done = 1U;
                break;
              }
            }
          }
          else
          {
            /* Call the error management function (command will be nacked */
            USBD_CtlError (pdev, req);
          }
          break;

        default:
          USBD_CtlError (pdev, req);
          retval = USBD_FAIL;
      }
  }
  return retval;
}

/**
 * @brief  USBD_COMP_DataIn
 *         handle data IN Stage
 * @param  pdev:  device instance
 * @param  epnum: endpoint index
 * @retval status
 */

static uint8_t  USBD_COMP_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  uint8_t retval = USBD_OK;
  uint8_t i;

  if (epnum == (CDC_IN_EP & 0x7F))   i = VCP;
  if (epnum == (AUDIO_IN_EP & 0x7F)) i = UAC;

  if (comp_dev[i].class->DataIn != NULL)
  {
    switchToClass (pdev, &comp_dev[i]);
    retval = comp_dev[i].class->DataIn (pdev, epnum);
  }

  return retval;
}

/**
 * @brief  USBD_COMP_DataOut
 *         handle data OUT Stage
 * @param  pdev:  device instance
 * @param  epnum: endpoint index
 * @retval status
 */

static uint8_t  USBD_COMP_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  uint8_t retval = USBD_OK;
  uint8_t i;

  if (epnum == CDC_OUT_EP)   i = VCP;
  if (epnum == AUDIO_OUT_EP) i = UAC;

  if (comp_dev[i].class->DataOut != NULL)
  {
    switchToClass (pdev, &comp_dev[i]);
    retval = comp_dev[i].class->DataOut (pdev, epnum);
  }

  return retval;
}

/**
 * @brief  USBD_COMP_SOF
 *         handle SOF event
 * @param  pdev: device instance
 * @retval status
 */

static uint8_t  USBD_COMP_SOF (USBD_HandleTypeDef *pdev)
{
  uint8_t retval = USBD_OK;

  for (uint8_t i = 0U; i < CLASS_NUM; i++)
  {
    if (comp_dev[i].class->SOF != NULL)
    {
      switchToClass (pdev, &comp_dev[i]);
      retval = comp_dev[i].class->SOF (pdev);
    }
  }
  return retval;
}

/**
 * @brief  USBD_COMP_EP0_RxReady
 *         handle EP0 Rx Ready event
 * @param  pdev: device instance
 * @retval status
 */

static uint8_t  USBD_COMP_EP0_RxReady (USBD_HandleTypeDef *pdev)
{
  uint8_t retval = USBD_OK;

  for (uint8_t i = 0U; i < CLASS_NUM; i++)
  {
    if (comp_dev[i].class->EP0_RxReady != NULL)
    {
      switchToClass (pdev, &comp_dev[i]);
      retval = comp_dev[i].class->EP0_RxReady (pdev);
    }
  }
  return retval;
}

/**
  * @brief  USBD_COMP_EP0_TxReady
  *         handle EP0 Tx Ready event
  * @param  pdev: device instance
  * @retval status
  */

static uint8_t  USBD_COMP_EP0_TxReady (USBD_HandleTypeDef *pdev)
{
  /* Only OUT control data are processed */
  return USBD_OK;
}

/**
  * @brief  USBD_COMP_IsoINIncomplete
  *         handle data ISO IN Incomplete event
  * @param  pdev:  device instance
  * @param  epnum: endpoint index
  * @retval status
  */

static uint8_t  USBD_COMP_IsoINIncomplete (USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  return USBD_OK;
}

/**
  * @brief  USBD_COMP_IsoOutIncomplete
  *         handle data ISO OUT Incomplete event
  * @param  pdev:  device instance
  * @param  epnum: endpoint index
  * @retval status
  */

static uint8_t  USBD_COMP_IsoOutIncomplete (USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  return USBD_OK;
}

/**
 * @brief  USBD_COMP_GetCfgDesc
 *         return Configuration descriptor
 * @param  length: pointer data length
 * @retval pointer to descriptor buffer
 */

static uint8_t* USBD_COMP_GetCfgDesc (uint16_t *length)
{
  *length = sizeof (USBD_COMP_CfgDesc);
  return USBD_COMP_CfgDesc;
}

/**
* @brief  DeviceQualifierDescriptor
*         return Device Qualifier descriptor
* @param  length: pointer data length
* @retval pointer to descriptor buffer
*/

static uint8_t* USBD_COMP_GetDeviceQualifierDesc (uint16_t *length)
{
  *length = sizeof (USBD_COMP_DeviceQualifierDesc);
  return USBD_COMP_DeviceQualifierDesc;
}

/**
 * @brief  USBD_COMP_RegisterInterface
 * @param  fops: COMP interface callback
 * @retval status
 */

uint8_t USBD_COMP_RegisterInterface (USBD_HandleTypeDef   *pdev,
                                     USBD_COMP_ItfTypeDef *fops)
{
  if (fops != NULL)
  {
    pdev->pUserData = fops;
  }
  return USBD_OK;
}


/**
  * @brief  CDC_Transmit_FS
  *         Data to send over USB IN endpoint are sent over CDC interface
  *         through this function.
  *         @note
  *
  *
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */

uint8_t COMP_CDC_Transmit_FS (uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;

  switchToClass (&hUsbDeviceFS, &comp_dev[VCP]);

//  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
//  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);

  return result;
}



/****END OF FILE****/
