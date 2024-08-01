/**
  *******************************************************************************
  *
  * @file    cat_if.c
  * @brief   FT817 CAT interface commands driver
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cat_if.h"
#include "ptt_if.h"
#include "usbd_cdc_if.h"

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  uint8_t  is_enable;
  uint8_t  is_connected;
  uint8_t  buff [CAT_BUFF_SIZE];
  uint32_t buff_wr_time;
  uint16_t wr_ptr;
  uint16_t rd_ptr;
} CAT_TypeDef;


typedef enum
{
//  FT817_LOCK_ON       = 0x00,
//  FT817_LOCK_OFF      = 0x80,
    FT817_PTT_ON        = 0x08,
    FT817_PTT_OFF       = 0x88,
    FT817_SET_FREQ      = 0x01,
    FT817_MODE_SET      = 0x07,
//  FT817_RIT_ON        = 0x05,
//  FT817_RIT_OFF       = 0x85,
//  FT817_RIT_OFFSET    = 0xF5,
    FT817_TOGGLE_VFO    = 0x81,
    FT817_SPLIT_ON      = 0x02,
    FT817_SPLIT_OFF     = 0x82,
//  FT817_READ_RX_STATE = 0xE7,
    FT817_READ_TX_STATE = 0xF7,
    FT817_GET_FREQ      = 0x03,
//  FT817_PWR_ON        = 0x0F,
//  FT817_PWR_OFF       = 0x8F,
} FT817_COMMAND;

typedef enum
{
  FT817_LSB = 0x00,
  FT817_USB = 0x01,
  FT817_CW  = 0x02,  //CW-U
  FT817_CWR = 0x03,  //CW-L
  FT817_AM  = 0x04,
  FT817_FM  = 0x08,
  FT817_DIG = 0x0A,
  FT817_PKT = 0x0C
} FT817_MODE;

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

CAT_TypeDef cat;

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

/* External variables --------------------------------------------------------*/

extern TRX_TypeDef trx;

/* Private functions with user code ------------------------------------------*/

/**
  * @brief This function sends а reply to the CAT command
  *
  * CDC_Transmit_FS() is declared in usbd_cdc_if.c
  *
  */

static inline void cat_send_reply (uint8_t *Reply, uint8_t Len)
{
  CDC_Transmit_FS (Reply, Len);
}

/**
  * @brief This function returns TRX system time to CAT
  *
  * @retval TRX system time
  *
  */

static inline uint32_t cat_get_time (void)
{
  return (trx.sysclock);
}

/**
  * @brief This function returns TRX main frequency in BCD to CAT
  *
  * VFO_Get_Tune_BCD () is declared in ptt_if.c
  *
  * @retval TRX main frequency
  *
  */

static inline uint32_t cat_get_freq_bcd (void)
{
  return VFO_Get_Tune_BCD ();
}

/**
  * @brief This function returns TRX operating mode to CAT
  *
  * @retval TRX operating mode according to FT817 User Manual
  *
  */

static inline FT817_MODE cat_get_mode (void)
{
  return (FT817_MODE) (trx.mode);
}

/**
  * @brief This function sets TRX main frequency in HEX from CAT command
  *
  * VFO_Get_Tune () is declared in ptt_if.c
  *
  * @param Frequency from CAT command
  *
  */

static inline void cat_set_freq (uint32_t freq)
{
  VFO_Set_Tune (freq);
}

/**
  * @brief This function sets TRX operating mode from CAT command
  *
  * @param TRX operating mode
  *
  */

static inline uint8_t cat_set_mode (uint8_t mode)
{
  PTT_Set_Mode (mode);
  return 0x00;
}

/**
  * @brief  This function sets Split Mode from CAT command
  *
  * VFO_Set_Split () is declared in ptt_if.c
  *
  * @param  0 - sets Split Mode OFF
  * @param  1 - sets Split Mode ON
  *
  */

static inline uint8_t cat_split (uint8_t split)
{
  VFO_Set_Split (split);
  return 0x00;
}

/**
  * @brief  This function toggle VFO from CAT command
  *
  * VFO_Toggle_VFO () is declared in ptt_if.c
  *
  */

static inline uint8_t cat_toggle_vfo (void)
{
  VFO_Toggle_VFO ();
  return 0x00;
}

/**
  * @brief  This function sets PTT status from CAT command
  *
  * PTT_CAT_TX () is declared in ptt_if.c
  *
  * @param  0 - sets PTT OFF
  * @param  1 - sets PTT ON
  *
  * @retval 0x00 - OK
  * @retval 0xF0 - PTT's already On/Off
  *
  */

uint8_t cat_ptt (uint8_t ptt)
{
  uint8_t retval = 0x00;

  if (ptt == trx.is_tx)
  {
    retval = 0xF0;
  }
  PTT_CAT_TX (ptt);

  return retval;
}

/**
  * @brief This function returns TX state to CAT
  *
  * @retval TX state according to FT817 User Manual
  *
  */

uint8_t cat_tx_state (void)
{
  uint8_t retval = 0xFF;

  if (trx.is_tx)
  {
    retval &= 0x7F;
  }

  if (trx.split)
  {
    retval &= 0xDF;
  }

  return retval;
}

/*----------------------------------------------------------------------------*/


/* Private functions ---------------------------------------------------------*/

/**
  * @brief  This function resets CAT buffer pointers
  *
  */

void cat_buff_init (void)
{
  cat.rd_ptr = 0U;
  cat.wr_ptr = 0U;
  cat.buff_wr_time = cat_get_time ();
}

/**
  * @brief  This function reads **Len** bytes from CAT buffer
  *
  * The function increments the CAT buffer read pointer by **Len** value
  *
  * @param  *Dest - pointer to destination buffer
  * @param  Len   - number of bytes to read
  *
  * @retval 0 - CAT buffer is empty
  * @retval 1 - OK
  */

uint8_t cat_buff_read_data (uint8_t *Dest, uint8_t Len)
{
  uint8_t i;
  uint8_t room = 0U;
  uint8_t retval = 0U;

  if (cat.wr_ptr >= cat.rd_ptr)
  {
    room = cat.wr_ptr - cat.rd_ptr;
  }
  else
  {
    room = CAT_BUFF_SIZE - cat.rd_ptr;
    room = room + cat.wr_ptr;
  }

  if (Len <= room)
  {
    for (i = 0; i < Len; i++)
    {
      Dest[i] = cat.buff[cat.rd_ptr];

      if ((cat.rd_ptr + 1U) == CAT_BUFF_SIZE)
      {
        cat.rd_ptr = 0U;
      }
      else
      {
        cat.rd_ptr++;
      }
    }

    retval = 1U;
  }
  return retval;
}

/**
  * @brief This function decodes and handles FT817 CAT commands
  *
  * The function reads 5 bytes from CAT buffer to **req[ ]** array and decodes CAT command
  * from **req[4]** byte.<br>
  * If CAT command is decoded successfully, the function sends reply.
  *
  * **List of FT817 CAT commands according to User Manual:**
  * | Command          | req[0] | req[1] | req[2] | req[3] | req[4] | Macros              |
  * |:-----------------|:------:|:------:|:------:|:------:|:------:|---------------------|
  * | LOCK ON          |   --   |   --   |   --   |   --   |  0x00  |                     |
  * | LOCK OFF         |   --   |   --   |   --   |   --   |  0x80  |                     |
  * | PTT ON           |   --   |   --   |   --   |   --   |  0x08  | FT817_PTT_ON        |
  * | PTT OFF          |   --   |   --   |   --   |   --   |  0x88  | FT817_PTT_OFF       |
  * | Set Frequency    |   P1   |   P2   |   P3   |   P4   |  0x01  | FT817_SET_FREQ      |
  * | Operating Mode   |   P1   |   --   |   --   |   --   |  0x07  | FT817_MODE_SET      |
  * | CLAR ON          |   --   |   --   |   --   |   --   |  0x05  |                     |
  * | CLAR OFF         |   --   |   --   |   --   |   --   |  0x85  |                     |
  * | CLAR Frequency   |   P1   |   --   |   P3   |   P4   |  0xF5  |                     |
  * | VFO-A/B          |   --   |   --   |   --   |   --   |  0x81  | FT817_TOGGLE_VFO    |
  * | SPLIT ON         |   --   |   --   |   --   |   --   |  0x02  | FT817_SPLIT_ON      |
  * | SPLIT OFF        |   --   |   --   |   --   |   --   |  0x82  | FT817_SPLIT_OFF     |
  * | Repeater Offset  |   P1   |   --   |   --   |   --   |  0x09  |                     |
  * | Repeater Offset  |   P1   |   P2   |   P3   |   P4   |  0xF9  |                     |
  * | CTCSS/DCS Mode   |   P1   |   --   |   --   |   --   |  0x0A  |                     |
  * | CTCSS Tone       |   P1   |   P2   |   --   |   --   |  0x0A  |                     |
  * | DCS Code         |   P1   |   P2   |   --   |   --   |  0x0A  |                     |
  * | Read RX Status   |   --   |   --   |   --   |   --   |  0xE7  |                     |
  * | Read TX Status   |   --   |   --   |   --   |   --   |  0xF7  | FT817_READ_TX_STATE |
  * | Read Freq & Mode |   --   |   --   |   --   |   --   |  0x03  | FT817_GET_FREQ      |
  * | POWER ON         |   --   |   --   |   --   |   --   |  0x0F  |                     |
  * | POWER OFF        |   --   |   --   |   --   |   --   |  0x8F  |                     |
  *
  *
  * <p> </p>
  * **List of FT817 Operating Modes according to User Manual:**
  *
  * <p>0x00 = LSB; 0x01 = USB; 0x02 = CW;  0x03 = CW-R;</p>
  * <p>0x04 = AM;  0x08 = FM;  0x0A = DIG; 0x0C = PKT</p>
  *
  */

void cat_ft817_handle_command (void)
{
  uint8_t  req [5];
  uint8_t  reply [6];
  uint16_t len = 0U;

  uint8_t  i;

  uint32_t fhex = 0U;  /* Frequency HEX value */
  uint32_t fbcd = 0U;  /* Frequency BCD value */


  while (cat_buff_read_data (req, 5U))
  {
    switch ((FT817_COMMAND) req [4])
    {
      case FT817_SET_FREQ:
        /* Convert CAT format to HEX */
        for (i = 0; i < 4; i++)
        {
          fhex *= 100U;
          fhex += (req [i] >> 4U) * 10U + (req [i] & 0x0F);
        }
        fhex *= 10U;

        cat_set_freq (fhex);  /* save frequency in HEX format to TRX */

        reply [0] = 0U;
        len = 1U;
        break;

      case FT817_GET_FREQ:
        /* Get frequency in CAT format from TRX */
        fbcd = cat_get_freq_bcd ();

        for (i = 0; i < 3; i++)
        {
          reply [i] = fbcd & 0xFF;
          fbcd = fbcd >> 8U;
        }
        reply [3] = fbcd & 0xFF;

        reply [4] = cat_get_mode ();
        len = 5U;
        break;

      case FT817_MODE_SET:
        reply [0] = cat_set_mode (req [0]);
        len = 1U;
        break;

      case FT817_TOGGLE_VFO:
        reply [0] = cat_toggle_vfo ();
        len = 1U;
        break;

      case FT817_SPLIT_ON:
        reply [0] = cat_split (1U);
        len = 1U;
        break;

      case FT817_SPLIT_OFF:
        reply [0] = cat_split (0U);
        len = 1U;
        break;

      case FT817_PTT_ON:
        reply [0] = cat_ptt (1U);
        len = 1U;
        break;

      case FT817_PTT_OFF:
        reply [0] = cat_ptt (0U);
        len = 1U;
        break;

      case FT817_READ_TX_STATE:
        reply [0] = cat_tx_state ();
        len = 1U;
        break;
    }
    cat_send_reply (reply, len);
  }
}

/* Public functions ----------------------------------------------------------*/

/**
  * @brief This function writes byte to CAT buffer
  *
  * The function increments buffer write pointer by 1
  *
  * @param Byte to write to CAT buffer
  *
  */

void CAT_Buff_Write_Byte (uint8_t Byte)
{
  uint16_t wr_ptr;

  if ((cat.wr_ptr + 1U) == CAT_BUFF_SIZE)
  {
    wr_ptr = 0U;
  }
  else
  {
    wr_ptr = cat.wr_ptr + 1U;
  }

  if (wr_ptr != cat.rd_ptr) /* there is enough room */
  {
    cat.buff [cat.wr_ptr] = Byte;
    cat.wr_ptr = wr_ptr;
    cat.buff_wr_time = cat_get_time ();
  }
}

/**
  * @brief  This function handles CAT interface state and starts CAT command handler if needs
  *
  */

void CAT_Handler (uint8_t connected)
{
  if (connected)
  {
    if (cat.is_connected)
    {
      if ((cat_get_time () - cat.buff_wr_time) < CAT_TIMEOUT)
      {
        cat_ft817_handle_command ();
      }
      else
      {
        cat_buff_init ();
      }
    }
    else
    {
      cat_buff_init ();
      cat.is_connected = 1U;
    }
  }
  else
  {
    cat.is_connected = 0U;
  }
}


/***END OF FILE***/
