/**
  *******************************************************************************
  *
  * @file    ptt_if.c
  * @brief   PTT driver
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "rxtx_if.h"
#include "si5351a.h"
#include "dsp_if.h"
#include "usbd_cdc_if.h"
#include <string.h>

/* Private typedef -----------------------------------------------------------*/

#define CAT_BUFF_SIZE 16U

typedef struct
{
  uint8_t  buff [CAT_BUFF_SIZE];
  uint16_t wr_ptr;
  uint16_t rd_ptr;
  uint8_t  get_freq [5U];
  uint8_t  read_tx_state;
} CAT_TypeDef;

typedef enum
{
  FT817_SET_FREQ      = 0x01,
  FT817_TOGGLE_VFO    = 0x81,
  FT817_SPLIT_ON      = 0x02,
  FT817_SPLIT_OFF     = 0x82,
  FT817_GET_FREQ      = 0x03,
  FT817_MODE_SET      = 0x07,
  FT817_PTT_ON        = 0x08,
  FT817_PTT_OFF       = 0x88,
  FT817_READ_TX_STATE = 0xF7,
} FT817_COMMAND;

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

TRX_TypeDef trx;
PTT_TypeDef ptt;
CAT_TypeDef cat;

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

/* External variables --------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
  * @brief This function sets si5351a frequency
  *
  */

void vfo_set_freq (uint32_t freq)
{
  Si5351a_Set_Freq (freq, 0, 90);
}

/**
  * @brief This function converts HEX to BCD format
  *
  */

uint32_t HEX_to_BCD (uint32_t input)
{
  int8_t   i;
  uint32_t output = 0U;

  if (input < 100000000U)
  {
    for (i = 26; i >= 0; i--)
    {
      if ((output  & 0xF) >= 5)               { output += 3; }
      if (((output & 0xF0) >> 4) >= 5)        { output += (3 << 4); }
      if (((output & 0xF00) >> 8) >= 5)       { output += (3 << 8); }
      if (((output & 0xF000) >> 12) >= 5)     { output += (3 << 12); }
      if (((output & 0xF0000) >> 16) >= 5)    { output += (3 << 16); }
      if (((output & 0xF00000) >> 20) >= 5)   { output += (3 << 20); }
      if (((output & 0xF000000) >> 24) >= 5)  { output += (3 << 24); }

      output = (output << 1) | ((input >> i) & 1);
    }
  }
  return output;
}

/**
  * @brief This function changes order of bytes from 0-1-2-3 to 3-2-1-0
  *
  */

uint32_t BCD_to_CAT (uint32_t input)
{
  uint8_t  i;
  uint32_t output = 0U;

  for (i = 0; i < 3; i++)
  {
    output |= (input & 0xFF);
    output = output << 8;
    input = input >> 8;
  }
  output |= (input & 0xFF);

  return output;
}

/**
  * @brief This function converts frequency from CAT to HEX format
  *
  */

uint32_t CAT_to_HEX (void)
{
  uint8_t  i;
  uint32_t output = 0U;

  for (i = 0; i < 4; i++)
  {
    output *= 100U;
    output += (cat.get_freq [i] >> 4U) * 10U + (cat.get_freq [i] & 0x0F);
  }
  output *= 10U;

  return output;
}

/**
  * @brief This function sets BPF band and mode
  *
  * The function sets BPF band according to VFO frequency and sets RX/TX mode
  *
  */

void ptt_set_bpf (uint32_t tune_new)
{
  uint8_t retval = 0x06;

  if (tune_new >  4000000U) { retval = 0x04; }
  if (tune_new >  8000000U) { retval = 0x02; }
  if (tune_new > 16000000U) { retval = 0x00; }
 
  if (retval & 0x02)
  {
    HAL_GPIO_WritePin (S1_GPIO_Port, S1_Pin, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin (S1_GPIO_Port, S1_Pin, GPIO_PIN_RESET);
  }

  if (retval & 0x04)
  {
    HAL_GPIO_WritePin (S2_GPIO_Port, S2_Pin, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin (S2_GPIO_Port, S2_Pin, GPIO_PIN_RESET);
  }

  if (trx.is_tx)
  {
    HAL_GPIO_WritePin (TX_GPIO_Port, TX_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin (QSE_EN_GPIO_Port, QSE_EN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin (QSD_EN_GPIO_Port, QSD_EN_Pin, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin (TX_GPIO_Port, TX_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin (QSE_EN_GPIO_Port, QSE_EN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin (QSD_EN_GPIO_Port, QSD_EN_Pin, GPIO_PIN_RESET);
  }
}

/**
  * @brief This function sets TX mode
  *
  * The function also sets TX mode for VFO and DSP
  *
  */

void ptt_set_tx (void)
{
  ptt.key_off_time = 0U;

  if (!trx.is_tx)
  {
    if (trx.split)
    {
      VFO_Toggle_VFO ();
    }

    trx.is_tx = 1U;
    cat.read_tx_state &= 0x7F;

    DSP_Set_TX ();

    if (trx.vfo)
    {
      ptt_set_bpf (trx.vfob);
    }
    else
    {
      ptt_set_bpf (trx.vfoa);
    }
  }
}

/**
  * @brief This function sets RX mode
  *
  * The function sets RX mode if the PTT button is not pressed,
  * the telegraph key is not used and CAT status is not TX.
  * The function also sets RX mode for VFO and DSP
  *
  */

void ptt_set_rx (void)
{
  if (ptt.cat_is_on || ptt.key_dah_is_on || ptt.key_dit_is_on ||
      ptt.dtr_is_on || ptt.rts_is_on) return;

  if (trx.is_tx)
  {
    trx.is_tx = 0U;
    cat.read_tx_state |= 0x80;

    if (trx.split)
    {
      VFO_Toggle_VFO ();
    }

    DSP_Set_RX ();

    if (trx.vfo)
    {
      ptt_set_bpf (trx.vfob);
    }
    else
    {
      ptt_set_bpf (trx.vfoa);
    }
  }
}

/**
  * @brief This function sets telegraph key off time
  *
  */

void PTT_Key_RX (void)
{
  ptt.key_off_time = trx.sysclock;
}

/**
  * @brief This function sets TX mode from DTR line
  *
  * DTR line works like external telegraph key
  *
  */

void PTT_DTR_TX (uint8_t dtr)
{
  if (ptt.dtr_is_on != dtr)
  {
    ptt.dtr_is_on = dtr;

    if (dtr)
    {
      ptt_set_tx ();
    }
    else
    {
      ptt.key_off_time = trx.sysclock;
    }
  }
}

/**
  * @brief This function sets TX mode from RTS line
  *
  * RTS line works like external PTT
  *
  */

void PTT_RTS_TX (uint8_t rts)
{
  if (ptt.rts_is_on != rts)
  {
    ptt.rts_is_on = rts;

    if (rts)
    {
      ptt_set_tx ();
    }
    else
    {
      ptt_set_rx ();
    }
  }
}

/**
  * @brief This function sets TX mode from CAT command
  *
  * @param  0 - sets PTT OFF
  * @param  1 - sets PTT ON
  *
  * @retval 0x00 - OK
  * @retval 0xF0 - PTT's already On/Off
  *
  */

uint8_t ptt_cat_tx (uint8_t cat)
{
  uint8_t retval = 0xF0;

  if (ptt.cat_is_on != cat)
  {
    ptt.cat_is_on = cat;

    if (cat)
    {
      ptt_set_tx ();
    }
    else
    {
      ptt_set_rx ();
    }
    retval = 0x00;
  }
  return retval;
}

/**
  * @brief  This function sets TRX main frequency according to toggled VFO and sets VFO and BPF
  *
  * @param  Main frequency in Hz
  */

void VFO_Set_Tune (uint32_t tune_new)
{
  switch (trx.vfo)
  {
    case 0:
      if (trx.vfoa != tune_new)
      {
        trx.vfoa = tune_new;
        trx.vfoa_bcd = HEX_to_BCD (trx.vfoa / 10U);
        trx.vfoa_bcd = BCD_to_CAT (trx.vfoa_bcd);
        *(uint32_t*)cat.get_freq = trx.vfoa_bcd;
      }
      break;
    case 1:
      if (trx.vfob != tune_new)
      {
        trx.vfob = tune_new;
        trx.vfob_bcd = HEX_to_BCD (trx.vfob / 10U);
        trx.vfob_bcd = BCD_to_CAT (trx.vfob_bcd);
        *(uint32_t*)cat.get_freq = trx.vfob_bcd;
      }
      break;
  }

  vfo_set_freq (tune_new);
  ptt_set_bpf  (tune_new);
}

/**
  * @brief  This function switches VFO from one to another
  *
  */

void VFO_Toggle_VFO (void)
{
  if (trx.is_tx) { return; }

  if (trx.vfo)
  {
    trx.vfo = 0U;
    *(uint32_t*)cat.get_freq = trx.vfoa_bcd;
    VFO_Set_Tune (trx.vfoa);
  }
  else
  {
    trx.vfo = 1U;
    *(uint32_t*)cat.get_freq = trx.vfob_bcd;
    VFO_Set_Tune (trx.vfob);
  }
}

/**
  * @brief This function sets VFO Split Mode On/Off
  *
  */

void VFO_Set_Split (uint8_t split)
{
  if (trx.is_tx) { return; }
  
  trx.split = split;

  if (trx.split)
  {
    cat.read_tx_state &= 0xDF;
  }
  else
  {
    cat.read_tx_state |= 0x20;
  }
}

/**
  * @brief  This function sets frequency from CAT
  *
  */

void vfo_set_tune_cat (void)
{
  uint32_t tune_new = CAT_to_HEX ();

  if (trx.vfo)
  {
    trx.vfob_bcd = *(uint32_t*)cat.get_freq;
    trx.vfob = tune_new;
  }
  else
  {
    trx.vfoa_bcd = *(uint32_t*)cat.get_freq;
    trx.vfoa = tune_new;
  }

  vfo_set_freq (tune_new);
  ptt_set_bpf  (tune_new);
}

/**
 * @brief This function decodes and handles FT817 CAT commands
 *
 * The function reads 5th byte from CAT buffer to and decodes CAT command *
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

void cat_cmd_handler (void)
{
  uint8_t cmd, reply;

  do
  {
    cmd   = cat.buff [cat.rd_ptr + 4];

    if (cmd == FT817_GET_FREQ)
    {
      CDC_Transmit_FS (cat.get_freq, 5U);
    }
    else
    {
      reply = 0U;

      switch (cmd)
      {
        case FT817_READ_TX_STATE:
          reply = cat.read_tx_state;
          break;

        case FT817_SET_FREQ:
          *(uint32_t*)cat.get_freq = *(uint32_t*)&cat.buff [cat.rd_ptr];
          vfo_set_tune_cat ();
          break;

        case FT817_MODE_SET:
          cat.get_freq [4] = cat.buff [cat.rd_ptr];
          PTT_Set_Mode (cat.buff [cat.rd_ptr]);
          break;

        case FT817_TOGGLE_VFO:
          VFO_Toggle_VFO ();
          break;

        case FT817_SPLIT_ON:
          VFO_Set_Split (1U);
          break;

        case FT817_SPLIT_OFF:
          VFO_Set_Split (0U);
          break;

        case FT817_PTT_ON:
          reply = ptt_cat_tx (1U);
          break;

        case FT817_PTT_OFF:
          reply = ptt_cat_tx (0U);
          break;

        default:
          break;
      }
      CDC_Transmit_FS (&reply, 1U);
    }

    cat.rd_ptr += 5U;

    if (cat.rd_ptr >= cat.wr_ptr)
    {
      cat.rd_ptr = 0;
      cat.wr_ptr = 0;
    }
  }
  while (cat.wr_ptr);
}

/**
 * @brief This function writes to CAT buffer
 *
 */

void CAT_Buff_Write (uint8_t* pbuf, uint32_t len)
{
  if ((cat.wr_ptr + len) <= CAT_BUFF_SIZE)
  {
    memcpy (&cat.buff[cat.wr_ptr], pbuf, len);
    cat.wr_ptr += len;
  }
}
/**
 * @brief This function sets TRX operating mode from CAT command
 *
 * @param TRX operating mode
 */

void PTT_Set_Mode (uint8_t trx_mode)
{
  if (trx.is_tx) { return; }

  trx.mode = trx_mode; 
  cat.get_freq[4] = trx.mode;
  
  DSP_Set_Mode (trx_mode);
}

/**
  * @brief This function calls VFO_Init () to sets HSE = 24.576 MHz
  *
  */

void VFO_Init (void)
{
  Si5351a_Init ();
}

/**
  * @brief This function initialize PTT, VFO and DSP
  *
  */

void PTT_Init (void)
{
  /* PTT and CAT init */
  trx.vfo   = 0U;
  trx.split = 0U;
  trx.is_tx = 0U;
  cat.read_tx_state = 0xFF;

  /* TRX mode init */
  trx.mode  = MODE_LSB;    
  cat.get_freq [4] = trx.mode;

  /* VFO frequency setting */
  trx.vfoa = 7050000U;
  trx.vfoa_bcd = HEX_to_BCD (trx.vfoa / 10U);
  trx.vfoa_bcd = BCD_to_CAT (trx.vfoa_bcd);

  trx.vfob = 7010000U;
  trx.vfob_bcd = HEX_to_BCD (trx.vfob / 10U);
  trx.vfob_bcd = BCD_to_CAT (trx.vfob_bcd);

  *(uint32_t*)cat.get_freq = trx.vfoa_bcd;
  VFO_Set_Tune (trx.vfoa);

  /* CAT buffer init */
  cat.rd_ptr = 0U;
  cat.wr_ptr = 0U;
  
  /* TRX timeouts init */
  trx.systicks  = 10U;
  trx.sysclock  = 0U;
  trx.displayed = trx.sysclock;

  DSP_Init ();
  //++++++
  /* Start here UI Init */
  //++++++
}

/**
  * @brief This function processes the timeout in switching to RX mode
  * after releasing the telegraph key
  *
  */

void PTT_Handler (void)
{
  if (cat.wr_ptr)
  {
    cat_cmd_handler ();
  }

  if (ptt.key_off_time != 0U)
  {
    if ((trx.sysclock - ptt.key_off_time) > KEY_TIMEOUT)
    {
      ptt.key_off_time = 0U;
      ptt_set_rx ();
    }
  }

  if ((trx.sysclock - trx.displayed) > 19)
  {
    trx.displayed = trx.sysclock;
    //++++++
    /* Start here UI handler */
    //++++++
  }

}

/**
  * @brief This function is GPIO_EXTI handler
  *
  * @param KEY_DAH_Pin - Telegraph key DAH paddle
  * @param KEY_DIT_Pin - Telegraph key DIT paddle
  *
  */

void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin)
{
  switch (GPIO_Pin)
  {
    case KEY_DAH_Pin:
      ptt.key_dah_is_on = !HAL_GPIO_ReadPin (KEY_DAH_GPIO_Port, KEY_DAH_Pin);

      if (ptt.key_dah_is_on)
      {
        ptt_set_tx ();
      }
      else
      {
        ptt.key_off_time = trx.sysclock;
      }

      break;
    case KEY_DIT_Pin:
      ptt.key_dit_is_on = !HAL_GPIO_ReadPin (KEY_DIT_GPIO_Port, KEY_DIT_Pin);

      if (ptt.key_dit_is_on)
      {
        ptt_set_tx ();
      }
      else
      {
        ptt.key_off_time = trx.sysclock;
      }

      break;
  }
}

/****END OF FILE****/
