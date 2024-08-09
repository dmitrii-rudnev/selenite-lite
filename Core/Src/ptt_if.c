/**
  *******************************************************************************
  *
  * @file    ptt_if.c
  * @brief   PTT driver
  * @version v1.0
  * @date    05.04.2020
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
#include "main.h"
#include "ptt_if.h"
#include "si5351a.h"
#include "cat_if.h"
#include "dsp_if.h"
#include "usb_device.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

PTT_TypeDef ptt;

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

/* External variables --------------------------------------------------------*/

extern USBD_HandleTypeDef hUsbDeviceFS;
extern TRX_TypeDef trx;

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
  int8_t   i;
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
  * @brief This function sets BPF band and mode
  *
  * The function sets BPF band according to VFO frequency and sets RX/TX mode
  *
  */

void ptt_set_bpf (uint32_t tune_new)
{
  uint8_t retval = 0x06;

  if (tune_new >  3000000U) { retval = 0x07; }
  if (tune_new >  4000000U) { retval = 0x04; }
  if (tune_new >  6000000U) { retval = 0x05; }
  if (tune_new >  8000000U) { retval = 0x02; }
  if (tune_new > 12000000U) { retval = 0x03; }
  if (tune_new > 16000000U) { retval = 0x00; }
  if (tune_new > 24000000U) { retval = 0x01; }


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
      }
      break;
    case 1:
      if (trx.vfob != tune_new)
      {
        trx.vfob = tune_new;
        trx.vfob_bcd = HEX_to_BCD (trx.vfob / 10U);
        trx.vfob_bcd = BCD_to_CAT (trx.vfob_bcd);
      }
      break;
  }

  vfo_set_freq (tune_new);
  ptt_set_bpf  (tune_new);
}

/**
  * @brief  This function sets trx.vfoX_bcd in (BCD to CAT) format
  *
  * @param  Main frequency in Hz
  */

void VFO_Set_Tune_BCD (uint32_t tune_new)
{
  switch (trx.vfo)
  {
    case 0:
      if (trx.vfoa_bcd != tune_new)
      {
        trx.vfoa_bcd = tune_new;
      }
      break;
    case 1:
      if (trx.vfob_bcd != tune_new)
      {
        trx.vfob_bcd = tune_new;
     }
      break;
  }
}

/**
  * @brief  This function returns TRX main frequency according to toggled VFO
  *
  * @retval Main frequency in Hz
  */

uint32_t VFO_Get_Tune (void)
{
  uint32_t retval;

  if (trx.vfo)
  {
    retval = trx.vfob;
  }
  else
  {
    retval = trx.vfoa;
  }

  return retval;
}

/**
  * @brief  This function returns TRX main frequency in BCD according to toggled VFO
  *
  * @retval Main frequency in Hz
  */

uint32_t VFO_Get_Tune_BCD (void)
{
  uint32_t retval;

  if (trx.vfo)
  {
    retval = trx.vfob_bcd;
  }
  else
  {
    retval = trx.vfoa_bcd;
  }

  return retval;
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
    VFO_Set_Tune (trx.vfoa);
  }
  else
  {
    trx.vfo = 1U;
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
}

/**
 * @brief This function sets TRX operating mode from CAT command
 *
 * @param TRX operating mode
 */

void PTT_Set_Mode (uint8_t trx_mode)
{
  if (trx.is_tx) { return; }

  switch (trx_mode)
  {
    case 0x00:
      trx.mode = MODE_LSB;
      break;
    case 0x01:
      trx.mode = MODE_USB;
      break;
    case 0x02:
      trx.mode = MODE_CW;   /* CW-USB */
      break;
    case 0x03:
      trx.mode = MODE_CWR;  /* CW-LSB */
      break;
    case 0x04:
      trx.mode = MODE_AM;
      break;
    case 0x08:
      trx.mode = MODE_FM;
      break;
    case 0x0A:
      trx.mode = MODE_DIG;  /* DIG-U */
      break;
    case 0x0C:
      trx.mode = MODE_PKT;  //DIG-L
      break;
    default:
      trx.mode = MODE_USB;
      break;
  }

  DSP_Set_Mode (trx_mode);
}


/**
  * @brief This function sets TX mode from CAT command
  *
  */

void PTT_CAT_TX (uint8_t cat)
{
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
  }
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
  * @brief This function calls VFO_Init () to sets HSE = 24.576 MHz
  *
  */

void PTT_Set_HSE (void)
{
  Si5351a_Init ();
}

/**
  * @brief This function initialize PTT, VFO and DSP
  *
  */

void PTT_Init (void)
{
  trx.split = 0U;
  trx.vfo   = 0U;
  trx.is_tx = 0U;
  trx.mode  = MODE_LSB;

  trx.vfoa = 7050000U;
  trx.vfob = 7010000U;

  trx.vfoa_bcd = HEX_to_BCD (trx.vfoa / 10U);
  trx.vfob_bcd = HEX_to_BCD (trx.vfob / 10U);

  trx.vfoa_bcd = BCD_to_CAT (trx.vfoa_bcd);
  trx.vfob_bcd = BCD_to_CAT (trx.vfob_bcd);

  trx.displayed = trx.sysclock;

  DSP_Init ();

  //++++++
  /* Start here UI Init */
  //++++++

  VFO_Set_Tune (trx.vfoa);
}

/**
  * @brief This function sets telegraph key off time
  *
  */

void PTT_Key_Off_Time (void)
{
  ptt.key_off_time = trx.sysclock;
}

/**
  * @brief This function processes the timeout in switching to RX mode
  * after releasing the telegraph key
  *
  */

void PTT_Handler (void)
{
  if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED)
  {
    CAT_Handler (1U);
  }
  else
  {
    CAT_Handler (0U);
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
