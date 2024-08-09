/**
  *******************************************************************************
  *
  * @file    dsp_if.c
  * @brief   Digital Signal Processor Interface
  * @version v2.2
  * @date    03.11.2022
  * @author  Dmitrii Rudnev
  *
  *******************************************************************************
  * Copyright &copy; 2022 Selenite Project. All rights reserved.
  *
  * This software component is licensed under [BSD 3-Clause license]
  * (http://opensource.org/licenses/BSD-3-Clause/), the "License".<br>
  * You may not use this file except in compliance with the License.
  *******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include "dsp_if.h"
#include "codec_if.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

I2S_Buff_TypeDef  i2s_buff;

DSP_Buff_TypeDef  dsp_out_buff;
DSP_Buff_TypeDef  dsp_in_buff;

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

/* External variables --------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
 * @brief This function is a handler of end of half transfer event of I2S DMA buffer
 *
 */

void HAL_I2SEx_TxRxHalfCpltCallback (I2S_HandleTypeDef *hi2s)
{
  DSP_Out_Buff_Read (i2s_buff.tx, I2S_BUFF_HALF_SIZE);
  DSP_In_Buff_Write (i2s_buff.rx, I2S_BUFF_HALF_SIZE);
}

/**
 * @brief This function is a handler of end of transfer event of I2S DMA buffer
 *
 * @attention This function works properly with STM32Cube_FW_F4_V1.26.2 or newer
 *
 */

void HAL_I2SEx_TxRxCpltCallback  (I2S_HandleTypeDef *hi2s)
{
  DSP_Out_Buff_Read (&i2s_buff.tx [I2S_BUFF_HALF_SIZE], I2S_BUFF_HALF_SIZE);
  DSP_In_Buff_Write (&i2s_buff.rx [I2S_BUFF_HALF_SIZE], I2S_BUFF_HALF_SIZE);
}

/**
 * @brief This function initialize I2S buffer and starts HAL_I2SEx_TransmitReceive_DMA ()
 *
 */

void i2s_buff_init (void)
{
  for (uint32_t i = 0U; i < I2S_BUFF_SIZE; i++)
  {
    i2s_buff.rx [i] = 0U;
    i2s_buff.tx [i] = 0U;
  }

  HAL_I2SEx_TransmitReceive_DMA (&I2S_IF, i2s_buff.tx, i2s_buff.rx, I2S_BUFF_SIZE);
}


/**
 * @brief This function writes a sample to DSP Out buffer
 *
 * @param i, q are samples to write
 *
 */

void dsp_out_buff_write (uint16_t i, uint16_t q)
{
  dsp_out_buff.i [dsp_out_buff.wr_ptr] = i;
  dsp_out_buff.q [dsp_out_buff.wr_ptr] = q;

  dsp_out_buff.wr_ptr++;

  if (dsp_out_buff.wr_ptr == DSP_BUFF_SIZE)
  {
    dsp_out_buff.wr_ptr = 0U;
  }
}

/**
 * @brief This function writes to DSP Out buffer
 *
 * This function writes to DSP Out buffer and prevent read/write areas overlay
 *
 * @param Source buffer pointer
 * @param Number of bytes to write
 *
 */

void DSP_Out_Buff_Write (uint8_t *pbuf, uint32_t size)
{
  uint16_t *buff = (uint16_t*) pbuf;

  size = size / 2U;

  uint16_t gap;

  if (dsp_out_buff.buff_enable == 0U)
  {
    dsp_out_buff.wr_ptr = dsp_out_buff.rd_ptr + DSP_BUFF_HALF_SIZE;

    if (dsp_out_buff.wr_ptr >= DSP_BUFF_SIZE)
    {
      dsp_out_buff.wr_ptr -= DSP_BUFF_SIZE;
    }

    dsp_out_buff.buff_enable = 1U;
  }

  gap = dsp_out_buff.wr_ptr;

  if (dsp_out_buff.rd_ptr > dsp_out_buff.wr_ptr)
  {
    gap += DSP_BUFF_SIZE;
  }

  gap -= dsp_out_buff.rd_ptr;

  if (gap > (3U * DSP_BUFF_SIZE / 4U))  /* wr is faster */
  {
    if (dsp_out_buff.wr_ptr < 1U)
    {
      dsp_out_buff.wr_ptr += DSP_BUFF_SIZE;
    }

    dsp_out_buff.wr_ptr--;              /* shift wr_ptr backward */
  }

  if (gap < (DSP_BUFF_SIZE / 4U))       /* rd is faster */
  {
    dsp_out_buff.wr_ptr++;              /* shift wr_ptr forward */

    if (dsp_out_buff.wr_ptr >= DSP_BUFF_SIZE)
    {
      dsp_out_buff.wr_ptr -= DSP_BUFF_SIZE;
    }
  }

  for (uint32_t i = 0; i < size; i += 2U)
  {
    dsp_out_buff_write (buff [i + 0], buff [i + 1]);
  }

  /* repeat last sample for synchronization when wr_ptr is shifted forward */

  dsp_out_buff_write (buff [size - 2], buff [size - 1]);

  if (dsp_out_buff.wr_ptr < 1U)
  {
    dsp_out_buff.wr_ptr += DSP_BUFF_SIZE;
  }

  dsp_out_buff.wr_ptr--;
}

/**
 * @brief This function flush DSP Out buffer
 *
 * This function writes to DSP Out buffer zeros
 */

void DSP_Out_Buff_Mute (void)
{
  for (uint32_t i = 0U; i < DSP_BUFF_SIZE; i++)
  {
    dsp_out_buff.i [i] = 0U;
    dsp_out_buff.q [i] = 0U;
  }
}

/**
 * @brief This function writes to Codec TX buffer
 *
 *@param Codec TX buffer pointer
 *@param Number of samples to write
 */

void DSP_Out_Buff_Read (uint16_t *pbuf, uint16_t size)
{
  for (uint16_t k = 0U; k < size; k += 2U)
  {
    pbuf [k + 0] = dsp_out_buff.i [dsp_out_buff.rd_ptr];
    pbuf [k + 1] = dsp_out_buff.q [dsp_out_buff.rd_ptr];

    dsp_out_buff.rd_ptr++;

    if (dsp_out_buff.rd_ptr >= DSP_BUFF_SIZE)
    {
      dsp_out_buff.rd_ptr = 0U;
    }
  }
  /* mix CW tone to speaker signal here*/
}

/**
 * @brief This function writes a sample to DSP In buffer
 *
 * @param i, q are samples to write
 */

void dsp_in_buff_write (uint16_t i, uint16_t q)
{
  dsp_in_buff.i [dsp_in_buff.wr_ptr] = i;
  dsp_in_buff.q [dsp_in_buff.wr_ptr] = q;

  dsp_in_buff.wr_ptr++;

  if (dsp_in_buff.wr_ptr == DSP_BUFF_SIZE)
  {
    dsp_in_buff.wr_ptr = 0U;
  }
}

/**
 * @brief This function writes to DSP In buffer
 *
 * This function writes to DSP In buffer and prevent read/write areas overlay
 *
 * @param Source buffer pointer
 * @param Number of samples to write
 *
 */

void DSP_In_Buff_Write (uint16_t *pbuf, uint16_t size)
{
  uint16_t gap = 0U;

  if (dsp_in_buff.buff_enable)
  {
    gap = dsp_in_buff.wr_ptr;

    if (dsp_in_buff.rd_ptr > dsp_in_buff.wr_ptr)
    {
      gap += DSP_BUFF_SIZE;
    }

    gap -= dsp_in_buff.rd_ptr;
  }

  if (gap > (3U * DSP_BUFF_SIZE / 4U))  /* wr is faster */
  {
    if (dsp_in_buff.wr_ptr < 1U)
    {
      dsp_in_buff.wr_ptr += DSP_BUFF_SIZE;
    }

    dsp_in_buff.wr_ptr--;               /* shift wr_ptr backward */
  }

  if (gap < (DSP_BUFF_SIZE / 4U))       /* rd is faster */
  {
    dsp_in_buff.wr_ptr++;               /* shift wr_ptr forward */

    if (dsp_in_buff.wr_ptr >= DSP_BUFF_SIZE)
    {
      dsp_in_buff.wr_ptr -= DSP_BUFF_SIZE;
    }
  }

  for (uint32_t i = 0; i < size; i += 2U)
  {
    dsp_in_buff_write (pbuf [i + 0], pbuf [i + 1]);
  }

  /* repeat last sample for synchronization when wr_ptr is shifted forward */

  dsp_in_buff_write (pbuf [size - 2], pbuf [size - 1]);

  if (dsp_in_buff.wr_ptr < 1U)
  {
    dsp_in_buff.wr_ptr += DSP_BUFF_SIZE;
  }

  dsp_in_buff.wr_ptr--;
}

/**
 * @brief This function writes to USBD In buffer
 *
 * @param USBD In buffer pointer
 * @param Number of bytes to write
 */

void DSP_In_Buff_Read (uint8_t *pbuf, uint32_t size)
{
  uint16_t *buff = (uint16_t*) pbuf;

  size = size / 2U;

  if (dsp_in_buff.buff_enable == 0U)
  {
    dsp_in_buff.rd_ptr = dsp_in_buff.wr_ptr + DSP_BUFF_HALF_SIZE;

    if (dsp_in_buff.rd_ptr >= DSP_BUFF_SIZE)
    {
      dsp_in_buff.rd_ptr = 0U;
    }

    dsp_in_buff.buff_enable = 1U;
  }

  for (uint32_t i = 0U; i < size; i += 2U)
  {
    buff [i + 0] = dsp_in_buff.i [dsp_in_buff.rd_ptr];
    buff [i + 1] = dsp_in_buff.q [dsp_in_buff.rd_ptr];

    dsp_in_buff.rd_ptr++;

    if (dsp_in_buff.rd_ptr >= DSP_BUFF_SIZE)
    {
      dsp_in_buff.rd_ptr = 0U;
    }
  }
}

/**
 * @brief This function sets DSP to RX mode
 *
 */

void DSP_Set_RX (void)
{
  Codec_Set_RX ();
}

/**
 * @brief This function sets DSP to TX mode
 *
 */

void DSP_Set_TX (void)
{
  Codec_Set_TX ();
}

/**
 * @brief This function sets DSP Mode
 *
 */

void DSP_Set_Mode (uint8_t mode)
{

}

/**
 * @brief This function initialize DSP and codec
 *
 */

void DSP_Init (void)
{
  i2s_buff_init ();

  Codec_Init (USBD_AUDIO_FREQ);
  Codec_AF_Vol (20U);
}

/****END OF FILE****/
