/**
  *******************************************************************************
  *
  * @file    codec_if.c
  * @brief   Codec interface
  * @version v2.1
  * @date    28.10.2022
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
#include "codec_if.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

const Output_Level output_level [21] =
{
    {0, 117}, { 5, 52}, {10, 40}, {15, 33}, //-32.1, -26.0, -20.0, -16.5 dB
    {20, 28}, {25, 24}, {30, 21}, {35, 18}, //-14.0, -12.0, -10.5,  -9.0 dB
    {40, 16}, {45, 14}, {50, 12}, {56, 10}, // -8.0,  -7.0   -6.0,  -5.0 dB
    {60,  9}, {67,  7}, {71,  6}, {75,  5}, // -4.5,  -3.5,  -3.0,  -2.5 dB
    {80,  4}, {85,  3}, {90,  2}, {95,  1}, // -2.0,  -1.5,  -1.0,  -0.5 dB
    {99,  0}                                //  0.0 dB
};

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

/* External variables --------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
 * @brief This function checks if the I2C device is ready
 *
 */

uint8_t codec_i2c_is_not_ready (void)
{
  if (HAL_I2C_IsDeviceReady (&CODEC_I2C_PORT,
                             (uint16_t) CODEC_BUS_BASE_ADDR << 1,
                             2, CODEC_I2C_TIMEOUT) != HAL_OK)
  {
    Error_Handler ();
    return 1U;
  }
  return 0U;
}

/**
 * @brief This function checks for I2C transmission errors
 *
 */

void codec_i2c_get_error (void)
{
  if (HAL_I2C_GetError (&CODEC_I2C_PORT) != HAL_I2C_ERROR_AF)
  {
    Error_Handler ();
  }
}


/**
 * @brief This function writes to a codec register
 *
 */

void codec_write_reg (uint8_t addr, uint8_t data)
{
  if (codec_i2c_is_not_ready ()) return;

  uint8_t d [2] = { addr, data };

  if (HAL_I2C_Master_Transmit (&CODEC_I2C_PORT,
                               (uint16_t) CODEC_BUS_BASE_ADDR << 1,
                               (uint8_t*) d, 2, CODEC_I2C_TIMEOUT) != HAL_OK)
  {
    codec_i2c_get_error ();
  }
}


/**
 * @brief This function reads from a codec register
 *
 */

void codec_read_reg (uint8_t addr, uint8_t *data)
{
  if (codec_i2c_is_not_ready ()) return;

  addr -= 1U;

  if (HAL_I2C_Master_Transmit (&CODEC_I2C_PORT,
                               (uint16_t) CODEC_BUS_BASE_ADDR << 1,
                               (uint8_t*) &addr, 1, CODEC_I2C_TIMEOUT) != HAL_OK)
  {
    codec_i2c_get_error ();
    return;
  }

  if (codec_i2c_is_not_ready ()) return;

  if (HAL_I2C_Master_Receive (&CODEC_I2C_PORT,
                              (uint16_t) CODEC_BUS_BASE_ADDR << 1,
                              data, 1, CODEC_I2C_TIMEOUT) != HAL_OK)
  {
    codec_i2c_get_error ();
  }
}

/**
 * @brief This function sets the bit in codec register
 *
 */

void codec_bit_set (uint8_t addr, uint8_t bit)
{
  uint8_t data = 0;

  if (bit < 8)
  {
    codec_read_reg (addr, &data);
    data |= (1 << bit);
    codec_write_reg (addr, data);
  }
}

/**
 * @brief This function resets the bit in codec register
 *
 */

void codec_bit_reset (uint8_t addr, uint8_t bit)
{
  uint8_t data = 0;

  if (bit < 8)
  {
    codec_read_reg (addr, &data);
    data &= ~(1 << bit);
    codec_write_reg (addr, data);
  }
}

/**
 * @brief This function mutes all codec channels
 *
 */

void codec_mute_all (void)
{
  codec_bit_reset (86, 3); /* Reset D3 of Register 86 to mute LEFT_LOP/M  */
  codec_bit_reset (93, 3); /* Reset D3 of Register 93 to mute RIGHT_LOP/M */

  codec_bit_reset (51, 3); /* Reset D3 of Register 51 to mute HPLOUT */
  codec_bit_reset (65, 3); /* Reset D3 of Register 65 to mute HPROUT */

  codec_bit_reset (58, 3); /* Reset D3 of Register 58 to mute HPLCOM */
  codec_bit_reset (72, 3); /* Reset D3 of Register 72 to mute HPRCOM */


  codec_bit_set (43, 7);   /* Set D7 of Register 43 to mute Left DAC channel  */
  codec_bit_set (44, 7);   /* Set D7 of Register 44 to mute Right DAC channel */

  codec_bit_set (15, 7);   /* Set D7 of Register 15 to mute PGA_L */
  codec_bit_set (16, 7);   /* Set D7 of Register 16 to mute PGA_R */
}

/**
 * @brief This function sets gain of PGA and DAC
 *
 */

void codec_set_gain (uint8_t addr, uint8_t gain)
{
  /* Used to set:
   * PGA gain 0...+59.5 dB (registers 15, 16)
   * DAC gain 0...-63.5 dB (registers 43, 44)
   * routed to outputs sources gain 0...-78.3 dB
   * (registers 45...50, 52...57, 59...64, 66...71, 80...85, 87...92)
   *
   * Registers 73...79 are reserved, do not write to these registers
   */

  uint8_t data = 0;

  if (gain > 127) gain = 127;

  codec_read_reg (addr, &data);

  data &= 0x80;
  data |= gain;
  codec_write_reg (addr, data);
}

/**
 * @brief This function sets bits D6...D3 of control registers 19...24
 *
 */

void codec_set_in_level (uint8_t addr, uint8_t level)
{
  /* Bits D6...D3 of control registers 19...24 set the input level in dB:
   * 0000 =  0.0; 0001 =  -1.5; 0010 =  -3.0;
   * 0011 = -4.5; 0100 =  -6.0; 0101 =  -7.5;
   * 0110 = -9.0; 0111 = -10.5; 1000 = -12.0;
   * 1001...1110 = reserved, do not write these sequences;
   * 1111 = not connected
   *
   * MIC2L is used as MICDET, D7...D4 of control registers 17, 18 are always 1111
   * MIC2R uses D3...D0 of control registers 17, 18 to set the input level
   */

  uint8_t data = 0;

  if (addr < 17) { return; }
  if (addr > 24) { return; }
  if (level < 0x0F)
  {
    if (level > 0x08) { level = 0x08; }
  }

  codec_read_reg (addr, &data);

  if (addr < 19)
  {
    /* MIC2R control registers 17, 18 */
    data &= 0xF0;
    data |= level;
  }
  else
  {
    data &= 0x87;
    data |= (level << 3);
  }

  codec_write_reg (addr, data);
}

/**
 * @brief This function sets bits D7...D4 of control registers 51, 58, 65, 72, 86, 93
 *
 */

void codec_set_out_level (uint8_t addr, uint8_t level)
{
  /* Bits D7...D4 of control registers 51, 58, 65, 72, 86, 93
   * set the output level in dB:
   * 0000 = 0; 0001 = 1; 0010 = 2; 0011 = 3; 0100 = 4;
   * 0101 = 5; 0110 = 6; 0111 = 7; 1000 = 8; 1001 = 9;
   * 1010...1111 = reserved, do not write these sequences;
   */

  uint8_t data = 0;

  if (level > 0x09) { level = 0x09; }

  codec_read_reg (addr, &data);

  data &= 0x0F;
  data |= (level << 4);
  codec_write_reg (addr, data);
}

/**
 * @brief This function sets codec to RX mode
 *
 */

void codec_set_rx (void)
{
  /* Mute all channels */
  codec_mute_all ();

  /* Power MIC down */
  codec_write_reg (25, 0x00);

  /* Unroute MIC3R from PGA */
  codec_set_in_level (17, 0x0F); /* Set level 1111 to unroute MIC3R from PGA_L */
  codec_set_in_level (18, 0x0F); /* Set level 1111 to unroute MIC3R from PGA_R */

  /* Route LINE1 to PGA */
  codec_set_in_level (19, 0x00); /* 0.0 dB */
  codec_set_in_level (22, 0x00); /* 0.0 dB */

  /* Unroute DAC from xLOMP */
  codec_bit_reset (82, 7);    /* Reset D7 of Register 82 to unroute DAC_L1 from LEFT_LOP/M  */
  codec_bit_reset (92, 7);    /* Reset D7 of Register 92 to unroute DAC_R1 from RIGHT_LOP/M */

  /* Route DAC to HPxOUT */
  codec_bit_set (47, 7);      /* Set D7 of Register 47 to route DAC_L1 output to HPLOUT */
  codec_bit_set (64, 7);      /* Set D7 of Register 64 to route DAC_R1 output to HPROUT */

  /* Route DAC to HPxCOM */
  codec_bit_set (54, 7);      /* Set D7 of Register 54 to route DAC_L1 output to HPLCOM */
  codec_bit_set (71, 7);      /* Set D7 of Register 71 to route DAC_R1 output to HPRCOM */

  /* Unmute RX channels */
  codec_bit_reset (15, 7);    /* Reset D7 of Register 15 to unmute PGA_L */
  codec_bit_reset (16, 7);    /* Reset D7 of Register 16 to unmute PGA_R */

  codec_bit_reset (43, 7);    /* Reset D7 of Register 43 to unmute Left DAC channel */
  codec_bit_reset (44, 7);    /* Reset D7 of Register 44 to unmute Right DAC channel */

  codec_bit_set (51, 3);      /* Set D3 of Register 51 to unmute HPLOUT */
  codec_bit_set (65, 3);      /* Set D3 of Register 65 to unmute HPROUT */

  codec_bit_set (58, 3);      /* Set D3 of Register 58 to unmute HPLCOM */
  codec_bit_set (72, 3);      /* Set D3 of Register 72 to unmute HPRCOM */

  /* Set RX channels levels */
  /*
  codec_set_gain (15, level);
  codec_set_gain (16, level);

  codec_set_gain (43, level);
  codec_set_gain (44, level);

  codec_set_out_level (51, level);
  codec_set_out_level (65, level);

  codec_set_out_level (58, level);
  codec_set_out_level (72, level);
  */
}

/**
 * @brief This function sets codec to TX mode
 *
 */

void codec_set_tx (void)
{
  /* Mute all channels */
  codec_mute_all ();

  /* Unroute LINE1 from PGA */
  codec_set_in_level (19, 0x0F); /* Set level 1111 to unroute LINE1L from PGA_L */
  codec_set_in_level (22, 0x0F); /* Set level 1111 to unroute LINE1R from PGA_R */

  /* Power MIC up */
  codec_write_reg (25, 0x40);

  /* Route MIC3R to PGA */
  codec_set_in_level (17, 0x00); /* 0.0 dB */
  codec_set_in_level (18, 0x00); /* 0.0 dB */

  /* Unroute DAC from HPxCOM */
  codec_bit_reset (54, 7);    /* Reset D7 of Register 54 to unroute DAC_L1 from HPLCOM */
  codec_bit_reset (71, 7);    /* Reset D7 of Register 71 to unroute DAC_R1 from HPRCOM */

  /* Unroute DAC from HPxOUT */
  codec_bit_reset (47, 7);    /* Reset D7 of Register 47 to unroute DAC_L1 output to HPLOUT */
  codec_bit_reset (64, 7);    /* Reset D7 of Register 64 to unroute DAC_R1 output to HPROUT */

  /* Route DAC to xLOMP */
  codec_bit_set (82, 7);      /* Set D7 of Register 82 to route DAC_L1 from LEFT_LOP/M  */
  codec_bit_set (92, 7);      /* Set D7 of Register 92 to route DAC_R1 from RIGHT_LOP/M */

  /* Unmute TX channels */
  codec_bit_reset (15, 7);    /* Reset D7 of Register 15 to unmute PGA_L */
  codec_bit_reset (16, 7);    /* Reset D7 of Register 16 to unmute PGA_R */

  codec_bit_reset (43, 7);    /* Reset D7 of Register 43 to unmute Left DAC channel  */
  codec_bit_reset (44, 7);    /* Reset D7 of Register 44 to unmute Right DAC channel */

  codec_bit_set (86, 3);      /* Set D3 of Register 86 to unmute LEFT_LOP/M  */
  codec_bit_set (93, 3);      /* Set D3 of Register 93 to unmute RIGHT_LOP/M */

  /* Set TX channels levels */
  /*
  codec_set_gain (15, level);
  codec_set_gain (16, level);

  codec_set_gain (43, level);
  codec_set_gain (44, level);

  codec_set_out_level (86, level);
  codec_set_out_level (93, level);
  */
}

/**
 * @brief This function initializes codec
 *
 */

void codec_init (uint32_t sr)
{
  HAL_GPIO_WritePin (I2S2_RES_GPIO_Port, I2S2_RES_Pin, GPIO_PIN_RESET);
  HAL_Delay (100);
  HAL_GPIO_WritePin (I2S2_RES_GPIO_Port, I2S2_RES_Pin, GPIO_PIN_SET);
  HAL_Delay (100);

  codec_write_reg (0, 0x00);   /* Write 0 to Register 0 DO for Page 0 activating */

/* Codec uses MCLK */
  /* Write 1 to Register 101 for CODEC_CLKIN uses CLKDIV_OUT */
  codec_write_reg (101, 0x01);
  /* Write 0x02 to Register 102 for CLKDIV_IN uses MCLK (default)
   * If MCLK = 12288 kHz write 0x10 to Register 3 to divide CODEC_CLKIN by 1 (Q = 2)(default)
   * If MCLK = 24576 kHz write 0x20 to Register 3 to divide CODEC_CLKIN by 2 (Q = 4) */

  /* Write 0x0A to Register 7 to route Left data to Left DAC, Route Right data to Right DAC (48 kHz)
   * Write 0x6A to Register 7 to the same routing with ADC, DAC dual-rate mode (96 kHz) */

  codec_write_reg (102, 0x02);

  /*
  if (sr == 96000U)
  {
    codec_write_reg (3, 0x20);
    codec_write_reg (7, 0x6A);
  }
  else
  {
    codec_write_reg (3, 0x10);
    codec_write_reg (7, 0x0A);
  }
  */

  codec_write_reg (3, 0x20);
  codec_write_reg (7, 0x6A);

  /* Set D7 of Register 14 to set HP outputs for ac-coupled driver configuration */
  codec_bit_set (14, 7);
  /* Set D5, reset D4 of Register 37 to configure HPLCOM as independent SE output
   * Set D6 of Register 37 to power up Right DAC
   * Set D7 of Register 37 to power up Left DAC */
  codec_write_reg (37, 0xE0);
  /* Set D4, reset D5, D3 of Register 38 to configure HPRCOM as independent SE output */
  codec_write_reg (38, 0x10);

  /* Write 0x50 to Register 12 to set ADC high-pass filter –3dB frequency = 0.0045 × ADC fs */
  codec_write_reg (12, 0x50);

  codec_set_gain (15, 0);      /* Set PGA_L gain 0 / 2 = 0dB */
  codec_set_gain (16, 0);      /* Set PGA_R gain 0 / 2 = 0dB */

  //++++++
  //codec_set_gain (15, 40);      /* Set PGA_L gain 40 / 2 = 20dB */
  //codec_set_gain (16, 40);      /* Set PGA_R gain 40 / 2 = 20dB */
  //codec_write_reg (26, 0x80);   /* Set PGA_L AGC enable with default values */
  //codec_write_reg (29, 0x80);   /* Set PGA_R AGC enable with default values */
  //++++++

  codec_bit_set (19, 2);       /* Set D2 of Register 19 to power up Left ADC  */
  codec_bit_set (22, 2);       /* Set D2 of Register 22 to power up Right ADC */

  codec_bit_set (58, 0);       /* Set D0 of Register 58 to power up HPLCOM */
  codec_bit_set (72, 0);       /* Set D0 of Register 72 to power up HPRCOM */

  codec_bit_set (51, 0);       /* Set D0 of Register 51 to power up HPLOUT */
  codec_bit_set (65, 0);       /* Set D0 of Register 65 to power up HPROUT */

  codec_bit_set (86, 0);       /* Set D0 of Register 86 to power up LEFT_LOP/M  */
  codec_bit_set (93, 0);       /* Set D0 of Register 93 to power up RIGHT_LOP/M */

  codec_set_out_level (86, 0); /* Set LEFT_LOP/M  output level = 0dB */
  codec_set_out_level (93, 0); /* Set RIGHT_LOP/M output level = 0dB */

  /* Write data to Register 25 to set MICBIAS voltage
   * data = 0x00 is powered down
   * data = 0x40 is powered to 2V
   * data = 0x80 is powered to 2.5V
   * data = 0xC0 is powered to AVDD */
  codec_write_reg (25, 0x40);

  codec_set_rx ( );
}

/* Public functions ----------------------------------------------------------*/

/**
 * @brief This function sets codec to RX mode
 *
 */

void Codec_Set_RX (void)
{
  codec_set_rx ( );
}

/**
 * @brief This function sets codec to TX mode
 *
 */

void Codec_Set_TX (void)
{
  codec_set_tx ( );
}

/**
 * @brief This function sets codec mode and starts I2S buffer
 *
 * @param none
 */

void Codec_Init (uint32_t sr)
{
  codec_init (sr);
}


/**
 * @brief This function sets volume to HP Outputs
 *
 * @param volume in percents
 * @retval volume in percents
 */

uint8_t Codec_AF_Vol (uint8_t vol)
{
  uint8_t af_vol = 0;
  af_vol = output_level[vol].reg_val;

  codec_set_gain (47, af_vol); /* set volume DAC_L1 to HPLOUT */
  codec_set_gain (64, af_vol); /* set volume DAC_R1 to HPROUT */

  return output_level[vol].percent;
}

/****END OF FILE****/
