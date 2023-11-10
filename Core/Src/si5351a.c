/**
  *******************************************************************************
  *
  * @file    si5351a.c
  * @brief   si5351a frequency synthesizer driver
  * @version v1.0
  * @date    18.10.2022
  * @author  Dmitrii Rudnev
  *
  *******************************************************************************
  * Copyrigh &copy; 2022 Selenite Project. All rights reserved.
  *
  * This software component is licensed under [BSD 3-Clause license]
  * (http://opensource.org/licenses/BSD-3-Clause/), the "License".<br>
  * You may not use this file except in compliance with the License.
  *
  * This software uses some source code from QCX-SSB.ino
  * https://github.com/threeme3/QCX-SSB
  * Copyright 2019, 2020, 2021 Guido PE1NNZ <pe1nnz@amsat.org>
  *
  *******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include "si5351a.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

#define _MSC     0x10000

/* Private macro -------------------------------------------------------------*/

 #define BB0(x) ((uint8_t) (x))           // Bash byte x of int32_t
 #define BB1(x) ((uint8_t)((x) >> 8 ))
 #define BB2(x) ((uint8_t)((x) >> 16))

/* Private variables ---------------------------------------------------------*/

uint32_t fxtal = F_XTAL;
int16_t  iqmsa = 0;

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

/* External variables --------------------------------------------------------*/

extern I2C_HandleTypeDef SI5351_I2C_PORT;

/* Private functions ---------------------------------------------------------*/

/**
 * @brief This function checks if the I2C device is ready
 *
 */

uint8_t si5351_i2c_is_not_ready (void)
{
  if (HAL_I2C_IsDeviceReady (&SI5351_I2C_PORT,
                             (uint16_t) SI5351_BUS_BASE_ADDR << 1,
                             2, SI5351_I2C_TIMEOUT) != HAL_OK)
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

void si5351_i2c_get_error (void)
{
  if (HAL_I2C_GetError (&SI5351_I2C_PORT) != HAL_I2C_ERROR_AF)
  {
    Error_Handler ();
  }
}

/**
 * @brief This function transfers some bytes to I2C device
 *
 * @param addr  - initial address of destination
 * @param *data - pointer to data source
 * @param bytes - number of bytes for transfer
 *
 */

void si5351_write_bulk (uint8_t addr, uint8_t *data, uint8_t bytes)
{
  if (si5351_i2c_is_not_ready ()) return;

  if (HAL_I2C_Mem_Write (&SI5351_I2C_PORT,
                         (uint16_t) SI5351_BUS_BASE_ADDR << 1,
                         addr, 1, data, bytes, SI5351_I2C_TIMEOUT) != HAL_OK)
  {
    si5351_i2c_get_error ();
  }
}

/**
 * @brief This function transfers one bytes to I2C device register
 *
 * @param addr - register address
 * @param data - value
 *
 */

void si5351_write (uint8_t addr, uint8_t data)
{
  if (si5351_i2c_is_not_ready ()) return;

  uint8_t d[2] = {addr, data};

  if (HAL_I2C_Master_Transmit (&SI5351_I2C_PORT,
                               (uint16_t) SI5351_BUS_BASE_ADDR << 1,
                               (uint8_t *) d, 2, SI5351_I2C_TIMEOUT) != HAL_OK)
  {
    si5351_i2c_get_error ();
  }
}

/**
 * @brief This function receives one bytes from I2C device register
 *
 * @param addr  - register address
 * @param *data - pointer to destination
 *
 */

void si5351_read (uint8_t addr, uint8_t *data)
{
  if (si5351_i2c_is_not_ready ()) return;

  while (HAL_I2C_Master_Transmit (&SI5351_I2C_PORT,
                                  (uint16_t) SI5351_BUS_BASE_ADDR << 1,
                                  (uint8_t*) &addr, 1, SI5351_I2C_TIMEOUT) != HAL_OK)
  {
    si5351_i2c_get_error ();
    return;
  }

  if (si5351_i2c_is_not_ready ()) return;

  while (HAL_I2C_Master_Receive (&SI5351_I2C_PORT,
                                 (uint16_t) SI5351_BUS_BASE_ADDR << 1,
                                 data, 1, SI5351_I2C_TIMEOUT) != HAL_OK)
  {
    si5351_i2c_get_error ();
  }
}

/**
 * @brief This function resets PLLs
 *
 */

void si5351_pll_reset ()
{
  /* Write 0x20 to Register 177 to reset PLLA */
  /* Write 0x80 to Register 177 to reset PLLB */
  /* Write 0xA0 to Register 177 to reset both PLL */

  si5351_write (177, 0x20);
}

/**
 * @brief This function calculate the multisynth stage
 *
 * @param n         - the stage instance (see ms_t enumerator)
 * @param div_nom   - the first Multisynth divider value
 * @param div_denom - the second Multisynth divider value
 * @param pll       - the PLL instance
 * @param _int      - _int = 1 if integer mode
 * @param phase     - the value of phase shift
 * @param rdiv      - the R Divider value
 *
 */

void si5351_set_ms (int8_t n, uint32_t div_nom, uint32_t div_denom, uint8_t pll, uint8_t _int, uint16_t phase, uint8_t rdiv)
{
  uint16_t msa;
  uint32_t msb, msc, msp1, msp2, msp3;

  msa = div_nom / div_denom;   /* integer part: msa must be in range 15..90 for PLL, 8+1/1048575..900 for MS */

  /* to satisfy the MSx_INT=1 requirement of AN619, section 4.1.3 which basically says
   * that for MS divider a value of 4 and integer mode must be used */

  if (msa == 4) { _int = 1; }

  msb = (_int) ? 0 : (((uint64_t)(div_nom % div_denom)*_MSC) / div_denom); /* fractional part */

  msc = (_int) ? 1 : _MSC;

  msp1 = 128 * msa + 128 * msb / msc - 512;
  msp2 = 128 * msb - 128 * msb / msc * msc;
  msp3 = msc;

  uint8_t ms_reg2 = BB2 (msp1) | (rdiv << 4) | ((msa == 4) * 0x0C);

  uint8_t ms_regs[8] = { BB1(msp3), BB0(msp3), ms_reg2, BB1(msp1), BB0(msp1), BB2(((msp3 & 0x0F0000)<<4) | msp2), BB1(msp2), BB0(msp2) };

  si5351_write_bulk (n * 8 + 42, ms_regs, 8); // Write to MSx

  if (n < 0)
  {
    /* MSNx PLLn: 0x40 = FBA_INT; 0x80 = CLKn_PDN */
    si5351_write (n + 16 + 8, 0x80 | (0x40 * _int));
  }
  else
  {
    /* MSx CLKn: 0x0C = PLLA, 0x2C = PLLB local msynth; 3 = 8mA; 0x40 = MSx_INT; 0x80 = CLKx_PDN */
    si5351_write (n + 16, ((pll) * 0x20) | 0x0C | 2 | (0x40 * _int));
    /* when using: make sure to configure MS in fractional-mode, perform reset afterwards */
    si5351_write (n + 165, (!_int) * phase * msa / 90);
  }
}

/**
 * @brief This function sets frequency and phases of CKL0 and CLK1 signals
 *
 * PLLA is used
 *
 * @param fout - TUNE frequency in Hz
 * @param i    - phase of CKL0 signal 0...90 deg
 * @param q    - phase of CKL1 signal 0...90 deg
 *
 */

void si5351_set_freqa (int32_t fout, uint16_t i, uint16_t q)
{
  uint8_t rdiv = 0;     /* CLK pin sees fout / (2 ^ rdiv) */

  if (fout > 300000000) /* for higher freqs, use 3rd harmonic */
  {
    i /= 3;
    q /= 3;
    fout /= 3;
  }

  if (fout < 500000)    /* Divide by 128 for fout = 4...500kHz */
  {
    rdiv = 7;
    fout *= 128;
  }

  uint16_t d;

  if (fout < 30000000)  /* Integer part  .. maybe 44? */
  {
    d = (16 * fxtal) / fout;
  }
  else
  {
    d = (32 * fxtal) / fout;
  }

  /* PLL at (27 * 7) = 189MHz cover 160m band (freq > 1.48MHz) when using 27MHz crystal */
  if (fout < 3500000) { d = (7 * fxtal) / fout; }

  /* for fout = 140...300MHz; AN619; 4.1.3, this implies integer mode */
  if (fout > 140000000) { d = 4; }

  /* even numbers preferred for divider (AN619 p.4 and p.6) */
  if (d % 2) { d++; }

  /* Test if multiplier remains same for freq deviation +/- 5kHz, if not use different divider to make same */
  if ((d * (fout - 5000) / fxtal) != (d * (fout + 5000) / fxtal))
  {
    d += 2;
  }

  uint32_t fvcoa = d * fout;
  /* Variable PLLA VCO frequency at integer multiple of fout at around 27MHz * 16 = 432MHz
   * si5351 spectral purity considerations: https://groups.io/g/QRPLabs/message/42662 */

  si5351_set_ms (MSNA, fvcoa, fxtal, PLLA, 0, 0, 0);     /* PLLA is in fractional mode */
  si5351_set_ms (MS0,  fvcoa, fout,  PLLA, 0, i, rdiv);  /* Multisynth stage with integer divider */
  si5351_set_ms (MS1,  fvcoa, fout,  PLLA, 0, q, rdiv);  /* but in frac mode due to phase setting */

  if (iqmsa != (((int8_t) i - (int8_t) q) * ((int16_t) (fvcoa / fout)) / 90))
  {
    iqmsa = ((int8_t) i - (int8_t) q) * ((int16_t) (fvcoa / fout)) / 90;
    si5351_pll_reset ();
  }
}

/**
 * @brief This function sets CLK2 signal frequency
 *
 * PLLB is used
 *
 * @param fout - TUNE frequency in Hz
 *
 */
void si5351_set_freqb (uint32_t fout)
{
    uint16_t d = (16 * fxtal) / fout;

    if (d % 2) { d++; }         /* even numbers preferred for divider (AN619 p.4 and p.6) */

    uint32_t fvcob = d * fout;  /* PLLB VCO frequency at integer multiple of fout at around 27MHz * 16 = 432MHz */

    si5351_set_ms (MSNB, fvcob, fxtal, PLLB, 0, 0, 0);
    si5351_set_ms (MS2,  fvcob, fout,  PLLB, 0, 0, 0);
}

/**
 * @brief This function resets si5351a chip to initial state
 *
 */

void si5351_power_down ()
{
  si5351_write (3, 0b11111111);       /* Disable all CLK outputs */

  /* Disable state: CLK2 HIGH state, CLK0 & CLK1 LOW state when disabled;
   * CLK2 needs to be in HIGH state to make sure that cap to gate is already charged,
   * preventing "exponential pulse is caused by CLK2,
   * which had been at 0v whilst it was disabled, suddenly generating a 5vpp waveform,
   * which is “added to” the 0v filtered PWM output and causing the output FETs
   * to be driven with the full 5v pp.", see: https://forum.dl2man.de/viewtopic.php?t=146&p=1307#p1307 */
  si5351_write (24, 0b00010000);

  si5351_write (25, 0b00000000);      /* Disable state: LOW state when disabled */

  for (int addr = 16; addr != 24; addr++)
  {
    si5351_write (addr, 0b10000000);  /* Conserve power when output is disabled */
  }

  si5351_write (187, 0);              /* Disable fanout (power-safe) */

  /* To initialise things as they should: */
  si5351_write (149, 0);              /* Disable spread spectrum enable */
  si5351_write (183, 0b11010010);     /* Internal CL = 10 pF (default) */
}


/* Public functions ----------------------------------------------------------*/

/**
 * @brief This function sets TUNE frequency and phases
 *
 * @param fout - TUNE frequency in Hz
 * @param i    - phase of CKL0 signal 0...90 deg
 * @param q    - phase of CKL1 signal 0...90 deg
 *
 */

void Si5351a_Set_Freq (uint32_t fout, uint16_t i, uint16_t q)
{
  si5351_set_freqa (fout, i, q);
}

/**
 * @brief This function resets chip and sets HSE and TUNE frequencies
 *
 */

void Si5351a_Init (void)
{
  fxtal = F_XTAL;

  si5351_power_down ();               /* Reset si5351a chip */
  HAL_Delay (100);

  si5351_set_freqb (24576000U);       /* Set HSE = 24.576 MHz */
  HAL_Delay (100);

  si5351_write (3, 0b11111000);       /* Enable all CLK outputs */
}

/****END OF FILE****/

