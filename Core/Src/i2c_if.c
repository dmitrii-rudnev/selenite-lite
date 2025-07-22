/**
  *******************************************************************************
  *
  * @file    i2c_if.c
  * @brief   I2C Bus Interface
  * @version v1.0
  * @date    22.09.2024
  * @author  Dmitrii Rudnev
  *
  *******************************************************************************
  * Copyright &copy; 2024 Selenite Project. All rights reserved.
  *
  * This software component is licensed under [BSD 3-Clause license]
  * (http://opensource.org/licenses/BSD-3-Clause/), the "License".<br>
  * You may not use this file except in compliance with the License.
  *******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c_if.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

/* External variables --------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/


/**
 * @brief This function checks if the I2C device is not ready
 *
 */

uint8_t i2c_is_not_ready (I2C_HandleTypeDef *hi2c, uint16_t dev_addr)
{
  if (HAL_I2C_IsDeviceReady (hi2c, dev_addr, 2, I2C_TIMEOUT) != HAL_OK)
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

void i2c_get_error (I2C_HandleTypeDef *hi2c)
{
  if (HAL_I2C_GetError (hi2c) != HAL_I2C_ERROR_AF)
  {
    Error_Handler ();
  }
}

/* Public functions ----------------------------------------------------------*/

/**
 * @brief This function transfers some bytes to I2C device
 *
 * @param dev_addr - device bus address
 * @param addr     - initial address of destination
 * @param *data    - pointer to data source
 * @param bytes    - number of bytes for transfer
 *
 */

void I2C_Transmit_Bulk (I2C_HandleTypeDef *hi2c, uint16_t dev_addr, uint8_t addr, uint8_t *data, uint8_t bytes)
{
  dev_addr = dev_addr << 1;

  if (i2c_is_not_ready (hi2c, dev_addr)) return;

  if (HAL_I2C_Mem_Write (hi2c, dev_addr, addr, 1, data, bytes, I2C_TIMEOUT) != HAL_OK)
  {
    i2c_get_error (hi2c);
  }
}


/**
 * @brief This function transfers one bytes to I2C device register
 *
 * @param dev_addr - device bus address
 * @param addr     - register address
 * @param data     - value
 *
 */

void I2C_Transmit (I2C_HandleTypeDef *hi2c, uint16_t dev_addr, uint8_t addr, uint8_t data)
{
  dev_addr = dev_addr << 1;

  if (i2c_is_not_ready (hi2c, dev_addr)) return;

  uint8_t d[2] = {addr, data};

  if (HAL_I2C_Master_Transmit (hi2c, (uint16_t) dev_addr, d, 2, I2C_TIMEOUT) != HAL_OK)
  {
    i2c_get_error (hi2c);
  }
}

/**
 * @brief This function receives one bytes from I2C device
 *
 * @param dev_addr - device bus address
 * @param addr     - register address
 * @param *data    - pointer to destination
 *
 */

void I2C_Receive (I2C_HandleTypeDef *hi2c, uint16_t dev_addr, uint8_t addr, uint8_t *data)
{
  dev_addr = dev_addr << 1;

  if (i2c_is_not_ready (hi2c, dev_addr)) return;

  while (HAL_I2C_Master_Transmit (hi2c, dev_addr, (uint8_t*) &addr, 1, I2C_TIMEOUT) != HAL_OK)
  {
    i2c_get_error (hi2c);
    return;
  }

  if (i2c_is_not_ready (hi2c, dev_addr)) return;

  while (HAL_I2C_Master_Receive (hi2c, dev_addr, data, 1, I2C_TIMEOUT) != HAL_OK)
  {
    i2c_get_error (hi2c);
  }
}

/**
 * @brief This functions made one byte transfer to I2C device with bit-banging
 *
 * @param addr    - register address
 * @param data    - pointer to destination
 *
 */

#define SCL          GPIO_PIN_8
#define SDA          GPIO_PIN_9
#define pI2C_Speed   30

void I2C_Init_ (void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
   
  GPIO_InitStructure.Pin   = SDA | SCL;
  GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull  = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init (GPIOB, &GPIO_InitStructure);

  HAL_GPIO_WritePin (GPIOB, SDA | SCL, GPIO_PIN_SET);
}
//-------------------------------------------------------

void i2c_start (void)
{
  uint8_t i = 0;
  GPIO_InitTypeDef GPIO_InitStructure;
   
  GPIO_InitStructure.Pin   = SDA;
  GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull  = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init (GPIOB, &GPIO_InitStructure);

  HAL_GPIO_WritePin (GPIOB, SDA, GPIO_PIN_SET);
  for (i = 0; i < pI2C_Speed; ++i) {asm ("nop");}

  HAL_GPIO_WritePin (GPIOB, SCL, GPIO_PIN_SET);
  for (int i = 0; i < pI2C_Speed; ++i) {asm ("nop");}

  HAL_GPIO_Init (GPIOB, &GPIO_InitStructure);

  HAL_GPIO_WritePin (GPIOB, SDA, GPIO_PIN_RESET);
  for (i = 0; i < pI2C_Speed; ++i) {asm ("nop");}

  HAL_GPIO_WritePin (GPIOB, SCL, GPIO_PIN_RESET);
  for (i = 0; i < pI2C_Speed; ++i) {asm ("nop");}
  for (i = 0; i < pI2C_Speed; ++i) {asm ("nop");}  
  for (i = 0; i < pI2C_Speed; ++i) {asm ("nop");}  
  for (i = 0; i < pI2C_Speed; ++i) {asm ("nop");}
}
//-------------------------------------------------------

void i2c_stop (void)
{
  uint8_t i = 0;
  GPIO_InitTypeDef GPIO_InitStructure;
   
  GPIO_InitStructure.Pin   = SDA;
  GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull  = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init (GPIOB, &GPIO_InitStructure);

  HAL_GPIO_WritePin (GPIOB, SDA, GPIO_PIN_RESET);
  for (i = 0; i < pI2C_Speed; ++i) {asm ("nop");}

  HAL_GPIO_WritePin (GPIOB, SCL, GPIO_PIN_SET);
  for (i = 0; i < pI2C_Speed; ++i) {asm ("nop");}

  HAL_GPIO_WritePin (GPIOB, SDA, GPIO_PIN_SET);
  for (i = 0; i < pI2C_Speed; ++i) {asm ("nop");}
  for (i = 0; i < pI2C_Speed; ++i) {asm ("nop");}  
  for (i = 0; i < pI2C_Speed; ++i) {asm ("nop");}  
  for (i = 0; i < pI2C_Speed; ++i) {asm ("nop");}
}
//-------------------------------------------------------

void i2c_write (uint8_t byte)
{
  uint8_t i = 0;
  uint8_t j = 0;
  uint8_t temp = byte;
  GPIO_InitTypeDef GPIO_InitStructure;
   
  GPIO_InitStructure.Pin   = SDA;
  GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull  = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init (GPIOB, &GPIO_InitStructure);

  for (j = 0; j < 8; ++j)
  {
    for (i = 0; i < pI2C_Speed; ++i) {asm ("nop");}
    /* Writing bit */
    if ((temp & 0x80) == 0x80) {HAL_GPIO_WritePin (GPIOB, SDA, GPIO_PIN_SET);}
    else {HAL_GPIO_WritePin (GPIOB, SDA, GPIO_PIN_RESET);}
    temp <<= 1;

    for (i = 0; i < pI2C_Speed; ++i) {asm ("nop");}
    HAL_GPIO_WritePin (GPIOB, SCL, GPIO_PIN_SET);
    for (i = 0; i < pI2C_Speed; ++i) {asm ("nop");}
    HAL_GPIO_WritePin (GPIOB, SCL, GPIO_PIN_RESET);
  }
  
  GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
  HAL_GPIO_Init (GPIOB, &GPIO_InitStructure);

  for (i = 0; i < pI2C_Speed; ++i) {asm ("nop");}
  HAL_GPIO_WritePin (GPIOB, SCL, GPIO_PIN_SET);
  for (i = 0; i < pI2C_Speed; ++i) {asm ("nop");}
  HAL_GPIO_WritePin (GPIOB, SCL, GPIO_PIN_RESET);
}
//-------------------------------------------------------

void I2C_Transmit_ (uint8_t addr, uint8_t data)
{
  i2c_start ();
  i2c_write (0x40);
  i2c_write (addr);
  i2c_write (data);
  i2c_stop  ();
}

/****END OF FILE****/
