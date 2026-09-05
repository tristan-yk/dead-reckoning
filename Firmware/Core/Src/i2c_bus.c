/**
  ******************************************************************************
  * @file           : i2c_bus.c
  * @brief          : Shared register-level access to the I2C1 bus.
  ******************************************************************************
  */

#include "i2c_bus.h"

#include "main.h"

extern I2C_HandleTypeDef hi2c1;

/* Long enough for the largest sensor burst at 400 kHz with room to spare, but
 * short enough that a wedged device cannot stall a tick indefinitely. */
#define I2C_TIMEOUT_MS 20u

bool i2c_read(uint8_t addr7, uint8_t reg, uint8_t *buf, uint16_t len)
{
  return HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(addr7 << 1), reg,
                          I2C_MEMADD_SIZE_8BIT, buf, len,
                          I2C_TIMEOUT_MS) == HAL_OK;
}

bool i2c_write(uint8_t addr7, uint8_t reg, uint8_t val)
{
  return HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(addr7 << 1), reg,
                           I2C_MEMADD_SIZE_8BIT, &val, 1,
                           I2C_TIMEOUT_MS) == HAL_OK;
}

uint8_t i2c_read_u8(uint8_t addr7, uint8_t reg)
{
  uint8_t v = 0;
  (void)i2c_read(addr7, reg, &v, 1);
  return v;
}

bool i2c_transmit(uint8_t addr7, const uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
  return HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(addr7 << 1),
                                 (uint8_t *)data, len, timeout_ms) == HAL_OK;
}
