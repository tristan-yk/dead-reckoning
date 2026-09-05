/**
  ******************************************************************************
  * @file           : i2c_bus.h
  * @brief          : Shared register-level access to the I2C1 bus.
  *
  * All three devices on the shield - the BNO055, the BMP390 and the SH1106 -
  * sit on I2C1, so the register helpers live here rather than being duplicated
  * per driver. Addresses are 7-bit and shifted internally.
  ******************************************************************************
  */

#ifndef I2C_BUS_H
#define I2C_BUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

bool i2c_read(uint8_t addr7, uint8_t reg, uint8_t *buf, uint16_t len);
bool i2c_write(uint8_t addr7, uint8_t reg, uint8_t val);
uint8_t i2c_read_u8(uint8_t addr7, uint8_t reg);

/* Raw transmit with no register byte, for the SH1106 control-byte protocol. */
bool i2c_transmit(uint8_t addr7, const uint8_t *data, uint16_t len, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* I2C_BUS_H */
