#ifndef I2C_H_INCLUDED
#define I2C_H_INCLUDED
#include <stdint.h>
#include "subbus.h"

#define I2C_BASE_ADDR 0x20
#define I2C_HIGH_ADDR 0x2B
#define I2C_ENABLE_DEFAULT true
#define I2C_TS_ENABLED     true
/** Temp Sensor IDs here use the 1-based numbering from 1 to 6 */
#define I2C_TS_ID_DEFAULT 4
#define I2C_SHT31_ENABLED  true
extern subbus_driver_t sb_i2c;
void i2c_enable(bool value);
void i2c_write(int16_t i2c_addr, const uint8_t *obuf, int16_t nbytes);
void i2c_read(int16_t i2c_addr, uint8_t *ibuf, int16_t nbytes);

#endif
