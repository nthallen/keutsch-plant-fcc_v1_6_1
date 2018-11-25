#ifndef I2C_H_INCLUDED
#define I2C_H_INCLUDED
#include "subbus.h"

#define I2C_BASE_ADDR 0x20
#define I2C_HIGH_ADDR 0x28
#define I2C_ENABLE_DEFAULT true
#define I2C_TS_ENABLED     true
#define I2C_TS_ID_DEFAULT 1
#define I2C_SHT31_ENABLED  true
extern subbus_driver_t sb_i2c;
void i2c_enable(bool value);

#endif
