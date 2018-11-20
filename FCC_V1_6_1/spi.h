/************************************************************************/
/* \file spi.h                                                          */
/************************************************************************/
#ifndef SPI_H_INCLUDED
#define SPI_H_INCLUDED
#include "subbus.h"

#define MAX_SPI_READ_LENGTH 4
#define ADC_CONVERT_TIMEOUT 500
#define SPI_ENABLE_DEFAULT true
#define SPI_ADC_U2_ENABLED true
#define SPI_ADC_U3_ENABLED true
#define SPI_DAC_U5_ENABLED true
#define SPI_BASE_ADDR 0x10
#define SPI_HIGH_ADDR 0x1A

extern subbus_driver_t sb_spi;
void spi_enable(bool value);

#endif
