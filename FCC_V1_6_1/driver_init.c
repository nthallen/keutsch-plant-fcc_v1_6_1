/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */

#include "driver_init.h"
#include <peripheral_clk_config.h>
#include <utils.h>
#include <hal_init.h>

/*! The buffer size for USART */
// #define USART_CTRL_BUFFER_SIZE 16

// struct usart_async_descriptor USART_CTRL;

// static uint8_t USART_CTRL_buffer[USART_CTRL_BUFFER_SIZE];

// struct spi_m_async_descriptor SPI_ADC;

struct i2c_m_sync_desc I2C_0;

// void SPI_ADC_PORT_init(void)
// {

	// gpio_set_pin_level(MOSI,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   // false);

	// Set pin direction to output
	// gpio_set_pin_direction(MOSI, GPIO_DIRECTION_OUT);

	// gpio_set_pin_function(MOSI, PINMUX_PA04D_SERCOM0_PAD0);

	// gpio_set_pin_level(SCK,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   // false);

	// Set pin direction to output
	// gpio_set_pin_direction(SCK, GPIO_DIRECTION_OUT);

	// gpio_set_pin_function(SCK, PINMUX_PA05D_SERCOM0_PAD1);

	// Set pin direction to input
	// gpio_set_pin_direction(MISO, GPIO_DIRECTION_IN);

	// gpio_set_pin_pull_mode(MISO,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       // GPIO_PULL_OFF);

	// gpio_set_pin_function(MISO, PINMUX_PA07D_SERCOM0_PAD3);
// }

// void SPI_ADC_CLOCK_init(void)
// {
	// hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM0_GCLK_ID_CORE, CONF_GCLK_SERCOM0_CORE_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	// hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM0_GCLK_ID_SLOW, CONF_GCLK_SERCOM0_SLOW_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	// hri_mclk_set_APBCMASK_SERCOM0_bit(MCLK);
// }

// void SPI_ADC_init(void)
// {
	// SPI_ADC_CLOCK_init();
	// spi_m_async_init(&SPI_ADC, SERCOM0);
	// SPI_ADC_PORT_init();
// }

void I2C_0_PORT_init(void)
{

	gpio_set_pin_pull_mode(UART_TX,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(UART_TX, PINMUX_PA16C_SERCOM1_PAD0);

	gpio_set_pin_pull_mode(UART_RX,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(UART_RX, PINMUX_PA17C_SERCOM1_PAD1);
}

void I2C_0_CLOCK_init(void)
{
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM1_GCLK_ID_CORE, CONF_GCLK_SERCOM1_CORE_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM1_GCLK_ID_SLOW, CONF_GCLK_SERCOM1_SLOW_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_mclk_set_APBCMASK_SERCOM1_bit(MCLK);
}

void I2C_0_init(void)
{
	I2C_0_CLOCK_init();
	i2c_m_sync_init(&I2C_0, SERCOM1);
	I2C_0_PORT_init();
}

/**
 * \brief USART Clock initialization function
 *
 * Enables register interface and peripheral clock
 */
// void USART_CTRL_CLOCK_init()
// {

	// hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM3_GCLK_ID_CORE, CONF_GCLK_SERCOM3_CORE_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	// hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM3_GCLK_ID_SLOW, CONF_GCLK_SERCOM3_SLOW_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	// hri_mclk_set_APBCMASK_SERCOM3_bit(MCLK);
// }

/**
 * \brief USART pinmux initialization function
 *
 * Set each required pin to USART functionality
 */
// void USART_CTRL_PORT_init()
// {
	// gpio_set_pin_function(FTDI_TX, PINMUX_PA24C_SERCOM3_PAD2);
	// gpio_set_pin_function(FTDI_RX, PINMUX_PA25C_SERCOM3_PAD3);
// }

/**
 * \brief USART initialization function
 *
 * Enables USART peripheral, clocks and initializes USART driver
 */
// void USART_CTRL_init(void)
// {
	// USART_CTRL_CLOCK_init();
	// usart_async_init(&USART_CTRL, SERCOM3, USART_CTRL_buffer, USART_CTRL_BUFFER_SIZE, (void *)NULL);
	// USART_CTRL_PORT_init();
// }

void system_init(void)
{
	init_mcu();

	// GPIO on PA02

	// gpio_set_pin_level(CS0,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   // true);

	// Set pin direction to output
	// gpio_set_pin_direction(CS0, GPIO_DIRECTION_OUT);

	// gpio_set_pin_function(CS0, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA03

	// gpio_set_pin_level(DACSYNC,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   // true);

	// Set pin direction to output
	// gpio_set_pin_direction(DACSYNC, GPIO_DIRECTION_OUT);

	// gpio_set_pin_function(DACSYNC, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA06

	// gpio_set_pin_level(CS1,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   // true);

	// Set pin direction to output
	// gpio_set_pin_direction(CS1, GPIO_DIRECTION_OUT);

	// gpio_set_pin_function(CS1, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA08

	// Set pin direction to input
	gpio_set_pin_direction(ADR0, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(ADR0,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(ADR0, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA09

	// Set pin direction to input
	gpio_set_pin_direction(ADR1, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(ADR1,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(ADR1, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA10

	gpio_set_pin_level(CLOSE0,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(CLOSE0, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(CLOSE0, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA11

	gpio_set_pin_level(OPEN0,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(OPEN0, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(OPEN0, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA14

	gpio_set_pin_level(CLOSE1,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(CLOSE1, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(CLOSE1, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA15

	gpio_set_pin_level(OPEN1,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(OPEN1, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(OPEN1, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA18

	gpio_set_pin_level(CLOSE2,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(CLOSE2, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(CLOSE2, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA19

	gpio_set_pin_level(OPEN2,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(OPEN2, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(OPEN2, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA22

	gpio_set_pin_level(CLOSE3,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(CLOSE3, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(CLOSE3, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA23

	gpio_set_pin_level(OPEN3,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(OPEN3, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(OPEN3, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA27

	// Set pin direction to input
	gpio_set_pin_direction(ADR3, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(ADR3,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(ADR3, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA28

	gpio_set_pin_level(LED,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   true);

	// Set pin direction to output
	gpio_set_pin_direction(LED, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(LED, GPIO_PIN_FUNCTION_OFF);

	// SPI_ADC_init();

	I2C_0_init();
	// USART_CTRL_init();
}
