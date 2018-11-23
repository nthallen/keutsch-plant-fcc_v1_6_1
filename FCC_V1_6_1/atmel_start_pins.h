/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */
#ifndef ATMEL_START_PINS_H_INCLUDED
#define ATMEL_START_PINS_H_INCLUDED

#include <hal_gpio.h>

// SAMC21 has 9 pin functions

#define GPIO_PIN_FUNCTION_A 0
#define GPIO_PIN_FUNCTION_B 1
#define GPIO_PIN_FUNCTION_C 2
#define GPIO_PIN_FUNCTION_D 3
#define GPIO_PIN_FUNCTION_E 4
#define GPIO_PIN_FUNCTION_F 5
#define GPIO_PIN_FUNCTION_G 6
#define GPIO_PIN_FUNCTION_H 7
#define GPIO_PIN_FUNCTION_I 8

#define CS0 GPIO(GPIO_PORTA, 2)
#define DACSYNC GPIO(GPIO_PORTA, 3)
#define MOSI GPIO(GPIO_PORTA, 4)
#define SCK GPIO(GPIO_PORTA, 5)
#define CS1 GPIO(GPIO_PORTA, 6)
#define MISO GPIO(GPIO_PORTA, 7)
#define ADR0 GPIO(GPIO_PORTA, 8)
#define ADR1 GPIO(GPIO_PORTA, 9)
#define CLOSE0 GPIO(GPIO_PORTA, 10)
#define OPEN0 GPIO(GPIO_PORTA, 11)
#define CLOSE1 GPIO(GPIO_PORTA, 14)
#define OPEN1 GPIO(GPIO_PORTA, 15)
#define SDA GPIO(GPIO_PORTA, 16)
#define SCL GPIO(GPIO_PORTA, 17)
#define CLOSE2 GPIO(GPIO_PORTA, 18)
#define OPEN2 GPIO(GPIO_PORTA, 19)
#define CLOSE3 GPIO(GPIO_PORTA, 22)
#define OPEN3 GPIO(GPIO_PORTA, 23)
#define FTDI_TX GPIO(GPIO_PORTA, 24)
#define FTDI_RX GPIO(GPIO_PORTA, 25)
#define ADR3 GPIO(GPIO_PORTA, 27)
#define LED GPIO(GPIO_PORTA, 28)

#endif // ATMEL_START_PINS_H_INCLUDED
