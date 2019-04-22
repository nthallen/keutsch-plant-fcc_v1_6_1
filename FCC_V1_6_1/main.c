#include <atmel_start.h>
#include "usart.h"
#include "subbus.h"
#include "control.h"
#include "spi.h"
#include "commands.h"
#include "i2c.h"

int main(void) {
	atmel_start_init();
  spi_enable(true);
  i2c_enable(I2C_ENABLE_DEFAULT);
  if (subbus_add_driver(&sb_base) ||
      subbus_add_driver(&sb_fail_sw) ||
      subbus_add_driver(&sb_board_desc) ||
      subbus_add_driver(&sb_spi) ||
      subbus_add_driver(&sb_cmd) ||
      subbus_add_driver(&sb_i2c)) {
    while (true) ; // some driver is misconfigured.
  }
  subbus_reset();
  uart_init();
  while (1) {
    poll_control();
    subbus_poll();
    #if SUBBUS_INTERRUPTS
    if (subbus_intr_req)
      intr_service();
    #endif
    // possibly check for watchdog features
    // delay_ms(500);
    // gpio_set_pin_level(SPR7, true);
    // delay_ms(500);
    // gpio_set_pin_level(SPR7, false);
    // uart_send_char('Z');
    // uart_flush_output();
  }
}
