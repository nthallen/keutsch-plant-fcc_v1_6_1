#include <atmel_start.h>
#include "usart.h"
#include "subbus.h"
#include "control.h"
#include "spi.h"

int main(void) {
	atmel_start_init();
  spi_enable(true);
  if (subbus_add_driver(&sb_base) ||
      subbus_add_driver(&sb_fail_sw) ||
      subbus_add_driver(&sb_spi)) {
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
