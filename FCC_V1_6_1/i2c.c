/** @file i2c.c */
#include <peripheral_clk_config.h>
#include <utils.h>
#include <hal_init.h>
#include <hal_i2c_m_async.h>
#include "atmel_start_pins.h"
#include "i2c.h"
#include "subbus.h"
#include "crc8nrsc_5_byte.h"

static struct i2c_m_async_desc I2C_0;
static bool i2c_enabled = I2C_ENABLE_DEFAULT;
static struct io_descriptor *I2C_0_io;
static volatile bool I2C_txfr_complete = true;
static volatile bool I2C_error_seen = false;
static volatile int32_t I2C_error = I2C_OK;
static volatile uint8_t ts_ov_status = 0;
static uint8_t ts_ibuf[4];
#define TS_OVERFLOW 1
#define TS_UNDERFLOW 2
static uint8_t I2C_intflag = 0;
static uint16_t I2C_status = 0;


/**
 * These addresses belong to the I2C module
 * 0x20 RW: TS0_Count
 * 0x21 R:  TS0_Address
 * 0x22 R:  TS0_Raw_LSW
 * 0x23 R:  TS0_Raw_MSW
 * 0x24 R:  ts_state, i2c_error
 * 0x25 R:  SHT31 Status
 * 0x26 R:  SHT31 Temperature
 * 0x27 R:  SHT31 Relative Humidity
 * 0x28 R:  sht31_state, i2c_error
 */
static subbus_cache_word_t i2c_cache[I2C_HIGH_ADDR-I2C_BASE_ADDR+1] = {
  { I2C_TS_ID_DEFAULT, 0, true,  false,  true, false }, // Offset 0: RW: TS0_Address
  { 0, 0, true,  false, false, false, false }, // Offset 1: R:  TS0_Count
  { 0, 0, true,  false, false, false, false }, // Offset 2: R:  TS0_Raw_LSW
  { 0, 0, true,  false, false, false, false }, // Offset 3: R:  TS0_Raw_MSW
  { 0, 0, true,  false, false, false, false }, // Offset 4: R:  ts_state, i2c_error
  { 0, 0, true,  false, false, false, false }, // Offset 5: RW: SHT31_Status
  { 0, 0, true,  false, false, false, false }, // Offset 6: R:  SHT31_Temperature
  { 0, 0, true,  false, false, false, false }, // Offset 7: R:  SHT31_Relative_Humidity
  { 0, 0, true,  false, false, false, false }, // Offset 8: R:  sht31_state, i2c_error
  { 0, 0, true,  false, false, false, false }, // Offset 9: R:  I2C error data
  { 0, 0, true,  false, false, false, false }, // Offset 10: R: I2C intflag
  { 0, 0, true,  false, false, false, false }  // Offset 11: R: I2C status
};

static int16_t ts_get_slave_addr(void) {
  switch (i2c_cache[0].cache) {
    case 1: return 0x14;
    case 2: return 0x15;
    case 3: return 0x17;
    case 4: return 0x24;
    case 5: return 0x26;
    case 6: return 0x27;
    default: return 0x14; // Invalid, default to 1
  }
}

enum ts_state_t {ts_init, ts_init_tx, ts_read_adc, ts_read_adc_tx };
static enum ts_state_t ts_state = ts_init;

static void  ts_record_i2c_error(enum ts_state_t ts_state, int32_t I2C_error) {
  uint16_t word = ((ts_state & 0x7) << 4) | ((-I2C_error) & 0xF);
  i2c_cache[4].cache = (i2c_cache[4].cache & 0xFF00) | word;
}

static void ts_record_ov_status(uint8_t ovs) {
  i2c_cache[4].cache = (i2c_cache[4].cache & 0xFCFF) | ((ovs & 3) << 8);
}

/**
 * @return true if the bus is free and available for another device
 */
static bool ts_poll(void) {
  static int nack_limit = 2047;
  static int n_readings = 0;
  static int64_t sum = 0;
  static bool read_observed = false;

  if (!read_observed && i2c_cache[1].was_read) {
    read_observed = true;
    n_readings = 0;
    sum = 0;
  }
  if (i2c_cache[1].was_read && i2c_cache[2].was_read && i2c_cache[3].was_read) {
    int32_t scaled = n_readings ?
      sum * 2 / n_readings : 0;
    read_observed = false;
    i2c_cache[1].cache = n_readings;
    i2c_cache[2].cache = scaled & 0xFFFF;
    i2c_cache[3].cache = (scaled >> 16) & 0xFFFF;
    i2c_cache[1].was_read = i2c_cache[2].was_read = i2c_cache[3].was_read = false;
  }
  if (i2c_cache[4].was_read) {
    ts_ov_status = 0;
    ts_record_ov_status(ts_ov_status);
    i2c_cache[4].was_read = false;
    ts_record_i2c_error(ts_init, I2C_OK);
   }

  switch (ts_state) {
    case ts_init:
      if (i2c_cache[0].written) {
        uint16_t newaddr = i2c_cache[0].wvalue;
        if (newaddr >= 1 && newaddr <= 6) {
          i2c_cache[0].cache = newaddr;
        }
        i2c_cache[0].written = false;
      }
      n_readings = 0;
      sum = 0;
      i2c_write(ts_get_slave_addr(), (const uint8_t *)"\x04", 1);
      ts_state = ts_init_tx;
      return false;
    case ts_init_tx:
      if (I2C_error_seen) {
        I2C_error_seen = false;
        if (I2C_error == I2C_NACK) {
          // This is sort of expected
        } else {
          // record the error in the I2C status
          ts_record_i2c_error(ts_state, I2C_error);
        }
        ts_state = ts_init;
      } else if (i2c_cache[0].written) {
        ts_state = ts_init;
      } else {
        ts_state = ts_read_adc;
      }
      return true;
    case ts_read_adc:
      if (i2c_cache[0].written) {
        ts_state = ts_init;
      } else {
        nack_limit = 2047;
        i2c_read(ts_get_slave_addr(), (uint8_t *)ts_ibuf, 4);
        ts_state = ts_read_adc_tx;
      }
      return false;
    case ts_read_adc_tx:
      if (i2c_cache[0].written) {
        ts_state = ts_init;
      } else if (I2C_error_seen) {
        I2C_error_seen = false;
        if (I2C_error == I2C_NACK) {
          if (--nack_limit <= 0) {
            ts_record_i2c_error(ts_state, I2C_NACK);
            ts_state = ts_init;
          } else {
            ts_state = ts_read_adc;
          }
        } else {
          ts_record_i2c_error(ts_state, I2C_error);
          ts_state = ts_init;
        }
      } else {
        // Process the data we've read.
        uint8_t tbits = ts_ibuf[0] & 0xE0;
        if (tbits == 0xC0) {
          ts_ov_status |= TS_OVERFLOW;
          ts_record_ov_status(ts_ov_status);
        } else if (tbits == 0x20) {
          ts_ov_status |= TS_UNDERFLOW;
          ts_record_ov_status(ts_ov_status);
        } else {
          int32_t acc = 0;
          if (ts_ibuf[0] & 0x40) {
            acc = 0xFFFFFF80 | (ts_ibuf[0] & 0x7F);
          } else {
            acc = ts_ibuf[0];
          }
          for (int i = 1; i < 4; ++i) {
            acc = (acc << 8) | ts_ibuf[i];
          }
          ++n_readings;
          sum += acc;
        }
        if (!read_observed) {
          i2c_cache[1].cache = n_readings;
          int32_t scaled = n_readings ?
            sum * 2 / n_readings : 0;
          i2c_cache[2].cache = scaled & 0xFFFF;
          i2c_cache[3].cache = (scaled >> 16) & 0xFFFF;
        }

        ts_state = ts_read_adc;
      }
      return true;
    default:
      assert(false, __FILE__, __LINE__);
  }
  return true; // will never actually get here
}

enum sht_state_t {sht_init, sht_init_tx,
    sht_issue_read_status, sht_issue_read_status_tx, sht_read_status_tx,
    sht_issue_convert_trh, sht_issue_convert_trh_tx, sht_read_trh_tx,
    sht_issue_clear_status, sht_issue_clear_status_tx };
static enum sht_state_t sht_state = sht_init;
#define SHT_SLAVE_ADDR 0x44
static uint8_t sht_ibuf[6];

static void sht_record_i2c_error(enum sht_state_t sht_state, int32_t I2C_error) {
  uint16_t word = ((sht_state & 0xF) << 4) | ((-I2C_error) & 0xF);
  i2c_cache[8].cache = (i2c_cache[8].cache & 0xFF00) | word;
}

static void sht_record_crc_error(uint16_t bit) {
  if (bit)
    i2c_cache[8].cache |= (bit << 8);
  else
    i2c_cache[8].cache &= 0x00FF;
}

/**
 * @param MSB The first byte transmitted
 * @param LSB The second byte transmitted
 * @param CRC The third byte transmitted, which should be the CRC
 * @return true if CRC checks out
 *
 * Calculates the CRC of the first two bytes and compares to the
 * provided CRC.
 *
 * This version does not actually check and returns true
 * until I can find a reasonably licensed version.
 */
bool 	sht_crc8(uint8_t *ibuf) {
  return (crc8nrsc_5_byte(ibuf, 2) == ibuf[2]);
}

/**
 * @return true if the bus is free and available for another device
 */
static bool sht31_poll(void) {
  if (i2c_cache[8].was_read) {
    sht_record_crc_error(0);
    sht_record_i2c_error(sht_init, I2C_OK);
    i2c_cache[8].was_read = false;
  }
  switch (sht_state) {
    case sht_init:
      i2c_write(SHT_SLAVE_ADDR, (const uint8_t *)"\x30\xA2", 2);
      sht_state = sht_init_tx;
      return false;
    case sht_init_tx:
      if (I2C_error_seen) {
        I2C_error_seen = false;
        sht_record_i2c_error(sht_state, I2C_error);
        sht_state = sht_init;
      } else {
        sht_state = sht_issue_read_status;
      }
      return true;
    case sht_issue_read_status:
      if (I2C_error_seen) {
        I2C_error_seen = false;
        sht_record_i2c_error(sht_state, I2C_error);
      }
      i2c_write(SHT_SLAVE_ADDR, (const uint8_t *)"\xF3\x2D", 2);
      sht_state = sht_issue_read_status_tx;
      return false;
    case sht_issue_read_status_tx:
      if (I2C_error_seen) {
        I2C_error_seen = false;
        sht_record_i2c_error(sht_state, I2C_error);
        sht_state = sht_issue_read_status;
        // Do we need to issue a stop?
        return true;
      }
      i2c_read(SHT_SLAVE_ADDR, sht_ibuf, 3);
      sht_state = sht_read_status_tx;
      return false;
    case sht_read_status_tx:
      if (I2C_error_seen) {
        I2C_error_seen = false;
        sht_record_i2c_error(sht_state, I2C_error);
        sht_state = sht_issue_read_status_tx;
        // Do we need to issue a stop?
        return true;
      }
      if (sht_crc8(sht_ibuf)) {
        i2c_cache[5].cache = (sht_ibuf[0] << 8) | sht_ibuf[1];
      } else {
        sht_record_crc_error(1);
      }
      sht_state = sht_issue_convert_trh;
      return true;
    case sht_issue_convert_trh:
      if (I2C_error_seen) {
        I2C_error_seen = false;
        sht_record_i2c_error(sht_state, I2C_error);
      }
      i2c_write(SHT_SLAVE_ADDR, (const uint8_t *)"\x24\x00", 2);
      sht_state = sht_issue_convert_trh_tx;
      return false;
    case sht_issue_convert_trh_tx:
      if (I2C_error_seen) {
        I2C_error_seen = false;
        sht_record_i2c_error(sht_state, I2C_error);
        sht_state = sht_issue_convert_trh;
        // Do we need to issue a stop?
        return true;
      }
      i2c_read(SHT_SLAVE_ADDR, sht_ibuf, 6);
      sht_state = sht_read_trh_tx;
      return false;
    case sht_read_trh_tx:
      if (I2C_error_seen) {
        I2C_error_seen = false;
        if (I2C_error == I2C_NACK) {
          sht_state = sht_issue_convert_trh_tx;
        } else {
          sht_record_i2c_error(sht_state, I2C_error);
          sht_state = sht_init;
        }
      } else {
        // process TRH data
        if (sht_crc8(sht_ibuf)) {
          i2c_cache[6].cache = (sht_ibuf[0] << 8) | sht_ibuf[1];
        } else {
          sht_record_crc_error(2);
        }
        if (sht_crc8(sht_ibuf)) {
          i2c_cache[7].cache = (sht_ibuf[3] << 8) | sht_ibuf[4];
        } else {
          sht_record_crc_error(4);
        }
        if (i2c_cache[5].was_read) {
          sht_state = sht_issue_clear_status;
        } else {
          sht_state = sht_issue_read_status;
        }
      }
      return true;
    case sht_issue_clear_status:
      if (I2C_error_seen) {
        I2C_error_seen = false;
        sht_record_i2c_error(sht_state, I2C_error);
      }
      i2c_write(SHT_SLAVE_ADDR, (const uint8_t *)"\x30\x41", 2);
      sht_state = sht_issue_clear_status_tx;
      return false;
    case sht_issue_clear_status_tx:
      if (I2C_error_seen) {
        I2C_error_seen = false;
        sht_record_i2c_error(sht_state, I2C_error);
        sht_state = sht_issue_clear_status;
      } else {
        sht_state = sht_issue_read_status;
      }
      return true;
    default:
      assert(false, __FILE__, __LINE__);
  }
  return true;
}

enum i2c_state_t {i2c_ts, i2c_sht31 };
static enum i2c_state_t i2c_state = i2c_ts;
static enum i2c_state_t i2c_tx_state;
static enum ts_state_t ts_tx_state;
static enum sht_state_t sht_tx_state;

static void i2c_record_tx_start() {
  uint16_t word = 0;
  i2c_tx_state = i2c_state;
  ts_tx_state = ts_state;
  sht_tx_state = sht_state;
  word = i2c_tx_state << 7;
  word |= (ts_tx_state & 7) << 4;
  word |= (sht_tx_state & 0xF);
  i2c_cache[9].cache = word;
}

void i2c_write(int16_t i2c_addr, const uint8_t *obuf, int16_t nbytes) {
  assert(I2C_txfr_complete, __FILE__, __LINE__);
  i2c_record_tx_start();
  I2C_txfr_complete = false;
  i2c_m_async_set_slaveaddr(&I2C_0, i2c_addr, I2C_M_SEVEN);
  io_write(I2C_0_io, obuf, nbytes);
}

void i2c_read(int16_t i2c_addr, uint8_t *ibuf, int16_t nbytes) {
  assert(I2C_txfr_complete, __FILE__, __LINE__);
  i2c_record_tx_start();
  I2C_txfr_complete = false;
  i2c_m_async_set_slaveaddr(&I2C_0, i2c_addr, I2C_M_SEVEN);
  io_read(I2C_0_io, ibuf, nbytes);
}

void i2c_enable(bool value) {
  i2c_enabled = value;
}

static void I2C_0_PORT_init(void) {
  gpio_set_pin_pull_mode(SDA, GPIO_PULL_OFF);
  gpio_set_pin_function(SDA, PINMUX_PA16C_SERCOM1_PAD0);
  gpio_set_pin_pull_mode(SCL, GPIO_PULL_OFF);
  gpio_set_pin_function(SCL, PINMUX_PA17C_SERCOM1_PAD1);
}

static void I2C_0_CLOCK_init(void) {
  hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM1_GCLK_ID_CORE, CONF_GCLK_SERCOM1_CORE_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
  hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM1_GCLK_ID_SLOW, CONF_GCLK_SERCOM1_SLOW_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
  hri_mclk_set_APBCMASK_SERCOM1_bit(MCLK);
}

#define I2C_INTFLAG_ERROR (1<<7)

/************************************************************************/
/* Possible error values:                                               */
/* I2C_OK 0                      Operation successful                   */
/* I2C_ACK -1                    Received ACK from device on I2C bus    */
/* I2C_NACK -2                   Received NACK from device on I2C bus   */
/* I2C_ERR_ARBLOST -3            Arbitration lost                       */
/* I2C_ERR_BAD_ADDRESS -4        Bad address                            */
/* I2C_ERR_BUS -5                Bus error                              */
/* I2C_ERR_BUSY -6               Device busy                            */
/* I2c_ERR_PACKAGE_COLLISION -7  Package collision                      */
/************************************************************************/
static void I2C_0_async_error(struct i2c_m_async_desc *const i2c, int32_t error) {
  I2C_txfr_complete = true;
  I2C_error_seen = true;
  I2C_error = error;
  I2C_intflag = hri_sercomi2cm_read_INTFLAG_reg(i2c->device.hw);
  I2C_status = hri_sercomi2cm_read_STATUS_reg(i2c->device.hw);
  i2c_cache[10].cache = I2C_intflag;
  i2c_cache[11].cache = I2C_status;
  if (error == I2C_ERR_BUS) {
    hri_sercomi2cm_write_STATUS_reg(I2C_0.device.hw, SERCOM_I2CM_STATUS_BUSERR);
    hri_sercomi2cm_clear_INTFLAG_reg(I2C_0.device.hw, I2C_INTFLAG_ERROR);
  }
}

static void I2C_0_txfr_complete(struct i2c_m_async_desc *const i2c) {
  I2C_txfr_complete = true;
}

static void I2C_0_init(void) {
  I2C_0_CLOCK_init();
  i2c_m_async_init(&I2C_0, SERCOM1);
  I2C_0_PORT_init();
  i2c_m_async_get_io_descriptor(&I2C_0, &I2C_0_io);
  i2c_m_async_enable(&I2C_0);
  i2c_m_async_register_callback(&I2C_0, I2C_M_ASYNC_ERROR, (FUNC_PTR)I2C_0_async_error);
  i2c_m_async_register_callback(&I2C_0, I2C_M_ASYNC_TX_COMPLETE, (FUNC_PTR)I2C_0_txfr_complete);
  i2c_m_async_register_callback(&I2C_0, I2C_M_ASYNC_RX_COMPLETE, (FUNC_PTR)I2C_0_txfr_complete);
}

static void i2c_reset() {
  I2C_0_init();
  I2C_txfr_complete = true;
  ts_state = ts_init;
  sht_state = sht_init;
}

// static void i2c_may_use() {
  // i2c_m_async_set_slaveaddr(&I2C_0, 0x12, I2C_M_SEVEN);
  // io_write(I2C_0_io, I2C_0_example_str, 12);
// }

void i2c_poll(void) {
  enum i2c_state_t input_state = i2c_state;
  if (i2c_cache[11].written) {
    i2c_cache[11].written = false;
    i2c_reset();
  }
  while (i2c_enabled && I2C_txfr_complete) {
    switch (i2c_state) {
      case i2c_ts:
        if (ts_poll()) {
          i2c_state = i2c_sht31;
        }
        break;
      case i2c_sht31:
        if (sht31_poll()) {
          i2c_state = i2c_ts;
        }
        break;
      default:
        assert(false, __FILE__, __LINE__);
    }
    if (i2c_state == input_state) break;
  }
}

subbus_driver_t sb_i2c = {
  I2C_BASE_ADDR, I2C_HIGH_ADDR, // address range
  i2c_cache,
  i2c_reset,
  i2c_poll,
  false
};
