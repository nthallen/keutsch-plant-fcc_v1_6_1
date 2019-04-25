/** @file serial_num.h
 * This file must define:
 *  CAN_BOARD_ID: The CAN Identifier for the board. This must be unique on a specific CAN Bus
 *  SUBBUS_BOARD_SN: The serial number of this board among boards of the same SUBBUS_BOARD_TYPE
 *  SUBBUS_BUILD_NUM:
 *  SUBBUS_BOARD_ID: Board Identification number (*not* the CAN_BOARD_ID). Defines the type of board
 *  SUBBUS_BOARD_INSTRUMENT_ID: Number that maps to Instrument name. (not yet used as of 4/18/19)
 *  SUBBUS_BOARD_SN:
 *  SUBBUS_BOARD_REV: String encapsulating almost anything here
 */
#ifndef SERIAL_NUM_H_INCLUDED
#define SERIAL_NUM_H_INCLUDED

// These parameters are common to all boards built with this code
#define SUBBUS_BOARD_ID 7
#define SUBBUS_BOARD_BOARD_TYPE "FCC"
#define SUBBUS_BOARD_BOARD_REV "Rev A"
#define SUBBUS_BOARD_FIRMWARE_REV "V1.0"
#define SUBBUS_BOARD_BUILD_NUM 3

/**
 * Build definitions
 * 3: Add software reset
 * 2: Add I2C support for SHT31 and Temp Sensor
 * 1:
 */
#if ! defined(SUBBUS_BOARD_SN)
#error Must define SUBBUS_BOARD_SN in Build Properties
#endif


#if SUBBUS_BOARD_SN == 1
#define SUBBUS_BOARD_INSTRUMENT "HCHO"
#define SUBBUS_BOARD_INSTRUMENT_ID 2
#endif
#if SUBBUS_BOARD_SN == 2
#define SUBBUS_BOARD_INSTRUMENT "HCHO"
#define SUBBUS_BOARD_INSTRUMENT_ID 2
#endif
#if SUBBUS_BOARD_SN == 3
#define SUBBUS_BOARD_INSTRUMENT "Plant Chamber"
#define SUBBUS_BOARD_INSTRUMENT_ID 3
#endif

#ifdef CAN_BOARD_ID

#define SUBBUS_BOARD_REV_STR(SN,ID) SUBBUS_BOARD_INSTRUMENT " " SUBBUS_BOARD_BOARD_TYPE " " \
SUBBUS_BOARD_BOARD_REV " " SUBBUS_BOARD_FIRMWARE_REV \
" S/N:" #SN " CAN ID:" #ID " " SUBBUS_BOARD_LOCATION
#define SUBBUS_BOARD_REV_XSTR(SUBBUS_BOARD_SN,CAN_BOARD_ID) SUBBUS_BOARD_REV_STR(SUBBUS_BOARD_SN,CAN_BOARD_ID)
#define SUBBUS_BOARD_REV SUBBUS_BOARD_REV_XSTR(SUBBUS_BOARD_SN,CAN_BOARD_ID)

#else

#define SUBBUS_BOARD_REV_STR(SN,ID) "V" #ID ":0:" SUBBUS_BOARD_INSTRUMENT " " SUBBUS_BOARD_BOARD_TYPE " " \
SUBBUS_BOARD_BOARD_REV " " SUBBUS_BOARD_FIRMWARE_REV \
" S/N:" #SN
#define SUBBUS_BOARD_REV_XSTR(SN,ID) SUBBUS_BOARD_REV_STR(SN,ID)
#define SUBBUS_BOARD_REV SUBBUS_BOARD_REV_XSTR(SUBBUS_BOARD_SN,SUBBUS_BOARD_ID)

#endif

#endif
