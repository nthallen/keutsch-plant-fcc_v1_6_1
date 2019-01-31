#ifndef CRC8NRSC_5_BYTE_H_INCLUDED
#define CRC8NRSC_5_BYTE_H_INCLUDED

#include <stddef.h>

/**
 * Generated using crcany https://github.com/madler/crcany
 * width=8 poly=0x31 init=0xff refin=false refout=false xorout=0x00
 *    check=0xf7 residue=0x00 name="CRC-8/NRSC-5"
 */
unsigned crc8nrsc_5_byte(unsigned char const *data, size_t len);

#endif
