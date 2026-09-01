#ifndef VESC_CRC_H_
#define VESC_CRC_H_
#include <stdint.h>
uint16_t vesc_crc16(const uint8_t *data, uint32_t len);
#endif
