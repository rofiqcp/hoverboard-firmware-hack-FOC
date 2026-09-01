#include "crc.h"
uint16_t vesc_crc16(const uint8_t *data, uint32_t len) {
    uint16_t crc = 0u;
    for (uint32_t i = 0u; i < len; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t bit = 0u; bit < 8u; ++bit) {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}
