#ifndef VESC_PROTOCOL_H_
#define VESC_PROTOCOL_H_
#include <stdint.h>
#include <stdbool.h>
void vesc_protocol_init(void);
bool vesc_protocol_rx_byte(uint8_t byte);
bool vesc_protocol_rx_in_progress(void);
void vesc_protocol_process_pending(void);
bool vesc_protocol_link_active(void);
uint32_t vesc_protocol_rx_ok_count(void);
uint32_t vesc_protocol_rx_crc_error_count(void);
#endif
