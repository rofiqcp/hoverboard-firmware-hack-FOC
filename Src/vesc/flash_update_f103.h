#ifndef FLASH_UPDATE_F103_H_
#define FLASH_UPDATE_F103_H_
#include <stdbool.h>
#include <stdint.h>

bool f103_fw_erase_staging(uint32_t fw_size);
bool f103_fw_write_staging(uint32_t offset, const uint8_t *data, uint32_t len);
bool f103_fw_stage_is_valid(uint32_t *size_out, uint16_t *crc_out);
bool f103_fw_mark_pending_or_recovery(void);
void f103_fw_reset_to_bootloader(void);
#endif
