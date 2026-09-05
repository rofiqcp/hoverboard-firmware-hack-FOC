#ifndef FLASH_UPDATE_F103_H_
#define FLASH_UPDATE_F103_H_
#include <stdbool.h>
#include <stdint.h>

bool f103_fw_erase_staging(uint32_t fw_size);
bool f103_fw_write_staging(uint32_t offset, const uint8_t *data, uint32_t len);
void f103_fw_reset_to_bootloader(void);
#endif
