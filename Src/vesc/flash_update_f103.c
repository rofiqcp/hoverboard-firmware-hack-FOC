#include "flash_update_f103.h"
#include "f103_boot_layout.h"
#include "crc.h"
#include "motor/mcpwm_foc.h"
#include "defines.h"
#include "stm32f1xx_hal.h"
#include <string.h>

static uint32_t be32(const uint8_t *p) {
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}
static uint16_t be16(const uint8_t *p) { return (uint16_t)(((uint16_t)p[0] << 8) | p[1]); }

static void release_both(void) {
    mcpwm_foc_release_motor(false);
    mcpwm_foc_release_motor(true);
    LEFT_TIM->BDTR &= ~TIM_BDTR_MOE;
    RIGHT_TIM->BDTR &= ~TIM_BDTR_MOE;
    HAL_Delay(20);
}

static bool erase_pages(uint32_t base, uint32_t bytes) {
    if ((base & (F103_FLASH_PAGE_SIZE - 1u)) != 0u || bytes == 0u) return false;
    FLASH_EraseInitTypeDef e = {0};
    uint32_t page_error = 0u;
    e.TypeErase = FLASH_TYPEERASE_PAGES;
    e.PageAddress = base;
    e.NbPages = (bytes + F103_FLASH_PAGE_SIZE - 1u) / F103_FLASH_PAGE_SIZE;
    HAL_FLASH_Unlock();
    const HAL_StatusTypeDef st = HAL_FLASHEx_Erase(&e, &page_error);
    HAL_FLASH_Lock();
    return st == HAL_OK && page_error == 0xFFFFFFFFu;
}

static bool program_halfwords(uint32_t base, const uint8_t *data, uint32_t len) {
    if (!data || (base & 1u)) return false;
    HAL_FLASH_Unlock();
    for (uint32_t i = 0; i < len; i += 2u) {
        uint16_t hw = data[i];
        if (i + 1u < len) hw |= (uint16_t)((uint16_t)data[i + 1u] << 8);
        else hw |= 0xFF00u;
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, base + i, hw) != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }
    }
    HAL_FLASH_Lock();
    return memcmp((const void *)base, data, len) == 0;
}

bool f103_fw_erase_staging(uint32_t fw_size) {
    if (fw_size == 0u || fw_size > F103_MAX_FW_IMAGE_SIZE) return false;
    release_both();
    if (!erase_pages(F103_STAGE_BASE_ADDR, F103_STAGE_REGION_SIZE)) return false;
    return erase_pages(F103_META_BASE_ADDR, F103_META_REGION_SIZE);
}

bool f103_fw_write_staging(uint32_t offset, const uint8_t *data, uint32_t len) {
    if (!data || len == 0u || (offset & 1u) != 0u) return false;
    if (offset > F103_STAGE_REGION_SIZE || len > F103_STAGE_REGION_SIZE - offset) return false;
    release_both();
    return program_halfwords(F103_STAGE_BASE_ADDR + offset, data, len);
}

bool f103_fw_stage_is_valid(uint32_t *size_out, uint16_t *crc_out) {
    const uint8_t *stage = (const uint8_t *)F103_STAGE_BASE_ADDR;
    const uint32_t size = be32(stage);
    const uint16_t crc = be16(stage + 4u);
    if (size == 0u || size > F103_MAX_FW_IMAGE_SIZE) return false;
    if (vesc_crc16(stage + F103_VESC_IMAGE_HEADER_SIZE, size) != crc) return false;
    if (size_out) *size_out = size;
    if (crc_out) *crc_out = crc;
    return true;
}

static bool write_meta(uint32_t state, uint32_t size, uint16_t crc) {
    f103_update_meta_t m;
    m.magic = F103_UPDATE_META_MAGIC;
    m.state = state;
    m.size = size;
    m.size_inv = ~size;
    m.crc16 = crc;
    m.crc16_inv = (uint16_t)~crc;
    m.version = F103_UPDATE_META_VERSION;
    m.version_inv = (uint16_t)~F103_UPDATE_META_VERSION;
    if (!erase_pages(F103_META_BASE_ADDR, F103_META_REGION_SIZE)) return false;
    return program_halfwords(F103_META_BASE_ADDR, (const uint8_t *)&m, (uint32_t)sizeof(m));
}

bool f103_fw_mark_pending_or_recovery(void) {
    release_both();
    uint32_t size = 0u;
    uint16_t crc = 0u;
    if (f103_fw_stage_is_valid(&size, &crc)) return write_meta(F103_UPDATE_STATE_PENDING, size, crc);
    return write_meta(F103_UPDATE_STATE_RECOVERY, 0u, 0u);
}

void f103_fw_reset_to_bootloader(void) {
    release_both();
    NVIC_SystemReset();
    for (;;) { }
}
