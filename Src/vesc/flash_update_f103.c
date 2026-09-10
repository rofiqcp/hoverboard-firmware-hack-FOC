#include "flash_update_f103.h"
#include "f103_boot_layout.h"
#include "motor/mcpwm_foc.h"
#include "defines.h"
#include "platform_watchdog.h"
#include "stm32f1xx_hal.h"
#include <string.h>

static bool stage_session_active = false;
static uint32_t stage_session_total = 0u;

static uint16_t crc16(const uint8_t *data, uint32_t len) {
    uint16_t crc = 0u;
    for (uint32_t i = 0u; i < len; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t b = 0u; b < 8u; ++b) {
            crc = (crc & 0x8000u) ?
                (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
        }
        if ((i & 0x3FFu) == 0u) platform_watchdog_maintenance_kick();
    }
    return crc;
}

static uint32_t be32(const uint8_t *p) {
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}
static uint16_t be16(const uint8_t *p) {
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static void release_both(void) {
    mcpwm_foc_release_motor(false);
    mcpwm_foc_release_motor(true);
    LEFT_TIM->BDTR &= ~TIM_BDTR_MOE;
    RIGHT_TIM->BDTR &= ~TIM_BDTR_MOE;
}

static bool page_erased(uint32_t address) {
    for (uint32_t off = 0u; off < F103_FLASH_PAGE_SIZE; off += 4u) {
        if (*(volatile const uint32_t *)(address + off) != 0xFFFFFFFFu) return false;
    }
    return true;
}

static bool erase_pages(uint32_t base, uint32_t bytes) {
    if ((base & (F103_FLASH_PAGE_SIZE - 1u)) != 0u || bytes == 0u) return false;
    const uint32_t pages = (bytes + F103_FLASH_PAGE_SIZE - 1u) /
                           F103_FLASH_PAGE_SIZE;
    HAL_FLASH_Unlock();
    for (uint32_t page = 0u; page < pages; ++page) {
        FLASH_EraseInitTypeDef e = {0};
        uint32_t page_error = 0u;
        e.TypeErase = FLASH_TYPEERASE_PAGES;
        e.PageAddress = base + page * F103_FLASH_PAGE_SIZE;
        e.NbPages = 1u;        platform_watchdog_maintenance_kick();
        const HAL_StatusTypeDef st = HAL_FLASHEx_Erase(&e, &page_error);
        if (st != HAL_OK || page_error != 0xFFFFFFFFu || !page_erased(e.PageAddress)) {
            HAL_FLASH_Lock();
            return false;
        }
        platform_watchdog_maintenance_kick();
    }
    HAL_FLASH_Lock();
    return true;
}

static bool program_halfwords(uint32_t base, const uint8_t *data, uint32_t len) {
    if (!data || (base & 1u) != 0u) return false;
    HAL_FLASH_Unlock();
    for (uint32_t i = 0u; i < len; i += 2u) {
        uint16_t wanted = data[i];
        wanted |= (uint16_t)((i + 1u < len ? data[i + 1u] : 0xFFu) << 8);
        volatile const uint16_t *dst = (volatile const uint16_t *)(base + i);
        if (*dst == wanted) continue;
        if (*dst != 0xFFFFu ||
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, base + i, wanted) != HAL_OK ||
            *dst != wanted) {
            HAL_FLASH_Lock();
            return false;
        }
        if ((i & 0x7Fu) == 0u) platform_watchdog_maintenance_kick();
    }
    HAL_FLASH_Lock();
    return memcmp((const void *)base, data, len) == 0;
}
static bool staged_vector_valid(uint32_t size) {
    if (size < 8u || size > F103_MAX_FW_IMAGE_SIZE) return false;
    const uint32_t base = F103_STAGE_BASE_ADDR + F103_VESC_IMAGE_HEADER_SIZE;
    const uint32_t sp = *(const uint32_t *)base;
    const uint32_t rv = *(const uint32_t *)(base + 4u);
    if (sp < 0x20000000u || sp > F103_BOOT_REQUEST_ADDR || (sp & 3u)) return false;
    if ((rv & 1u) == 0u) return false;
    const uint32_t pc = rv & ~1u;
    return pc >= F103_APP_BASE_ADDR && pc < (F103_APP_BASE_ADDR + size);
}

static bool staged_image_valid(uint32_t *size_out, uint16_t *crc_out) {
    if (!stage_session_active) return false;
    const uint8_t *s = (const uint8_t *)F103_STAGE_BASE_ADDR;
    const uint32_t size = be32(s);
    const uint16_t wanted = be16(s + 4u);
    if (size == 0u || size > F103_MAX_FW_IMAGE_SIZE) return false;
    if (stage_session_total != size + F103_VESC_IMAGE_HEADER_SIZE) return false;
    if (!staged_vector_valid(size)) return false;
    if (crc16(s + F103_VESC_IMAGE_HEADER_SIZE, size) != wanted) return false;
    if (size_out) *size_out = size;
    if (crc_out) *crc_out = wanted;
    return true;
}

static bool write_pending_meta(uint32_t size, uint16_t crc) {
    f103_update_meta_t m;
    m.magic = F103_UPDATE_META_MAGIC;
    m.state = F103_UPDATE_STATE_PENDING;    m.size = size;
    m.size_inv = ~size;
    m.crc16 = crc;
    m.crc16_inv = (uint16_t)~crc;
    m.version = F103_UPDATE_META_VERSION;
    m.version_inv = (uint16_t)~F103_UPDATE_META_VERSION;
    if (!erase_pages(F103_META_BASE_ADDR, F103_META_REGION_SIZE)) return false;
    return program_halfwords(F103_META_BASE_ADDR, (const uint8_t *)&m, sizeof(m));
}

bool f103_fw_erase_staging(uint32_t fw_size) {
    if (fw_size == 0u || fw_size > F103_MAX_FW_IMAGE_SIZE) return false;
    release_both();
    stage_session_active = false;
    stage_session_total = fw_size + F103_VESC_IMAGE_HEADER_SIZE;
    if (!erase_pages(F103_META_BASE_ADDR, F103_META_REGION_SIZE)) return false;
    if (!erase_pages(F103_STAGE_BASE_ADDR, F103_STAGE_REGION_SIZE)) return false;
    stage_session_active = true;
    return true;
}

bool f103_fw_write_staging(uint32_t offset, const uint8_t *data, uint32_t len) {
    if (!stage_session_active || !data || len == 0u || (offset & 1u) != 0u) return false;
    if (offset > stage_session_total || len > stage_session_total - offset) return false;
    release_both();
    const uint32_t dst = F103_STAGE_BASE_ADDR + offset;
    if (memcmp((const void *)dst, data, len) == 0) return true;
    return program_halfwords(dst, data, len);
}
void f103_fw_reset_to_bootloader(void) {
    release_both();
    platform_watchdog_maintenance_kick();

    uint32_t size = 0u;
    uint16_t crc = 0u;
    if (!staged_image_valid(&size, &crc)) return;
    if (!write_pending_meta(size, crc)) return;

    *(volatile uint32_t *)F103_RESET_REASON_ADDR = F103_RESET_REASON_FW_UPDATE;
    __DSB();
    __ISB();
    NVIC_SystemReset();
    for (;;) { }
}