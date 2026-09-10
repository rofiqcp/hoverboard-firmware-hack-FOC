#include "flash_update_f103.h"
#include "f103_boot_layout.h"
#include "motor/mcpwm_foc.h"
#include "defines.h"
#include "platform_watchdog.h"
#include "stm32f1xx_hal.h"
#include <string.h>

static bool stage_session_active = false;
static uint32_t stage_session_total = 0u;

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
    const uint32_t pages = (bytes + F103_FLASH_PAGE_SIZE - 1u) / F103_FLASH_PAGE_SIZE;
    HAL_FLASH_Unlock();
    platform_watchdog_maintenance_kick();
    for (uint32_t page = 0u; page < pages; ++page) {
        FLASH_EraseInitTypeDef e = {0};
        uint32_t page_error = 0u;
        e.TypeErase = FLASH_TYPEERASE_PAGES;
        e.PageAddress = base + page * F103_FLASH_PAGE_SIZE;
        e.NbPages = 1u;
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
    platform_watchdog_maintenance_kick();
    HAL_FLASH_Unlock();
    for (uint32_t i = 0u; i < len; i += 2u) {
        uint16_t wanted = data[i];
        wanted |= (uint16_t)((i + 1u < len ? data[i + 1u] : 0xFFu) << 8);
        volatile const uint16_t *dst = (volatile const uint16_t *)(base + i);
        const uint16_t current = *dst;
        if (current == wanted) continue;
        if (current != 0xFFFFu) {
            HAL_FLASH_Lock();
            return false;
        }
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, base + i, wanted) != HAL_OK || *dst != wanted) {
            HAL_FLASH_Lock();
            return false;
        }
        if ((i & 0x7Fu) == 0u) platform_watchdog_maintenance_kick();
    }
    HAL_FLASH_Lock();
    platform_watchdog_maintenance_kick();
    return memcmp((const void *)base, data, len) == 0;
}

bool f103_fw_erase_staging(uint32_t fw_size) {
    if (fw_size == 0u || fw_size > F103_MAX_FW_IMAGE_SIZE) return false;
    release_both();
    stage_session_active = false;
    stage_session_total = fw_size + F103_VESC_IMAGE_HEADER_SIZE;

    /* VESC-standard staging semantics: erase the complete update buffer before
     * writing size+CRC+image. Old APP remains untouched until bootloader entry. */
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

    /* Lost ACKs are safe: exact duplicate chunks are accepted without another
     * flash program operation, matching VESC Tool retry behaviour. */
    if (memcmp((const void *)dst, data, len) == 0) return true;
    return program_halfwords(dst, data, len);
}

void f103_fw_reset_to_bootloader(void) {
    release_both();
    platform_watchdog_maintenance_kick();
    volatile uint32_t *const request = (volatile uint32_t *)F103_BOOT_REQUEST_ADDR;
    request[0] = F103_BOOT_REQUEST_MAGIC;
    request[1] = F103_BOOT_REQUEST_MAGIC_INV;
    *(volatile uint32_t *)F103_RESET_REASON_ADDR = F103_RESET_REASON_FW_UPDATE;
    __DSB();
    __ISB();
    NVIC_SystemReset();
    for (;;) { }
}
