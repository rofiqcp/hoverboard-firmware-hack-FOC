#include "flash_update_f103.h"
#include "f103_boot_layout.h"
#include "motor/mcpwm_foc.h"
#include "defines.h"
#include "stm32f1xx_hal.h"
#include <string.h>

#define F103_STAGE_PAGE_COUNT (F103_STAGE_REGION_SIZE / F103_FLASH_PAGE_SIZE)
static bool stage_session_active = false;
static uint32_t stage_session_total = 0u;
static uint8_t stage_page_erased[F103_STAGE_PAGE_COUNT];

static void release_both(void) {
    mcpwm_foc_release_motor(false);
    mcpwm_foc_release_motor(true);
    LEFT_TIM->BDTR &= ~TIM_BDTR_MOE;
    RIGHT_TIM->BDTR &= ~TIM_BDTR_MOE;
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

static bool erase_one_page(uint32_t address) {
    return erase_pages(address, F103_FLASH_PAGE_SIZE);
}

static bool ensure_stage_pages_erased(uint32_t offset, uint32_t len) {
    if (len == 0u || offset > F103_STAGE_REGION_SIZE || len > F103_STAGE_REGION_SIZE - offset) return false;
    const uint32_t first = offset / F103_FLASH_PAGE_SIZE;
    const uint32_t last = (offset + len - 1u) / F103_FLASH_PAGE_SIZE;
    if (last >= F103_STAGE_PAGE_COUNT) return false;
    for (uint32_t page = first; page <= last; ++page) {
        if (stage_page_erased[page] == 0u) {
            if (!erase_one_page(F103_STAGE_BASE_ADDR + page * F103_FLASH_PAGE_SIZE)) return false;
            stage_page_erased[page] = 1u;
        }
    }
    return true;
}

static bool program_halfwords(uint32_t base, const uint8_t *data, uint32_t len) {
    if (!data || (base & 1u)) return false;
    HAL_FLASH_Unlock();
    for (uint32_t i = 0; i < len; i += 2u) {
        uint16_t hw = data[i];
        if (i + 1u < len) hw |= (uint16_t)((uint16_t)data[i + 1u] << 8);
        else hw |= 0xFF00u;
        volatile const uint16_t *dst = (volatile const uint16_t *)(base + i);
        const uint16_t current = *dst;
        if (current == hw) continue;
        if (current != 0xFFFFu) {
            HAL_FLASH_Lock();
            return false;
        }
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
    memset(stage_page_erased, 0, sizeof(stage_page_erased));
    stage_session_active = true;
    stage_session_total = fw_size + F103_VESC_IMAGE_HEADER_SIZE;
    /* Do not block for a 120-KiB mass erase while USART is live. Metadata is
     * cleared immediately; staging pages are erased lazily before first write. */
    return erase_pages(F103_META_BASE_ADDR, F103_META_REGION_SIZE);
}

bool f103_fw_write_staging(uint32_t offset, const uint8_t *data, uint32_t len) {
    if (!stage_session_active || !data || len == 0u || (offset & 1u) != 0u) return false;
    if (offset > stage_session_total || len > stage_session_total - offset) return false;
    release_both();
    const uint32_t dst = F103_STAGE_BASE_ADDR + offset;
    /* ACK loss is recoverable: duplicate chunks are accepted without touching
     * flash again. */
    if (memcmp((const void *)dst, data, len) == 0) return true;
    if (!ensure_stage_pages_erased(offset, len)) return false;
    return program_halfwords(dst, data, len);
}

void f103_fw_reset_to_bootloader(void) {
    release_both();
    volatile uint32_t *const request = (volatile uint32_t *)F103_BOOT_REQUEST_ADDR;
    request[0] = F103_BOOT_REQUEST_MAGIC;
    request[1] = F103_BOOT_REQUEST_MAGIC_INV;
    *(volatile uint32_t *)F103_RESET_REASON_ADDR = F103_RESET_REASON_FW_UPDATE;
    __DSB();
    __ISB();
    NVIC_SystemReset();
    for (;;) { }
}
