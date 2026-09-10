#include "stm32f1xx_hal.h"
#include "vesc/f103_boot_layout.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define RAMFUNC __attribute__((section(".ramfunc"), noinline, long_call))
#define FLASH_ERROR_MASK (FLASH_SR_PGERR | FLASH_SR_WRPRTERR)
#define COPY_RETRIES 3u

static uint16_t crc16(const uint8_t *data, uint32_t len) {
    uint16_t crc = 0u;
    for (uint32_t i = 0u; i < len; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t b = 0u; b < 8u; ++b) {
            crc = (crc & 0x8000u) ?
                (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
        }
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
static void safe_gpio_init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_13 |
                     GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

    GPIO_InitTypeDef g = {0};
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    g.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8;
    HAL_GPIO_Init(GPIOC, &g);
    g.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 |
            GPIO_PIN_5 | GPIO_PIN_4;
    HAL_GPIO_Init(GPIOA, &g);
    g.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_13 |
            GPIO_PIN_14 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOB, &g);
}
static RAMFUNC bool flash_wait_ready(uint32_t guard) {
    while ((FLASH->SR & FLASH_SR_BSY) != 0u) {
        if (guard-- == 0u) return false;
    }
    return true;
}

static RAMFUNC bool flash_unlock(void) {
    if ((FLASH->CR & FLASH_CR_LOCK) != 0u) {
        FLASH->KEYR = FLASH_KEY1;
        FLASH->KEYR = FLASH_KEY2;
    }
    return (FLASH->CR & FLASH_CR_LOCK) == 0u;
}

static RAMFUNC void flash_clear_status(void) {
    FLASH->SR = FLASH_SR_EOP | FLASH_SR_PGERR | FLASH_SR_WRPRTERR;
}

static RAMFUNC bool flash_erase_page(uint32_t address) {
    if (!flash_wait_ready(8000000u) || !flash_unlock()) return false;
    flash_clear_status();
    FLASH->CR |= FLASH_CR_PER;
    FLASH->AR = address;
    FLASH->CR |= FLASH_CR_STRT;
    const bool ready = flash_wait_ready(8000000u);
    const uint32_t sr = FLASH->SR;
    FLASH->CR &= ~FLASH_CR_PER;
    FLASH->CR |= FLASH_CR_LOCK;
    flash_clear_status();
    return ready && (sr & FLASH_ERROR_MASK) == 0u;
}
static RAMFUNC bool flash_program_block(uint32_t base,
                                        const uint8_t *data,
                                        uint32_t len) {
    if (!data || (base & 1u) != 0u ||
        !flash_wait_ready(8000000u) || !flash_unlock()) return false;
    flash_clear_status();
    for (uint32_t i = 0u; i < len; i += 2u) {
        uint16_t wanted = data[i];
        wanted |= (uint16_t)((i + 1u < len ? data[i + 1u] : 0xFFu) << 8);
        volatile uint16_t *dst = (volatile uint16_t *)(base + i);
        if (*dst == wanted) continue;
        if (*dst != 0xFFFFu) {
            FLASH->CR |= FLASH_CR_LOCK;
            return false;
        }
        FLASH->CR |= FLASH_CR_PG;
        *dst = wanted;
        const bool ready = flash_wait_ready(1000000u);
        const uint32_t sr = FLASH->SR;
        FLASH->CR &= ~FLASH_CR_PG;
        flash_clear_status();
        if (!ready || (sr & FLASH_ERROR_MASK) != 0u || *dst != wanted) {
            FLASH->CR |= FLASH_CR_LOCK;
            return false;
        }
    }
    FLASH->CR |= FLASH_CR_LOCK;
    return true;
}
static bool page_erased(uint32_t address) {
    for (uint32_t off = 0u; off < F103_FLASH_PAGE_SIZE; off += 4u) {
        if (*(volatile const uint32_t *)(address + off) != 0xFFFFFFFFu) return false;
    }
    return true;
}

static bool erase_page(uint32_t address) {
    return flash_erase_page(address) && page_erased(address);
}

static bool erase_meta(void) {
    return erase_page(F103_META_BASE_ADDR);
}

static bool stage_valid(uint32_t *size_out, uint16_t *crc_out) {
    const uint8_t *s = (const uint8_t *)F103_STAGE_BASE_ADDR;
    const uint32_t size = be32(s);
    const uint16_t wanted = be16(s + 4u);
    if (size == 0u || size > F103_MAX_FW_IMAGE_SIZE) return false;
    if (crc16(s + F103_VESC_IMAGE_HEADER_SIZE, size) != wanted) return false;
    if (size_out) *size_out = size;
    if (crc_out) *crc_out = wanted;
    return true;
}

static bool pending_valid(const f103_update_meta_t *m) {
    if (!m || m->magic != F103_UPDATE_META_MAGIC) return false;
    if (m->state != F103_UPDATE_STATE_PENDING) return false;
    if (m->size == 0u || m->size > F103_MAX_FW_IMAGE_SIZE) return false;
    if (m->size != ~m->size_inv) return false;
    if ((uint16_t)(m->crc16 ^ m->crc16_inv) != 0xFFFFu) return false;
    if (m->version != F103_UPDATE_META_VERSION) return false;
    if ((uint16_t)(m->version ^ m->version_inv) != 0xFFFFu) return false;
    return true;
}

static bool vector_valid_at(uint32_t base, uint32_t image_size) {
    if (image_size < 8u || image_size > F103_APP_REGION_SIZE) return false;
    const uint32_t sp = *(const uint32_t *)base;
    const uint32_t rv = *(const uint32_t *)(base + 4u);
    if (sp < 0x20000000u || sp > F103_BOOT_REQUEST_ADDR || (sp & 3u)) return false;
    if ((rv & 1u) == 0u) return false;
    const uint32_t pc = rv & ~1u;
    return pc >= F103_APP_BASE_ADDR && pc < (F103_APP_BASE_ADDR + image_size);
}

static bool app_valid(void) {
    const f103_update_meta_t *m = (const f103_update_meta_t *)F103_META_BASE_ADDR;
    if (pending_valid(m)) return false;
    const uint32_t sp = *(const uint32_t *)F103_APP_BASE_ADDR;
    const uint32_t rv = *(const uint32_t *)(F103_APP_BASE_ADDR + 4u);
    if (sp < 0x20000000u || sp > F103_BOOT_REQUEST_ADDR || (sp & 3u)) return false;
    if ((rv & 1u) == 0u) return false;
    const uint32_t pc = rv & ~1u;
    return pc >= F103_APP_BASE_ADDR && pc <
           (F103_APP_BASE_ADDR + F103_APP_REGION_SIZE);
}
static bool copy_pending_image(void) {
    const f103_update_meta_t *m = (const f103_update_meta_t *)F103_META_BASE_ADDR;
    if (!pending_valid(m)) return false;

    uint32_t size = 0u;
    uint16_t wanted = 0u;
    if (!stage_valid(&size, &wanted)) return false;
    if (size != m->size || wanted != m->crc16) return false;

    const uint8_t *stage = (const uint8_t *)(F103_STAGE_BASE_ADDR +
                                            F103_VESC_IMAGE_HEADER_SIZE);
    if (!vector_valid_at((uint32_t)stage, size)) return false;

    static uint8_t page_buf[F103_FLASH_PAGE_SIZE];
    uint32_t copied = 0u;
    while (copied < size) {
        const uint32_t remain = size - copied;
        const uint32_t chunk = remain < F103_FLASH_PAGE_SIZE ?
                               remain : F103_FLASH_PAGE_SIZE;
        const uint32_t dst = F103_APP_BASE_ADDR + copied;
        memcpy(page_buf, stage + copied, chunk);
        if (!erase_page(dst)) return false;
        if (!flash_program_block(dst, page_buf, chunk)) return false;
        if (memcmp((const void *)dst, page_buf, chunk) != 0) return false;
        copied += chunk;
    }

    if (crc16((const uint8_t *)F103_APP_BASE_ADDR, size) != wanted) return false;
    if (!vector_valid_at(F103_APP_BASE_ADDR, size)) return false;
    return erase_meta();
}
__attribute__((naked, noreturn)) static void branch_to_app(uint32_t sp, uint32_t rv) {
    (void)sp;
    (void)rv;
    __asm volatile (
        "msr msp, r0\n"
        "bx r1\n"
    );
}

static void jump_app(void) {
    const uint32_t sp = *(const uint32_t *)F103_APP_BASE_ADDR;
    const uint32_t rv = *(const uint32_t *)(F103_APP_BASE_ADDR + 4u);
    __disable_irq();
    SysTick->CTRL = 0u;
    SysTick->LOAD = 0u;
    SysTick->VAL = 0u;
    for (uint32_t i = 0u; i < 8u; ++i) {
        NVIC->ICER[i] = 0xFFFFFFFFu;
        NVIC->ICPR[i] = 0xFFFFFFFFu;
    }
    SCB->VTOR = F103_APP_BASE_ADDR;
    __DSB();
    __ISB();
    branch_to_app(sp, rv);
}

static __attribute__((noreturn)) void safe_fault(void) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
    __disable_irq();
    for (;;) { __WFI(); }
}
int main(void) {
    safe_gpio_init();

    const f103_update_meta_t *m =
        (const f103_update_meta_t *)F103_META_BASE_ADDR;

    if (pending_valid(m)) {
        for (uint32_t attempt = 0u; attempt < COPY_RETRIES; ++attempt) {
            if (copy_pending_image()) {
                NVIC_SystemReset();
            }
        }
        safe_fault();
    }

    if (app_valid()) {
        jump_app();
    }

    safe_fault();
}