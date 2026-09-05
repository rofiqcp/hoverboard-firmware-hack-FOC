#include "stm32f1xx_hal.h"
#include "vesc/f103_boot_layout.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define COMM_FW_VERSION          0u
#define COMM_JUMP_TO_BOOTLOADER  1u
#define COMM_ERASE_NEW_APP       2u
#define COMM_WRITE_NEW_APP_DATA  3u
#define COMM_REBOOT             29u
#define COMM_ALIVE              30u

#define RX_MAX_PAYLOAD 512u
#define RECOVERY_IDLE_BLINK_MS  250u

static UART_HandleTypeDef huart3;
static uint8_t rx_payload[RX_MAX_PAYLOAD];

static uint16_t crc16(const uint8_t *data, uint32_t len) {
    uint16_t crc = 0u;
    for (uint32_t i = 0u; i < len; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t b = 0u; b < 8u; ++b) {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static uint32_t be32(const uint8_t *p) {
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}
static uint16_t be16(const uint8_t *p) { return (uint16_t)(((uint16_t)p[0] << 8) | p[1]); }

static void safe_gpio_init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_USART3_CLK_ENABLE();

    /* Hoverboard half-bridges: high-side pins LOW, complementary low-side pins HIGH. */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); /* keep power latch on */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); /* buzzer off */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET); /* LED off */

    GPIO_InitTypeDef g = {0};
    g.Mode = GPIO_MODE_OUTPUT_PP; g.Pull = GPIO_NOPULL; g.Speed = GPIO_SPEED_FREQ_HIGH;
    g.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8; HAL_GPIO_Init(GPIOC, &g);
    g.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_5 | GPIO_PIN_4; HAL_GPIO_Init(GPIOA, &g);
    g.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15; HAL_GPIO_Init(GPIOB, &g);

    g.Mode = GPIO_MODE_AF_PP; g.Pin = GPIO_PIN_10; HAL_GPIO_Init(GPIOB, &g);
    g.Mode = GPIO_MODE_INPUT; g.Pull = GPIO_NOPULL; g.Pin = GPIO_PIN_11; HAL_GPIO_Init(GPIOB, &g);
}


static bool boot_clock_init(void) {
    /* Match the application clock tree so USART3 can run the project-wide
     * 2 Mbaud VESC transport: HSI/2 * 16 = 64 MHz SYSCLK, APB1 = 32 MHz.
     * HAL_Init() is called first with a correct 8-MHz SystemCoreClock model,
     * therefore the oscillator-switch timeouts and SysTick are valid. */
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
    osc.PLL.PLLMUL = RCC_PLL_MUL16;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) return false;
    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV2;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    return HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) == HAL_OK;
}

static void uart_init(void) {
    huart3.Instance = USART3;
    huart3.Init.BaudRate = F103_VESC_UART_BAUD;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    (void)HAL_UART_Init(&huart3);
}

static bool erase_pages(uint32_t base, uint32_t bytes) {
    if ((base & (F103_FLASH_PAGE_SIZE - 1u)) != 0u || bytes == 0u) return false;
    FLASH_EraseInitTypeDef e = {0};
    uint32_t page_error = 0u;
    e.TypeErase = FLASH_TYPEERASE_PAGES;
    e.PageAddress = base;
    e.NbPages = (bytes + F103_FLASH_PAGE_SIZE - 1u) / F103_FLASH_PAGE_SIZE;
    HAL_FLASH_Unlock();
    HAL_StatusTypeDef st = HAL_FLASHEx_Erase(&e, &page_error);
    HAL_FLASH_Lock();
    return st == HAL_OK && page_error == 0xFFFFFFFFu;
}

static bool program_halfwords(uint32_t base, const uint8_t *data, uint32_t len) {
    if (!data || (base & 1u)) return false;
    HAL_FLASH_Unlock();
    for (uint32_t i = 0u; i < len; i += 2u) {
        uint16_t hw = data[i];
        hw |= (uint16_t)((i + 1u < len ? data[i + 1u] : 0xFFu) << 8);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, base + i, hw) != HAL_OK) {
            HAL_FLASH_Lock(); return false;
        }
    }
    HAL_FLASH_Lock();
    return memcmp((const void *)base, data, len) == 0;
}

static bool stage_valid(uint32_t *size_out, uint16_t *crc_out) {
    const uint8_t *s = (const uint8_t *)F103_STAGE_BASE_ADDR;
    uint32_t size = be32(s); uint16_t wanted = be16(s + 4u);
    if (size == 0u || size > F103_MAX_FW_IMAGE_SIZE) return false;
    if (crc16(s + F103_VESC_IMAGE_HEADER_SIZE, size) != wanted) return false;
    if (size_out) *size_out = size;
    if (crc_out) *crc_out = wanted;
    return true;
}

static bool app_vector_valid(void) {
    const uint32_t sp = *(const uint32_t *)F103_APP_BASE_ADDR;
    const uint32_t rv = *(const uint32_t *)(F103_APP_BASE_ADDR + 4u);
    if (sp < 0x20000000u || sp > 0x2000C000u || (sp & 3u)) return false;
    if ((rv & 1u) == 0u) return false;
    const uint32_t pc = rv & ~1u;
    return pc >= F103_APP_BASE_ADDR && pc < (F103_APP_BASE_ADDR + F103_APP_REGION_SIZE);
}

static bool meta_valid(const f103_update_meta_t *m) {
    if (!m || m->magic != F103_UPDATE_META_MAGIC) return false;
    if (m->size != ~m->size_inv) return false;
    if ((uint16_t)(m->crc16 ^ m->crc16_inv) != 0xFFFFu) return false;
    if (m->version != F103_UPDATE_META_VERSION || (uint16_t)(m->version ^ m->version_inv) != 0xFFFFu) return false;
    return m->state == F103_UPDATE_STATE_PENDING || m->state == F103_UPDATE_STATE_RECOVERY;
}

static bool write_meta(uint32_t state, uint32_t size, uint16_t crc) {
    f103_update_meta_t m;
    m.magic = F103_UPDATE_META_MAGIC; m.state = state;
    m.size = size; m.size_inv = ~size;
    m.crc16 = crc; m.crc16_inv = (uint16_t)~crc;
    m.version = F103_UPDATE_META_VERSION; m.version_inv = (uint16_t)~F103_UPDATE_META_VERSION;
    if (!erase_pages(F103_META_BASE_ADDR, F103_META_REGION_SIZE)) return false;
    return program_halfwords(F103_META_BASE_ADDR, (const uint8_t *)&m, sizeof(m));
}

static bool stage_to_pending_meta(void) {
    uint32_t size = 0u; uint16_t crc = 0u;
    if (!stage_valid(&size, &crc)) return false;
    return write_meta(F103_UPDATE_STATE_PENDING, size, crc);
}

static bool copy_pending_image(void) {
    const f103_update_meta_t *m = (const f103_update_meta_t *)F103_META_BASE_ADDR;
    if (!meta_valid(m) || m->state != F103_UPDATE_STATE_PENDING) return false;
    uint32_t size = 0u; uint16_t wanted = 0u;
    if (!stage_valid(&size, &wanted) || size != m->size || wanted != m->crc16) return false;
    if (!erase_pages(F103_APP_BASE_ADDR, F103_APP_REGION_SIZE)) return false;
    if (!program_halfwords(F103_APP_BASE_ADDR,
                           (const uint8_t *)(F103_STAGE_BASE_ADDR + F103_VESC_IMAGE_HEADER_SIZE), size)) return false;
    if (crc16((const uint8_t *)F103_APP_BASE_ADDR, size) != wanted) return false;
    if (!app_vector_valid()) return false;
    return erase_pages(F103_META_BASE_ADDR, F103_META_REGION_SIZE);
}

static void jump_app(void) {
    const uint32_t sp = *(const uint32_t *)F103_APP_BASE_ADDR;
    const uint32_t rv = *(const uint32_t *)(F103_APP_BASE_ADDR + 4u);
    typedef void (*entry_t)(void);
    entry_t entry = (entry_t)rv;
    (void)HAL_UART_DeInit(&huart3);
    HAL_SuspendTick();
    __disable_irq();
    SysTick->CTRL = 0u; SysTick->LOAD = 0u; SysTick->VAL = 0u;
    for (uint32_t i = 0u; i < 8u; ++i) { NVIC->ICER[i] = 0xFFFFFFFFu; NVIC->ICPR[i] = 0xFFFFFFFFu; }
    SCB->VTOR = F103_APP_BASE_ADDR;
    __set_MSP(sp);
    /* Cortex-M reset state enters application with PRIMASK clear. The
     * bootloader disabled IRQs while tearing down its peripherals, so restore
     * that reset invariant before branching to the application's Reset_Handler. */
    __enable_irq();
    entry();
    for (;;) { }
}

static void send_payload(const uint8_t *p, uint16_t len) {
    uint8_t tx[RX_MAX_PAYLOAD + 7u]; uint16_t i = 0u;
    if (!p || len == 0u || len > RX_MAX_PAYLOAD) return;
    if (len <= 255u) { tx[i++] = 2u; tx[i++] = (uint8_t)len; }
    else { tx[i++] = 3u; tx[i++] = (uint8_t)(len >> 8); tx[i++] = (uint8_t)len; }
    memcpy(&tx[i], p, len); i = (uint16_t)(i + len);
    uint16_t c = crc16(p, len); tx[i++] = (uint8_t)(c >> 8); tx[i++] = (uint8_t)c; tx[i++] = 3u;
    (void)HAL_UART_Transmit(&huart3, tx, i, 1000u);
}

static bool recv_payload(uint32_t timeout_ms, uint16_t *len_out) {
    uint32_t start_ms = HAL_GetTick(); uint8_t b = 0u;
    while ((HAL_GetTick() - start_ms) < timeout_ms) {
        if (HAL_UART_Receive(&huart3, &b, 1u, 10u) != HAL_OK) continue;
        if (b != 2u && b != 3u) continue;
        uint16_t len = 0u;
        if (b == 2u) {
            if (HAL_UART_Receive(&huart3, &b, 1u, 50u) != HAL_OK) continue;
            len = b;
        } else {
            uint8_t l[2]; if (HAL_UART_Receive(&huart3, l, 2u, 50u) != HAL_OK) continue;
            len = (uint16_t)(((uint16_t)l[0] << 8) | l[1]);
        }
        if (len == 0u || len > RX_MAX_PAYLOAD) continue;
        if (HAL_UART_Receive(&huart3, rx_payload, len, 1000u) != HAL_OK) continue;
        uint8_t tail[3]; if (HAL_UART_Receive(&huart3, tail, 3u, 100u) != HAL_OK) continue;
        uint16_t got = (uint16_t)(((uint16_t)tail[0] << 8) | tail[1]);
        if (tail[2] != 3u || got != crc16(rx_payload, len)) continue;
        if (len_out) *len_out = len;
        return true;
    }
    return false;
}

static void reply_fw_version(void) {
    uint8_t b[72]; uint16_t i = 0u;
    const char hw[] = "f103rc_bootloader"; const char fw[] = "f103rc_bootloader";
    b[i++] = COMM_FW_VERSION; b[i++] = 6u; b[i++] = 0u;
    memcpy(&b[i], hw, sizeof(hw)); i += sizeof(hw);
    memcpy(&b[i], (const void *)0x1FFFF7E8u, 12u); i += 12u;
    b[i++] = 1u; b[i++] = 0u; b[i++] = 0u; b[i++] = 0u;
    b[i++] = 0u; b[i++] = 0u; b[i++] = 0u; b[i++] = 0u;
    memcpy(&b[i], fw, sizeof(fw)); i += sizeof(fw);
    send_payload(b, i);
}

static bool recovery_command(uint16_t len) {
    if (len == 0u) return true;
    const uint8_t id = rx_payload[0]; const uint8_t *d = rx_payload + 1u; uint16_t n = len - 1u;
    if (id == COMM_FW_VERSION) { reply_fw_version(); return true; }
    if (id == COMM_ALIVE) return true;
    if (id == COMM_ERASE_NEW_APP) {
        uint8_t r[2] = {COMM_ERASE_NEW_APP, 0u};
        if (n >= 4u) {
            uint32_t size = be32(d);
            if (size > 0u && size <= F103_MAX_FW_IMAGE_SIZE &&
                erase_pages(F103_STAGE_BASE_ADDR, F103_STAGE_REGION_SIZE) &&
                erase_pages(F103_META_BASE_ADDR, F103_META_REGION_SIZE)) r[1] = 1u;
        }
        send_payload(r, sizeof(r)); return true;
    }
    if (id == COMM_WRITE_NEW_APP_DATA) {
        uint8_t r[6] = {COMM_WRITE_NEW_APP_DATA,0u,0u,0u,0u,0u};
        if (n >= 4u) {
            uint32_t off = be32(d); uint32_t dl = n - 4u;
            r[2] = (uint8_t)(off >> 24); r[3] = (uint8_t)(off >> 16); r[4] = (uint8_t)(off >> 8); r[5] = (uint8_t)off;
            if ((off & 1u) == 0u && dl > 0u && off <= F103_STAGE_REGION_SIZE && dl <= F103_STAGE_REGION_SIZE - off)
                r[1] = program_halfwords(F103_STAGE_BASE_ADDR + off, d + 4u, dl) ? 1u : 0u;
        }
        send_payload(r, sizeof(r)); return true;
    }
    if (id == COMM_JUMP_TO_BOOTLOADER) {
        if (stage_to_pending_meta() && copy_pending_image()) NVIC_SystemReset();
        (void)write_meta(F103_UPDATE_STATE_RECOVERY, 0u, 0u); return true;
    }
    if (id == COMM_REBOOT) {
        (void)erase_pages(F103_META_BASE_ADDR, F103_META_REGION_SIZE);
        if (app_vector_valid()) NVIC_SystemReset();
        return true;
    }
    return true;
}

int main(void) {
    /* Startup SystemInit() leaves the MCU on HSI=8 MHz but the CMSIS variable
     * defaults to 72 MHz. Fix the software model first, then raise the actual
     * clock to the same 64/32-MHz tree as the application before USART3 init. */
    SystemCoreClockUpdate();
    HAL_Init();
    if (!boot_clock_init()) {
        for (;;) { }
    }
    safe_gpio_init();
    uart_init();

    const f103_update_meta_t *m = (const f103_update_meta_t *)F103_META_BASE_ADDR;
    if (meta_valid(m) && m->state == F103_UPDATE_STATE_PENDING) {
        if (copy_pending_image()) NVIC_SystemReset();
        (void)write_meta(F103_UPDATE_STATE_RECOVERY, 0u, 0u);
    }

    bool recovery = meta_valid((const f103_update_meta_t *)F103_META_BASE_ADDR) &&
                    ((const f103_update_meta_t *)F103_META_BASE_ADDR)->state == F103_UPDATE_STATE_RECOVERY;
    if (!recovery && app_vector_valid()) {
        /* A valid application boots immediately. Runtime F411 traffic must never
         * trap a healthy system in recovery. Firmware update enters here only
         * through PENDING/RECOVERY metadata written by the application. */
        jump_app();
    } else {
        recovery = true;
    }

    uint32_t blink = HAL_GetTick();
    while (recovery) {
        uint16_t len = 0u;
        if (recv_payload(50u, &len)) recovery_command(len);
        if ((HAL_GetTick() - blink) >= RECOVERY_IDLE_BLINK_MS) {
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2); blink = HAL_GetTick();
        }
    }
    for (;;) { }
}
