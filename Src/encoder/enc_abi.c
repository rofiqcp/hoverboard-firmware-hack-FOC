#include <string.h>
#include "encoder/enc_abi.h"

static uint32_t abi_counts_sane(uint32_t counts) {
    if (counts < 4u) counts = 4u;
    if (counts > 65536u) counts = 65536u;
    return counts;
}

bool enc_abi_init(ABI_config_t *cfg) {
    if (!cfg || !cfg->timer || !cfg->A_gpio || !cfg->B_gpio) return false;
    cfg->counts = abi_counts_sane(cfg->counts);
    memset(&cfg->state, 0, sizeof(cfg->state));

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();

    GPIO_InitTypeDef io = {0};
    io.Mode = GPIO_MODE_INPUT;
    io.Pull = GPIO_PULLUP;
    io.Speed = GPIO_SPEED_FREQ_HIGH;
    io.Pin = cfg->A_pin;
    HAL_GPIO_Init(cfg->A_gpio, &io);
    io.Pin = cfg->B_pin;
    HAL_GPIO_Init(cfg->B_gpio, &io);

    cfg->timer->CR1 = 0u;
    cfg->timer->CR2 = 0u;
    cfg->timer->SMCR = 0u;
    cfg->timer->DIER = 0u;
    cfg->timer->CCER = 0u;
    cfg->timer->CCMR1 = 0u;
    cfg->timer->CCMR2 = 0u;
    cfg->timer->PSC = 0u;
    cfg->timer->ARR = cfg->counts - 1u;
    cfg->timer->CNT = 0u;

    /* VESC enc_abi.c: TIM encoder mode TI12, rising/rising, IC filter=6. */
    cfg->timer->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0 |
                        (6u << TIM_CCMR1_IC1F_Pos) |
                        (6u << TIM_CCMR1_IC2F_Pos);
    cfg->timer->SMCR = TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;
    cfg->timer->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P);
    cfg->timer->EGR = TIM_EGR_UG;
    cfg->timer->CR1 |= TIM_CR1_CEN;

    /* Sama seperti VESC: state ABI mulai belum tersinkron. Karena hardware ini
     * hanya A/B tanpa index I, encoder_set_deg() setelah electrical alignment
     * yang mengubah index_found menjadi true. */
    return true;
}

void enc_abi_deinit(ABI_config_t *cfg) {
    if (!cfg || !cfg->timer) return;
    cfg->timer->CR1 &= ~TIM_CR1_CEN;
    GPIO_InitTypeDef io = {0};
    io.Mode = GPIO_MODE_INPUT;
    io.Pull = GPIO_NOPULL;
    io.Speed = GPIO_SPEED_FREQ_LOW;
    io.Pin = cfg->A_pin;
    HAL_GPIO_Init(cfg->A_gpio, &io);
    io.Pin = cfg->B_pin;
    HAL_GPIO_Init(cfg->B_gpio, &io);
}

uint32_t enc_abi_read_cnt(ABI_config_t *cfg) {
    if (!cfg || !cfg->timer) return 0u;
    return cfg->timer->CNT;
}

float enc_abi_read_deg(ABI_config_t *cfg) {
    if (!cfg || !cfg->timer || cfg->counts < 1u) return 0.0f;
    return ((float)cfg->timer->CNT * 360.0f) / (float)cfg->counts;
}

void enc_abi_set_deg(ABI_config_t *cfg, float deg) {
    if (!cfg || !cfg->timer || cfg->counts < 1u) return;
    while (deg >= 360.0f) deg -= 360.0f;
    while (deg < 0.0f) deg += 360.0f;
    /* VESC encoder_set_deg() memakai truncation, bukan rounding. */
    uint32_t cnt = (uint32_t)(deg * (float)cfg->counts / 360.0f);
    if (cnt >= cfg->counts) cnt = 0u;
    cfg->timer->CNT = cnt;
    cfg->state.index_found = true;
}

void enc_abi_pin_isr(ABI_config_t *cfg) {
    if (!cfg || !cfg->I_gpio || cfg->I_pin == 0u) return;
    __NOP(); __NOP(); __NOP(); __NOP();
    if (HAL_GPIO_ReadPin(cfg->I_gpio, cfg->I_pin) == GPIO_PIN_SET) {
        const uint32_t cnt = cfg->timer->CNT;
        const uint32_t lim = cfg->counts / 20u;
        cfg->state.cnt_at_ind_last = cnt;
        cfg->state.index_pulse_cnt++;
        if (!cfg->state.index_found || cnt > cfg->counts - lim || cnt < lim) {
            cfg->timer->CNT = 0u;
            cfg->state.index_found = true;
            cfg->state.bad_pulses = 0;
        } else if (++cfg->state.bad_pulses > 5) {
            cfg->state.index_found = false;
        }
    }
}
