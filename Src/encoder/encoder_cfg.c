#include "encoder/encoder_cfg.h"

#ifndef STM32F103xE
static TIM_TypeDef s_encoder_host_tim4;
#define ENCODER_ABI_TIMER (&s_encoder_host_tim4)
#else
#define ENCODER_ABI_TIMER TIM4
#endif

/* VESC-style global ABI configuration.
 * LEFT encoder A/B: PB6=TIM4_CH1, PB7=TIM4_CH2. Index tidak dipakai. */
ABI_config_t encoder_cfg_ABI = {
    4096u,
    GPIOB, GPIO_PIN_6,
    GPIOB, GPIO_PIN_7,
    NULL, 0u,
    ENCODER_ABI_TIMER,
    0u,
    0u,
    0u,
    0u,
    {false, 0u, 0, 0u}
};
