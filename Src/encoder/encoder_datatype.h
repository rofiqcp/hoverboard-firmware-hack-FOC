#ifndef ENCODER_DATATYPE_H_
#define ENCODER_DATATYPE_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32f1xx_hal.h"

/* Nama enum mengikuti VESC. Target ini hanya mengimplementasikan ABI. */
typedef enum {
    ENCODER_TYPE_NONE = 0,
    ENCODER_TYPE_AS504x,
    ENCODER_TYPE_MT6816,
    ENCODER_TYPE_TLE5012,
    ENCODER_TYPE_AD2S1205_SPI,
    ENCODER_TYPE_SINCOS,
    ENCODER_TYPE_TS5700N8501,
    ENCODER_TYPE_ABI,
    ENCODER_TYPE_AS5x47U,
    ENCODER_TYPE_BISSC,
    ENCODER_TYPE_CUSTOM,
    ENCODER_TYPE_PWM,
    ENCODER_TYPE_PWM_ABI,
    ENCODER_TYPE_MA782,
    ENCODER_TYPE_AMT22,
    ENCODER_TYPE_MT6835
} encoder_type_t;

typedef struct {
    volatile bool index_found;
    volatile uint32_t cnt_at_ind_last;
    volatile int bad_pulses;
    volatile uint32_t index_pulse_cnt;
} ABI_state;

typedef struct {
    uint32_t counts;
    GPIO_TypeDef *A_gpio; uint16_t A_pin;
    GPIO_TypeDef *B_gpio; uint16_t B_pin;
    GPIO_TypeDef *I_gpio; uint16_t I_pin;
    TIM_TypeDef *timer;
    uint8_t tim_af;
    uint8_t exti_portsrc;
    uint8_t exti_pinsrc;
    uint32_t exti_line;
    ABI_state state;
} ABI_config_t;

#endif /* ENCODER_DATATYPE_H_ */
