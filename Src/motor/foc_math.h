#ifndef FOC_MATH_H_
#define FOC_MATH_H_

#include <stdint.h>

int16_t foc_math_clamp_s16(int32_t value);
int16_t foc_math_clamp_s16_range(int32_t value, int16_t min_value, int16_t max_value);
uint16_t foc_math_phase_from_degrees_x16(int32_t degrees_x16);
void foc_math_clarke_park_q4(int16_t i1, int16_t i2, uint8_t sample_map,
                             int16_t sin_q15, int16_t cos_q15,
                             int16_t *iq_q4, int16_t *id_q4);
void foc_math_inv_park_q15(int16_t vd, int16_t vq, int16_t sin_q15, int16_t cos_q15,
                           int32_t *v_alpha, int32_t *v_beta);

#endif
