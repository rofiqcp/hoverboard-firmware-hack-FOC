#include "foc_math.h"
#include <limits.h>

#define ONE_BY_SQRT3_Q15 18919

int16_t foc_math_clamp_s16(int32_t value) {
  if (value > INT16_MAX) return INT16_MAX;
  if (value < INT16_MIN) return INT16_MIN;
  return (int16_t)value;
}

int16_t foc_math_clamp_s16_range(int32_t value, int16_t min_value, int16_t max_value) {
  if (value > max_value) return max_value;
  if (value < min_value) return min_value;
  return (int16_t)value;
}

uint16_t foc_math_phase_from_degrees_x16(int32_t degrees_x16) {
  degrees_x16 %= 5760;
  if (degrees_x16 < 0) degrees_x16 += 5760;
  return (uint16_t)(((uint32_t)degrees_x16 * 65536u) / 5760u);
}

void foc_math_clarke_park_q4(int16_t i1, int16_t i2, uint8_t sample_map,
                             int16_t sin_q15, int16_t cos_q15,
                             int16_t *iq_q4, int16_t *id_q4) {
  int32_t ia = (int32_t)i1 << 4;
  int32_t ib = (int32_t)i2 << 4;
  int32_t alpha;
  int32_t beta;
  if (sample_map == 0u) {
    alpha = ia;
    beta = ((ONE_BY_SQRT3_Q15 * ia) >> 15) + ((ONE_BY_SQRT3_Q15 * ib) >> 14);
  } else {
    alpha = -ia - ib;
    beta = (ONE_BY_SQRT3_Q15 * (ia - ib)) >> 15;
  }
  alpha = foc_math_clamp_s16(alpha);
  beta = foc_math_clamp_s16(beta);
  *iq_q4 = foc_math_clamp_s16((beta * cos_q15 - alpha * sin_q15) >> 15);
  *id_q4 = foc_math_clamp_s16((alpha * cos_q15 + beta * sin_q15) >> 15);
}

void foc_math_inv_park_q15(int16_t vd, int16_t vq, int16_t sin_q15, int16_t cos_q15,
                           int32_t *v_alpha, int32_t *v_beta) {
  *v_alpha = ((int32_t)vd * cos_q15 - (int32_t)vq * sin_q15) >> 15;
  *v_beta = ((int32_t)vq * cos_q15 + (int32_t)vd * sin_q15) >> 15;
}
