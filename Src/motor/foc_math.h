#ifndef FOC_MATH_H_
#define FOC_MATH_H_

#include <stdint.h>
#include <stdbool.h>

#define FOC_Q15_ONE                32767
#define FOC_SQRT3_BY_2_Q15         28378
#define FOC_INV_SQRT3_Q15          18919
#define FOC_CURRENT_Q4_PER_A       (A2BIT_CONV * 16)

typedef struct {
    int32_t integrator;
    uint8_t sat_hold;
} foc_pi_fixed_t;

typedef struct {
    int32_t state_q16[2];
} foc_lpf2_fixed_t;

typedef struct {
    int16_t alpha;
    int16_t beta;
} foc_ab_t;

typedef struct {
    int16_t d;
    int16_t q;
} foc_dq_t;

typedef struct {
    int16_t a;
    int16_t b;
    int16_t c;
} foc_abc_t;

int16_t foc_sat_s16(int32_t x);
void foc_sin_cos_q15(uint16_t phase, int16_t *s, int16_t *c);
void foc_clarke_ab_q4(int16_t ia_q4, int16_t ib_q4, foc_ab_t *out);
void foc_clarke_bc_q4(int16_t ib_q4, int16_t ic_q4, foc_ab_t *out);
void foc_park_q4(const foc_ab_t *ab, uint16_t phase, foc_dq_t *dq);
void foc_inv_park(const foc_dq_t *vdvq, uint16_t phase, foc_ab_t *ab);
void foc_lpf2_run(foc_lpf2_fixed_t *f, uint16_t coef, const foc_dq_t *in, foc_dq_t *out);
int16_t foc_pi_run(foc_pi_fixed_t *pi, int16_t err, uint16_t kp, uint16_t ki,
                   int16_t sat_max, int16_t sat_min);
void foc_vector_limit(foc_dq_t *vdvq, int16_t max_mag);
void foc_centered_svpwm(const foc_dq_t *vdvq, uint16_t phase, foc_abc_t *pwm_signed);
uint32_t foc_isqrt_u32(uint32_t x);

#endif
