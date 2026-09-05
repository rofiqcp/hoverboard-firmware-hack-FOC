#include "motor/foc_math.h"

static const int16_t s_sin_q15[257] = {
    0, 804, 1608, 2410, 3212, 4011, 4808, 5602, 6393, 7179, 7962, 8739, 9512, 10278, 11039, 11793,
    12539, 13279, 14010, 14732, 15446, 16151, 16846, 17530, 18204, 18868, 19519, 20159, 20787, 21403, 22005, 22594,
    23170, 23731, 24279, 24811, 25329, 25832, 26319, 26790, 27245, 27683, 28105, 28510, 28898, 29268, 29621, 29956,
    30273, 30571, 30852, 31113, 31356, 31580, 31785, 31971, 32137, 32285, 32412, 32521, 32609, 32678, 32728, 32757,
    32767, 32757, 32728, 32678, 32609, 32521, 32412, 32285, 32137, 31971, 31785, 31580, 31356, 31113, 30852, 30571,
    30273, 29956, 29621, 29268, 28898, 28510, 28105, 27683, 27245, 26790, 26319, 25832, 25329, 24811, 24279, 23731,
    23170, 22594, 22005, 21403, 20787, 20159, 19519, 18868, 18204, 17530, 16846, 16151, 15446, 14732, 14010, 13279,
    12539, 11793, 11039, 10278, 9512, 8739, 7962, 7179, 6393, 5602, 4808, 4011, 3212, 2410, 1608, 804,
    0, -804, -1608, -2410, -3212, -4011, -4808, -5602, -6393, -7179, -7962, -8739, -9512, -10278, -11039, -11793,
    -12539, -13279, -14010, -14732, -15446, -16151, -16846, -17530, -18204, -18868, -19519, -20159, -20787, -21403, -22005, -22594,
    -23170, -23731, -24279, -24811, -25329, -25832, -26319, -26790, -27245, -27683, -28105, -28510, -28898, -29268, -29621, -29956,
    -30273, -30571, -30852, -31113, -31356, -31580, -31785, -31971, -32137, -32285, -32412, -32521, -32609, -32678, -32728, -32757,
    -32767, -32757, -32728, -32678, -32609, -32521, -32412, -32285, -32137, -31971, -31785, -31580, -31356, -31113, -30852, -30571,
    -30273, -29956, -29621, -29268, -28898, -28510, -28105, -27683, -27245, -26790, -26319, -25832, -25329, -24811, -24279, -23731,
    -23170, -22594, -22005, -21403, -20787, -20159, -19519, -18868, -18204, -17530, -16846, -16151, -15446, -14732, -14010, -13279,
    -12539, -11793, -11039, -10278, -9512, -8739, -7962, -7179, -6393, -5602, -4808, -4011, -3212, -2410, -1608, -804,
    0,
};

int16_t foc_sat_s16(int32_t x) {
    if (x > 32767) return 32767;
    if (x < -32768) return -32768;
    return (int16_t)x;
}


void foc_sin_cos_q15(uint16_t phase, int16_t *s, int16_t *c) {
    const uint8_t idx = (uint8_t)(phase >> 8);
    const uint8_t frac = (uint8_t)phase;
    int32_t y0 = s_sin_q15[idx];
    int32_t y1 = s_sin_q15[(uint16_t)idx + 1u];
    *s = (int16_t)(y0 + (((y1 - y0) * frac) >> 8));
    uint16_t cp = (uint16_t)(phase + 16384u);
    const uint8_t cidx = (uint8_t)(cp >> 8);
    const uint8_t cfrac = (uint8_t)cp;
    y0 = s_sin_q15[cidx];
    y1 = s_sin_q15[(uint16_t)cidx + 1u];
    *c = (int16_t)(y0 + (((y1 - y0) * cfrac) >> 8));
}

void foc_clarke_ab_q4(int16_t ia_q4, int16_t ib_q4, foc_ab_t *out) {
    /* Same AB shunt convention as the generated EFeru controller:
     * alpha=A, beta=(A+2B)/sqrt(3). */
    out->alpha = ia_q4;
    out->beta = foc_sat_s16(((int32_t)FOC_INV_SQRT3_Q15 *
                             ((int32_t)ia_q4 + 2 * (int32_t)ib_q4)) >> 15);
}

void foc_clarke_bc_q4(int16_t ib_q4, int16_t ic_q4, foc_ab_t *out) {
    /* Reconstruct A=-(B+C), then standard Clarke. */
    const int16_t ia = foc_sat_s16(-(int32_t)ib_q4 - (int32_t)ic_q4);
    out->alpha = ia;
    out->beta = foc_sat_s16(((int32_t)FOC_INV_SQRT3_Q15 *
                             ((int32_t)ib_q4 - (int32_t)ic_q4)) >> 15);
}

void foc_park_q4(const foc_ab_t *ab, uint16_t phase, foc_dq_t *dq) {
    int16_t s, c; foc_sin_cos_q15(phase, &s, &c);
    /* Preserve generated-controller axis convention: exported index0=iq, index1=id. */
    dq->q = foc_sat_s16((((int32_t)ab->beta * c) - ((int32_t)ab->alpha * s)) >> 15);
    dq->d = foc_sat_s16((((int32_t)ab->alpha * c) + ((int32_t)ab->beta * s)) >> 15);
}

void foc_inv_park(const foc_dq_t *vdvq, uint16_t phase, foc_ab_t *ab) {
    int16_t s, c; foc_sin_cos_q15(phase, &s, &c);
    ab->alpha = foc_sat_s16((((int32_t)vdvq->d * c) - ((int32_t)vdvq->q * s)) >> 15);
    ab->beta  = foc_sat_s16((((int32_t)vdvq->d * s) + ((int32_t)vdvq->q * c)) >> 15);
}


uint32_t foc_isqrt_u32(uint32_t x) {
    uint32_t op=x, res=0, one=1u<<30;
    while (one>op) one>>=2;
    while (one!=0) {
        if (op>=res+one) { op-=res+one; res=(res>>1)+one; } else { res>>=1; }
        one>>=2;
    }
    return res;
}

void foc_vector_limit(foc_dq_t *v, int16_t max_mag) {
    uint32_t mag2=(uint32_t)((int32_t)v->d*v->d)+(uint32_t)((int32_t)v->q*v->q);
    uint32_t max2=(uint32_t)((int32_t)max_mag*max_mag);
    if (mag2<=max2 || mag2==0) return;
    uint32_t mag=foc_isqrt_u32(mag2);
    if (!mag) return;
    v->d=(int16_t)(((int32_t)v->d*max_mag)/(int32_t)mag);
    v->q=(int16_t)(((int32_t)v->q*max_mag)/(int32_t)mag);
}

void foc_centered_svpwm(const foc_dq_t *vdvq, uint16_t phase, foc_abc_t *pwm_signed) {
    foc_ab_t ab; foc_inv_park(vdvq, phase, &ab);
    int32_t a=ab.alpha;
    int32_t b=-(ab.alpha>>1)+(((int32_t)FOC_SQRT3_BY_2_Q15*ab.beta)>>15);
    int32_t c=-(ab.alpha>>1)-(((int32_t)FOC_SQRT3_BY_2_Q15*ab.beta)>>15);
    int32_t vmax=a; if (b>vmax)vmax=b; if(c>vmax)vmax=c;
    int32_t vmin=a; if (b<vmin)vmin=b; if(c<vmin)vmin=c;
    int32_t common=(vmax+vmin)>>1;
    a-=common; b-=common; c-=common;
    /* Exact generated-FERU scaling: gain 18919 / 2^14, then DC output >>4. */
    pwm_signed->a=foc_sat_s16((((int32_t)18919*a)>>14)>>4);
    pwm_signed->b=foc_sat_s16((((int32_t)18919*b)>>14)>>4);
    pwm_signed->c=foc_sat_s16((((int32_t)18919*c)>>14)>>4);
}
