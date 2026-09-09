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


/* sqrt(n) Q7 untuk n pada grid 0,256,...,65536. Tabel uint16 berada di
 * Flash. ISR menormalisasi uint32 ke mantissa 14/15-bit, interpolasi linear,
 * lalu koreksi integer bounded. Tidak ada loop sqrt bit-by-bit atau libm. */
static const uint16_t s_sqrt_q7[257] = {
    0u, 2048u, 2896u, 3547u, 4096u, 4579u, 5016u, 5418u, 5792u, 6144u, 6476u, 6792u,
    7094u, 7384u, 7662u, 7931u, 8192u, 8444u, 8688u, 8927u, 9158u, 9385u, 9605u, 9821u,
    10033u, 10240u, 10442u, 10641u, 10836u, 11028u, 11217u, 11402u, 11585u, 11764u, 11941u, 12116u,
    12288u, 12457u, 12624u, 12789u, 12952u, 13113u, 13272u, 13429u, 13584u, 13738u, 13890u, 14040u,
    14188u, 14336u, 14481u, 14625u, 14768u, 14909u, 15049u, 15188u, 15325u, 15462u, 15597u, 15730u,
    15863u, 15995u, 16125u, 16255u, 16384u, 16511u, 16638u, 16763u, 16888u, 17011u, 17134u, 17256u,
    17377u, 17498u, 17617u, 17736u, 17854u, 17971u, 18087u, 18203u, 18317u, 18432u, 18545u, 18658u,
    18770u, 18881u, 18992u, 19102u, 19211u, 19320u, 19429u, 19536u, 19643u, 19750u, 19856u, 19961u,
    20066u, 20170u, 20274u, 20377u, 20480u, 20582u, 20683u, 20784u, 20885u, 20985u, 21085u, 21184u,
    21283u, 21381u, 21479u, 21577u, 21673u, 21770u, 21866u, 21962u, 22057u, 22152u, 22246u, 22341u,
    22434u, 22528u, 22620u, 22713u, 22805u, 22897u, 22988u, 23079u, 23170u, 23260u, 23350u, 23440u,
    23529u, 23618u, 23707u, 23795u, 23883u, 23971u, 24058u, 24145u, 24232u, 24318u, 24404u, 24490u,
    24576u, 24661u, 24746u, 24830u, 24914u, 24999u, 25082u, 25166u, 25249u, 25332u, 25415u, 25497u,
    25579u, 25661u, 25742u, 25824u, 25905u, 25986u, 26066u, 26147u, 26227u, 26307u, 26386u, 26465u,
    26545u, 26624u, 26702u, 26781u, 26859u, 26937u, 27014u, 27092u, 27169u, 27246u, 27323u, 27400u,
    27476u, 27553u, 27629u, 27704u, 27780u, 27855u, 27930u, 28005u, 28080u, 28155u, 28229u, 28303u,
    28377u, 28451u, 28525u, 28598u, 28672u, 28745u, 28817u, 28890u, 28963u, 29035u, 29107u, 29179u,
    29251u, 29322u, 29394u, 29465u, 29536u, 29607u, 29678u, 29748u, 29819u, 29889u, 29959u, 30029u,
    30099u, 30168u, 30238u, 30307u, 30376u, 30445u, 30514u, 30583u, 30651u, 30720u, 30788u, 30856u,
    30924u, 30991u, 31059u, 31126u, 31194u, 31261u, 31328u, 31395u, 31461u, 31528u, 31595u, 31661u,
    31727u, 31793u, 31859u, 31925u, 31990u, 32056u, 32121u, 32186u, 32251u, 32316u, 32381u, 32446u,
    32510u, 32575u, 32639u, 32703u, 32768u,
};

int16_t foc_sat_s16(int32_t x) {
    if (x > 32767) return 32767;
    if (x < -32768) return -32768;
    return (int16_t)x;
}


void foc_sin_cos_q15(uint16_t phase, int16_t *s, int16_t *c) {
    /* 256 intervals + duplicated 360-deg endpoint make idx+1 always safe.
     * phase is Q0.16 turns: high byte selects the LUT interval and low byte
     * linearly interpolates it. Cosine is exactly a +90-deg (= +64 index)
     * shift, so its fractional byte is identical; avoid a second 16-bit phase
     * add/split in the ISR hot path. No float, division, modulo, sinf or cosf. */
    const uint8_t idx = (uint8_t)(phase >> 8);
    const uint8_t frac = (uint8_t)phase;
    int32_t y0 = s_sin_q15[idx];
    int32_t y1 = s_sin_q15[(uint16_t)idx + 1u];
    *s = (int16_t)(y0 + (((y1 - y0) * frac) >> 8));

    const uint8_t cidx = (uint8_t)(idx + 64u);
    y0 = s_sin_q15[cidx];
    y1 = s_sin_q15[(uint16_t)cidx + 1u];
    *c = (int16_t)(y0 + (((y1 - y0) * frac) >> 8));
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
    if(x==0u)return 0u;
    const uint32_t bit=31u-(uint32_t)__builtin_clz(x);
    const int32_t target=(bit&1u)?15:14;
    const int32_t shift=(int32_t)bit-target; /* selalu genap */
    const uint32_t n=shift>=0?(x>>(uint32_t)shift):(x<<(uint32_t)(-shift));
    const uint32_t idx=n>>8;
    const uint32_t frac=n&0xffu;
    const uint32_t y0=s_sqrt_q7[idx];
    const uint32_t yq7=y0+(((uint32_t)s_sqrt_q7[idx+1u]-y0)*frac>>8);
    uint32_t y=shift>=0?((yq7<<(uint32_t)(shift/2))>>7):
                         (yq7>>(uint32_t)(7+((-shift)/2)));
    /* Q7 lower-estimate maksimum 4 count pada domain uint32. Empat koreksi
     * bounded menghasilkan floor(sqrt(x)) persis tanpa latency data-dependent
     * dari algoritma restoring 16 iterasi. */
    for(uint8_t k=0u;k<4u && y<65535u;++k){
        const uint32_t yp=y+1u;
        if(yp*yp<=x)y=yp; else break;
    }
    return y;
}

/* Akar float ringan untuk commissioning/detect non-ISR. Seed berasal dari
 * eksponen IEEE-754 lalu empat Newton step. Tidak dipakai oleh ADC ISR; fungsi
 * ini menggantikan sqrtf/libm yang mahal pada tiga slow-path VESC detect. */
float foc_sqrtf_slow(float x) {
    if (!(x > 0.0f)) return 0.0f;
    union { float f; uint32_t u; } seed;
    seed.f = x;
    seed.u = (seed.u >> 1) + 0x1FC00000u;
    float g = seed.f;
    for (uint8_t k=0u;k<4u;++k) g = 0.5f * (g + x / g);
    return g;
}

/* atan2 ringan untuk commissioning/diagnostik non-ISR. Pada octant 0..45 deg
 * gunakan atan(r) ~= r*(pi/4 + 0.273*(1-r)); error teoritis sekitar 0,3 deg.
 * Output memakai phase Q0.16 agar langsung kompatibel dengan phase FOC. */
void foc_deadtime_sign_q15(int16_t alpha_q4, int16_t beta_q4, int32_t *sign_alpha_q15, int32_t *sign_beta_q15) {
    /* Tanda phase-current yang sama dengan update_valpha_vbeta() VESC.
     * Tidak perlu magnitude ampere: dead-time model hanya memakai SIGN(Ia/b/c).
     * Proyeksi B/C memakai -0.5*alpha +/- sqrt(3)/2*beta dalam Q15. */
    const int32_t a=(int32_t)alpha_q4;
    const int32_t bproj=-a*16384+(int32_t)beta_q4*FOC_SQRT3_BY_2_Q15;
    const int32_t cproj=-a*16384-(int32_t)beta_q4*FOC_SQRT3_BY_2_Q15;
    const int32_t sa=(a>0)-(a<0);
    const int32_t sb=(bproj>0)-(bproj<0);
    const int32_t sc=(cproj>0)-(cproj<0);
    if(sign_alpha_q15)*sign_alpha_q15=((2*sa-sb-sc)*32768)/3;
    if(sign_beta_q15)*sign_beta_q15=(sb-sc)*FOC_INV_SQRT3_Q15;
}

uint16_t foc_atan2_phase_u16(int32_t y, int32_t x) {
    if(x==0 && y==0)return 0u;
    const uint32_t ax=(uint32_t)(x<0?-(int64_t)x:(int64_t)x);
    const uint32_t ay=(uint32_t)(y<0?-(int64_t)y:(int64_t)y);
    const uint32_t hi=ax>ay?ax:ay;
    const uint32_t lo=ax>ay?ay:ax;
    uint32_t r_q15=hi?((uint32_t)(((uint64_t)lo<<15)/hi)):0u;
    if(r_q15>32768u)r_q15=32768u;
    const uint32_t correction=(2847u*(32768u-r_q15))>>15;
    const uint32_t base=(r_q15*(8192u+correction))>>15;
    uint32_t phase=ax>=ay?base:(16384u-base);
    if(x<0)phase=32768u-phase;
    if(y<0)phase=(65536u-phase)&0xffffu;
    return (uint16_t)phase;
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
