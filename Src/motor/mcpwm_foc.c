#include <string.h>
#include <stdlib.h>
#include <limits.h>
#include "stm32f1xx_hal.h"
#include "config.h"
#include "defines.h"
#include "setup.h"
#include "util.h"
#include "motor/mcconf_default.h"
#include "motor/foc_math.h"
#include "motor/mcpwm_foc.h"

/* ========================================================================== */
/* Bare-metal VESC-style dual FOC state                                       */
/* ========================================================================== */
mcpwm_foc_motor_t m_motor_1;
mcpwm_foc_motor_t m_motor_2;

extern volatile adc_buf_t adc_buffer;
extern uint8_t ctrlModReq;

volatile int pwml = 0;
volatile int pwmr = 0;
uint8_t buzzerFreq = 0;
uint8_t buzzerPattern = 0;
uint8_t buzzerCount = 0;
volatile uint32_t buzzerTimer = 0;
static uint8_t buzzerPrev = 0;
static uint8_t buzzerIdx = 0;
uint8_t enable = 0;
volatile uint8_t motorRunReq = 1u;
volatile uint16_t svpwmOpenloopRpm = SVPWM_OPENLOOP_RPM_DEFAULT;
volatile int32_t positionCommandL = 0;
volatile int32_t positionCommandR = 0;

volatile uint32_t foc_isr_cycles = 0;
volatile uint32_t foc_isr_cycles_max = 0;
volatile int16_t foc_iqL_q4 = 0;
volatile int16_t foc_iqR_q4 = 0;
volatile int16_t foc_idL_q4 = 0;
volatile int16_t foc_idR_q4 = 0;

int16_t curL_phaA = 0, curL_phaB = 0, curL_DC = 0;
int16_t curR_phaB = 0, curR_phaC = 0, curR_DC = 0;
int16_t batVoltage = (400 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE;
static int32_t batVoltageFixdt = (400 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE << 16;

/* ADC offset calibration is intentionally identical to the proven EFeru ISR. */
static uint16_t offsetcount = 0;
static int16_t offsetrlA = 2000;
static int16_t offsetrlB = 2000;
static int16_t offsetrrB = 2000;
static int16_t offsetrrC = 2000;
static int16_t offsetdcl = 2000;
static int16_t offsetdcr = 2000;

static const uint16_t pwm_res = 64000000 / 2 / PWM_FREQ; /* 2000 */
static int16_t pwm_margin = MCCONF_PWM_MARGIN_COUNTS;
static int16_t curDC_max  = (I_DC_MAX * A2BIT_CONV);

int16_t odom_l = 0, odom_r = 0;
static volatile uint8_t s_overrun = 0;
/* VESC command ownership is separate from its safety timeout. Upstream VESC
 * keeps the last motor setpoint while COMM_ALIVE resets timeout_reset(); when
 * the timeout expires the motor is stopped/braked, but an unrelated legacy
 * input source must not immediately overwrite that VESC state. */
static volatile uint8_t s_vesc_owned[2] = {0u, 0u};
static volatile uint8_t s_vesc_timeout_braking[2] = {0u, 0u};
static volatile uint32_t s_vesc_timeout_ticks[2] = {0u, 0u};
static volatile uint32_t s_vesc_timeout_ms[2] = {1000u, 1000u};
static volatile float s_vesc_timeout_brake_a[2] = {0.0f, 0.0f};
static uint32_t s_energy_last_ms = 0u;
static uint8_t s_foc_control_div = 0u;

/* VESC FOC Hall table: 0..199 = 0..360 electrical degrees, 255 = invalid.
 * These defaults reproduce the previously proven hard-coded sector centers. */
static const uint8_t s_default_foc_hall_table[8] = {255u, 83u, 17u, 50u, 150u, 117u, 183u, 255u};

static uint16_t default_motor_poles(bool second) {
#if MCCONF_POLE_PAIRS_LEFT == MCCONF_POLE_PAIRS_RIGHT
    (void)second;
    return (uint16_t)(2u * MCCONF_POLE_PAIRS_LEFT);
#else
    return (uint16_t)(2u * (second ? MCCONF_POLE_PAIRS_RIGHT : MCCONF_POLE_PAIRS_LEFT));
#endif
}

static uint16_t motor_pole_pairs(bool second) {
    const mc_configuration *c = second ? &m_motor_2.m_conf : &m_motor_1.m_conf;
    uint16_t poles = c->si_motor_poles;
    if (poles < 2u || (poles & 1u)) {
        poles = default_motor_poles(second);
    }
    return (uint16_t)(poles / 2u);
}

static float motor_gear_ratio(bool second) {
    const float ratio = (second ? m_motor_2.m_conf.si_gear_ratio : m_motor_1.m_conf.si_gear_ratio);
    return (ratio >= 0.01f && ratio <= 1000.0f) ? ratio : 1.0f;
}

static int32_t user_position_to_internal(int32_t position_counts, bool second) {
    if (!second) return position_counts;
    /* Right hardware/Hall direction is mirrored. Avoid UB on -INT32_MIN. */
    return (position_counts == INT32_MIN) ? INT32_MAX : -position_counts;
}

static int32_t internal_position_to_user(int32_t position_counts, bool second) {
    if (!second) return position_counts;
    return (position_counts == INT32_MIN) ? INT32_MAX : -position_counts;
}

static int16_t erpm_to_mech_rpm(float erpm, bool second) {
    const float pp = (float)motor_pole_pairs(second);
    float mech = (pp > 0.0f) ? (erpm / pp) : erpm;
    if (mech > (float)MCCONF_MOTOR_RPM_MAX) mech = (float)MCCONF_MOTOR_RPM_MAX;
    if (mech < -(float)MCCONF_MOTOR_RPM_MAX) mech = -(float)MCCONF_MOTOR_RPM_MAX;
    return (int16_t)mech;
}

static int32_t erpm_to_mech_rpm_q16(float erpm, bool second) {
    const float pp = (float)motor_pole_pairs(second);
    float mech = (pp > 0.0f) ? (erpm / pp) : erpm;
    if (mech > (float)MCCONF_MOTOR_RPM_MAX) mech = (float)MCCONF_MOTOR_RPM_MAX;
    if (mech < -(float)MCCONF_MOTOR_RPM_MAX) mech = -(float)MCCONF_MOTOR_RPM_MAX;
    const float scaled = mech * 65536.0f;
    return (int32_t)(scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f);
}

static int32_t measured_mech_rpm_q16(const mcpwm_foc_motor_t *m, bool second) {
    /* One Hall transition is 60 electrical degrees, i.e. six transitions per
     * electrical revolution. At a 16-kHz estimator tick rate this is
     * ERPM = PWM_FREQ * 10 / hall_period. Keep the mechanical speed in Q16 so
     * a VESC target such as 50 ERPM @15 pole-pairs remains 3.333... RPM instead
     * of being truncated to 3 RPM inside the controller. */
    if (m->m_hall_initialized && m->m_hall_direction != 0 &&
        m->m_hall_period > 0u && m->m_hall_period < MCCONF_HALL_TIMEOUT_TICKS &&
        m->m_hall_ticks <= MCCONF_HALL_TIMEOUT_TICKS) {
        const uint32_t pp = motor_pole_pairs(second);
        if (pp > 0u) {
            /* Cortex-M3 64-bit integer division is a software helper and was
             * one of the largest speed-loop ISR costs. Q12 mechanical RPM is
             * already far finer than Hall timing resolution, so compute with a
             * single 32-bit divide then promote to Q16. Numerator 16000*10*4096
             * is 655,360,000 and safely fits uint32_t. */
            const uint32_t den = (uint32_t)m->m_hall_period * pp;
            uint32_t mag_q12 = ((uint32_t)PWM_FREQ * 10u * 4096u) / den;
            int32_t q16 = (int32_t)(mag_q12 << 4);
            if (m->m_hall_direction < 0) q16 = -q16;
            return q16;
        }
    }
    return (int32_t)m->m_rpm << 16;
}

static int16_t voltage_circle_q_limit(int16_t vd, int16_t vmax) {
    const int32_t d = vd;
    const int32_t max = vmax;
    const uint32_t d2 = (uint32_t)(d * d);
    const uint32_t max2 = (uint32_t)(max * max);
    if (d2 >= max2) return 0;
    return (int16_t)foc_isqrt_u32(max2 - d2);
}

static int16_t current_circle_iq_limit_q4(const mcpwm_foc_motor_t *m, int16_t iq_cmd_q4) {
    /* Motor-current limit and VESC input-current limit are distinct. Estimate
     * |Ibus| ~= |Iq|*|duty| for the current envelope, then keep the requested
     * Iq inside whichever limit is tighter. The raw DC shunt retains the 17 A
     * immediate hard-fault layer below. */
    int32_t lim = (m && m->m_current_limit_q4 > 0) ?
                  m->m_current_limit_q4 : MCCONF_MOTOR_CURRENT_MAX_Q4;
    if (m) {
        int32_t duty=m->m_duty_now_permille;
        if (m->m_control_mode==CONTROL_MODE_DUTY) {
            const int32_t rd=m->m_duty_ramp_permille;
            if (ABS(rd)>ABS(duty)) duty=rd;
        }
        const int32_t ad=ABS(duty);
        if (ad > 0) {
            const bool drawing=((iq_cmd_q4>=0)==(duty>=0));
            const int32_t in_lim=drawing?m->m_input_current_max_q4:m->m_input_current_regen_q4;
            if (in_lim > 0) {
                int32_t motor_from_input=(in_lim*1000)/ad;
                if (motor_from_input < lim) lim=motor_from_input;
            }
        }
    }
    if (lim < 1) lim=1;
    int32_t id = m ? m->m_id_q4 : 0;
    if (id < 0) id = -id;
    if (id >= lim) return 0;
    const uint32_t lim2 = (uint32_t)(lim * lim);
    const uint32_t id2 = (uint32_t)(id * id);
    const int32_t qmax = (int32_t)foc_isqrt_u32(lim2 - id2);
    return (int16_t)CLAMP((int32_t)iq_cmd_q4, -qmax, qmax);
}

static void duty_setpoint_slew_step(mcpwm_foc_motor_t *m) {
    int32_t target=m->m_duty_set_permille;
    const int32_t lim=m->m_duty_limit_permille>0?m->m_duty_limit_permille:1000;
    target=CLAMP(target,-lim,lim);
    int32_t now=m->m_duty_ramp_permille;
    int32_t step=m->m_duty_ramp_step_permille;
    if(step<1)step=1;
    if(now<target){now+=step;if(now>target)now=target;}
    else if(now>target){now-=step;if(now<target)now=target;}
    m->m_duty_ramp_permille=(int16_t)now;
}

static void foc_isr_monitor_end(uint32_t start) {
    uint32_t elapsed = DWT->CYCCNT - start;
    foc_isr_cycles = elapsed;
    if (elapsed > foc_isr_cycles_max) foc_isr_cycles_max = elapsed;
}

static void conf_defaults(mc_configuration *c, bool second) {
    memset(c, 0, sizeof(*c));
    c->motor_type = MOTOR_TYPE_FOC;
    c->sensor_mode = SENSOR_MODE_SENSORED;
    c->foc_sensor_mode = FOC_SENSOR_MODE_HALL;
    c->l_current_max = MCCONF_L_CURRENT_MAX;
    c->l_current_min = MCCONF_L_CURRENT_MIN;
    c->l_abs_current_max = MCCONF_L_ABS_CURRENT_MAX;
    /* This two-low-side-shunt board cannot safely use reconstructed raw phase
     * samples as an ABS source near switching boundaries. "Fast" therefore
     * means the FOC feedback D/Q current; "slow" means the additional VESC
     * monitoring LPF. Both preserve the VESC setting semantics without using an
     * unobservable raw phase as a fault source. */
    c->l_slow_abs_current = false;
    c->l_min_duty = MCCONF_L_MIN_DUTY;
    c->l_max_duty = MCCONF_L_MAX_DUTY;
    c->m_fault_stop_time_ms = (int32_t)MCCONF_FAULT_STOP_TIME_MS;
    c->m_duty_ramp_step = MCCONF_DUTY_RAMP_STEP_DEFAULT;
    c->cc_min_current = MCCONF_CC_MIN_CURRENT;
    c->foc_duty_dowmramp_kp = MCCONF_FOC_DUTY_DOWNRAMP_KP;
    c->foc_duty_dowmramp_ki = MCCONF_FOC_DUTY_DOWNRAMP_KI;
    c->l_in_current_max = MCCONF_L_IN_CURRENT_MAX;
    c->l_in_current_min = MCCONF_L_IN_CURRENT_MIN;
    c->l_max_erpm = MCCONF_L_MAX_ERPM;
    c->l_min_erpm = MCCONF_L_MIN_ERPM;
    /* Expose VESC configuration in physical units. The ISR remains fixed-point:
     * Kp ~= 0.800 V/A and Ki ~= 266.7 V/(A*s) at the 5.333-kHz control cadence. */
    c->foc_current_kp = 0.80013f;
    c->foc_current_ki = 266.710f;
    c->foc_current_filter_const = MCCONF_FOC_TELEMETRY_FILTER_DEFAULT;
    c->foc_openloop_rpm = (float)MCCONF_OPENLOOP_RPM_DEFAULT;
    c->foc_hall_interp_erpm = 0.0f;
    c->s_pid_ramp_erpms_s = (float)MCCONF_SPEED_RAMP_ERPMS_S;
    c->s_pid_min_erpm = (float)MCCONF_SPEED_RELEASE_ERPM;
    c->s_pid_allow_braking = true;
    c->s_pid_kp = (float)MCCONF_SPEED_KP_Q11 / (float)MCCONF_SPEED_GAIN_SCALE;
    c->s_pid_ki = (float)MCCONF_SPEED_KI_Q16 / (float)MCCONF_SPEED_GAIN_SCALE;
    c->s_pid_kd = (float)MCCONF_SPEED_KD_Q11 / (float)MCCONF_SPEED_GAIN_SCALE;
    c->p_pid_kp = (float)MCCONF_POSITION_KP_Q11 / 1000.0f;
    c->p_pid_ki = (float)MCCONF_POSITION_KI_Q16 / 1000.0f;
    c->p_pid_kd = (float)MCCONF_POSITION_KD_Q11 / 1000.0f;
    c->p_pid_kd_filter = (float)MCCONF_POSITION_KD_FILTER_Q16 / 65536.0f;
    c->p_pid_kd_proc = 0.00035f; /* upstream VESC default process-D damping */
    c->p_pid_ang_div = 1.0f;
    c->si_motor_poles = (uint8_t)default_motor_poles(second);
    c->si_gear_ratio = 1.0f; /* direct drive; set >1 in VESC Tool for a gearbox */
    for (int i=0;i<8;i++) c->foc_hall_table[i] = (int8_t)s_default_foc_hall_table[i];
}

void mcpwm_foc_get_default_configuration(mc_configuration *conf, bool second) {
    if (conf) conf_defaults(conf, second);
}

static void speed_pid_recompute_coeff(mcpwm_foc_motor_t *m) {
    if (!m) return;
    const float kp=(float)m->m_kps_q11/(float)MCCONF_SPEED_GAIN_SCALE;
    const float ki=(float)m->m_kis_q16/(float)MCCONF_SPEED_GAIN_SCALE;
    const float kd=(float)m->m_kds_q11/(float)MCCONF_SPEED_GAIN_SCALE;
    const float lim=(float)(m->m_current_limit_q4>0?m->m_current_limit_q4:MCCONF_MOTOR_CURRENT_MAX_Q4);
    /* For error_q2 = ERPM*4:
     *   P_norm_q15 = error_q2 * kp * 409.6
     *   I_step(q4Q16) = error_q2 * ki * IqLimit(q4) * 0.1536
     *   D_q4 = delta_error_q2 * kd * IqLimit(q4) * 66.6666667
     * Store extra fractional bits once here; ISR uses only 64-bit multiply and
     * right-shift, which Cortex-M3 handles far cheaper than software division. */
    double v=(double)kp*26843545.6;
    if(v<0.0)v=0.0;
    if(v>4294967295.0)v=4294967295.0;
    m->m_speed_kp_coeff_q16=(uint32_t)(v+0.5);
    v=(double)ki*(double)lim*10066.3296;
    if(v<0.0)v=0.0;
    if(v>4294967295.0)v=4294967295.0;
    m->m_speed_ki_coeff_q16=(uint32_t)(v+0.5);
    v=(double)kd*(double)lim*17066.6666667;
    if(v<0.0)v=0.0;
    if(v>4294967295.0)v=4294967295.0;
    m->m_speed_kd_coeff_q8=(uint32_t)(v+0.5);
}

static void position_pid_recompute_coeff(mcpwm_foc_motor_t *m) {
    if (!m) return;
    /* Count-position process D uses signed mechanical RPM -> electrical deg/s. */
    double v=(double)m->m_conf.p_pid_kd_proc*6.0*32768.0*65536.0;
    if(v<0.0)v=0.0;
    if(v>4294967295.0)v=4294967295.0;
    m->m_position_kd_proc_coeff_q16=(uint32_t)(v+0.5);
    /* Stock VESC SET_POS tracks electrical phase directly. Cache a coefficient
     * for -d(phase)/dt * kd_proc in normalized Q15, preserving the sign of the
     * actual phase delta instead of relying on Hall ERPM sign conventions. */
    v=(double)m->m_conf.p_pid_kd_proc*(double)PWM_FREQ*180.0*16.0/
      (double)MCCONF_FOC_CONTROL_DIV;
    if(v<0.0)v=0.0;
    if(v>65535.0)v=65535.0;
    m->m_position_kd_proc_phase_coeff_q4=(uint16_t)(v+0.5);
}

static bool hall_table_runtime_sane(const uint8_t t[8]) {
    uint8_t u[8], sorted[6];
    for (uint8_t i=0u;i<8u;++i) u[i]=(uint8_t)t[i];
    if (u[0]!=255u || u[7]!=255u) return false;
    for (uint8_t h=1u;h<=6u;++h) { if (u[h]>=200u) return false; sorted[h-1u]=u[h]; }
    for (uint8_t i=0u;i<5u;++i) for(uint8_t j=(uint8_t)(i+1u);j<6u;++j)
        if(sorted[j]<sorted[i]){uint8_t x=sorted[i];sorted[i]=sorted[j];sorted[j]=x;}
    for (uint8_t i=0u;i<6u;++i) {
        const uint16_t a=sorted[i], b=(i==5u)?(uint16_t)sorted[0]+200u:sorted[i+1u];
        const uint16_t gap=b-a; if(gap<18u || gap>48u) return false;
    }
    return true;
}

static void motor_fault_set(mcpwm_foc_motor_t *m, mc_fault_code code) {
    uint32_t ms = m->m_conf.m_fault_stop_time_ms > 0 ? (uint32_t)m->m_conf.m_fault_stop_time_ms : MCCONF_FAULT_STOP_TIME_MS;
    if (ms < 50u) ms = 50u;
    m->m_fault = code;
    m->m_fault_recovery_ticks = (uint32_t)(((uint64_t)ms * (uint64_t)PWM_FREQ + 999u) / 1000u);
    if (m->m_fault_recovery_ticks == 0u) m->m_fault_recovery_ticks = 1u;
}

static void motor_fault_recovery_tick(mcpwm_foc_motor_t *m) {
    if (m->m_fault == FAULT_CODE_NONE) return;
    if (m->m_fault_recovery_ticks > 0u) m->m_fault_recovery_ticks--;
    if (m->m_fault_recovery_ticks == 0u) {
        m->m_fault = FAULT_CODE_NONE;
        m->m_state = MC_STATE_OFF;
    }
}

static void motor_reset(mcpwm_foc_motor_t *m, bool second) {
    memset(m, 0, sizeof(*m));
    conf_defaults(&m->m_conf, second);
    m->m_state = MC_STATE_OFF;
    m->m_control_mode = CONTROL_MODE_NONE;
    m->m_fault = FAULT_CODE_NONE;
    m->m_kpq_q11=MCCONF_FOC_CURRENT_KP_Q11; m->m_kiq_q16=MCCONF_FOC_CURRENT_KI_Q16;
    m->m_kpd_q11=MCCONF_FOC_ID_KP_Q11; m->m_kid_q16=MCCONF_FOC_ID_KI_Q16;
    m->m_kps_q11=MCCONF_SPEED_KP_Q11; m->m_kis_q16=MCCONF_SPEED_KI_Q16; m->m_kds_q11=MCCONF_SPEED_KD_Q11;
    m->m_kpp_q11=MCCONF_POSITION_KP_Q11; m->m_kip_q16=MCCONF_POSITION_KI_Q16; m->m_kdp_q11=MCCONF_POSITION_KD_Q11;
    m->m_position_kd_filter_q16=MCCONF_POSITION_KD_FILTER_Q16;
    m->m_position_dt_ticks=1u;
    m->m_position_min_counts=INT32_MIN; m->m_position_max_counts=INT32_MAX;
    m->m_current_limit_q4=MCCONF_MOTOR_CURRENT_MAX_Q4;
    m->m_input_current_max_q4=(int16_t)(MCCONF_L_IN_CURRENT_MAX*FOC_CURRENT_Q4_PER_A+0.5f);
    m->m_input_current_regen_q4=(int16_t)(-MCCONF_L_IN_CURRENT_MIN*FOC_CURRENT_Q4_PER_A+0.5f);
    m->m_duty_ramp_step_permille=(uint16_t)(MCCONF_DUTY_RAMP_STEP_DEFAULT*1000.0f+0.5f);
    speed_pid_recompute_coeff(m);
    position_pid_recompute_coeff(m);
    m->m_abs_current_limit_counts=(int16_t)(MCCONF_L_ABS_CURRENT_MAX*(float)A2BIT_CONV+0.5f);
    m->m_duty_limit_permille=(int16_t)(MCCONF_L_MAX_DUTY*1000.0f+0.5f);
    m->m_telem_current_filter_q16=(uint16_t)(MCCONF_FOC_TELEMETRY_FILTER_DEFAULT*65535.0f+0.5f);
    {
        const uint16_t pp = motor_pole_pairs(second);
        m->m_speed_ramp_rpm_s = (uint16_t)(MCCONF_SPEED_RAMP_ERPMS_S / pp);
        if (m->m_speed_ramp_rpm_s == 0u) m->m_speed_ramp_rpm_s = 1u;
        m->m_speed_release_rpm = (uint16_t)(MCCONF_SPEED_RELEASE_ERPM / pp);
        if (m->m_speed_release_rpm == 0u) m->m_speed_release_rpm = 1u;
    }
    {
        const float kscale=(32768.0f*4096.0f)/(1000.0f*MCCONF_DUTY_PI_BUS_NOMINAL_V);
        const float dt=(float)MCCONF_FOC_CONTROL_DIV/(float)PWM_FREQ;
        m->m_duty_kp_q12_per_permille=(uint32_t)(MCCONF_FOC_DUTY_DOWNRAMP_KP*kscale+0.5f);
        m->m_duty_ki_q12_per_permille=(uint32_t)(MCCONF_FOC_DUTY_DOWNRAMP_KI*dt*kscale+0.5f);
    }
    m->m_hall_pos_prev = 0;
    m->m_hall_reject_counted_state = 0xffu;
    m->m_hall_period = MCCONF_HALL_TIMEOUT_TICKS;
    for (int i=0;i<4;i++) m->m_hall_period_hist[i] = MCCONF_HALL_TIMEOUT_TICKS;
    m->m_phase_openloop = SVPWM_ALIGN_PHASE;
    m->m_openloop_phase_acc_q32 = ((uint32_t)SVPWM_ALIGN_PHASE) << 16;
}

void mcpwm_foc_init(void) {
    motor_reset(&m_motor_1, false);
    motor_reset(&m_motor_2, true);
    offsetcount = 0;
    offsetrlA = offsetrlB = offsetrrB = offsetrrC = offsetdcl = offsetdcr = 2000;
    m_motor_1.m_driven_offset0=offsetrlA; m_motor_1.m_driven_offset1=offsetrlB; m_motor_1.m_driven_offsetdc=offsetdcl;
    m_motor_2.m_driven_offset0=offsetrrB; m_motor_2.m_driven_offset1=offsetrrC; m_motor_2.m_driven_offsetdc=offsetdcr;
    m_motor_1.m_off_offset0=offsetrlA; m_motor_1.m_off_offset1=offsetrlB; m_motor_1.m_off_offsetdc=offsetdcl;
    m_motor_2.m_off_offset0=offsetrrB; m_motor_2.m_off_offset1=offsetrrC; m_motor_2.m_off_offsetdc=offsetdcr;
    foc_isr_cycles = foc_isr_cycles_max = 0;
    s_overrun = 0;
    s_vesc_owned[0]=s_vesc_owned[1]=0u;
    s_vesc_timeout_braking[0]=s_vesc_timeout_braking[1]=0u;
    s_vesc_timeout_ticks[0]=s_vesc_timeout_ticks[1]=0u;
    s_vesc_timeout_ms[0]=s_vesc_timeout_ms[1]=1000u;
    s_vesc_timeout_brake_a[0]=s_vesc_timeout_brake_a[1]=0.0f;
    s_energy_last_ms = 0u;
    s_foc_control_div = 0u;
}

mcpwm_foc_motor_t *mcpwm_foc_get_motor(bool second) { return second ? &m_motor_2 : &m_motor_1; }
const mcpwm_foc_motor_t *mcpwm_foc_get_motor_const(bool second) { return second ? &m_motor_2 : &m_motor_1; }

static int16_t amp_to_q4(const mcpwm_foc_motor_t *m, float current);

void mcpwm_foc_set_configuration(const mc_configuration *conf, bool second) {
    if (!conf) return;
    mcpwm_foc_motor_t *m = mcpwm_foc_get_motor(second);
    mc_configuration next = *conf;
    /* VESC stores the number of motor poles (not pole-pairs). FOC speed math
     * uses this value at runtime, so changing Motor Poles in VESC Tool really
     * changes ERPM <-> mechanical RPM conversion without recompiling. */
    if (next.si_motor_poles < 2u || (next.si_motor_poles & 1u)) {
        next.si_motor_poles = m->m_conf.si_motor_poles;
        if (next.si_motor_poles < 2u || (next.si_motor_poles & 1u))
            next.si_motor_poles = (uint8_t)default_motor_poles(second);
    }
    if (!(next.si_gear_ratio >= 0.01f && next.si_gear_ratio <= 1000.0f)) next.si_gear_ratio = 1.0f;
    if (!(next.l_max_duty > 0.0f) || next.l_max_duty > MCCONF_L_MAX_DUTY) next.l_max_duty=MCCONF_L_MAX_DUTY;
    if (!(next.l_in_current_max >= 0.1f) || next.l_in_current_max > (float)I_DC_MAX) next.l_in_current_max=MCCONF_L_IN_CURRENT_MAX;
    if (!(next.l_in_current_min <= -0.1f) || next.l_in_current_min < -(float)I_DC_MAX) next.l_in_current_min=MCCONF_L_IN_CURRENT_MIN;
    if (!(next.m_duty_ramp_step >= 0.0001f && next.m_duty_ramp_step <= 0.20f)) next.m_duty_ramp_step=MCCONF_DUTY_RAMP_STEP_DEFAULT;
    if (!(next.cc_min_current >= 0.001f && next.cc_min_current <= 1.0f)) next.cc_min_current=MCCONF_CC_MIN_CURRENT;
    /* Absolute phase-current fault must cover both motoring and regenerative
     * current ranges. Checking only l_current_max lets a large negative
     * l_current_min exceed the ABS threshold during braking. Keep the VESC Tool
     * field authoritative when valid, bounded by the board hard ceiling. */
    {
        float commanded_abs = next.l_current_max;
        if (-next.l_current_min > commanded_abs) commanded_abs = -next.l_current_min;
        if (!(next.l_abs_current_max >= commanded_abs) ||
            next.l_abs_current_max > MCCONF_L_ABS_CURRENT_MAX) {
            next.l_abs_current_max = MCCONF_L_ABS_CURRENT_MAX;
        }
    }
    if (!(next.foc_duty_dowmramp_kp > 0.0f)) next.foc_duty_dowmramp_kp=MCCONF_FOC_DUTY_DOWNRAMP_KP;
    if (!(next.foc_duty_dowmramp_ki > 0.0f)) next.foc_duty_dowmramp_ki=MCCONF_FOC_DUTY_DOWNRAMP_KI;
    /* Upstream uses foc_current_filter_const for less time-critical filtered
     * currents. Keep the current-loop feedback filter independent and make this
     * standard MC-config field control monitoring smoothness only. */
    if (!(next.foc_current_filter_const >= 0.001f && next.foc_current_filter_const <= 1.0f))
        next.foc_current_filter_const=MCCONF_FOC_TELEMETRY_FILTER_DEFAULT;
    if (!(next.p_pid_kd_filter >= 0.0f && next.p_pid_kd_filter <= 1.0f))
        next.p_pid_kd_filter=(float)MCCONF_POSITION_KD_FILTER_Q16/65536.0f;
    const bool poles_changed = m->m_conf.si_motor_poles != next.si_motor_poles;
    /* Never let a malformed VESC Tool/EEPROM Hall table become the live FOC
     * angle source. Preserve the last known-good table while still accepting
     * the other configuration fields. */
    if (!hall_table_runtime_sane(next.foc_hall_table)) {
        for (uint8_t h=0u;h<8u;++h) next.foc_hall_table[h]=m->m_conf.foc_hall_table[h];
    }
    m->m_conf = next;
    if (poles_changed) {
        /* A live pole-count change would instantly rescale the speed loop.
         * Release first, matching upstream's stop-on-structural-config-change policy. */
        mcpwm_foc_release_motor(second);
        m->m_speed_set_rpm = 0;
        m->m_speed_target_rpm = 0;
        m->m_speed_target_rpm_q16 = 0;
        m->m_speed_set_ramp_q16 = 0;
    }

    /* VESC configuration uses electrical units. Convert once outside the ISR
     * and keep the actual speed-loop ramp integer/fixed-point. */
    const float pp = (float)motor_pole_pairs(second);
    float ramp_mech = next.s_pid_ramp_erpms_s / pp;
    if (ramp_mech < 1.0f) ramp_mech = 1.0f;
    if (ramp_mech > 5000.0f) ramp_mech = 5000.0f;
    m->m_speed_ramp_rpm_s = (uint16_t)(ramp_mech + 0.5f);

    float release_mech = next.s_pid_min_erpm / pp;
    if (release_mech < 1.0f) release_mech = 1.0f;
    if (release_mech > 100.0f) release_mech = 100.0f;
    m->m_speed_release_rpm = (uint16_t)(release_mech + 0.5f);
    int32_t kpc=(int32_t)(conf->foc_current_kp*1536.0f+0.5f);
    int32_t kic=(int32_t)(conf->foc_current_ki*4.608f+0.5f);
    kpc=CLAMP(kpc,0,65535); kic=CLAMP(kic,0,65535);
    m->m_kpq_q11=m->m_kpd_q11=(uint16_t)kpc;
    m->m_kiq_q16=m->m_kid_q16=(uint16_t)kic;
    m->m_kps_q11=(uint16_t)CLAMP((int32_t)(next.s_pid_kp*(float)MCCONF_SPEED_GAIN_SCALE+0.5f),0,65535);
    m->m_kis_q16=(uint16_t)CLAMP((int32_t)(next.s_pid_ki*(float)MCCONF_SPEED_GAIN_SCALE+0.5f),0,65535);
    m->m_kds_q11=(uint16_t)CLAMP((int32_t)(next.s_pid_kd*(float)MCCONF_SPEED_GAIN_SCALE+0.5f),0,65535);
    m->m_kpp_q11=(uint16_t)CLAMP((int32_t)(conf->p_pid_kp*1000.0f+0.5f),0,65535);
    m->m_kip_q16=(uint16_t)CLAMP((int32_t)(conf->p_pid_ki*1000.0f+0.5f),0,65535);
    m->m_kdp_q11=(uint16_t)CLAMP((int32_t)(conf->p_pid_kd*1000.0f+0.5f),0,65535);
    m->m_position_kd_filter_q16=(uint16_t)CLAMP((int32_t)(next.p_pid_kd_filter*65535.0f+0.5f),0,65535);
    m->m_current_limit_q4=amp_to_q4(m,next.l_current_max);
    m->m_input_current_max_q4=(int16_t)CLAMP((int32_t)(next.l_in_current_max*FOC_CURRENT_Q4_PER_A+0.5f),1,I_DC_MAX*FOC_CURRENT_Q4_PER_A);
    m->m_input_current_regen_q4=(int16_t)CLAMP((int32_t)(-next.l_in_current_min*FOC_CURRENT_Q4_PER_A+0.5f),1,I_DC_MAX*FOC_CURRENT_Q4_PER_A);
    m->m_duty_ramp_step_permille=(uint16_t)CLAMP((int32_t)(next.m_duty_ramp_step*1000.0f+0.5f),1,200);
    {
        int32_t a=(int32_t)(next.foc_current_filter_const*65535.0f+0.5f);
        m->m_telem_current_filter_q16=(uint16_t)CLAMP(a,1,65535);
    }
    speed_pid_recompute_coeff(m);
    position_pid_recompute_coeff(m);
    m->m_abs_current_limit_counts=(int16_t)CLAMP((int32_t)(next.l_abs_current_max*(float)A2BIT_CONV+0.5f),1,32767);
    m->m_duty_limit_permille=(int16_t)CLAMP((int32_t)(next.l_max_duty*1000.0f+0.5f),1,1000);
    {
        const float kscale=(32768.0f*4096.0f)/(1000.0f*MCCONF_DUTY_PI_BUS_NOMINAL_V);
        const float dt=(float)MCCONF_FOC_CONTROL_DIV/(float)PWM_FREQ;
        m->m_duty_kp_q12_per_permille=(uint32_t)(next.foc_duty_dowmramp_kp*kscale+0.5f);
        m->m_duty_ki_q12_per_permille=(uint32_t)(next.foc_duty_dowmramp_ki*dt*kscale+0.5f);
    }
}
void mcpwm_foc_sync_tuning_to_conf(bool second) {
    mcpwm_foc_motor_t *m=mcpwm_foc_get_motor(second);
    speed_pid_recompute_coeff(m);
    position_pid_recompute_coeff(m);
    m->m_conf.foc_current_kp=(float)m->m_kpq_q11/1536.0f;
    m->m_conf.foc_current_ki=(float)m->m_kiq_q16/4.608f;
    m->m_conf.s_pid_kp=(float)m->m_kps_q11/(float)MCCONF_SPEED_GAIN_SCALE;
    m->m_conf.s_pid_ki=(float)m->m_kis_q16/(float)MCCONF_SPEED_GAIN_SCALE;
    m->m_conf.s_pid_kd=(float)m->m_kds_q11/(float)MCCONF_SPEED_GAIN_SCALE;
    m->m_conf.p_pid_kp=(float)m->m_kpp_q11/1000.0f; m->m_conf.p_pid_ki=(float)m->m_kip_q16/1000.0f; m->m_conf.p_pid_kd=(float)m->m_kdp_q11/1000.0f;
}
const volatile mc_configuration *mcpwm_foc_get_configuration(bool second) { return &mcpwm_foc_get_motor(second)->m_conf; }

static int16_t amp_to_q4(const mcpwm_foc_motor_t *m, float current) {
    float max_a = m ? m->m_conf.l_current_max : (float)I_MOT_MAX;
    float min_a = m ? m->m_conf.l_current_min : -(float)I_MOT_MAX;
    if (max_a <= 0.0f || max_a > (float)I_MOT_MAX) max_a = (float)I_MOT_MAX;
    if (min_a >= 0.0f || min_a < -(float)I_MOT_MAX) min_a = -(float)I_MOT_MAX;
    if (current > max_a) current = max_a;
    if (current < min_a) current = min_a;
    float q = current * (float)FOC_CURRENT_Q4_PER_A;
    if (q > (float)MCCONF_MOTOR_CURRENT_MAX_Q4) q = (float)MCCONF_MOTOR_CURRENT_MAX_Q4;
    if (q < -(float)MCCONF_MOTOR_CURRENT_MAX_Q4) q = -(float)MCCONF_MOTOR_CURRENT_MAX_Q4;
    return (int16_t)q;
}

static void reset_position_pid(mcpwm_foc_motor_t *m){
    m->m_position_integrator=0;m->m_position_prev_error=0;m->m_position_prev_error_mdeg=0;m->m_position_sat_hold=0;
    m->m_position_dt_ticks=1u;m->m_position_d_filter_q15=0;m->m_position_d_proc_filter_q15=0;
    m->m_position_prev_proc_phase=m->m_phase;m->m_position_proc_dt_ticks=1u;
    m->m_position_breakaway_ticks=0u;m->m_position_no_motion_ticks=0u;m->m_position_motion_seen=0u;
    m->m_position_drive_direction=0;m->m_position_settle_ticks=0;
}

static void reset_current_pi(mcpwm_foc_motor_t *m) {
    m->m_iq_integrator = 0; m->m_id_integrator = 0;
    m->m_iq_sat_hold = 0; m->m_id_sat_hold = 0;
}

static void iq_setpoint_slew_step(mcpwm_foc_motor_t *m) {
    const int32_t target_q16 = (int32_t)m->m_iq_target_q4 << 16;
    int32_t step_q16 = ((int32_t)MCCONF_CURRENT_SLEW_A_PER_S *
                        FOC_CURRENT_Q4_PER_A * 65536 *
                        (int32_t)MCCONF_FOC_CONTROL_DIV) / PWM_FREQ;
    if (m->m_iq_set_ramp_q16 < target_q16) {
        m->m_iq_set_ramp_q16 += step_q16;
        if (m->m_iq_set_ramp_q16 > target_q16) m->m_iq_set_ramp_q16 = target_q16;
    } else if (m->m_iq_set_ramp_q16 > target_q16) {
        m->m_iq_set_ramp_q16 -= step_q16;
        if (m->m_iq_set_ramp_q16 < target_q16) m->m_iq_set_ramp_q16 = target_q16;
    }
    m->m_iq_set_q4 = (int16_t)(m->m_iq_set_ramp_q16 >> 16);
}

static void set_control_mode(mcpwm_foc_motor_t *m, mc_control_mode mode);

static void speed_setpoint_slew_step(mcpwm_foc_motor_t *m) {
    const int32_t target_q16 = m->m_speed_target_rpm_q16;
    int32_t step_q16 = ((int32_t)m->m_speed_ramp_rpm_s * 65536 *
                        (int32_t)MCCONF_FOC_CONTROL_DIV) / PWM_FREQ;
    if (step_q16 < 1) step_q16 = 1;

    if (m->m_speed_set_ramp_q16 < target_q16) {
        m->m_speed_set_ramp_q16 += step_q16;
        if (m->m_speed_set_ramp_q16 > target_q16) m->m_speed_set_ramp_q16 = target_q16;
    } else if (m->m_speed_set_ramp_q16 > target_q16) {
        m->m_speed_set_ramp_q16 -= step_q16;
        if (m->m_speed_set_ramp_q16 < target_q16) m->m_speed_set_ramp_q16 = target_q16;
    }
    m->m_speed_set_rpm = (int16_t)(m->m_speed_set_ramp_q16 >> 16);
}

static void speed_mode_enter(mcpwm_foc_motor_t *m) {
    if (m->m_control_mode != CONTROL_MODE_SPEED) {
        set_control_mode(m, CONTROL_MODE_SPEED);
        /* VESC-style speed ramp starts from measured speed when entering speed
         * mode, avoiding a discontinuous speed error after a mode transition. */
        m->m_speed_set_rpm = m->m_rpm;
        m->m_speed_set_ramp_q16 = (int32_t)m->m_rpm << 16;
    }
}

static void set_control_mode(mcpwm_foc_motor_t *m, mc_control_mode mode) {
    if (m->m_control_mode != mode) {
        /* Preserve the brake direction across BRAKE->NONE at zero speed so a
         * repeated brake packet cannot re-arm on mechanical/Hall rebound. Any
         * real drive/handbrake command starts a new event and clears it. */
        if (mode != CONTROL_MODE_CURRENT_BRAKE && mode != CONTROL_MODE_NONE)
            m->m_brake_direction=0;
        reset_current_pi(m);
        m->m_speed_integrator=0; m->m_speed_prev_error=0; m->m_speed_sat_hold=0; reset_position_pid(m);
        m->m_duty_i_q15=0; m->m_duty_pi_active=0u;
        /* Never seed a new torque reference from measured Iq. With low-side
         * shunts the phase current is not observable in high-impedance/coast
         * states, so OFF telemetry can legitimately be biased/noisy. Preserve
         * the previously commanded/slewed reference across active-mode changes;
         * mcpwm_foc_release_motor() already guarantees this is zero from NONE. */
        m->m_iq_set_ramp_q16 = (int32_t)m->m_iq_set_q4 << 16;
        m->m_iq_target_q4 = m->m_iq_set_q4;
        if(mode==CONTROL_MODE_DUTY)m->m_duty_ramp_permille=m->m_duty_now_permille;
        m->m_control_mode = mode;
    }
}

void mcpwm_foc_set_duty(float duty, bool second) {
    mcpwm_foc_motor_t *m=mcpwm_foc_get_motor(second);
    int32_t dpm=(int32_t)(duty>=0.0f?duty*1000.0f+0.5f:duty*1000.0f-0.5f);
    const int32_t lim=m->m_duty_limit_permille>0?m->m_duty_limit_permille:1000;
    dpm=CLAMP(dpm,-lim,lim);
    /* VESC Tool STOP / zero duty must be true coast. Do not keep a 50%%
     * synchronous zero vector active; release the advanced-timer MOE. */
    if (dpm == 0) { mcpwm_foc_release_motor(second); return; }
    set_control_mode(m, CONTROL_MODE_DUTY);
    m->m_duty_set_permille=(int16_t)dpm;
}
void mcpwm_foc_set_pid_speed(float erpm, bool second) {
    /* VESC COMM_SET_RPM is ERPM. As in VESC foc_run_pid_control_speed, keep a
     * command setpoint and a separately ramped active setpoint. A zero command
     * therefore decelerates through the configured ramp instead of becoming an
     * abrupt zero-speed servo. When the ramp reaches the low-speed release
     * threshold the controller integrators are reset and the bridge is released. */
    mcpwm_foc_motor_t *m = mcpwm_foc_get_motor(second);
    const int16_t mech_rpm = erpm_to_mech_rpm(erpm, second);
    m->m_speed_target_rpm = mech_rpm;
    m->m_speed_target_rpm_q16 = erpm_to_mech_rpm_q16(erpm, second);
    if (m->m_speed_target_rpm_q16 != 0 || m->m_control_mode == CONTROL_MODE_SPEED) {
        speed_mode_enter(m);
    } else {
        /* Zero ERPM while not already in a speed ramp is simply release. */
        mcpwm_foc_release_motor(second);
    }
}
void mcpwm_foc_set_pid_pos(float position_deg,bool second){
    /* Match vedderb/bldc foc_run_pid_control_pos: without a dedicated encoder,
     * VESC uses the live FOC electrical rotor phase as m_pos_pid_now and closes
     * a shortest-path angular PID on that continuous phase. Do NOT quantize the
     * command to Hall transition counts. */
    mcpwm_foc_motor_t *m=mcpwm_foc_get_motor(second);
    while(position_deg>=360.0f)position_deg-=360.0f;
    while(position_deg<0.0f)position_deg+=360.0f;
    uint32_t ph=(uint32_t)(position_deg*(65536.0f/360.0f)+0.5f);
    if(ph>=65536u)ph=0u;
    const uint16_t new_phase=(uint16_t)ph;
    const bool branch_change=(m->m_control_mode==CONTROL_MODE_POS && m->m_pos_pid_phase_mode==0u);
    const int16_t target_delta=(int16_t)(new_phase-m->m_pos_pid_set_phase);
    const bool target_changed=(target_delta>64 || target_delta<-64);
    set_control_mode(m,CONTROL_MODE_POS);
    if(branch_change) reset_position_pid(m);
    if(target_changed && !branch_change){
        m->m_position_breakaway_ticks=0u;m->m_position_no_motion_ticks=0u;m->m_position_motion_seen=0u;
        m->m_position_prev_proc_phase=m->m_phase;m->m_position_proc_dt_ticks=1u;
    }
    m->m_pos_pid_phase_mode=1u;
    m->m_pos_pid_set_phase=new_phase;
}
void mcpwm_foc_set_position_counts(int32_t pc,bool second){
    mcpwm_foc_motor_t*m=mcpwm_foc_get_motor(second);
    if(pc<m->m_position_min_counts)pc=m->m_position_min_counts;
    if(pc>m->m_position_max_counts)pc=m->m_position_max_counts;
    const bool branch_change=(m->m_control_mode==CONTROL_MODE_POS && m->m_pos_pid_phase_mode!=0u);
    set_control_mode(m,CONTROL_MODE_POS);
    if(branch_change) reset_position_pid(m);
    m->m_pos_pid_phase_mode=0u;
    m->m_position_target_counts=pc;
}
void mcpwm_foc_set_position_user_counts(int32_t pc,bool second){
    mcpwm_foc_set_position_counts(user_position_to_internal(pc,second),second);
}
void mcpwm_foc_set_position_user_limits(int32_t minc,int32_t maxc,bool second){
    mcpwm_foc_motor_t*m=mcpwm_foc_get_motor(second);
    if(minc>maxc){int32_t t=minc;minc=maxc;maxc=t;}
    if(!second){
        m->m_position_min_counts=minc;
        m->m_position_max_counts=maxc;
    }else{
        /* user [min,max] maps to internal [-max,-min] */
        m->m_position_min_counts=user_position_to_internal(maxc,true);
        m->m_position_max_counts=user_position_to_internal(minc,true);
        if(m->m_position_min_counts>m->m_position_max_counts){
            int32_t t=m->m_position_min_counts;m->m_position_min_counts=m->m_position_max_counts;m->m_position_max_counts=t;
        }
    }
    if(m->m_position_target_counts<m->m_position_min_counts)m->m_position_target_counts=m->m_position_min_counts;
    if(m->m_position_target_counts>m->m_position_max_counts)m->m_position_target_counts=m->m_position_max_counts;
}
int32_t mcpwm_foc_get_position_user_counts(bool second){
    return internal_position_to_user(mcpwm_foc_get_motor_const(second)->m_position_counts,second);
}
int32_t mcpwm_foc_get_position_target_user_counts(bool second){
    return internal_position_to_user(mcpwm_foc_get_motor_const(second)->m_position_target_counts,second);
}
int32_t mcpwm_foc_get_position_min_user_counts(bool second){
    const mcpwm_foc_motor_t*m=mcpwm_foc_get_motor_const(second);
    return second?internal_position_to_user(m->m_position_max_counts,true):m->m_position_min_counts;
}
int32_t mcpwm_foc_get_position_max_user_counts(bool second){
    const mcpwm_foc_motor_t*m=mcpwm_foc_get_motor_const(second);
    return second?internal_position_to_user(m->m_position_min_counts,true):m->m_position_max_counts;
}
void mcpwm_foc_reset_position(bool second){mcpwm_foc_motor_t*m=mcpwm_foc_get_motor(second);m->m_position_counts=0;m->m_position_target_counts=0;reset_position_pid(m);}

void mcpwm_foc_set_current(float current, bool second) {
    mcpwm_foc_motor_t *m=mcpwm_foc_get_motor(second);
    const float min_i=(m->m_conf.cc_min_current>0.0f)?m->m_conf.cc_min_current:MCCONF_CC_MIN_CURRENT;
    if (current < min_i && current > -min_i) { mcpwm_foc_release_motor(second); return; }
    set_control_mode(m, CONTROL_MODE_CURRENT);
    m->m_iq_target_q4=amp_to_q4(m,current);
    m->m_id_set_q4=0;
}
void mcpwm_foc_set_brake_current(float current, bool second) {
    mcpwm_foc_motor_t *m=mcpwm_foc_get_motor(second);
    const float min_i=(m->m_conf.cc_min_current>0.0f)?m->m_conf.cc_min_current:MCCONF_CC_MIN_CURRENT;
    if (current < min_i && current > -min_i) { mcpwm_foc_release_motor(second); return; }
    set_control_mode(m, CONTROL_MODE_CURRENT_BRAKE);
    m->m_brake_current_q4=amp_to_q4(m,current<0?-current:current);
    if(m->m_brake_current_q4<0)m->m_brake_current_q4=(int16_t)-m->m_brake_current_q4;
    /* Upstream VESC derives brake sign from speed. Hall speed is quantized, so
     * latch the first fresh non-zero direction for this stopping event and do
     * not re-arm in the opposite direction after zero-crossing/rebound. */
    if(m->m_brake_direction==0 && m->m_hall_initialized && m->m_hall_direction!=0 &&
       m->m_hall_period>0u && m->m_hall_period<MCCONF_HALL_TIMEOUT_TICKS){
        uint32_t fresh=(uint32_t)m->m_hall_period*2u;
        if(fresh>MCCONF_HALL_TIMEOUT_TICKS)fresh=MCCONF_HALL_TIMEOUT_TICKS;
        if(m->m_hall_ticks<=fresh){
            if(m->m_rpm>MCCONF_TRQ_STOP_RPM_DEADBAND)m->m_brake_direction=1;
            else if(m->m_rpm<-MCCONF_TRQ_STOP_RPM_DEADBAND)m->m_brake_direction=-1;
        }
    }
    m->m_iq_target_q4=0;
    m->m_id_set_q4=0;
}
void mcpwm_foc_set_handbrake(float current, bool second) {
    mcpwm_foc_motor_t *m=mcpwm_foc_get_motor(second);
    const float min_i=(m->m_conf.cc_min_current>0.0f)?m->m_conf.cc_min_current:MCCONF_CC_MIN_CURRENT;
    if (current < min_i && current > -min_i) { mcpwm_foc_release_motor(second); return; }
    set_control_mode(m, CONTROL_MODE_HANDBRAKE);
    m->m_handbrake_current_q4=amp_to_q4(m,current<0?-current:current);
    if(m->m_handbrake_current_q4<0)m->m_handbrake_current_q4=(int16_t)-m->m_handbrake_current_q4;
    m->m_iq_target_q4=m->m_handbrake_current_q4;
    m->m_id_set_q4=0;
}
void mcpwm_foc_set_openloop_current(float current, float rpm, bool second) {
    mcpwm_foc_motor_t *m=mcpwm_foc_get_motor(second); set_control_mode(m, CONTROL_MODE_OPENLOOP);
    m->m_openloop_id_target_q4=amp_to_q4(m,current<0?-current:current); m->m_iq_target_q4=0; m->m_iq_set_q4=0; m->m_speed_set_rpm=(int16_t)rpm; m->m_phase_override=1;
}
void mcpwm_foc_set_openloop_phase(float current, float phase, bool second) {
    mcpwm_foc_motor_t *m=mcpwm_foc_get_motor(second); set_control_mode(m, CONTROL_MODE_OPENLOOP_PHASE);
    m->m_openloop_id_target_q4=amp_to_q4(m,current<0?-current:current); m->m_iq_target_q4=0; m->m_iq_set_q4=0;
    while (phase < 0.0f) phase += 360.0f;
    while (phase >= 360.0f) phase -= 360.0f;
    m->m_phase_openloop=(uint16_t)(phase*(65536.0f/360.0f)); m->m_phase_override=1;
}
void mcpwm_foc_release_motor(bool second) {
    mcpwm_foc_motor_t *m=mcpwm_foc_get_motor(second);
    const bool was_active=(m->m_control_mode!=CONTROL_MODE_NONE);
    set_control_mode(m, CONTROL_MODE_NONE);
    if(was_active){
        /* The high-impedance amplifier common-mode point can move after a
         * driven interval. Re-acquire its telemetry-only zero once the rotor is
         * stationary; never reuse a stale OFF baseline from before motion. */
        m->m_off_offset_valid=0u; m->m_off_offset_samples=0u;
        m->m_off_settle_ticks=MCCONF_OFF_TELEM_SETTLE_SAMPLES;
        m->m_off_offset_sum0=0; m->m_off_offset_sum1=0; m->m_off_offset_sumdc=0;
        m->m_current_lpf_q16[0]=m->m_current_lpf_q16[1]=0;
        m->m_telem_current_lpf_q16[0]=m->m_telem_current_lpf_q16[1]=m->m_telem_current_lpf_q16[2]=0;
        m->m_telem_sum_id_q4=0; m->m_telem_sum_iq_q4=0; m->m_telem_sum_ibus_counts=0;
        m->m_telem_avg_samples=0u;
    }
    m->m_iq_set_q4=0; m->m_iq_target_q4=0; m->m_iq_set_ramp_q16=0;
    m->m_duty_set_permille=0; m->m_duty_ramp_permille=0;
    m->m_id_set_q4=0; m->m_openloop_id_target_q4=0;
    m->m_openloop_id_ramp_q16=0;
    m->m_speed_target_rpm=0; m->m_speed_target_rpm_q16=0; m->m_speed_set_rpm=0; m->m_speed_set_ramp_q16=0;
    m->m_speed_integrator=0; m->m_speed_prev_error=0; m->m_speed_sat_hold=0; reset_position_pid(m);
    m->m_state=MC_STATE_OFF;
    if (second) {
        RIGHT_TIM->BDTR &= ~TIM_BDTR_MOE;
        RIGHT_TIM->RIGHT_TIM_U=pwm_res/2u; RIGHT_TIM->RIGHT_TIM_V=pwm_res/2u; RIGHT_TIM->RIGHT_TIM_W=pwm_res/2u;
    } else {
        LEFT_TIM->BDTR &= ~TIM_BDTR_MOE;
        LEFT_TIM->LEFT_TIM_U=pwm_res/2u; LEFT_TIM->LEFT_TIM_V=pwm_res/2u; LEFT_TIM->LEFT_TIM_W=pwm_res/2u;
    }
}

static int16_t trq_ca_to_q4(const mcpwm_foc_motor_t *m, int16_t ca) {
    (void)m;
    /* ISR-safe integer-only centiampere scaling: 50 -> 0.50 A,
     * 1500 -> 15.00 A. Runtime VESC commands are clamped separately by the
     * non-ISR amp_to_q4 API. */
    int32_t q4=((int32_t)ca*(int32_t)A2BIT_CONV*16)/100;
    if(q4>MCCONF_MOTOR_CURRENT_MAX_Q4)q4=MCCONF_MOTOR_CURRENT_MAX_Q4;
    else if(q4<-MCCONF_MOTOR_CURRENT_MAX_Q4)q4=-MCCONF_MOTOR_CURRENT_MAX_Q4;
    return (int16_t)q4;
}

void mcpwm_foc_set_mode_command(uint8_t mode, int16_t command, bool run_request,
                                uint16_t openloop_rpm, bool second) {
    mcpwm_foc_motor_t *m=mcpwm_foc_get_motor(second);
    if (mode==TRQ_MODE) {
        if (!run_request) {
            /* Explicit STOP in torque mode is VESC-style release, not braking.
             * First slew Iq_ref to zero; once the active reference is zero,
             * release the bridge so the wheel is truly free-running. Once it
             * is released, keep it released until a new RUN request arrives. */
            if (m->m_control_mode != CONTROL_MODE_NONE) {
                if (m->m_control_mode != CONTROL_MODE_CURRENT) {
                    set_control_mode(m, CONTROL_MODE_CURRENT);
                }
                m->m_iq_target_q4 = 0;
                m->m_id_set_q4 = 0;
                if (m->m_iq_set_q4 == 0 && m->m_iq_set_ramp_q16 == 0) {
                    mcpwm_foc_release_motor(second);
                }
            }
        } else {
            set_control_mode(m, CONTROL_MODE_CURRENT);
            m->m_iq_target_q4 = trq_ca_to_q4(m, command);
            m->m_id_set_q4 = 0;
        }
    } else if (mode==SPD_MODE) {
        /* Speed mode follows the VESC concept of command_rpm -> ramped set_rpm.
         * STOP sets the target to zero but keeps SPEED active while the setpoint
         * ramps down. The low-speed release happens inside motor_control_step,
         * where the speed and current integrators are reset before free-running. */
        m->m_speed_target_rpm = run_request ? command : 0;
        m->m_speed_target_rpm_q16 = (int32_t)m->m_speed_target_rpm << 16;
        if (m->m_speed_target_rpm_q16 != 0 || m->m_control_mode == CONTROL_MODE_SPEED) {
            speed_mode_enter(m);
        }
    } else if (mode==VLT_MODE) {
        /* Zero voltage on this hardware must mean high-impedance/free-run, not
         * a synchronously switched zero vector. After the legacy command ramp
         * reaches zero, release the bridge. */
        if (!run_request || command == 0) {
            mcpwm_foc_release_motor(second);
        } else {
            /* Integer equivalent of the VESC duty setter for the legacy ISR
             * source. Do not execute software floating-point at 16 kHz. */
            set_control_mode(m, CONTROL_MODE_DUTY);
            const int32_t lim=m->m_duty_limit_permille>0?m->m_duty_limit_permille:1000;
            m->m_duty_set_permille=(int16_t)CLAMP((int32_t)command,-lim,lim);
        }
    } else if(mode==5u){
        if (!run_request) {
            mcpwm_foc_release_motor(second);
        } else {
            const int32_t user_target = second ? positionCommandR : positionCommandL;
            mcpwm_foc_set_position_counts(user_position_to_internal(user_target, second), second);
        }
    } else if (mode==SVPWM_MODE) {
        set_control_mode(m, CONTROL_MODE_OPENLOOP);
        int32_t a=command<0?-(int32_t)command:(int32_t)command;
        if(a>(int32_t)SVPWM_MAX_ID_A)a=(int32_t)SVPWM_MAX_ID_A;
        m->m_openloop_id_target_q4=(int16_t)(a*FOC_CURRENT_Q4_PER_A); m->m_iq_target_q4=0; m->m_iq_set_q4=0;
        int32_t rpm=(int32_t)openloop_rpm; if(rpm>(int32_t)MCCONF_OPENLOOP_RPM_MAX)rpm=(int32_t)MCCONF_OPENLOOP_RPM_MAX;
        m->m_speed_set_rpm=(command<0)?-(int16_t)rpm:(command>0?(int16_t)rpm:0);
        m->m_phase_override=1;
    } else {
        mcpwm_foc_release_motor(second);
    }
}

static uint8_t hall_read(bool second) {
    if (!second) {
        uint8_t u=!(LEFT_HALL_U_PORT->IDR & LEFT_HALL_U_PIN), v=!(LEFT_HALL_V_PORT->IDR & LEFT_HALL_V_PIN), w=!(LEFT_HALL_W_PORT->IDR & LEFT_HALL_W_PIN);
        return (uint8_t)((u<<2)|(v<<1)|w);
    }
    uint8_t u=!(RIGHT_HALL_U_PORT->IDR & RIGHT_HALL_U_PIN), v=!(RIGHT_HALL_V_PORT->IDR & RIGHT_HALL_V_PIN), w=!(RIGHT_HALL_W_PORT->IDR & RIGHT_HALL_W_PIN);
    return (uint8_t)((u<<2)|(v<<1)|w);
}

static uint8_t hall_table_angle(const mcpwm_foc_motor_t *m, uint8_t hall) {
    return (uint8_t)m->m_conf.foc_hall_table[hall & 7u];
}

static int16_t hall_angle_diff(uint8_t now, uint8_t prev) {
    int16_t d = (int16_t)now - (int16_t)prev;
    if (d > 100) d -= 200;
    if (d < -100) d += 200;
    return d;
}

static void hall_estimator_reset(mcpwm_foc_motor_t *m) {
    m->m_hall_initialized = 0u;
    m->m_hall_direction = 0;
    m->m_hall_debounce_initialized = 0u;
    m->m_hall_candidate_count = 0u;
    m->m_hall_direction_stable_edges = 0u;
    m->m_hall_ticks = 0u;
    m->m_hall_period = MCCONF_HALL_TIMEOUT_TICKS;
    m->m_hall_interp_step_q16 = 0u;
    m->m_hall_rate_limit_step = 1u;
    for (int i = 0; i < 4; i++) m->m_hall_period_hist[i] = MCCONF_HALL_TIMEOUT_TICKS;
    m->m_hall_hist_pos = 0u;
    m->m_hall_interp_active = 0u;
    m->m_hall_reject_counted_state = 0xffu;
    m->m_phase_hall_target = m->m_phase_hall;
    m->m_rpm = 0;
}

static int16_t phase_diff_u16(uint16_t target, uint16_t actual) {
    return (int16_t)(target - actual);
}

static uint16_t hall_angle200_to_phase(uint8_t a) {
    return (uint16_t)(((uint32_t)a * 65536u) / 200u);
}

static uint8_t hall_midpoint200(uint8_t previous_center, int16_t center_delta) {
    int16_t edge = (int16_t)previous_center + center_delta / 2;
    while (edge < 0) edge += 200;
    while (edge >= 200) edge -= 200;
    return (uint8_t)edge;
}

static void hall_update(mcpwm_foc_motor_t *m, bool second) {
    const uint8_t raw_h = hall_read(second);
    m->m_hall_raw_state = raw_h;

    /* GPIO Hall inputs are asynchronous to the 16-kHz ADC ISR. One transient
     * sample during a switching edge must never become an electrical-sector
     * transition. Accept a new raw code only after consecutive agreement. */
    if (!m->m_hall_debounce_initialized) {
        m->m_hall_debounce_initialized = 1u;
        m->m_hall_state = raw_h;
        m->m_hall_candidate_state = raw_h;
        m->m_hall_candidate_count = 0u;
    } else if (raw_h == m->m_hall_state) {
        m->m_hall_candidate_state = raw_h;
        m->m_hall_candidate_count = 0u;
    } else {
        if (raw_h != m->m_hall_candidate_state) {
            m->m_hall_candidate_state = raw_h;
            m->m_hall_candidate_count = 1u;
        } else if (m->m_hall_candidate_count < 0xffu) {
            m->m_hall_candidate_count++;
        }
        if (m->m_hall_candidate_count >= MCCONF_HALL_DEBOUNCE_SAMPLES) {
            m->m_hall_state = m->m_hall_candidate_state;
            m->m_hall_candidate_count = 0u;
        }
    }

    const uint8_t h = m->m_hall_state;
    const uint8_t angle = hall_table_angle(m, h);
    const bool valid = (h != 0u && h != 7u && angle < 200u);
    if (m->m_hall_ticks < 0xffffu) m->m_hall_ticks++;

    if (valid) {
        if (!m->m_hall_initialized) {
            m->m_hall_initialized = 1u;
            m->m_hall_pos_prev = angle;          /* current sector center */
            m->m_hall_pos = angle;               /* edge/base until first valid transition */
            m->m_hall_ticks = 0u;
            m->m_hall_direction = 0;
            m->m_phase_hall = hall_angle200_to_phase(angle);
            m->m_phase_hall_target = m->m_phase_hall;
            m->m_rpm = 0;
        } else if (angle != m->m_hall_pos_prev) {
            const uint8_t previous_center = m->m_hall_pos_prev;
            const int16_t ad = hall_angle_diff(angle, previous_center);
            const int16_t aad = ad < 0 ? (int16_t)-ad : ad;
            int8_t dir = 0;
            /* One Hall edge should be about 200/6 = 33.3 VESC angle units.
             * 15..50 tolerates calibration spread while rejecting skipped states. */
            if (aad >= 15 && aad <= 50) dir = ad > 0 ? 1 : -1;

            if (dir != 0) {
                uint16_t period = m->m_hall_ticks;
                if (period == 0u) period = 1u;
                const bool period_outlier =
                    (m->m_hall_direction != 0 &&
                     m->m_hall_direction == dir &&
                     m->m_hall_direction_stable_edges >= MCCONF_HALL_PERIOD_FILTER_WARMUP_EDGES &&
                     m->m_hall_period < MCCONF_HALL_TIMEOUT_TICKS &&
                     ((uint32_t)period * MCCONF_HALL_PERIOD_OUTLIER_RATIO) < m->m_hall_period);
                uint16_t period_for_filter = period;
                if (period_outlier) {
                    /* Never reject an electrically valid adjacent Hall state just
                     * because its timing changed quickly. During acceleration that
                     * creates a false skipped-state cascade on the next edge. VESC
                     * uses Hall state for phase correction; timing is an estimator
                     * input. Accept the state, but slew-limit the period estimate. */
                    if (m->m_hall_reject_counted_state != h) {
                        m->m_hall_period_reject_count++;
                        m->m_hall_last_reject_reason = 1u;
                        m->m_hall_last_reject_from = 0xffu;
                        for(uint8_t rh=1u;rh<=6u;++rh) if(hall_table_angle(m,rh)==m->m_hall_pos_prev){m->m_hall_last_reject_from=rh;break;}
                        m->m_hall_last_reject_to = h;
                    }
                    const uint16_t floor_period=(uint16_t)(m->m_hall_period/MCCONF_HALL_PERIOD_OUTLIER_RATIO);
                    if(floor_period>0u && period_for_filter<floor_period) period_for_filter=floor_period;
                }
                m->m_hall_reject_counted_state = 0xffu;
                const bool direction_reset =
                    (m->m_hall_direction == 0 || m->m_hall_direction != dir ||
                     m->m_hall_period_hist[0] == MCCONF_HALL_TIMEOUT_TICKS);
                if (direction_reset) {
                    for (int i = 0; i < 4; i++) m->m_hall_period_hist[i] = period_for_filter;
                    m->m_hall_hist_pos = 0u;
                    m->m_hall_direction_stable_edges = 0u;
                } else {
                    m->m_hall_period_hist[m->m_hall_hist_pos++ & 3u] = period_for_filter;
                }
                uint32_t sum = 0u;
                for (int i = 0; i < 4; i++) sum += m->m_hall_period_hist[i];
                m->m_hall_period = (uint16_t)(sum / 4u);
                if (!m->m_hall_period) m->m_hall_period = 1u;
                {
                    const uint32_t sector=65536u/6u;
                    m->m_hall_interp_step_q16=(uint32_t)(((sector<<16)+(m->m_hall_period/2u))/m->m_hall_period);
                    /* Derive the phase-correction slew from the interpolation
                     * reciprocal we just computed. This is mathematically the
                     * same sector*3/(2*period), but avoids a second integer
                     * division on every Hall edge. */
                    uint32_t rs=(m->m_hall_interp_step_q16*3u + 65535u) >> 17;
                    if(rs==0u)rs=1u;
                    const uint32_t min_erpm=(uint32_t)MCCONF_HALL_INTERP_ON_RPM*(uint32_t)motor_pole_pairs(second);
                    uint32_t min_step=(uint32_t)((min_erpm*65536u)/(60u*PWM_FREQ));
                    if (min_step == 0u) min_step = 1u;
                    if (rs < min_step) rs = min_step;
                    if (rs > 32767u) rs = 32767u;
                    m->m_hall_rate_limit_step=(uint16_t)rs;
                }
                m->m_hall_ticks = 0u;
                m->m_hall_direction = dir;
                /* Hall period only changes on an accepted edge. Calculate the
                 * mechanical RPM once here instead of dividing at 16 kHz. */
                {
                    int32_t rpm = 10667 / (int32_t)m->m_hall_period;
                    if (rpm > MCCONF_MOTOR_RPM_MAX) rpm = MCCONF_MOTOR_RPM_MAX;
                    m->m_rpm = (int16_t)(rpm * dir);
                }
                if (m->m_hall_direction_stable_edges < 0xffu) m->m_hall_direction_stable_edges++;
                if (dir > 0 && m->m_position_counts < INT32_MAX) m->m_position_counts++;
                else if (dir < 0 && m->m_position_counts > INT32_MIN) m->m_position_counts--;
                m->m_hall_pos = hall_midpoint200(previous_center, ad);
                m->m_hall_pos_prev = angle;
            } else {
                if (m->m_hall_reject_counted_state != h) {
                    m->m_hall_invalid_transition_count++;
                    m->m_hall_sequence_reject_count++;
                    m->m_hall_last_reject_reason = 2u;
                    m->m_hall_last_reject_from = 0xffu;
                    for(uint8_t rh=1u;rh<=6u;++rh) if(hall_table_angle(m,rh)==m->m_hall_pos_prev){m->m_hall_last_reject_from=rh;break;}
                    m->m_hall_last_reject_to = h;
                    m->m_hall_reject_counted_state = h;
                }
            }
        } else {
            /* Returned to the last accepted Hall sector. A later departure is
             * a new transition attempt and may be counted once again. */
            m->m_hall_reject_counted_state = 0xffu;
        }
    }

    if (!valid || !m->m_hall_initialized || m->m_hall_ticks > MCCONF_HALL_TIMEOUT_TICKS ||
        !m->m_hall_period || m->m_hall_direction == 0) {
        m->m_rpm = 0;
    }

    const int16_t abs_rpm = (int16_t)abs(m->m_rpm);
    if (abs_rpm >= MCCONF_HALL_INTERP_ON_RPM) m->m_hall_interp_active = 1u;
    else if (abs_rpm <= MCCONF_HALL_INTERP_OFF_RPM) m->m_hall_interp_active = 0u;

    uint16_t desired = m->m_phase_hall;
    if (valid) {
        if (m->m_hall_interp_active && m->m_hall_direction != 0 &&
            m->m_hall_period > 0u && m->m_hall_period < MCCONF_HALL_TIMEOUT_TICKS) {
            const uint16_t edge_phase = hall_angle200_to_phase(m->m_hall_pos);
            uint32_t ticks = m->m_hall_ticks;
            if (ticks > m->m_hall_period) ticks = m->m_hall_period;
            const uint32_t frac = (uint32_t)((ticks * m->m_hall_interp_step_q16) >> 16);
            const uint32_t debounce_adv=(uint32_t)((MCCONF_HALL_PHASE_ADVANCE_TICKS * m->m_hall_interp_step_q16) >> 16);
            const uint32_t phase_frac=frac+debounce_adv;
            desired = (uint16_t)(m->m_hall_direction > 0 ?
                                (uint32_t)edge_phase + phase_frac :
                                (uint32_t)edge_phase - phase_frac);
        } else {
            /* At low speed use the calibrated Hall-sector center directly,
             * just like foc_correct_hall() in VESC 6.00. */
            desired = hall_angle200_to_phase(angle);
        }
    }
    m->m_phase_hall_target = desired;

    /* VESC rate-limits corrected Hall phase to avoid current spikes when a
     * Hall edge or noisy sample moves the target abruptly. The actual Hall
     * sector rate is the primary limit; retain a small minimum around the
     * interpolation-on threshold so low-speed center corrections stay smooth. */
    uint32_t max_step=m->m_hall_rate_limit_step;
    if(max_step==0u)max_step=1u;
    const int16_t pd = phase_diff_u16(desired, m->m_phase_hall);
    if (pd > (int16_t)max_step) m->m_phase_hall = (uint16_t)(m->m_phase_hall + (uint16_t)max_step);
    else if (pd < -(int16_t)max_step) m->m_phase_hall = (uint16_t)(m->m_phase_hall - (uint16_t)max_step);
    else m->m_phase_hall = desired;
}

static void openloop_current_ramp_update(mcpwm_foc_motor_t *m) {
    /* Shared mode-4/Hall-detect Id slew. This helper deliberately never changes
     * m_phase_openloop: COMM_DETECT_HALL_FOC depends on the requested synthetic
     * electrical phase remaining exactly where the detector put it. */
    const int32_t target_id_q16=(int32_t)m->m_openloop_id_target_q4<<16;
    int32_t id_step_q16=((int32_t)MCCONF_OPENLOOP_ID_SLEW_A_S*FOC_CURRENT_Q4_PER_A<<16)/PWM_FREQ;
    if(m->m_openloop_id_ramp_q16<target_id_q16){
        m->m_openloop_id_ramp_q16+=id_step_q16;
        if(m->m_openloop_id_ramp_q16>target_id_q16)m->m_openloop_id_ramp_q16=target_id_q16;
    } else if(m->m_openloop_id_ramp_q16>target_id_q16){
        m->m_openloop_id_ramp_q16-=id_step_q16;
        if(m->m_openloop_id_ramp_q16<target_id_q16)m->m_openloop_id_ramp_q16=target_id_q16;
    }
    m->m_id_set_q4=(int16_t)(m->m_openloop_id_ramp_q16>>16);
    m->m_iq_target_q4=0;
    m->m_iq_set_q4=0;
}

static void openloop_update(mcpwm_foc_motor_t *m, bool second) {
    const int16_t target=m->m_speed_set_rpm;
    const int8_t requested_dir=(target>0)?1:(target<0?-1:0);
    const uint16_t absrpm=(uint16_t)(target<0?-target:target);

    openloop_current_ramp_update(m);

    if (requested_dir!=0 && m->m_openloop_direction!=requested_dir) {
        m->m_openloop_direction=requested_dir;
        m->m_openloop_speed_q16=0;
        m->m_openloop_phase_acc_q32=((uint32_t)SVPWM_ALIGN_PHASE)<<16;
        m->m_phase_openloop=SVPWM_ALIGN_PHASE;
        m->m_openloop_align_ticks=(uint16_t)(((uint32_t)PWM_FREQ*MCCONF_OPENLOOP_ALIGN_MS)/1000u);
        m->m_openloop_primed=0u;
        m->m_openloop_id_ramp_q16=0;
        m->m_id_set_q4=0;
        reset_current_pi(m);
    }

    if (!m->m_openloop_primed && requested_dir!=0) {
        reset_current_pi(m);
        m->m_openloop_primed=1u;
    }

    if (m->m_openloop_align_ticks && requested_dir!=0) {
        m->m_openloop_align_ticks--;
        m->m_phase_openloop=(uint16_t)(m->m_openloop_phase_acc_q32>>16);
        return;
    }

    const int32_t target_speed_q16=(int32_t)absrpm<<16;
    int32_t speed_step=((int32_t)MCCONF_OPENLOOP_ACCEL_RPM_S<<16)/PWM_FREQ;
    if(m->m_openloop_speed_q16<target_speed_q16){
        m->m_openloop_speed_q16+=speed_step;
        if(m->m_openloop_speed_q16>target_speed_q16)m->m_openloop_speed_q16=target_speed_q16;
    } else if(m->m_openloop_speed_q16>target_speed_q16){
        m->m_openloop_speed_q16-=speed_step;
        if(m->m_openloop_speed_q16<target_speed_q16)m->m_openloop_speed_q16=target_speed_q16;
    }

    const int8_t phase_dir=(requested_dir!=0)?requested_dir:m->m_openloop_direction;
    const uint32_t stepPerRpm=(uint32_t)(((uint64_t)motor_pole_pairs(second)*4294967296ULL)/(60ULL*PWM_FREQ));
    const uint32_t phaseStep=(uint32_t)(((uint64_t)(uint32_t)m->m_openloop_speed_q16*stepPerRpm)>>16);
    if(phase_dir>0)m->m_openloop_phase_acc_q32+=phaseStep;
    else if(phase_dir<0)m->m_openloop_phase_acc_q32-=phaseStep;
    m->m_phase_openloop=(uint16_t)(m->m_openloop_phase_acc_q32>>16);

    if(requested_dir==0 && m->m_openloop_speed_q16==0 && m->m_id_set_q4==0){
        m->m_openloop_direction=0;
        m->m_openloop_primed=0u;
        m->m_openloop_phase_acc_q32=((uint32_t)SVPWM_ALIGN_PHASE)<<16;
        m->m_phase_openloop=SVPWM_ALIGN_PHASE;
    }
}

static int16_t pi_run_state(int16_t err, uint16_t kp, uint16_t ki, int16_t max, int16_t min,
                            int32_t *integ, uint8_t *hold) {
    foc_pi_fixed_t pi={*integ,*hold}; int16_t out=foc_pi_run(&pi,err,kp,ki,max,min); *integ=pi.integrator; *hold=pi.sat_hold; return out;
}

static int16_t position_pid_iq_target_step(mcpwm_foc_motor_t *m, bool second) {
    /* Stock COMM_SET_POS remains VESC-compatible in electrical degrees. On
     * this Hall-only hoverboard, direct position->Iq is under-damped because one
     * Hall sector is 60 electrical degrees. Use the hardware-safe cascade
     * position PID -> ERPM -> speed PI -> Iq -> current PI -> Vq for phase mode.
     * The custom long-range Hall-count API remains a separate branch. */
    int32_t error_mdeg;
    int32_t count_error=0;
    if(m->m_pos_pid_phase_mode){
        const int16_t phase_err=(int16_t)(m->m_pos_pid_set_phase-m->m_phase);
        error_mdeg=(int32_t)(((int64_t)phase_err*360000LL)/65536LL);
    }else{
        int64_t ec64=(int64_t)m->m_position_target_counts-(int64_t)m->m_position_counts;
        if(ec64>32767)ec64=32767; else if(ec64<-32768)ec64=-32768;
        count_error=(int32_t)ec64;
        const int32_t pp=(int32_t)motor_pole_pairs(second);
        const int32_t mdeg_per_count=360000/(6*pp);
        error_mdeg=count_error*mdeg_per_count;
        m->m_position_prev_error=(int16_t)count_error;
    }
    const int32_t limit_q4=m->m_current_limit_q4>0?m->m_current_limit_q4:MCCONF_MOTOR_CURRENT_MAX_Q4;

    int64_t p64=(int64_t)error_mdeg*(int32_t)m->m_kpp_q11*32768LL;
    int32_t p_q15=(int32_t)(p64/1000000LL);
    p_q15=CLAMP(p_q15,-32768,32768);

    if(m->m_kip_q16==0u){
        m->m_position_integrator=0;
    }else{
        int64_t istep=(int64_t)error_mdeg*(int32_t)m->m_kip_q16*32768LL*65536LL*
                      (int32_t)MCCONF_FOC_CONTROL_DIV;
        istep/=(1000000LL*(int32_t)PWM_FREQ);
        int64_t isum=(int64_t)m->m_position_integrator+istep;
        int32_t i_lim_q15=32768-(p_q15<0?-p_q15:p_q15);
        if(i_lim_q15<0)i_lim_q15=0;
        int64_t ilim=(int64_t)i_lim_q15<<16;
        if(isum>ilim)isum=ilim; else if(isum<-ilim)isum=-ilim;
        m->m_position_integrator=(int32_t)isum;
    }

    if(m->m_kdp_q11!=0u){
        int32_t d_raw_q15=0;
        if(error_mdeg==m->m_position_prev_error_mdeg){
            if(m->m_position_dt_ticks<65535u)m->m_position_dt_ticks++;
        }else{
            const uint32_t dt_ticks=m->m_position_dt_ticks?m->m_position_dt_ticks:1u;
            const int32_t de_mdeg=error_mdeg-m->m_position_prev_error_mdeg;
            int64_t d64=(int64_t)de_mdeg*(int32_t)m->m_kdp_q11*32768LL*(int32_t)PWM_FREQ;
            d64/=(1000000LL*(int32_t)MCCONF_FOC_CONTROL_DIV*(int64_t)dt_ticks);
            if(d64>32768)d64=32768; else if(d64<-32768)d64=-32768;
            d_raw_q15=(int32_t)d64;
            m->m_position_dt_ticks=1u;
        }
        const int32_t dd=d_raw_q15-m->m_position_d_filter_q15;
        m->m_position_d_filter_q15 += (int32_t)(((int64_t)dd*m->m_position_kd_filter_q16)>>16);
    }else{
        m->m_position_dt_ticks=1u; m->m_position_d_filter_q15=0;
    }
    m->m_position_prev_error_mdeg=error_mdeg;
    const int32_t d_q15=m->m_position_d_filter_q15;

    if(m->m_pos_pid_phase_mode){
        /* Upstream VESC position controller: normalized position PID -> Iq.
         * Hall-only hardware cannot resolve better than one electrical sector at
         * standstill, so use a small target deadband and a 0.6 A hardware-safe
         * current scale. Unlike the discarded position->speed cascade, process-D
         * is derived from the actual electrical phase delta, exactly matching the
         * quantity that SET_POS controls. */
        if(m->m_position_proc_dt_ticks<65535u)m->m_position_proc_dt_ticks++;
        const int16_t proc_delta=(int16_t)(m->m_phase-m->m_position_prev_proc_phase);
        int32_t dproc_raw_q15=0;
        if(proc_delta!=0){
            const uint32_t dt_ticks=m->m_position_proc_dt_ticks?m->m_position_proc_dt_ticks:1u;
            int32_t dp=-(int32_t)proc_delta*(int32_t)m->m_position_kd_proc_phase_coeff_q4;
            if(dt_ticks>1u)dp/=(int32_t)dt_ticks;
            dp>>=4;
            if(dp>32768)dp=32768; else if(dp<-32768)dp=-32768;
            dproc_raw_q15=dp;
            m->m_position_prev_proc_phase=m->m_phase;
            m->m_position_proc_dt_ticks=1u;
            m->m_position_motion_seen=1u;
            m->m_position_no_motion_ticks=0u;
        }else if(m->m_position_no_motion_ticks<65535u){
            m->m_position_no_motion_ticks++;
        }
        const int32_t dpdiff=dproc_raw_q15-m->m_position_d_proc_filter_q15;
        m->m_position_d_proc_filter_q15 +=
            (int32_t)(((int64_t)dpdiff*m->m_position_kd_filter_q16)>>16);

        const int32_t abs_error=error_mdeg<0?-error_mdeg:error_mdeg;
        int32_t base_q15=p_q15+(m->m_position_integrator>>16)+d_q15;
        if(abs_error<=(int32_t)MCCONF_POSITION_PHASE_DEADBAND_MDEG){
            base_q15=0;
            m->m_position_integrator=0;
            m->m_position_d_filter_q15=0;
        }
        int32_t out_q15=base_q15+m->m_position_d_proc_filter_q15;
        out_q15=CLAMP(out_q15,-32768,32768);

        const uint32_t kick_max=((uint32_t)MCCONF_POSITION_BREAKAWAY_KICK_MS*(uint32_t)PWM_FREQ)/
                                (1000u*(uint32_t)MCCONF_FOC_CONTROL_DIV);
        const uint32_t rearm_ticks=((uint32_t)MCCONF_POSITION_BREAKAWAY_REARM_MS*(uint32_t)PWM_FREQ)/
                                   (1000u*(uint32_t)MCCONF_FOC_CONTROL_DIV);
        if(m->m_position_motion_seen && abs_error>(int32_t)MCCONF_POSITION_PHASE_DEADBAND_MDEG &&
           m->m_position_no_motion_ticks>=rearm_ticks){
            m->m_position_motion_seen=0u;m->m_position_breakaway_ticks=0u;m->m_position_no_motion_ticks=0u;
        }
        const bool kick=(abs_error>(int32_t)MCCONF_POSITION_PHASE_DEADBAND_MDEG &&
                         !m->m_position_motion_seen && m->m_position_breakaway_ticks<kick_max);
        if(kick){
            m->m_position_breakaway_ticks++;
            int32_t kick_q4=((int32_t)FOC_CURRENT_Q4_PER_A*(int32_t)MCCONF_POSITION_BREAKAWAY_CURRENT_MA)/1000;
            if(kick_q4>limit_q4)kick_q4=limit_q4;
            return (int16_t)(error_mdeg>0?kick_q4:-kick_q4);
        }
        int32_t pos_lim_q4=((int32_t)FOC_CURRENT_Q4_PER_A*(int32_t)MCCONF_POSITION_RUN_CURRENT_MAX_MA)/1000;
        if(pos_lim_q4>limit_q4)pos_lim_q4=limit_q4;
        m->m_position_drive_direction=0;
        m->m_position_settle_ticks=0;
        return (int16_t)(((int64_t)out_q15*pos_lim_q4)/32768LL);
    }

    /* VESC p_pid_kd_proc: derivative of the measured electrical position is
     * a direct damping term and remains useful even when Hall angle is coarse at
     * standstill. m_rpm*pp is signed ERPM, so this is exactly -deg/s*Kd_proc. */
    int64_t dproc64=-(int64_t)m->m_rpm*(int32_t)motor_pole_pairs(second)*
                    (int64_t)m->m_position_kd_proc_coeff_q16;
    dproc64 >>= 16;
    if(dproc64>32768)dproc64=32768; else if(dproc64<-32768)dproc64=-32768;
    const int32_t dproc_raw_q15=(int32_t)dproc64;
    const int32_t dpdiff=dproc_raw_q15-m->m_position_d_proc_filter_q15;
    m->m_position_d_proc_filter_q15 +=
        (int32_t)(((int64_t)dpdiff*m->m_position_kd_filter_q16)>>16);
    const int32_t dproc_q15=m->m_position_d_proc_filter_q15;

    int32_t out_q15=p_q15+(m->m_position_integrator>>16)+d_q15+dproc_q15;
    out_q15=CLAMP(out_q15,-32768,32768);

    /* Keep VESC's normalized position PID, but scale it by a hardware-safe
     * position current range instead of multiplying by the full 15 A motor
     * limit and clipping afterwards. Post-clipping turns small position errors
     * into bang-bang current; pre-scaling preserves proportional authority near
     * the target while still providing enough breakaway torque at large error. */
    int32_t pos_lim_q4=((int32_t)FOC_CURRENT_Q4_PER_A*(int32_t)MCCONF_POSITION_CURRENT_MAX_MA)/1000;
    if(pos_lim_q4>limit_q4)pos_lim_q4=limit_q4;
    int32_t iq_cmd_q4=(int32_t)(((int64_t)out_q15*pos_lim_q4)/32768LL);

    {
        if(count_error>0){
            m->m_position_drive_direction=1; m->m_position_settle_ticks=0;
        }else if(count_error<0){
            m->m_position_drive_direction=-1; m->m_position_settle_ticks=0;
        }else if(m->m_position_drive_direction!=0){
            const uint32_t settle_max=((uint32_t)MCCONF_POSITION_SETTLE_MS*(uint32_t)PWM_FREQ)/
                                      (1000u*(uint32_t)MCCONF_FOC_CONTROL_DIV);
            if(m->m_position_settle_ticks<settle_max){
                const int32_t settle_q4=((int32_t)FOC_CURRENT_Q4_PER_A*
                                         (int32_t)MCCONF_POSITION_SETTLE_CURRENT_MA)/1000;
                iq_cmd_q4=(int32_t)m->m_position_drive_direction*settle_q4;
                m->m_position_settle_ticks++;
            }else{
                iq_cmd_q4=0; m->m_position_drive_direction=0;
            }
        }else{
            iq_cmd_q4=0;
        }
    }
    return (int16_t)iq_cmd_q4;
}

static int16_t speed_pid_iq_target_erpm_step(mcpwm_foc_motor_t *m, bool second,
                                                   int32_t target_erpm_q16, int32_t output_limit_q4) {
    /* VESC speed PID -> Iq. The normal speed mode uses the full configured
     * current range. Hall-position mode reuses the exact same regulator with a
     * smaller output ceiling, so position cannot wind the speed integrator into
     * multi-ampere torque while a wheel is mechanically blocked. */
    const int32_t pp=(int32_t)motor_pole_pairs(second);
    const int32_t full_limit_q4=m->m_current_limit_q4>0?m->m_current_limit_q4:MCCONF_MOTOR_CURRENT_MAX_Q4;
    int32_t limit_q4=output_limit_q4;
    if(limit_q4<=0 || limit_q4>full_limit_q4)limit_q4=full_limit_q4;
    const int64_t target64=(int64_t)target_erpm_q16;
    int64_t measured64=(int64_t)measured_mech_rpm_q16(m,second)*pp;
    int64_t error64 = target64 - measured64;
    if (error64 > INT32_MAX) error64 = INT32_MAX;
    if (error64 < INT32_MIN) error64 = INT32_MIN;
    const int32_t error_q16 = (int32_t)error64;

    /* VESC foc_run_pid_control_speed zeros the torque request below
     * s_pid_min_erpm. With Hall sensing this also prevents a low-speed
     * boundary-hunting limit cycle where one Hall sector spans a large
     * fraction of the requested speed period. */
    const int64_t min_erpm_q16 = (int64_t)m->m_speed_release_rpm * pp * 65536LL;
    const int64_t target_abs_q16 = target64 < 0 ? -target64 : target64;
    if (target_abs_q16 < min_erpm_q16) {
        m->m_speed_integrator = 0;
        m->m_speed_sat_hold = 0u;
        m->m_speed_prev_error = error_q16;
        return 0;
    }

    /* Keep 0.25-ERPM resolution while avoiding three software 64-bit divides.
     * Coefficients are recomputed when VESC Tool/EEPROM tuning changes. */
    const int32_t error_q2=error_q16>>14;
    int64_t pnorm64=((int64_t)error_q2*(int64_t)m->m_speed_kp_coeff_q16)>>16;
    if(pnorm64>32768)pnorm64=32768; else if(pnorm64<-32768)pnorm64=-32768;
    const int32_t p_q4=(int32_t)((pnorm64*(int64_t)full_limit_q4)>>15);

    const int64_t i_step=((int64_t)error_q2*(int64_t)m->m_speed_ki_coeff_q16)>>16;
    const int64_t i_lim = (int64_t)limit_q4 << 16;
    int64_t i_candidate = (int64_t)m->m_speed_integrator + i_step;
    if (i_candidate > i_lim) i_candidate = i_lim;
    if (i_candidate < -i_lim) i_candidate = -i_lim;

    int32_t d_q4 = 0;
    if (m->m_speed_kd_coeff_q8 != 0u) {
        int64_t de64=(int64_t)error_q16-(int64_t)m->m_speed_prev_error;
        if(de64>INT32_MAX)de64=INT32_MAX; else if(de64<INT32_MIN)de64=INT32_MIN;
        const uint32_t de_mag_q16=(uint32_t)(de64<0 ? -de64 : de64);
        const int32_t de_q2=de64<0 ? -(int32_t)(de_mag_q16>>14) : (int32_t)(de_mag_q16>>14);
        int64_t dq=((int64_t)de_q2*(int64_t)m->m_speed_kd_coeff_q8)>>8;
        if(dq>limit_q4)dq=limit_q4; else if(dq<-limit_q4)dq=-limit_q4;
        d_q4=(int32_t)dq;
    }

    int32_t out_q4 = p_q4 + (int32_t)(i_candidate >> 16) + d_q4;
    const bool sat_hi = out_q4 > limit_q4;
    const bool sat_lo = out_q4 < -limit_q4;
    /* Conditional integration: when saturated, only integrate an error that
     * moves the output back toward the linear region. */
    if ((sat_hi && i_step > 0) || (sat_lo && i_step < 0)) {
        i_candidate = m->m_speed_integrator;
        out_q4 = p_q4 + (int32_t)(i_candidate >> 16) + d_q4;
        m->m_speed_sat_hold = 1u;
    } else {
        m->m_speed_integrator = (int32_t)i_candidate;
        m->m_speed_sat_hold = 0u;
    }
    m->m_speed_prev_error = error_q16;

    if (!m->m_conf.s_pid_allow_braking) {
        const int64_t erpm20_q16 = 20LL << 16;
        if (measured64 > erpm20_q16 && out_q4 < 0) out_q4 = 0;
        if (measured64 < -erpm20_q16 && out_q4 > 0) out_q4 = 0;
    }
    return (int16_t)CLAMP(out_q4,-limit_q4,limit_q4);
}

static int16_t speed_pid_iq_target_step(mcpwm_foc_motor_t *m,bool second){
    const int32_t pp=(int32_t)motor_pole_pairs(second);
    int64_t target64=(int64_t)m->m_speed_set_ramp_q16*pp;
    if(target64>INT32_MAX)target64=INT32_MAX;
    if(target64<INT32_MIN)target64=INT32_MIN;
    const int32_t full_limit=m->m_current_limit_q4>0?m->m_current_limit_q4:MCCONF_MOTOR_CURRENT_MAX_Q4;
    return speed_pid_iq_target_erpm_step(m,second,(int32_t)target64,full_limit);
}

static int16_t duty_control_iq_target_step(mcpwm_foc_motor_t *m) {
    int32_t set=m->m_duty_ramp_permille;
    const int32_t max_permille=m->m_duty_limit_permille>0?m->m_duty_limit_permille:1000;
    set=CLAMP(set,-max_permille,max_permille);
    if(set==0){m->m_duty_i_q15=0;m->m_duty_pi_active=0u;return 0;}
    const int32_t now=m->m_duty_now_permille;
    const int32_t aset=set<0?-set:set, anow=now<0?-now:now;
    const bool same_sign=(now==0)||((set>0)==(now>0));
    const int32_t limit=m->m_current_limit_q4>0?m->m_current_limit_q4:MCCONF_MOTOR_CURRENT_MAX_Q4;
    if(!(same_sign && anow>aset+10)){
        m->m_duty_i_q15=0; m->m_duty_pi_active=0u;
        return (int16_t)(set>0?limit:-limit);
    }
    /* Upstream VESC uses a duty down-ramp PI that commands Iq while the current
     * PI remains the inner loop. Fixed-point equivalent uses nominal bus scaling
     * precomputed outside the ISR, avoiding float/division in the 16-kHz path. */
    const int32_t err=set-now;
    const int32_t p_q15=(int32_t)(((int64_t)err*m->m_duty_kp_q12_per_permille)>>12);
    int32_t i=m->m_duty_i_q15+(int32_t)(((int64_t)err*m->m_duty_ki_q12_per_permille)>>12);
    i=CLAMP(i,-32768,32768); m->m_duty_i_q15=i; m->m_duty_pi_active=1u;
    int32_t out=CLAMP(p_q15+i,-32768,32768);
    return (int16_t)(((int64_t)out*limit)/32768LL);
}

static int16_t phase_current_counts_to_q4(const mcpwm_foc_motor_t *m, int16_t counts) {
    /* Keep a one/two-sample high-duty shunt glitch from kicking the current PI
     * far outside the absolute-current envelope while the qualified ABS fault
     * logic below decides whether it is persistent. */
    const int32_t max_counts=(m && m->m_abs_current_limit_counts>0)?
                             m->m_abs_current_limit_counts:
                             (int32_t)(MCCONF_L_ABS_CURRENT_MAX*(float)A2BIT_CONV);
    int32_t c=CLAMP((int32_t)counts,-max_counts,max_counts);
    int32_t q4=c<<4;
    /* Preserve the generated-controller numeric saturation as a final guard. */
    if(q4>27200)q4=27200; else if(q4<-27200)q4=-27200;
    return (int16_t)q4;
}

static int16_t telemetry_lpf_step(int32_t *state_q16, uint16_t alpha_q16, int16_t sample) {
    const int32_t target=(int32_t)sample<<16;
    const int32_t diff=target-*state_q16;
    *state_q16 += (int32_t)(((int64_t)diff*(int32_t)alpha_q16)>>16);
    return (int16_t)(*state_q16>>16);
}

static int16_t off_telem_deadband_counts(int16_t sample) {
    /* High-impedance current amplifiers have a separate common-mode operating
     * point. After its stationary zero calibration, preserve changes around
     * that baseline for passive/back-drive telemetry, but strip ADC noise. */
    const int16_t db=(int16_t)MCCONF_OFF_TELEM_DEADBAND_COUNTS;
    if (sample > db) return (int16_t)(sample-db);
    if (sample < -db) return (int16_t)(sample+db);
    return 0;
}

static void telemetry_avg_push(mcpwm_foc_motor_t *m, int16_t id_q4, int16_t iq_q4, int16_t ibus_counts) {
    /* COMM_GET_VALUES normally consumes this every ~20 ms. Bound the window so
     * a disconnected VESC Tool can never overflow the 32-bit accumulators. */
    if (m->m_telem_avg_samples >= 1024u) {
        m->m_telem_sum_id_q4=0; m->m_telem_sum_iq_q4=0; m->m_telem_sum_ibus_counts=0;
        m->m_telem_avg_samples=0u;
    }
    m->m_telem_sum_id_q4 += id_q4;
    m->m_telem_sum_iq_q4 += iq_q4;
    m->m_telem_sum_ibus_counts += ibus_counts;
    m->m_telem_avg_samples++;
}

static void motor_control_step(mcpwm_foc_motor_t *m, bool second, int16_t i0_counts,
                               int16_t i1_counts, int16_t idc_counts, bool control_update) {
    hall_update(m, second);
    if (m->m_control_mode==CONTROL_MODE_OPENLOOP) {
        openloop_update(m, second);
    } else if (m->m_control_mode==CONTROL_MODE_OPENLOOP_PHASE) {
        /* Hall detection/static phase mode: ramp Id only. Never run the rotating
         * open-loop updater here, otherwise it overwrites m_phase_openloop. */
        openloop_current_ramp_update(m);
    }

    /* Hall estimator needs the generated +30deg sector-center offset. Synthetic
     * measured/open-loop phase does NOT: the old generated measurement branch
     * subtracts 30deg before a +30deg sine table, giving a net zero offset. */
    if (m->m_control_mode==CONTROL_MODE_HANDBRAKE) {
        /* Upstream VESC fixes electrical phase at zero in handbrake mode. The
         * requested current then produces a stationary locking field rather
         * than a rotating torque command. */
        m->m_phase=0u;
    } else {
        m->m_phase=(m->m_phase_override && (m->m_control_mode==CONTROL_MODE_OPENLOOP || m->m_control_mode==CONTROL_MODE_OPENLOOP_PHASE)) ?
            m->m_phase_openloop : m->m_phase_hall;
    }

    /* Control-current offset is calibrated once in the original EFeru
     * startup ADC window. Do not pause/reset a new command to re-learn offset:
     * doing that while a rotor is already moving can fold real BEMF/current into
     * the offset and adds a visible dead time at every OFF->RUN transition. */

    /* Keep the ADC/current estimator running for hardware diagnostics, but match
     * upstream VESC semantics at the public motor-state boundary: when the motor
     * is released, standard Id/Iq/i_abs/i_bus telemetry is zero. Raw ADC and the
     * high-impedance baseline remain available only through custom diagnostics. */
    const int16_t i0_q4=phase_current_counts_to_q4(m,i0_counts);
    const int16_t i1_q4=phase_current_counts_to_q4(m,i1_counts);
    foc_ab_t ab; if(second)foc_clarke_bc_q4(i0_q4,i1_q4,&ab); else foc_clarke_ab_q4(i0_q4,i1_q4,&ab);
    m->m_i_alpha_q4=ab.alpha; m->m_i_beta_q4=ab.beta;
    foc_dq_t raw, filt; foc_park_q4(&ab,m->m_phase,&raw);
    foc_lpf2_fixed_t lf={{m->m_current_lpf_q16[0],m->m_current_lpf_q16[1]}};
    foc_lpf2_run(&lf,MCCONF_FOC_CURRENT_FILTER_Q16,&raw,&filt);
    m->m_current_lpf_q16[0]=lf.state_q16[0];m->m_current_lpf_q16[1]=lf.state_q16[1];
    m->m_current_in_counts=idc_counts;
    m->m_iq_q4=filt.q; m->m_id_q4=filt.d;
    if(control_update){
        const uint16_t a=m->m_telem_current_filter_q16?m->m_telem_current_filter_q16:6553u;
        m->m_id_telem_q4=telemetry_lpf_step(&m->m_telem_current_lpf_q16[0],a,m->m_id_q4);
        m->m_iq_telem_q4=telemetry_lpf_step(&m->m_telem_current_lpf_q16[1],a,m->m_iq_q4);
        m->m_current_in_telem_counts=telemetry_lpf_step(&m->m_telem_current_lpf_q16[2],a,m->m_current_in_counts);
        telemetry_avg_push(m,m->m_id_telem_q4,m->m_iq_telem_q4,m->m_current_in_telem_counts);
    }

    const bool source_enabled = (enable != 0u) || mcpwm_foc_vesc_command_live(second);
    if (!source_enabled || m->m_fault!=FAULT_CODE_NONE || m->m_control_mode==CONTROL_MODE_NONE) {
        m->m_state=MC_STATE_OFF;
        reset_current_pi(m); m->m_speed_integrator=0; m->m_speed_prev_error=0;
        m->m_speed_sat_hold=0; reset_position_pid(m);
        m->m_iq_set_q4=0; m->m_iq_target_q4=0; m->m_iq_set_ramp_q16=0;
        m->m_id_set_q4=0; m->m_openloop_id_target_q4=0; m->m_openloop_id_ramp_q16=0;
        /* Upstream VESC reports zero current while the motor is undriven. Clear
         * both public/current-filter state and the read/reset averaging window so
         * an OFF high-impedance amplifier bias cannot leak into COMM_GET_VALUES. */
        m->m_i_alpha_q4=0; m->m_i_beta_q4=0; m->m_id_q4=0; m->m_iq_q4=0; m->m_current_in_counts=0;
        m->m_current_lpf_q16[0]=0; m->m_current_lpf_q16[1]=0;
        m->m_id_telem_q4=0; m->m_iq_telem_q4=0; m->m_current_in_telem_counts=0;
        m->m_telem_current_lpf_q16[0]=0; m->m_telem_current_lpf_q16[1]=0; m->m_telem_current_lpf_q16[2]=0;
        m->m_telem_sum_id_q4=0; m->m_telem_sum_iq_q4=0; m->m_telem_sum_ibus_counts=0; m->m_telem_avg_samples=0u;
        m->m_vd=0; m->m_vq=0;
        m->m_pwm_a=0; m->m_pwm_b=0; m->m_pwm_c=0; m->m_duty_now_permille=0;
        m->m_ccr_a=pwm_res/2u; m->m_ccr_b=pwm_res/2u; m->m_ccr_c=pwm_res/2u;
        m->m_isr_count++;
        return;
    }

    /* A newly-enabled advanced-timer bridge changes the low-side current
     * amplifier common-mode operating point. Hold a true zero vector for only
     * a few synchronized ADC frames; do not recalibrate and do not touch the
     * requested current/speed/position setpoints. */
    if (m->m_bridge_settle_ticks != 0u) {
        m->m_state=MC_STATE_RUNNING;
        reset_current_pi(m);
        m->m_vd=0; m->m_vq=0;
        m->m_pwm_a=0; m->m_pwm_b=0; m->m_pwm_c=0; m->m_duty_now_permille=0;
        m->m_ccr_a=pwm_res/2u; m->m_ccr_b=pwm_res/2u; m->m_ccr_c=pwm_res/2u;
        m->m_isr_count++;
        return;
    }

    foc_dq_t v={m->m_vd,m->m_vq};
    m->m_state=MC_STATE_RUNNING;
        if (control_update) {
            /* The proven generated controller updates its regulators once every
             * three 16-kHz ADC frames (~5.333 kHz). */
            /* Current/speed/position modes may use the same configured physical
             * full-safe modulation ceiling as VESC duty mode. */
            /* VESC duty is normalized to [-1,1], but the physical hoverboard
             * FOC bridge must retain EFeru's 110-count sampling margin. Map
             * normalized duty to the board-scaled vector from config.h. The
             * separate FOC_SVPWM_VECTOR_FULL_SAFE constant retains EFeru's
             * exact 110..1890 electrical ceiling for reference/testing. */
            int32_t vcfg=((int32_t)(m->m_duty_limit_permille>0?m->m_duty_limit_permille:1000)*
                          (int32_t)MCCONF_FOC_DUTY_VOLTAGE_MAX)/1000;
            vcfg=CLAMP(vcfg,1,MCCONF_FOC_DUTY_VOLTAGE_MAX);
            const int16_t v_closed=(int16_t)vcfg;
            int16_t vector_limit=v_closed;

            if (m->m_control_mode==CONTROL_MODE_DUTY) duty_setpoint_slew_step(m);
            if (m->m_control_mode==CONTROL_MODE_SPEED) {
                speed_setpoint_slew_step(m);
                /* Do not release the bridge while a non-zero braking Iq is
                 * still active. That abrupt torque->coast transition is the
                 * physical "glek". Below s_pid_min_erpm the speed PID is reset,
                 * then Iq is slewed to zero; MOE is released only after the
                 * current reference has actually reached zero. */
            }

            m->m_id_set_q4 = (m->m_control_mode==CONTROL_MODE_OPENLOOP ||
                              m->m_control_mode==CONTROL_MODE_OPENLOOP_PHASE) ?
                              m->m_id_set_q4 : 0;
            const int16_t ed=(int16_t)CLAMP((int32_t)m->m_id_set_q4-m->m_id_q4,-32768,32767);
            v.d=pi_run_state(ed,m->m_kpd_q11,m->m_kid_q16,
                             v_closed,-v_closed,&m->m_id_integrator,&m->m_id_sat_hold);
            int16_t q_lim=voltage_circle_q_limit(v.d,v_closed);
            if (m->m_control_mode==CONTROL_MODE_DUTY) {
                int32_t dp=m->m_duty_ramp_permille<0?-(int32_t)m->m_duty_ramp_permille:(int32_t)m->m_duty_ramp_permille;
                const int32_t cfgmax=m->m_duty_limit_permille>0?m->m_duty_limit_permille:1000;
                if(dp>cfgmax)dp=cfgmax;
                int32_t duty_v=(dp*MCCONF_FOC_DUTY_VOLTAGE_MAX)/1000;
                if(duty_v>MCCONF_FOC_DUTY_VOLTAGE_MAX)duty_v=MCCONF_FOC_DUTY_VOLTAGE_MAX;
                vector_limit=(int16_t)duty_v;
                q_lim=voltage_circle_q_limit(v.d,vector_limit);
            }
            if (m->m_control_mode==CONTROL_MODE_SPEED) {
                const int32_t pp=(int32_t)motor_pole_pairs(second);
                const int64_t set_erpm_q16=(int64_t)m->m_speed_set_ramp_q16*pp;
                const int64_t abs_set_erpm_q16=set_erpm_q16<0?-set_erpm_q16:set_erpm_q16;
                const int64_t min_erpm_q16=(int64_t)m->m_speed_release_rpm*pp*65536LL;
                const bool stop_zone=(m->m_speed_target_rpm_q16==0 && abs_set_erpm_q16<min_erpm_q16);
                if(stop_zone){
                    m->m_speed_integrator=0; m->m_speed_prev_error=0; m->m_speed_sat_hold=0;
                    m->m_iq_target_q4=0;
                    iq_setpoint_slew_step(m);
                    if(m->m_iq_set_q4==0 && m->m_iq_set_ramp_q16==0){
                        reset_current_pi(m); mcpwm_foc_release_motor(second);
                        v.q=0; v.d=0; goto control_done;
                    }
                }else{
                    /* Normal VESC speed control writes PID Iq directly. Only
                     * the final STOP handoff uses current slew to remove torque
                     * before release. */
                    m->m_iq_target_q4 = speed_pid_iq_target_step(m, second);
                    m->m_iq_set_q4 = m->m_iq_target_q4;
                    m->m_iq_set_ramp_q16 = (int32_t)m->m_iq_set_q4 << 16;
                }
                m->m_iq_set_q4=current_circle_iq_limit_q4(m,m->m_iq_set_q4);
                const int16_t eq=(int16_t)CLAMP((int32_t)m->m_iq_set_q4-m->m_iq_q4,-32768,32767);
                v.q=pi_run_state(eq,m->m_kpq_q11,m->m_kiq_q16,
                                 q_lim,(int16_t)-q_lim,
                                 &m->m_iq_integrator,&m->m_iq_sat_hold);
            } else {
                if(m->m_control_mode==CONTROL_MODE_DUTY){
                    m->m_iq_target_q4=duty_control_iq_target_step(m);
                    if(m->m_duty_pi_active){
                        m->m_iq_set_q4=m->m_iq_target_q4;
                        m->m_iq_set_ramp_q16=(int32_t)m->m_iq_set_q4<<16;
                    }else{
                        /* Hardware adaptation: VESC duty remains current-limited,
                         * but ramp positive torque on this two-shunt hoverboard so
                         * bridge enable cannot create a large one-frame current step. */
                        iq_setpoint_slew_step(m);
                    }
                } else if(m->m_control_mode==CONTROL_MODE_POS){
                    m->m_iq_target_q4=position_pid_iq_target_step(m,second);
                    /* Position cascade is intentionally current-slewed. The
                     * outer loop runs at low Hall positioning speed, so a 10 A/s
                     * slew removes bridge torque steps without hiding damping. */
                    iq_setpoint_slew_step(m);
                } else if (m->m_control_mode==CONTROL_MODE_CURRENT_BRAKE) {
                    bool same_motion=false;
                    if(m->m_brake_direction!=0 && m->m_hall_initialized &&
                       m->m_hall_period>0u && m->m_hall_period<MCCONF_HALL_TIMEOUT_TICKS){
                        /* Hall RPM is only refreshed on an edge. During hard braking
                         * the last period can therefore claim 20..30 rpm even after
                         * the rotor has nearly stopped. Use elapsed ticks since the
                         * last edge as a monotonic upper-bound speed estimate. This
                         * releases brake torque before the first reverse Hall edge,
                         * preventing the classic Hall-brake zero-cross kick. */
                        uint32_t age=m->m_hall_ticks;
                        if(age<m->m_hall_period) age=m->m_hall_period;
                        if(age==0u) age=1u;
                        const int32_t est_rpm=(int32_t)(10667u/age);
                        const bool direction_same=(m->m_hall_direction==m->m_brake_direction);
                        if(direction_same && est_rpm>MCCONF_TRQ_STOP_RPM_DEADBAND){
                            same_motion=true;
                        }
                    }
                    m->m_iq_target_q4=same_motion?
                        (m->m_brake_direction>0?(int16_t)-m->m_brake_current_q4:m->m_brake_current_q4):0;
                    iq_setpoint_slew_step(m);
                    if(!same_motion && m->m_iq_set_q4==0 && m->m_iq_set_ramp_q16==0){
                        reset_current_pi(m); mcpwm_foc_release_motor(second);
                        v.q=0; v.d=0; goto control_done;
                    }
                } else if (m->m_control_mode==CONTROL_MODE_HANDBRAKE) {
                    m->m_iq_target_q4=m->m_handbrake_current_q4;
                    iq_setpoint_slew_step(m);
                } else if (m->m_control_mode==CONTROL_MODE_CURRENT) {
                    iq_setpoint_slew_step(m);
                } else {
                    m->m_iq_target_q4=0;
                    m->m_iq_set_q4=0;
                    m->m_iq_set_ramp_q16=0;
                }
                m->m_iq_set_q4=current_circle_iq_limit_q4(m,m->m_iq_set_q4);
                const int16_t eq=(int16_t)CLAMP((int32_t)m->m_iq_set_q4-m->m_iq_q4,-32768,32767);
                v.q=pi_run_state(eq,m->m_kpq_q11,m->m_kiq_q16,
                                 q_lim,(int16_t)-q_lim,
                                 &m->m_iq_integrator,&m->m_iq_sat_hold);
            }
            /* Backup only; q_lim already enforces the voltage circle before
             * the Iq PI, so its anti-windup sees the real available headroom. */
            foc_vector_limit(&v,vector_limit);
        }
control_done:
    m->m_vd=v.d; m->m_vq=v.q;
    foc_abc_t pwm; foc_centered_svpwm(&v,m->m_phase,&pwm);
    /* Keep EFeru's exact hardware clamp: ARR=2000, pwm_margin=110, therefore
     * every CCR remains in 110..1890. Normal VESC commands are additionally
     * scaled in config.h, leaving margin below this absolute electrical clamp. */
    m->m_pwm_a=pwm.a; m->m_pwm_b=pwm.b; m->m_pwm_c=pwm.c;
    /* VESC Tool still uses normalized duty [-1,1]. Report the physical full-safe
     * vector as 1.000, so command 1.0 and telemetry 1.0 both mean EFeru's real
     * PWM ceiling rather than an unsafe mathematical CCR rail. */
    if (control_update) {
        uint32_t mag2=(uint32_t)((int32_t)v.d*v.d)+(uint32_t)((int32_t)v.q*v.q);
        uint32_t mag=foc_isqrt_u32(mag2);
        int32_t dpm=(int32_t)(((uint64_t)mag*1000u+(MCCONF_FOC_DUTY_VOLTAGE_MAX/2u))/MCCONF_FOC_DUTY_VOLTAGE_MAX);
        if(dpm>1000)dpm=1000;
        if(v.q<0)dpm=-dpm;
        m->m_duty_now_permille=(int16_t)dpm;
    }
    m->m_ccr_a=(uint16_t)CLAMP((int32_t)pwm.a+pwm_res/2,pwm_margin,pwm_res-pwm_margin);
    m->m_ccr_b=(uint16_t)CLAMP((int32_t)pwm.b+pwm_res/2,pwm_margin,pwm_res-pwm_margin);
    m->m_ccr_c=(uint16_t)CLAMP((int32_t)pwm.c+pwm_res/2,pwm_margin,pwm_res-pwm_margin);
    m->m_isr_count++;
}

void mcpwm_foc_adc_int_handler(void) {
    motor_fault_recovery_tick(&m_motor_1);
    motor_fault_recovery_tick(&m_motor_2);
    /* Source arbitration follows VESC timeout semantics. A SET_* packet claims
     * the motor; COMM_ALIVE only refreshes that command timeout. Expiry stops or
     * brakes the motor but deliberately keeps VESC ownership, preventing stale
     * legacy pwml/pwmr data from taking over immediately after a timeout. */
    for (uint8_t vi=0u; vi<2u; ++vi) {
        if (s_vesc_owned[vi]) {
            uint32_t t=s_vesc_timeout_ticks[vi];
            if (t != 0u && t != UINT32_MAX) {
                t--; s_vesc_timeout_ticks[vi]=t;
                if (t == 0u) {
                    const bool second=(vi!=0u);
                    const float brake=s_vesc_timeout_brake_a[vi];
                    if (brake > 0.001f) {
                        mcpwm_foc_set_brake_current(brake,second);
                        s_vesc_timeout_braking[vi]=1u;
                    } else {
                        mcpwm_foc_release_motor(second);
                        s_vesc_timeout_braking[vi]=0u;
                    }
                }
            }
        } else if (vi==0u) {
            mcpwm_foc_set_mode_command(ctrlModReq,(int16_t)pwml,motorRunReq!=0u,svpwmOpenloopRpm,false);
        } else {
            mcpwm_foc_set_mode_command(ctrlModReq,(int16_t)pwmr,motorRunReq!=0u,svpwmOpenloopRpm,true);
        }
    }
    /* Keep each motor at the configured 5.33-kHz regulator cadence, but do
     * not execute both heavy PI updates on the same 16-kHz ADC interrupt. With
     * DIV=3: left updates slot 0, right slot 1, slot 2 is estimator/SVPWM only. */
    const uint8_t control_slot=s_foc_control_div;
    if(++s_foc_control_div>=MCCONF_FOC_CONTROL_DIV)s_foc_control_div=0u;
    const bool update_left=(control_slot==0u);
    const bool update_right=(MCCONF_FOC_CONTROL_DIV<=1u)?update_left:(control_slot==1u);
    motor_control_step(&m_motor_1,false,curL_phaA,curL_phaB,curL_DC,update_left);
    motor_control_step(&m_motor_2,true,curR_phaB,curR_phaC,curR_DC,update_right);
    foc_iqL_q4=m_motor_1.m_iq_q4;foc_idL_q4=m_motor_1.m_id_q4;
    foc_iqR_q4=m_motor_2.m_iq_q4;foc_idR_q4=m_motor_2.m_id_q4;
    LEFT_TIM->LEFT_TIM_U=m_motor_1.m_ccr_a;LEFT_TIM->LEFT_TIM_V=m_motor_1.m_ccr_b;LEFT_TIM->LEFT_TIM_W=m_motor_1.m_ccr_c;
    RIGHT_TIM->RIGHT_TIM_U=m_motor_2.m_ccr_a;RIGHT_TIM->RIGHT_TIM_V=m_motor_2.m_ccr_b;RIGHT_TIM->RIGHT_TIM_W=m_motor_2.m_ccr_c;
}

void mcpwm_foc_vesc_timeout_configure(bool second, uint32_t timeout_ms, float brake_current) {
    const uint8_t i=second?1u:0u;
    if (brake_current < 0.0f) brake_current=-brake_current;
    if (brake_current > (float)I_MOT_MAX) brake_current=(float)I_MOT_MAX;
    s_vesc_timeout_ms[i]=timeout_ms;
    s_vesc_timeout_brake_a[i]=brake_current;
    if (s_vesc_owned[i]) mcpwm_foc_vesc_override_touch(second);
}

void mcpwm_foc_vesc_override_touch(bool second) {
    const uint8_t i=second?1u:0u;
    const uint32_t ms=s_vesc_timeout_ms[i];
    s_vesc_owned[i]=1u;
    s_vesc_timeout_braking[i]=0u;
    if (ms == 0u) {
        /* VESC App Config timeout_msec=0 explicitly disables the timeout. */
        s_vesc_timeout_ticks[i]=UINT32_MAX;
    } else {
        uint64_t ticks=((uint64_t)PWM_FREQ*(uint64_t)ms+999u)/1000u;
        if (ticks == 0u) ticks=1u;
        if (ticks >= (uint64_t)UINT32_MAX) ticks=(uint64_t)UINT32_MAX-1u;
        s_vesc_timeout_ticks[i]=(uint32_t)ticks;
    }
}
bool mcpwm_foc_vesc_override_active(bool second) { return s_vesc_owned[second?1u:0u] != 0u; }
bool mcpwm_foc_vesc_command_live(bool second) {
    const uint8_t i=second?1u:0u;
    return s_vesc_owned[i] && (s_vesc_timeout_ticks[i] != 0u || s_vesc_timeout_braking[i]);
}
void mcpwm_foc_vesc_override_clear(bool second) {
    const uint8_t i=second?1u:0u;
    s_vesc_owned[i]=0u; s_vesc_timeout_braking[i]=0u; s_vesc_timeout_ticks[i]=0u;
}

/* ========================================================================== */
/* Original EFeru ADC DMA ISR timing/calibration path                          */
/* ========================================================================== */
void DMA1_Channel1_IRQHandler(void) {
    const uint32_t focIsrStartCycles = DWT->CYCCNT;
    DMA1->IFCR = DMA_IFCR_CTCIF1;

    if(offsetcount < 2000) {  // calibrate ADC offsets
        offsetcount++;
        offsetrlA = (int16_t)(((int32_t)adc_buffer.rlA + (int32_t)offsetrlA) / 2);
        offsetrlB = (int16_t)(((int32_t)adc_buffer.rlB + (int32_t)offsetrlB) / 2);
        offsetrrB = (int16_t)(((int32_t)adc_buffer.rrB + (int32_t)offsetrrB) / 2);
        offsetrrC = (int16_t)(((int32_t)adc_buffer.rrC + (int32_t)offsetrrC) / 2);
        offsetdcl = (int16_t)(((int32_t)adc_buffer.dcl + (int32_t)offsetdcl) / 2);
        offsetdcr = (int16_t)(((int32_t)adc_buffer.dcr + (int32_t)offsetdcr) / 2);
        if(offsetcount==2000u){
            m_motor_1.m_driven_offset0=offsetrlA; m_motor_1.m_driven_offset1=offsetrlB; m_motor_1.m_driven_offsetdc=offsetdcl;
            m_motor_2.m_driven_offset0=offsetrrB; m_motor_2.m_driven_offset1=offsetrrC; m_motor_2.m_driven_offsetdc=offsetdcr;
            m_motor_1.m_driven_offset_valid=1u; m_motor_2.m_driven_offset_valid=1u;
            m_motor_1.m_driven_offset_calibrating=0u; m_motor_2.m_driven_offset_calibrating=0u;
            m_motor_1.m_driven_offset_samples=2000u; m_motor_2.m_driven_offset_samples=2000u;
        }
        foc_isr_monitor_end(focIsrStartCycles);
        return;
    }

    if (buzzerTimer % 1000 == 0) {
        filtLowPass32(adc_buffer.batt1, BAT_FILT_COEF, &batVoltageFixdt);
        batVoltage = (int16_t)(batVoltageFixdt >> 16);
    }

    /* VESC ownership is per motor. A right-motor forwarded command must not
     * accidentally energize a stale left control mode (and vice versa). */
    const uint8_t leftSourceEnable=(enable!=0u)||mcpwm_foc_vesc_override_active(false);
    const uint8_t rightSourceEnable=(enable!=0u)||mcpwm_foc_vesc_override_active(true);
    const uint8_t leftDriveRequest=(leftSourceEnable!=0u)&&(m_motor_1.m_control_mode!=CONTROL_MODE_NONE);
    const uint8_t rightDriveRequest=(rightSourceEnable!=0u)&&(m_motor_2.m_control_mode!=CONTROL_MODE_NONE);
    const uint8_t leftBridgeWasOn=(LEFT_TIM->BDTR&TIM_BDTR_MOE)?1u:0u;
    const uint8_t rightBridgeWasOn=(RIGHT_TIM->BDTR&TIM_BDTR_MOE)?1u:0u;
    if(leftDriveRequest && !leftBridgeWasOn && m_motor_1.m_fault==FAULT_CODE_NONE){
        m_motor_1.m_bridge_settle_ticks=MCCONF_BRIDGE_SETTLE_SAMPLES;
        LEFT_TIM->LEFT_TIM_U=pwm_res/2u; LEFT_TIM->LEFT_TIM_V=pwm_res/2u; LEFT_TIM->LEFT_TIM_W=pwm_res/2u;
        reset_current_pi(&m_motor_1);
        m_motor_1.m_current_lpf_q16[0]=m_motor_1.m_current_lpf_q16[1]=0;
    }
    if(rightDriveRequest && !rightBridgeWasOn && m_motor_2.m_fault==FAULT_CODE_NONE){
        m_motor_2.m_bridge_settle_ticks=MCCONF_BRIDGE_SETTLE_SAMPLES;
        RIGHT_TIM->RIGHT_TIM_U=pwm_res/2u; RIGHT_TIM->RIGHT_TIM_V=pwm_res/2u; RIGHT_TIM->RIGHT_TIM_W=pwm_res/2u;
        reset_current_pi(&m_motor_2);
        m_motor_2.m_current_lpf_q16[0]=m_motor_2.m_current_lpf_q16[1]=0;
    }
    /* Match upstream hoverboard-firmware-hack-FOC: the phase/DC current
     * control offsets are calibrated once during the first 2000 synchronized
     * ADC frames and then remain fixed. There is deliberately no per-start
     * driven-offset calibration and no 50%% zero-vector hold before a command.
     * m_driven_offset* mirrors this fixed control baseline for diagnostics only. */
    m_motor_1.m_driven_offset_calibrating=0u;
    m_motor_2.m_driven_offset_calibrating=0u;

    /* On the first ISR after a RUN->OFF transition the MOE state sampled at
     * entry is still ON, but it is disabled earlier in this same ISR. The
     * phase-current amplifier operating point can jump immediately to its OFF
     * bias. Reset measurement filters so that one mixed-offset sample cannot
     * appear as a 10 A telemetry spike. */
    if(!leftDriveRequest && leftBridgeWasOn){
        m_motor_1.m_current_lpf_q16[0]=m_motor_1.m_current_lpf_q16[1]=0;
        m_motor_1.m_telem_current_lpf_q16[0]=m_motor_1.m_telem_current_lpf_q16[1]=m_motor_1.m_telem_current_lpf_q16[2]=0;
    }
    if(!rightDriveRequest && rightBridgeWasOn){
        m_motor_2.m_current_lpf_q16[0]=m_motor_2.m_current_lpf_q16[1]=0;
        m_motor_2.m_telem_current_lpf_q16[0]=m_motor_2.m_telem_current_lpf_q16[1]=m_motor_2.m_telem_current_lpf_q16[2]=0;
    }

    /* Calibrate the high-impedance current-amplifier operating point only after
     * its common-mode transient has settled. After a driven interval, wait 50 ms
     * and (if the wheel moved) until the last Hall edge is stale before learning
     * 256 zero-current samples. This prevents post-release 5..20 A telemetry
     * artifacts while keeping manual back-drive current observable afterwards. */
    if(!leftBridgeWasOn && !leftDriveRequest && m_motor_1.m_off_settle_ticks>0u) m_motor_1.m_off_settle_ticks--;
    if(!rightBridgeWasOn && !rightDriveRequest && m_motor_2.m_off_settle_ticks>0u) m_motor_2.m_off_settle_ticks--;
    const bool leftOffStationary=(m_motor_1.m_rpm==0) &&
        (m_motor_1.m_hall_direction==0 || m_motor_1.m_hall_ticks>=MCCONF_HALL_TIMEOUT_TICKS);
    const bool rightOffStationary=(m_motor_2.m_rpm==0) &&
        (m_motor_2.m_hall_direction==0 || m_motor_2.m_hall_ticks>=MCCONF_HALL_TIMEOUT_TICKS);
    /* The low-side current amplifier high-Z operating point drifts for hundreds
     * of milliseconds after MOE is disabled. A one-shot OFF zero therefore
     * becomes stale and can show tens of amps while the bridge is open. Track
     * the OFF zero slowly only while Hall motion is stale/stationary. Once the
     * rotor moves, freeze the baseline so passive/manual-spin current remains
     * observable. This path is telemetry-only and never feeds protection. */
    if(m_motor_1.m_off_offset_valid && !leftBridgeWasOn && !leftDriveRequest && leftOffStationary){
        if((++m_motor_1.m_off_offset_samples & 7u)==0u){
            if(adc_buffer.rlA>m_motor_1.m_off_offset0)m_motor_1.m_off_offset0++; else if(adc_buffer.rlA<m_motor_1.m_off_offset0)m_motor_1.m_off_offset0--;
            if(adc_buffer.rlB>m_motor_1.m_off_offset1)m_motor_1.m_off_offset1++; else if(adc_buffer.rlB<m_motor_1.m_off_offset1)m_motor_1.m_off_offset1--;
            if(adc_buffer.dcl>m_motor_1.m_off_offsetdc)m_motor_1.m_off_offsetdc++; else if(adc_buffer.dcl<m_motor_1.m_off_offsetdc)m_motor_1.m_off_offsetdc--;
        }
    }
    if(m_motor_2.m_off_offset_valid && !rightBridgeWasOn && !rightDriveRequest && rightOffStationary){
        if((++m_motor_2.m_off_offset_samples & 7u)==0u){
            if(adc_buffer.rrB>m_motor_2.m_off_offset0)m_motor_2.m_off_offset0++; else if(adc_buffer.rrB<m_motor_2.m_off_offset0)m_motor_2.m_off_offset0--;
            if(adc_buffer.rrC>m_motor_2.m_off_offset1)m_motor_2.m_off_offset1++; else if(adc_buffer.rrC<m_motor_2.m_off_offset1)m_motor_2.m_off_offset1--;
            if(adc_buffer.dcr>m_motor_2.m_off_offsetdc)m_motor_2.m_off_offsetdc++; else if(adc_buffer.dcr<m_motor_2.m_off_offsetdc)m_motor_2.m_off_offsetdc--;
        }
    }
    if(!m_motor_1.m_off_offset_valid && m_motor_1.m_off_settle_ticks==0u && !leftBridgeWasOn && !leftDriveRequest && leftOffStationary){
        m_motor_1.m_off_offset_sum0+=(int32_t)adc_buffer.rlA;
        m_motor_1.m_off_offset_sum1+=(int32_t)adc_buffer.rlB;
        m_motor_1.m_off_offset_sumdc+=(int32_t)adc_buffer.dcl;
        if(++m_motor_1.m_off_offset_samples>=256u){
            const int32_t n=(int32_t)m_motor_1.m_off_offset_samples;
            m_motor_1.m_off_offset0=(int16_t)(m_motor_1.m_off_offset_sum0/n);
            m_motor_1.m_off_offset1=(int16_t)(m_motor_1.m_off_offset_sum1/n);
            m_motor_1.m_off_offsetdc=(int16_t)(m_motor_1.m_off_offset_sumdc/n);
            m_motor_1.m_off_offset_valid=1u;
            m_motor_1.m_current_lpf_q16[0]=m_motor_1.m_current_lpf_q16[1]=0;
            m_motor_1.m_telem_current_lpf_q16[0]=m_motor_1.m_telem_current_lpf_q16[1]=m_motor_1.m_telem_current_lpf_q16[2]=0;
        }
    }
    if(!m_motor_2.m_off_offset_valid && m_motor_2.m_off_settle_ticks==0u && !rightBridgeWasOn && !rightDriveRequest && rightOffStationary){
        m_motor_2.m_off_offset_sum0+=(int32_t)adc_buffer.rrB;
        m_motor_2.m_off_offset_sum1+=(int32_t)adc_buffer.rrC;
        m_motor_2.m_off_offset_sumdc+=(int32_t)adc_buffer.dcr;
        if(++m_motor_2.m_off_offset_samples>=256u){
            const int32_t n=(int32_t)m_motor_2.m_off_offset_samples;
            m_motor_2.m_off_offset0=(int16_t)(m_motor_2.m_off_offset_sum0/n);
            m_motor_2.m_off_offset1=(int16_t)(m_motor_2.m_off_offset_sum1/n);
            m_motor_2.m_off_offsetdc=(int16_t)(m_motor_2.m_off_offset_sumdc/n);
            m_motor_2.m_off_offset_valid=1u;
            m_motor_2.m_current_lpf_q16[0]=m_motor_2.m_current_lpf_q16[1]=0;
            m_motor_2.m_telem_current_lpf_q16[0]=m_motor_2.m_telem_current_lpf_q16[1]=m_motor_2.m_telem_current_lpf_q16[2]=0;
        }
    }

    /* Three current-sampling states are kept deliberately separate:
     *  1) DRIVEN+settled: use the original 2000-sample control offset.
     *  2) Stable bridge-OFF: use the frozen high-impedance offset for telemetry
     *     only, so manual back-drive/passive regeneration remains observable.
     *  3) OFF<->RUN transition/settling: sample is ambiguous, publish zero.
     * Only state (1) is ever allowed to feed over-current protection below. */
    const uint8_t leftCurrentSampleValid=leftBridgeWasOn&&leftDriveRequest&&(m_motor_1.m_bridge_settle_ticks==0u);
    const uint8_t rightCurrentSampleValid=rightBridgeWasOn&&rightDriveRequest&&(m_motor_2.m_bridge_settle_ticks==0u);
    const uint8_t leftOffTelemValid=(!leftBridgeWasOn)&&(!leftDriveRequest)&&m_motor_1.m_off_offset_valid;
    const uint8_t rightOffTelemValid=(!rightBridgeWasOn)&&(!rightDriveRequest)&&m_motor_2.m_off_offset_valid;
    if(leftCurrentSampleValid){
        curL_phaA=(int16_t)(offsetrlA-adc_buffer.rlA);
        curL_phaB=(int16_t)(offsetrlB-adc_buffer.rlB);
        curL_DC=(int16_t)(offsetdcl-adc_buffer.dcl);
    }else if(leftOffTelemValid){
        curL_phaA=off_telem_deadband_counts((int16_t)(m_motor_1.m_off_offset0-(int16_t)adc_buffer.rlA));
        curL_phaB=off_telem_deadband_counts((int16_t)(m_motor_1.m_off_offset1-(int16_t)adc_buffer.rlB));
        curL_DC=off_telem_deadband_counts((int16_t)(m_motor_1.m_off_offsetdc-(int16_t)adc_buffer.dcl));
    }else{ curL_phaA=0; curL_phaB=0; curL_DC=0; }
    if(rightCurrentSampleValid){
        curR_phaB=(int16_t)(offsetrrB-adc_buffer.rrB);
        curR_phaC=(int16_t)(offsetrrC-adc_buffer.rrC);
        curR_DC=(int16_t)(offsetdcr-adc_buffer.dcr);
    }else if(rightOffTelemValid){
        curR_phaB=off_telem_deadband_counts((int16_t)(m_motor_2.m_off_offset0-(int16_t)adc_buffer.rrB));
        curR_phaC=off_telem_deadband_counts((int16_t)(m_motor_2.m_off_offset1-(int16_t)adc_buffer.rrC));
        curR_DC=off_telem_deadband_counts((int16_t)(m_motor_2.m_off_offsetdc-(int16_t)adc_buffer.dcr));
    }else{ curR_phaB=0; curR_phaC=0; curR_DC=0; }

    const int32_t curL_phaC=-(int32_t)curL_phaA-(int32_t)curL_phaB;
    const int32_t curR_phaA=-(int32_t)curR_phaB-(int32_t)curR_phaC;
    const uint8_t leftOpenloop = (m_motor_1.m_control_mode==CONTROL_MODE_OPENLOOP ||
                                  m_motor_1.m_control_mode==CONTROL_MODE_OPENLOOP_PHASE);
    const uint8_t rightOpenloop = (m_motor_2.m_control_mode==CONTROL_MODE_OPENLOOP ||
                                   m_motor_2.m_control_mode==CONTROL_MODE_OPENLOOP_PHASE);
    const int32_t leftPhaseLimit=leftOpenloop?((int32_t)SVPWM_PHASE_LIMIT_A*A2BIT_CONV):m_motor_1.m_abs_current_limit_counts;
    const int32_t rightPhaseLimit=rightOpenloop?((int32_t)SVPWM_PHASE_LIMIT_A*A2BIT_CONV):m_motor_2.m_abs_current_limit_counts;
    /* Phase over-current protection must apply to every powered mode. Hall
     * detection uses the mode-4 current-control power path, so its fixed-phase
     * submode gets the same stricter phase/DC limits even when the legacy
     * ctrlModReq is not SVPWM_MODE. */
    /* VESC suppresses current-unbalance diagnostics when phase sampling loses
     * observability at high duty, but ABS over-current protection stays active.
     * On this fixed low-side-shunt board, qualify a >80% phase excursion for a
     * few consecutive ADC frames instead of disabling the fault outright. */
    /* ABS current is a VESC motor-current protection, not a raw two-shunt ADC
     * plausibility test. EFeru feeds the raw phase channels into the current
     * controller and uses DC-link current as the immediate hardware-level trip.
     * Near high duty one phase can be poorly observable; reconstructing C=-A-B
     * and faulting on that raw sample created false ABS trips at ~0.96 duty and
     * even on 1 A startup commands. Use D/Q current magnitude instead:
     *   slow_abs=false -> fast FOC feedback LPF (control current)
     *   slow_abs=true  -> additional monitoring LPF (slower VESC ABS option)
     * The raw DC-link trip below remains immediate. */
    const int32_t leftAbsQ4=leftPhaseLimit*16;
    const int32_t rightAbsQ4=rightPhaseLimit*16;
    const int32_t leftFastD=m_motor_1.m_id_q4, leftFastQ=m_motor_1.m_iq_q4;
    const int32_t rightFastD=m_motor_2.m_id_q4, rightFastQ=m_motor_2.m_iq_q4;
    const int32_t leftSlowD=m_motor_1.m_id_telem_q4, leftSlowQ=m_motor_1.m_iq_telem_q4;
    const int32_t rightSlowD=m_motor_2.m_id_telem_q4, rightSlowQ=m_motor_2.m_iq_telem_q4;
    const uint32_t leftFastMag2=(uint32_t)(leftFastD*leftFastD)+(uint32_t)(leftFastQ*leftFastQ);
    const uint32_t rightFastMag2=(uint32_t)(rightFastD*rightFastD)+(uint32_t)(rightFastQ*rightFastQ);
    const uint32_t leftSlowMag2=(uint32_t)(leftSlowD*leftSlowD)+(uint32_t)(leftSlowQ*leftSlowQ);
    const uint32_t rightSlowMag2=(uint32_t)(rightSlowD*rightSlowD)+(uint32_t)(rightSlowQ*rightSlowQ);
    const uint32_t leftLimit2=(uint32_t)(leftAbsQ4*leftAbsQ4);
    const uint32_t rightLimit2=(uint32_t)(rightAbsQ4*rightAbsQ4);
    const uint8_t leftPhaseExceeded=leftCurrentSampleValid&&
        (m_motor_1.m_conf.l_slow_abs_current?(leftSlowMag2>leftLimit2):(leftFastMag2>leftLimit2));
    const uint8_t rightPhaseExceeded=rightCurrentSampleValid&&
        (m_motor_2.m_conf.l_slow_abs_current?(rightSlowMag2>rightLimit2):(rightFastMag2>rightLimit2));
    /* Three consecutive D/Q samples reject turn-on/sampling transients while
     * remaining sub-millisecond. Catastrophic DC-link overcurrent is still the
     * immediate ISR-level shutdown path. */
    if(!leftPhaseExceeded || !leftCurrentSampleValid)m_motor_1.m_phase_overcurrent_streak=0u;
    else if(m_motor_1.m_phase_overcurrent_streak<MCCONF_ABS_CURRENT_QUAL_SAMPLES)m_motor_1.m_phase_overcurrent_streak++;
    if(!rightPhaseExceeded || !rightCurrentSampleValid)m_motor_2.m_phase_overcurrent_streak=0u;
    else if(m_motor_2.m_phase_overcurrent_streak<MCCONF_ABS_CURRENT_QUAL_SAMPLES)m_motor_2.m_phase_overcurrent_streak++;
    const uint8_t leftPhaseTrip=leftPhaseExceeded&&m_motor_1.m_phase_overcurrent_streak>=MCCONF_ABS_CURRENT_QUAL_SAMPLES;
    const uint8_t rightPhaseTrip=rightPhaseExceeded&&m_motor_2.m_phase_overcurrent_streak>=MCCONF_ABS_CURRENT_QUAL_SAMPLES;
    const int32_t leftDcLimit=leftOpenloop?((int32_t)SVPWM_DC_LIMIT_A*A2BIT_CONV):curDC_max;
    const int32_t rightDcLimit=rightOpenloop?((int32_t)SVPWM_DC_LIMIT_A*A2BIT_CONV):curDC_max;
    const uint8_t leftDcTrip = leftCurrentSampleValid && (ABS(curL_DC) > leftDcLimit);
    const uint8_t rightDcTrip = rightCurrentSampleValid && (ABS(curR_DC) > rightDcLimit);
    const uint8_t leftCurrentTrip = leftPhaseTrip || leftDcTrip;
    const uint8_t rightCurrentTrip = rightPhaseTrip || rightDcTrip;
    /* Safety gate phase 1: trip/stop disables the bridge immediately. Do NOT
     * arm a previously-off bridge here. The FOC step below must first compute
     * and write CCRs for the new command; otherwise the first powered PWM frame
     * uses the stale OFF-state 50% zero vector and can create a current spike. */
    if(leftCurrentTrip || leftDriveRequest==0u || m_motor_1.m_fault!=FAULT_CODE_NONE){
        LEFT_TIM->BDTR&=~TIM_BDTR_MOE;
        if(leftCurrentTrip){
            m_motor_1.m_current_trip_count++; if(leftPhaseTrip)m_motor_1.m_phase_trip_count++; if(leftDcTrip)m_motor_1.m_dc_trip_count++;
            m_motor_1.m_last_trip_source=(leftPhaseTrip?1u:0u)|(leftDcTrip?2u:0u);
            m_motor_1.m_last_trip_phase0_counts=curL_phaA; m_motor_1.m_last_trip_phase1_counts=curL_phaB; m_motor_1.m_last_trip_phase2_counts=(int16_t)CLAMP(curL_phaC,-32768,32767);
            m_motor_1.m_last_trip_dc_counts=curL_DC; m_motor_1.m_last_trip_duty_permille=m_motor_1.m_duty_now_permille;
            motor_fault_set(&m_motor_1,FAULT_CODE_ABS_OVER_CURRENT);
        }
    }
    if(rightCurrentTrip || rightDriveRequest==0u || m_motor_2.m_fault!=FAULT_CODE_NONE){
        RIGHT_TIM->BDTR&=~TIM_BDTR_MOE;
        if(rightCurrentTrip){
            m_motor_2.m_current_trip_count++; if(rightPhaseTrip)m_motor_2.m_phase_trip_count++; if(rightDcTrip)m_motor_2.m_dc_trip_count++;
            m_motor_2.m_last_trip_source=(rightPhaseTrip?1u:0u)|(rightDcTrip?2u:0u);
            m_motor_2.m_last_trip_phase0_counts=(int16_t)CLAMP(curR_phaA,-32768,32767); m_motor_2.m_last_trip_phase1_counts=curR_phaB; m_motor_2.m_last_trip_phase2_counts=curR_phaC;
            m_motor_2.m_last_trip_dc_counts=curR_DC; m_motor_2.m_last_trip_duty_permille=m_motor_2.m_duty_now_permille;
            motor_fault_set(&m_motor_2,FAULT_CODE_ABS_OVER_CURRENT);
        }
    }

    buzzerTimer++;
    if (buzzerFreq != 0 && (buzzerTimer / 5000) % (buzzerPattern + 1) == 0) {
        if (buzzerPrev == 0) { buzzerPrev=1; if(++buzzerIdx>(buzzerCount+2))buzzerIdx=1; }
        if (buzzerTimer % buzzerFreq == 0 && (buzzerIdx <= buzzerCount || buzzerCount == 0)) HAL_GPIO_TogglePin(BUZZER_PORT,BUZZER_PIN);
    } else if (buzzerPrev) { HAL_GPIO_WritePin(BUZZER_PORT,BUZZER_PIN,GPIO_PIN_RESET); buzzerPrev=0; }

    if (s_overrun) { m_motor_1.m_overrun_count++;m_motor_2.m_overrun_count++;foc_isr_monitor_end(focIsrStartCycles);return; }
    s_overrun=1;
    mcpwm_foc_adc_int_handler();

    /* Odom follows the validated Hall accumulator, but integer division is far
     * too expensive to execute on every 16-kHz frame. s_foc_control_div==0 here
     * means the just-finished slot was slot 2 (the no-PI slot), so odom still
     * updates at 5.33 kHz without extending either motor's regulator peak ISR. */
    if (s_foc_control_div == 0u) {
        int32_t ol = m_motor_1.m_position_counts % 9000;
        int32_t oraw = (-m_motor_2.m_position_counts) % 9000;
        if (ol < 0) ol += 9000;
        if (oraw < 0) oraw += 9000;
        odom_l = (int16_t)ol;
        odom_r = (int16_t)oraw;
    }
    if(leftBridgeWasOn && leftDriveRequest && m_motor_1.m_bridge_settle_ticks>0u) m_motor_1.m_bridge_settle_ticks--;
    if(rightBridgeWasOn && rightDriveRequest && m_motor_2.m_bridge_settle_ticks>0u) m_motor_2.m_bridge_settle_ticks--;

    /* Safety gate phase 2: only after motor_control_step has written the CCRs
     * may a previously-off bridge be armed. Existing powered bridges remain on
     * unless phase/DC protection, fault state, or drive request disabled them. */
    if(leftDriveRequest && !leftCurrentTrip && m_motor_1.m_fault==FAULT_CODE_NONE) LEFT_TIM->BDTR|=TIM_BDTR_MOE;
    if(rightDriveRequest && !rightCurrentTrip && m_motor_2.m_fault==FAULT_CODE_NONE) RIGHT_TIM->BDTR|=TIM_BDTR_MOE;
    s_overrun=0;
    foc_isr_monitor_end(focIsrStartCycles);
}



static uint8_t hall_detect_angle200(int64_t sum_s, int64_t sum_c, uint16_t n, uint16_t min_samples) {
    if (n <= min_samples) return 255u;
    int64_t best_dot = INT64_MIN;
    uint8_t best = 255u;
    for (uint16_t a = 0u; a < 200u; ++a) {
        int16_t sn, cs;
        const uint16_t ph = (uint16_t)(((uint32_t)a * 65536u) / 200u);
        foc_sin_cos_q15(ph, &sn, &cs);
        const int64_t dot = sum_s * sn + sum_c * cs;
        if (dot > best_dot) { best_dot = dot; best = (uint8_t)a; }
    }
    return best;
}

static uint8_t hall_detect_distance200(uint8_t a, uint8_t b) {
    if (a >= 200u || b >= 200u) return 200u;
    int16_t d = (int16_t)a - (int16_t)b;
    if (d < 0) d = (int16_t)-d;
    if (d > 100) d = (int16_t)(200 - d);
    return (uint8_t)d;
}

bool mcpwm_foc_detect_hall(float current, bool second, uint8_t table[8]) {
    if (!table) return false;
    mcpwm_foc_motor_t *m = mcpwm_foc_get_motor(second);
    float max_i = m->m_conf.l_current_max;
    if (max_i <= 0.0f || max_i > (float)I_MOT_MAX) max_i = (float)I_MOT_MAX;
    if (current < 0.0f) current = -current;
    if (current < 0.5f) current = 0.5f;
    if (current > max_i * 0.40f) current = max_i * 0.40f;
    if (current > 4.0f) current = 4.0f;

    int64_t sum_s[8] = {0};
    int64_t sum_c[8] = {0};
    uint16_t samples[8] = {0};
    for (uint8_t i = 0u; i < 8u; ++i) table[i] = 255u;

    /* VESC 6.00-style Hall FOC detection: ramp d-axis alignment current for
     * ~1 s, then sweep the complete electrical revolution three times forward
     * and three times reverse in one-degree, 5-ms steps. This is intentionally
     * slower than V15's coarse 2-degree alternating sweep, but gives the Hall
     * sector-center table the same semantics expected by foc_correct_hall(). */
    bool detect_ok = true;
    for (uint16_t k = 0u; k < 1000u; ++k) {
        const float align_i = current * (float)(k + 1u) / 1000.0f;
        mcpwm_foc_set_openloop_phase(align_i, 0.0f, second);
        mcpwm_foc_vesc_override_touch(second);
        HAL_Delay(1u);
        if (m->m_fault != FAULT_CODE_NONE ||
            m->m_control_mode != CONTROL_MODE_OPENLOOP_PHASE ||
            m->m_iq_target_q4 != 0 || m->m_iq_set_q4 != 0) {
            detect_ok = false; break;
        }
    }
    if (!detect_ok) {
        mcpwm_foc_release_motor(second);
        mcpwm_foc_vesc_override_clear(second);
        return false;
    }

    uint8_t forward_ref[8], reverse_ref[8];
    for (uint8_t h = 0u; h < 8u; ++h) forward_ref[h] = reverse_ref[h] = 255u;
    bool repeatable = true;
    for (uint8_t pass = 0u; pass < 6u; ++pass) {
        const bool reverse = pass >= 3u;
        int64_t pass_s[8] = {0};
        int64_t pass_c[8] = {0};
        uint16_t pass_n[8] = {0};
        for (uint16_t k = 0u; k < 360u; ++k) {
            const uint16_t deg = reverse ? (uint16_t)(359u - k) : k;
            mcpwm_foc_set_openloop_phase(current, (float)deg, second);
            mcpwm_foc_vesc_override_touch(second);
            HAL_Delay(5u);
            if (m->m_fault != FAULT_CODE_NONE ||
                m->m_control_mode != CONTROL_MODE_OPENLOOP_PHASE ||
                m->m_iq_target_q4 != 0 || m->m_iq_set_q4 != 0 ||
                !m->m_phase_override) {
                detect_ok = false; break;
            }
            /* Electrical drive phase is always the explicit open-loop override.
             * Hall is observation only and never selects the FOC phase here. */
            const uint8_t h = m->m_hall_state;
            if (h != 0u && h != 7u) {
                int16_t sn, cs;
                const uint16_t ph = (uint16_t)(((uint32_t)deg * 65536u) / 360u);
                foc_sin_cos_q15(ph, &sn, &cs);
                sum_s[h] += sn; sum_c[h] += cs;
                pass_s[h] += sn; pass_c[h] += cs;
                if (samples[h] < 0xffffu) samples[h]++;
                if (pass_n[h] < 0xffffu) pass_n[h]++;
            }
        }
        if (!detect_ok) break;
        /* Do not hide a noisy pass inside the six-pass average. Three forward
         * passes must agree with each other, and the three reverse passes must
         * agree with each other. 4/200 = 7.2 electrical degrees. Forward-vs-
         * reverse may include rotor magnetic hysteresis, but must remain within
         * 8/200 = 14.4 electrical degrees. */
        for (uint8_t h = 1u; h <= 6u; ++h) {
            const uint8_t a = hall_detect_angle200(pass_s[h], pass_c[h], pass_n[h], 20u);
            if (a >= 200u) { repeatable = false; continue; }
            uint8_t *ref = reverse ? reverse_ref : forward_ref;
            const uint8_t first_pass = reverse ? 3u : 0u;
            if (pass == first_pass) ref[h] = a;
            else if (hall_detect_distance200(a, ref[h]) > 4u) repeatable = false;
        }
    }
    mcpwm_foc_release_motor(second);
    mcpwm_foc_vesc_override_clear(second);
    if (!detect_ok || !repeatable) return false;
    for (uint8_t h = 1u; h <= 6u; ++h) {
        if (hall_detect_distance200(forward_ref[h], reverse_ref[h]) > 8u) return false;
    }

    uint8_t valid = 0u;
    for (uint8_t h = 1u; h <= 6u; ++h) {
        table[h] = hall_detect_angle200(sum_s[h], sum_c[h], samples[h], 30u);
        if (table[h] < 200u) valid++;
    }
    table[0] = 255u; table[7] = 255u;
    if (valid != 6u) return false;

    /* A valid 3-Hall sequence has six roughly 60-degree electrical sectors.
     * Reject an ambiguous/noisy result instead of saving a dangerous table. */
    uint8_t sorted[6];
    for (uint8_t i = 0u; i < 6u; ++i) sorted[i] = table[i + 1u];
    for (uint8_t i = 0u; i < 5u; ++i) {
        for (uint8_t j = (uint8_t)(i + 1u); j < 6u; ++j) {
            if (sorted[j] < sorted[i]) { uint8_t t = sorted[i]; sorted[i] = sorted[j]; sorted[j] = t; }
        }
    }
    for (uint8_t i = 0u; i < 6u; ++i) {
        const uint16_t a = sorted[i];
        const uint16_t b = (i == 5u) ? (uint16_t)sorted[0] + 200u : sorted[i + 1u];
        const uint16_t gap = b - a;
        if (gap < 18u || gap > 48u) return false;
    }

    mc_configuration c = m->m_conf;
    for (uint8_t i = 0u; i < 8u; ++i) c.foc_hall_table[i] = table[i];
    c.foc_sensor_mode = FOC_SENSOR_MODE_HALL;
    mcpwm_foc_set_configuration(&c, second);
    hall_estimator_reset(m);
    return true;
}

bool mcpwm_foc_dc_cal_done(void){return offsetcount>=2000u;}
void mcpwm_foc_get_current_offsets(int16_t *p0,int16_t *p1,int16_t *dc,bool second){if(!second){if(p0)*p0=offsetrlA;if(p1)*p1=offsetrlB;if(dc)*dc=offsetdcl;}else{if(p0)*p0=offsetrrB;if(p1)*p1=offsetrrC;if(dc)*dc=offsetdcr;}}
uint32_t mcpwm_foc_get_isr_cycles(void){return foc_isr_cycles;}uint32_t mcpwm_foc_get_isr_cycles_max(void){return foc_isr_cycles_max;}

static float q4_to_amp(int16_t q){return (float)q/(float)FOC_CURRENT_Q4_PER_A;}
static float motor_current_vesc_a(const mcpwm_foc_motor_t *m){
    const int32_t id=m->m_id_telem_q4;
    const int32_t iq=m->m_iq_telem_q4;
    const uint32_t mag2=(uint32_t)(id*id)+(uint32_t)(iq*iq);
    int32_t mag=(int32_t)foc_isqrt_u32(mag2);
    /* Monitoring API follows the same filtered DQ/Ibus snapshot used by
     * COMM_GET_VALUES. The fast current controller still uses m_id_q4/m_iq_q4. */
    if(m->m_current_in_telem_counts>0)mag=-mag;
    return (float)mag/(float)FOC_CURRENT_Q4_PER_A;
}
float mcpwm_foc_get_tot_current_motor(bool s){return motor_current_vesc_a(mcpwm_foc_get_motor_const(s));}
float mcpwm_foc_get_tot_current_in_motor(bool s){return -(float)mcpwm_foc_get_motor_const(s)->m_current_in_telem_counts/(float)A2BIT_CONV;}
float mcpwm_foc_get_motor_mechanical_rpm(bool s){
    const float pp=(float)motor_pole_pairs(s);
    return pp>0.0f ? mcpwm_foc_get_erpm_motor(s)/pp : 0.0f;
}
float mcpwm_foc_get_output_rpm(bool s){return mcpwm_foc_get_motor_mechanical_rpm(s)/motor_gear_ratio(s);}
uint16_t mcpwm_foc_get_pole_pairs(bool s){return motor_pole_pairs(s);}
float mcpwm_foc_get_gear_ratio(bool s){return motor_gear_ratio(s);}
float mcpwm_foc_get_erpm_motor(bool s){
    const mcpwm_foc_motor_t*m=mcpwm_foc_get_motor_const(s);
    /* One Hall edge is 60 electrical degrees. Derive ERPM directly from edge
     * period so low-speed values such as 50 ERPM are not quantized through an
     * integer mechanical-RPM intermediate (50 ERPM @15pp used to display 45). */
    if(m->m_hall_initialized && m->m_hall_direction!=0 &&
       m->m_hall_period>0u && m->m_hall_period<MCCONF_HALL_TIMEOUT_TICKS &&
       m->m_hall_ticks<=MCCONF_HALL_TIMEOUT_TICKS){
        const float erpm=((float)PWM_FREQ*10.0f)/(float)m->m_hall_period;
        return erpm*(float)m->m_hall_direction;
    }
    return (float)m->m_rpm*(float)motor_pole_pairs(s);
}
float mcpwm_foc_get_duty_cycle_motor(bool s){return (float)mcpwm_foc_get_motor_const(s)->m_duty_now_permille/1000.0f;}
float mcpwm_foc_get_id_motor(bool s){return q4_to_amp(mcpwm_foc_get_motor_const(s)->m_id_telem_q4);}float mcpwm_foc_get_iq_motor(bool s){return q4_to_amp(mcpwm_foc_get_motor_const(s)->m_iq_telem_q4);}
static float bus_voltage_now(void){return (float)(batVoltage*BAT_CALIB_REAL_VOLTAGE/BAT_CALIB_ADC)/100.0f;}

void mcpwm_foc_energy_update(uint32_t now_ms) {
    if (s_energy_last_ms == 0u) { s_energy_last_ms = now_ms; return; }
    uint32_t dt_ms = (uint32_t)(now_ms - s_energy_last_ms);
    if (dt_ms == 0u) return;
    s_energy_last_ms = now_ms;
    /* Do not integrate a long debugger/power-stall gap as real energy. */
    if (dt_ms > 100u) dt_ms = 100u;
    const float dt_s = (float)dt_ms * 0.001f;
    const float vin = bus_voltage_now();
    mcpwm_foc_motor_t *motors[2] = {&m_motor_1, &m_motor_2};
    for (uint8_t k = 0u; k < 2u; ++k) {
        mcpwm_foc_motor_t *m = motors[k];
        const float current_in = -(float)m->m_current_in_counts / (float)A2BIT_CONV;
        const float amp_s = current_in * dt_s;
        const float watt_s = current_in * vin * dt_s;
        if (current_in >= 0.0f) {
            m->m_amp_seconds += amp_s;
            m->m_watt_seconds += watt_s;
        } else {
            m->m_amp_seconds_charged -= amp_s;
            m->m_watt_seconds_charged -= watt_s;
        }
    }
}
float mcpwm_foc_get_vd_motor(bool s){return (float)mcpwm_foc_get_motor_const(s)->m_vd*(bus_voltage_now()/(float)MCCONF_FOC_VOLTAGE_MAX);}
float mcpwm_foc_get_vq_motor(bool s){return (float)mcpwm_foc_get_motor_const(s)->m_vq*(bus_voltage_now()/(float)MCCONF_FOC_VOLTAGE_MAX);}
float mcpwm_foc_get_phase_motor(bool s){return (float)mcpwm_foc_get_motor_const(s)->m_phase*(360.0f/65536.0f);}
mc_state mcpwm_foc_get_state_motor(bool s){return mcpwm_foc_get_motor_const(s)->m_state;}mc_fault_code mcpwm_foc_get_fault_motor(bool s){return mcpwm_foc_get_motor_const(s)->m_fault;}

void mcpwm_foc_get_values(mc_values *v,bool second){
    if (!v) return;
    memset(v, 0, sizeof(*v));
    const mcpwm_foc_motor_t *m = mcpwm_foc_get_motor_const(second);
    /* Snapshot every ISR-owned field in one very short critical section. Do
     * all floating-point conversion afterwards, so one VESC telemetry packet
     * can never mix Id/Iq/RPM/phase values from different 16-kHz frames. */
    int16_t id_q4,iq_q4,ibus_counts,rpm_i,duty_i,vd_i,vq_i;
    int32_t sum_id_q4,sum_iq_q4,sum_ibus_counts; uint16_t avg_n;
    int32_t pos_i;
    uint16_t phase_i,hall_period_i,hall_ticks_i;
    uint8_t hall_init_i;
    int8_t hall_dir_i;
    mc_fault_code fault_i;
    __disable_irq();
    id_q4=m->m_id_telem_q4; iq_q4=m->m_iq_telem_q4; ibus_counts=m->m_current_in_telem_counts;
    sum_id_q4=m->m_telem_sum_id_q4; sum_iq_q4=m->m_telem_sum_iq_q4; sum_ibus_counts=m->m_telem_sum_ibus_counts; avg_n=m->m_telem_avg_samples;
    ((mcpwm_foc_motor_t *)m)->m_telem_sum_id_q4=0; ((mcpwm_foc_motor_t *)m)->m_telem_sum_iq_q4=0;
    ((mcpwm_foc_motor_t *)m)->m_telem_sum_ibus_counts=0; ((mcpwm_foc_motor_t *)m)->m_telem_avg_samples=0u;
    rpm_i=m->m_rpm; duty_i=m->m_duty_now_permille; vd_i=m->m_vd; vq_i=m->m_vq;
    pos_i=m->m_position_counts; phase_i=m->m_phase; fault_i=m->m_fault;
    hall_init_i=m->m_hall_initialized; hall_dir_i=m->m_hall_direction;
    hall_period_i=m->m_hall_period; hall_ticks_i=m->m_hall_ticks;
    __enable_irq();
    if(avg_n>0u){
        id_q4=(int16_t)(sum_id_q4/(int32_t)avg_n);
        iq_q4=(int16_t)(sum_iq_q4/(int32_t)avg_n);
        ibus_counts=(int16_t)(sum_ibus_counts/(int32_t)avg_n);
    }

    const float vin=bus_voltage_now();
    v->v_in=vin;
    v->amp_hours=m->m_amp_seconds/3600.0f;
    v->amp_hours_charged=m->m_amp_seconds_charged/3600.0f;
    v->watt_hours=m->m_watt_seconds/3600.0f;
    v->watt_hours_charged=m->m_watt_seconds_charged/3600.0f;
    v->id=q4_to_amp(id_q4); v->iq=q4_to_amp(iq_q4);
    { uint32_t mag2=(uint32_t)((int32_t)id_q4*id_q4)+(uint32_t)((int32_t)iq_q4*iq_q4);
      int32_t mag=(int32_t)foc_isqrt_u32(mag2); if(ibus_counts>0)mag=-mag;
      v->current_motor=(float)mag/(float)FOC_CURRENT_Q4_PER_A; }
    v->current_in=-(float)ibus_counts/(float)A2BIT_CONV;
    if(hall_init_i && hall_dir_i!=0 && hall_period_i>0u && hall_period_i<MCCONF_HALL_TIMEOUT_TICKS && hall_ticks_i<=MCCONF_HALL_TIMEOUT_TICKS)
        v->rpm=((float)PWM_FREQ*10.0f/(float)hall_period_i)*(float)hall_dir_i;
    else v->rpm=(float)rpm_i*(float)motor_pole_pairs(second);
    v->tachometer=pos_i;
    v->tachometer_abs=(pos_i==INT32_MIN)?INT32_MAX:(pos_i<0?-pos_i:pos_i);
    v->position=(float)phase_i*(360.0f/65536.0f);
    v->duty_now=(float)duty_i/1000.0f; v->fault_code=fault_i; v->vesc_id=second?2:1;
    v->vd=(float)vd_i*(vin/(float)MCCONF_FOC_VOLTAGE_MAX);
    v->vq=(float)vq_i*(vin/(float)MCCONF_FOC_VOLTAGE_MAX);
}
