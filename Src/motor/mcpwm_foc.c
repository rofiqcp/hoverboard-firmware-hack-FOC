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
static int16_t pwm_margin = 110;
static int16_t curDC_max  = (I_DC_MAX * A2BIT_CONV);
static int16_t curPha_max = (I_MOT_MAX * A2BIT_CONV);

int16_t odom_l = 0, odom_r = 0;
static volatile uint8_t s_overrun = 0;
static volatile uint16_t s_vesc_override_ticks[2] = {0u, 0u};
static uint8_t s_foc_control_div = 0u;

/* VESC FOC Hall table: 0..199 = 0..360 electrical degrees, 255 = invalid.
 * These defaults reproduce the previously proven hard-coded sector centers. */
static const uint8_t s_default_foc_hall_table[8] = {255u, 83u, 17u, 50u, 150u, 117u, 183u, 255u};

static uint16_t motor_pole_pairs(bool second) {
    return second ? MCCONF_POLE_PAIRS_RIGHT : MCCONF_POLE_PAIRS_LEFT;
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
            int64_t q16 = ((int64_t)PWM_FREQ * 10LL * 65536LL) /
                          ((int64_t)m->m_hall_period * (int64_t)pp);
            if (m->m_hall_direction < 0) q16 = -q16;
            if (q16 > INT32_MAX) q16 = INT32_MAX;
            if (q16 < INT32_MIN) q16 = INT32_MIN;
            return (int32_t)q16;
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

static int16_t modulo_i16(int16_t m, int16_t classes) {
    return (int16_t)(((m % classes) + classes) % classes);
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
    c->l_in_current_max = MCCONF_L_IN_CURRENT_MAX;
    c->l_in_current_min = MCCONF_L_IN_CURRENT_MIN;
    c->l_max_erpm = MCCONF_L_MAX_ERPM;
    c->l_min_erpm = MCCONF_L_MIN_ERPM;
    /* Expose VESC configuration in physical units. The ISR remains fixed-point:
     * Kp ~= 0.800 V/A and Ki ~= 266.7 V/(A*s) at the 5.333-kHz control cadence. */
    c->foc_current_kp = 0.80013f;
    c->foc_current_ki = 266.710f;
    c->foc_current_filter_const = (float)MCCONF_FOC_CURRENT_FILTER_Q16 / 65536.0f;
    c->foc_openloop_rpm = (float)MCCONF_OPENLOOP_RPM_DEFAULT;
    c->foc_hall_interp_erpm = 0.0f;
    c->s_pid_ramp_erpms_s = (float)MCCONF_SPEED_RAMP_ERPMS_S;
    c->s_pid_min_erpm = (float)MCCONF_SPEED_RELEASE_ERPM;
    c->s_pid_allow_braking = true;
    c->s_pid_kp = (float)MCCONF_SPEED_KP_Q11 / 1000.0f;
    c->s_pid_ki = (float)MCCONF_SPEED_KI_Q16 / 1000.0f;
    c->s_pid_kd = (float)MCCONF_SPEED_KD_Q11 / 1000.0f;
    c->p_pid_kp = (float)MCCONF_POSITION_KP_Q11 / 1000.0f;
    c->p_pid_ki = (float)MCCONF_POSITION_KI_Q16 / 1000.0f;
    c->p_pid_kd = (float)MCCONF_POSITION_KD_Q11 / 1000.0f;
    c->p_pid_ang_div = 1.0f;
    c->si_motor_poles = (uint8_t)(2u * (second ? MCCONF_POLE_PAIRS_RIGHT : MCCONF_POLE_PAIRS_LEFT));
    for (int i=0;i<8;i++) c->foc_hall_table[i] = (int8_t)s_default_foc_hall_table[i];
}

void mcpwm_foc_get_default_configuration(mc_configuration *conf, bool second) {
    if (conf) conf_defaults(conf, second);
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
    m->m_position_min_counts=INT32_MIN; m->m_position_max_counts=INT32_MAX;
    m->m_current_limit_q4=MCCONF_MOTOR_CURRENT_MAX_Q4;
    {
        const uint16_t pp = second ? MCCONF_POLE_PAIRS_RIGHT : MCCONF_POLE_PAIRS_LEFT;
        m->m_speed_ramp_rpm_s = (uint16_t)(MCCONF_SPEED_RAMP_ERPMS_S / pp);
        if (m->m_speed_ramp_rpm_s == 0u) m->m_speed_ramp_rpm_s = 1u;
        m->m_speed_release_rpm = (uint16_t)(MCCONF_SPEED_RELEASE_ERPM / pp);
        if (m->m_speed_release_rpm == 0u) m->m_speed_release_rpm = 1u;
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
    foc_isr_cycles = foc_isr_cycles_max = 0;
    s_overrun = 0;
    s_vesc_override_ticks[0] = s_vesc_override_ticks[1] = 0u;
    s_foc_control_div = 0u;
}

mcpwm_foc_motor_t *mcpwm_foc_get_motor(bool second) { return second ? &m_motor_2 : &m_motor_1; }
const mcpwm_foc_motor_t *mcpwm_foc_get_motor_const(bool second) { return second ? &m_motor_2 : &m_motor_1; }

static int16_t amp_to_q4(const mcpwm_foc_motor_t *m, float current);

void mcpwm_foc_set_configuration(const mc_configuration *conf, bool second) {
    if (!conf) return;
    mcpwm_foc_motor_t *m = mcpwm_foc_get_motor(second);
    mc_configuration next = *conf;
    /* Never let a malformed VESC Tool/EEPROM Hall table become the live FOC
     * angle source. Preserve the last known-good table while still accepting
     * the other configuration fields. */
    if (!hall_table_runtime_sane(next.foc_hall_table)) {
        for (uint8_t h=0u;h<8u;++h) next.foc_hall_table[h]=m->m_conf.foc_hall_table[h];
    }
    m->m_conf = next;

    /* VESC configuration uses electrical units. Convert once outside the ISR
     * and keep the actual speed-loop ramp integer/fixed-point. */
    const float pp = (float)(second ? MCCONF_POLE_PAIRS_RIGHT : MCCONF_POLE_PAIRS_LEFT);
    float ramp_mech = conf->s_pid_ramp_erpms_s / pp;
    if (ramp_mech < 1.0f) ramp_mech = 1.0f;
    if (ramp_mech > 5000.0f) ramp_mech = 5000.0f;
    m->m_speed_ramp_rpm_s = (uint16_t)(ramp_mech + 0.5f);

    float release_mech = conf->s_pid_min_erpm / pp;
    if (release_mech < 1.0f) release_mech = 1.0f;
    if (release_mech > 100.0f) release_mech = 100.0f;
    m->m_speed_release_rpm = (uint16_t)(release_mech + 0.5f);
    int32_t kpc=(int32_t)(conf->foc_current_kp*1536.0f+0.5f);
    int32_t kic=(int32_t)(conf->foc_current_ki*4.608f+0.5f);
    kpc=CLAMP(kpc,0,65535); kic=CLAMP(kic,0,65535);
    m->m_kpq_q11=m->m_kpd_q11=(uint16_t)kpc;
    m->m_kiq_q16=m->m_kid_q16=(uint16_t)kic;
    m->m_kps_q11=(uint16_t)CLAMP((int32_t)(conf->s_pid_kp*1000.0f+0.5f),0,65535);
    m->m_kis_q16=(uint16_t)CLAMP((int32_t)(conf->s_pid_ki*1000.0f+0.5f),0,65535);
    m->m_kds_q11=(uint16_t)CLAMP((int32_t)(conf->s_pid_kd*1000.0f+0.5f),0,65535);
    m->m_kpp_q11=(uint16_t)CLAMP((int32_t)(conf->p_pid_kp*1000.0f+0.5f),0,65535);
    m->m_kip_q16=(uint16_t)CLAMP((int32_t)(conf->p_pid_ki*1000.0f+0.5f),0,65535);
    m->m_kdp_q11=(uint16_t)CLAMP((int32_t)(conf->p_pid_kd*1000.0f+0.5f),0,65535);
    m->m_current_limit_q4=amp_to_q4(m,conf->l_current_max);
}
void mcpwm_foc_sync_tuning_to_conf(bool second) {
    mcpwm_foc_motor_t *m=mcpwm_foc_get_motor(second);
    m->m_conf.foc_current_kp=(float)m->m_kpq_q11/1536.0f;
    m->m_conf.foc_current_ki=(float)m->m_kiq_q16/4.608f;
    m->m_conf.s_pid_kp=(float)m->m_kps_q11/1000.0f; m->m_conf.s_pid_ki=(float)m->m_kis_q16/1000.0f; m->m_conf.s_pid_kd=(float)m->m_kds_q11/1000.0f;
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
    m->m_position_integrator=0;m->m_position_prev_error=0;m->m_position_sat_hold=0;
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
    if (step_q16 < 1) step_q16 = 1;
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
        reset_current_pi(m);
        m->m_speed_integrator=0; m->m_speed_prev_error=0; m->m_speed_sat_hold=0; reset_position_pid(m);
        m->m_iq_set_ramp_q16 = (int32_t)m->m_iq_q4 << 16;
        m->m_iq_set_q4 = m->m_iq_q4;
        m->m_iq_target_q4 = m->m_iq_q4;
        m->m_control_mode = mode;
    }
}

void mcpwm_foc_set_duty(float duty, bool second) {
    mcpwm_foc_motor_t *m=mcpwm_foc_get_motor(second);
    if (duty > 1.0f) duty = 1.0f;
    if (duty < -1.0f) duty = -1.0f;
    set_control_mode(m, CONTROL_MODE_DUTY);
    m->m_duty_set_permille=(int16_t)(duty*1000.0f);
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
    /* Stock VESC COMM_SET_POS is a single-turn rotor electrical-angle command.
     * The long-range Hall-count position requested by this project remains the
     * separate mode-5/PSETL/PSETR API. With Hall sensing at standstill the
     * physical position resolution is one 60-degree electrical sector, so map
     * the requested VESC angle to the nearest equivalent Hall-count target. */
    mcpwm_foc_motor_t *m=mcpwm_foc_get_motor(second);
    while(position_deg>=360.0f)position_deg-=360.0f;
    while(position_deg<0.0f)position_deg+=360.0f;
    float now=mcpwm_foc_get_phase_motor(second);
    float diff=position_deg-now;
    while(diff>180.0f)diff-=360.0f;
    while(diff<-180.0f)diff+=360.0f;
    int32_t delta=(int32_t)(diff>=0.0f?(diff/60.0f+0.5f):(diff/60.0f-0.5f));
    int64_t target=(int64_t)m->m_position_counts+delta;
    if(target>INT32_MAX)target=INT32_MAX;
    if(target<INT32_MIN)target=INT32_MIN;
    mcpwm_foc_set_position_counts((int32_t)target,second);
}
void mcpwm_foc_set_position_counts(int32_t pc,bool second){
    mcpwm_foc_motor_t*m=mcpwm_foc_get_motor(second);if(pc<m->m_position_min_counts)pc=m->m_position_min_counts;if(pc>m->m_position_max_counts)pc=m->m_position_max_counts;
    set_control_mode(m,CONTROL_MODE_POS);m->m_position_target_counts=pc;
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
    set_control_mode(m, CONTROL_MODE_CURRENT);
    m->m_iq_target_q4=amp_to_q4(m,current);
    m->m_id_set_q4=0;
}
void mcpwm_foc_set_brake_current(float current, bool second) {
    mcpwm_foc_motor_t *m=mcpwm_foc_get_motor(second);
    const bool entering = m->m_control_mode != CONTROL_MODE_CURRENT_BRAKE;
    set_control_mode(m, CONTROL_MODE_CURRENT_BRAKE);
    int16_t q=amp_to_q4(m,current<0?-current:current);

    /* COMM_SET_CURRENT_BRAKE is a stop request, not a reverse-speed command.
     * Latch the rotor direction once when brake mode is entered. When measured
     * speed reaches/crosses the deadband, clear the latch and command zero Iq.
     * Repeated brake packets then stay at zero even if Hall quantization or
     * mechanical rebound briefly reports the opposite sign. */
    if (entering) {
        if (m->m_rpm > MCCONF_TRQ_STOP_RPM_DEADBAND) m->m_brake_direction=1;
        else if (m->m_rpm < -MCCONF_TRQ_STOP_RPM_DEADBAND) m->m_brake_direction=-1;
        else m->m_brake_direction=0;
    }
    if (m->m_brake_direction > 0) {
        if (m->m_rpm <= MCCONF_TRQ_STOP_RPM_DEADBAND) {
            m->m_brake_direction=0; m->m_iq_target_q4=0;
        } else {
            m->m_iq_target_q4=-q;
        }
    } else if (m->m_brake_direction < 0) {
        if (m->m_rpm >= -MCCONF_TRQ_STOP_RPM_DEADBAND) {
            m->m_brake_direction=0; m->m_iq_target_q4=0;
        } else {
            m->m_iq_target_q4=q;
        }
    } else {
        m->m_iq_target_q4=0;
    }
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
    mcpwm_foc_motor_t *m=mcpwm_foc_get_motor(second); set_control_mode(m, CONTROL_MODE_NONE);
    m->m_iq_set_q4=0; m->m_iq_target_q4=0; m->m_iq_set_ramp_q16=0;
    m->m_id_set_q4=0; m->m_openloop_id_target_q4=0;
    m->m_openloop_id_ramp_q16=0;
    m->m_speed_target_rpm=0; m->m_speed_target_rpm_q16=0; m->m_speed_set_rpm=0; m->m_speed_set_ramp_q16=0;
    m->m_speed_integrator=0; m->m_speed_prev_error=0; m->m_speed_sat_hold=0; reset_position_pid(m);
    m->m_state=MC_STATE_OFF;
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
            set_control_mode(m, CONTROL_MODE_DUTY);
            m->m_duty_set_permille = (int16_t)CLAMP(command,-1000,1000);
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
                if (period_outlier) {
                    /* Period comparison is meaningful only while continuing in
                     * the same direction. A legitimate reversal deliberately
                     * resets the Hall period history and must not inherit the old
                     * direction's short/long-period rejection threshold.
                     * Keep re-evaluating a persistent candidate as m_hall_ticks
                     * grows, but count this raw edge only once. The old code
                     * incremented this counter on every 16-kHz ISR until the
                     * hold time elapsed, turning one chatter/outlier edge into
                     * hundreds of reported invalid transitions. */
                    if (m->m_hall_reject_counted_state != h) {
                        /* Period rejection is a timing/filter event, not an
                         * impossible Hall electrical sequence. Track it
                         * separately so hall_bad means an actual mapping/state
                         * error. */
                        m->m_hall_period_reject_count++;
                        m->m_hall_last_reject_reason = 1u;
                        m->m_hall_last_reject_from = 0xffu;
                        for(uint8_t rh=1u;rh<=6u;++rh) if(hall_table_angle(m,rh)==m->m_hall_pos_prev){m->m_hall_last_reject_from=rh;break;}
                        m->m_hall_last_reject_to = h;
                        m->m_hall_reject_counted_state = h;
                    }
                } else {
                    m->m_hall_reject_counted_state = 0xffu;
                    const bool direction_reset =
                        (m->m_hall_direction == 0 || m->m_hall_direction != dir ||
                         m->m_hall_period_hist[0] == MCCONF_HALL_TIMEOUT_TICKS);
                    if (direction_reset) {
                        for (int i = 0; i < 4; i++) m->m_hall_period_hist[i] = period;
                        m->m_hall_hist_pos = 0u;
                        m->m_hall_direction_stable_edges = 0u;
                    } else {
                        m->m_hall_period_hist[m->m_hall_hist_pos++ & 3u] = period;
                    }
                    uint32_t sum = 0u;
                    for (int i = 0; i < 4; i++) sum += m->m_hall_period_hist[i];
                    m->m_hall_period = (uint16_t)(sum / 4u);
                    if (!m->m_hall_period) m->m_hall_period = 1u;
                    m->m_hall_ticks = 0u;
                    m->m_hall_direction = dir;
                    if (m->m_hall_direction_stable_edges < 0xffu) m->m_hall_direction_stable_edges++;
                    if (dir > 0 && m->m_position_counts < INT32_MAX) m->m_position_counts++;
                    else if (dir < 0 && m->m_position_counts > INT32_MIN) m->m_position_counts--;
                    /* VESC Hall tables contain SECTOR CENTERS. At an edge the
                     * rotor is halfway between old and new centers. Starting
                     * interpolation at the new center (old V15 behavior) adds
                     * an erroneous +30 electrical degrees immediately, then
                     * another sector during interpolation. */
                    m->m_hall_pos = hall_midpoint200(previous_center, ad);
                    m->m_hall_pos_prev = angle;
                }
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
    } else {
        int32_t rpm = 10667 / (int32_t)m->m_hall_period;
        if (rpm > MCCONF_MOTOR_RPM_MAX) rpm = MCCONF_MOTOR_RPM_MAX;
        m->m_rpm = (int16_t)(rpm * m->m_hall_direction);
    }

    const int16_t abs_rpm = (int16_t)abs(m->m_rpm);
    if (abs_rpm >= MCCONF_HALL_INTERP_ON_RPM) m->m_hall_interp_active = 1u;
    else if (abs_rpm <= MCCONF_HALL_INTERP_OFF_RPM) m->m_hall_interp_active = 0u;

    uint16_t desired = m->m_phase_hall;
    if (valid) {
        if (m->m_hall_interp_active && m->m_hall_direction != 0 &&
            m->m_hall_period > 0u && m->m_hall_period < MCCONF_HALL_TIMEOUT_TICKS) {
            const uint16_t edge_phase = hall_angle200_to_phase(m->m_hall_pos);
            const uint32_t sector = 65536u / 6u;
            uint32_t ticks = m->m_hall_ticks;
            if (ticks > m->m_hall_period) ticks = m->m_hall_period;
            const uint32_t frac = (ticks * sector) / m->m_hall_period;
            desired = (uint16_t)(m->m_hall_direction > 0 ?
                                (uint32_t)edge_phase + frac :
                                (uint32_t)edge_phase - frac);
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
    uint32_t max_step = 1u;
    if (m->m_hall_period > 0u && m->m_hall_period < MCCONF_HALL_TIMEOUT_TICKS) {
        max_step = ((65536u / 6u) * 3u) / (2u * m->m_hall_period);
        if (max_step == 0u) max_step = 1u;
    }
    {
        const uint32_t min_erpm = (uint32_t)MCCONF_HALL_INTERP_ON_RPM * (uint32_t)motor_pole_pairs(second);
        uint32_t min_step = (uint32_t)(((uint64_t)min_erpm * 65536ULL) / (60ULL * PWM_FREQ));
        if (min_step == 0u) min_step = 1u;
        if (max_step < min_step) max_step = min_step;
    }
    if (max_step > 32767u) max_step = 32767u;
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
    if(id_step_q16<1)id_step_q16=1;
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
    if(speed_step<1)speed_step=1;
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
    /* VESC foc_run_pid_control_pos architecture: position PID output is
     * normalized to [-1,1] and then multiplied by the configured motor-current
     * limit. This Hall port stores position in sector counts; one count is
     * 360/(6*pole_pairs) mechanical degrees. Gains are stored as gain*1000. */
    int64_t ec64=(int64_t)m->m_position_target_counts-(int64_t)m->m_position_counts;
    if(ec64>32767)ec64=32767; else if(ec64<-32768)ec64=-32768;
    const int32_t ec=(int32_t)ec64;
    const int32_t pp=(int32_t)motor_pole_pairs(second);
    const int32_t mdeg_per_count=360000/(6*pp);
    const int32_t error_mdeg=ec*mdeg_per_count;
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

    int32_t d_q15=0;
    if(m->m_kdp_q11!=0u){
        int32_t de=ec-(int32_t)m->m_position_prev_error;
        int64_t d64=(int64_t)de*mdeg_per_count*(int32_t)m->m_kdp_q11*32768LL*
                    (int32_t)PWM_FREQ;
        d64/=(1000000LL*(int32_t)MCCONF_FOC_CONTROL_DIV);
        if(d64>32768)d64=32768; else if(d64<-32768)d64=-32768;
        d_q15=(int32_t)d64;
    }
    m->m_position_prev_error=(int16_t)ec;
    int32_t out_q15=p_q15+(m->m_position_integrator>>16)+d_q15;
    out_q15=CLAMP(out_q15,-32768,32768);

    int32_t iq_cmd_q4=(int32_t)(((int64_t)out_q15*limit_q4)/32768LL);
    const int32_t pos_lim_q4=((int32_t)FOC_CURRENT_Q4_PER_A*
                              (int32_t)MCCONF_POSITION_CURRENT_MAX_MA)/1000;
    iq_cmd_q4=CLAMP(iq_cmd_q4,-pos_lim_q4,pos_lim_q4);

    /* Hall position is quantized to sector edges. Using instantaneous Hall RPM
     * as damping is unstable near an edge because one accepted reversal can
     * imply hundreds of RPM for a few samples. Instead, remember the validated
     * position-error direction. On first reaching the target sector, keep a
     * small bounded current briefly to move away from the boundary toward the
     * sector interior, then command exactly zero current. */
    if(ec>0){
        m->m_position_drive_direction=1;
        m->m_position_settle_ticks=0;
    }else if(ec<0){
        m->m_position_drive_direction=-1;
        m->m_position_settle_ticks=0;
    }else if(m->m_position_drive_direction!=0){
        const uint32_t settle_max=((uint32_t)MCCONF_POSITION_SETTLE_MS*
                                  (uint32_t)PWM_FREQ)/
                                  (1000u*(uint32_t)MCCONF_FOC_CONTROL_DIV);
        if(m->m_position_settle_ticks<settle_max){
            const int32_t settle_q4=((int32_t)FOC_CURRENT_Q4_PER_A*
                                     (int32_t)MCCONF_POSITION_SETTLE_CURRENT_MA)/1000;
            iq_cmd_q4=(int32_t)m->m_position_drive_direction*settle_q4;
            m->m_position_settle_ticks++;
        }else{
            iq_cmd_q4=0;
            m->m_position_drive_direction=0;
        }
    }else{
        iq_cmd_q4=0;
    }
    return (int16_t)iq_cmd_q4;
}

static int16_t speed_pid_iq_target_step(mcpwm_foc_motor_t *m, bool second) {
    /* Match vedderb/bldc foc_run_pid_control_speed architecture: speed PID
     * produces an Iq/current request, then the inner FOC current PI produces Vq.
     * Upstream scales speed PID by 1/20 and clamps it to +/- current limit.
     * Keep that behavior in integer math: gains are stored as gain*1000, speed
     * error is ERPM Q16, and m_speed_integrator is Iq(q4) Q16. */
    const int32_t pp = (int32_t)motor_pole_pairs(second);
    const int32_t limit_q4 = m->m_current_limit_q4 > 0 ?
                             m->m_current_limit_q4 : MCCONF_MOTOR_CURRENT_MAX_Q4;
    int64_t target64 = (int64_t)m->m_speed_set_ramp_q16 * pp;
    int64_t measured64 = (int64_t)measured_mech_rpm_q16(m, second) * pp;
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

    const int64_t p_num = (int64_t)error_q16 * (int32_t)m->m_kps_q11 * limit_q4;
    int32_t p_q4 = (int32_t)(p_num / (65536LL * 1000LL * 20LL));

    const int64_t i_step =
        ((int64_t)error_q16 * (int32_t)m->m_kis_q16 * limit_q4 *
         (int32_t)MCCONF_FOC_CONTROL_DIV) /
        (1000LL * 20LL * (int32_t)PWM_FREQ);
    const int64_t i_lim = (int64_t)limit_q4 << 16;
    int64_t i_candidate = (int64_t)m->m_speed_integrator + i_step;
    if (i_candidate > i_lim) i_candidate = i_lim;
    if (i_candidate < -i_lim) i_candidate = -i_lim;

    int32_t d_q4 = 0;
    if (m->m_kds_q11 != 0u) {
        const int64_t de_q16 = (int64_t)error_q16 - m->m_speed_prev_error;
        const int64_t d_num = de_q16 * (int32_t)PWM_FREQ *
                              (int32_t)m->m_kds_q11 * limit_q4;
        d_q4 = (int32_t)(d_num /
               (65536LL * (int32_t)MCCONF_FOC_CONTROL_DIV * 1000LL * 20LL));
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
    return (int16_t)CLAMP(out_q4, -limit_q4, limit_q4);
}

static int16_t phase_current_counts_to_q4(int16_t counts) {
    int32_t q4=(int32_t)counts<<4;
    /* Exact generated input saturation before Clarke/Park. */
    if(q4>27200)q4=27200; else if(q4<-27200)q4=-27200;
    return (int16_t)q4;
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
    m->m_phase=(m->m_phase_override && (m->m_control_mode==CONTROL_MODE_OPENLOOP || m->m_control_mode==CONTROL_MODE_OPENLOOP_PHASE)) ?
        m->m_phase_openloop : m->m_phase_hall;

    const int16_t i0_q4=phase_current_counts_to_q4(i0_counts);
    const int16_t i1_q4=phase_current_counts_to_q4(i1_counts);
    foc_ab_t ab; if(second)foc_clarke_bc_q4(i0_q4,i1_q4,&ab); else foc_clarke_ab_q4(i0_q4,i1_q4,&ab);
    m->m_i_alpha_q4=ab.alpha; m->m_i_beta_q4=ab.beta;
    foc_dq_t raw, filt; foc_park_q4(&ab,m->m_phase,&raw);
    foc_lpf2_fixed_t lf={{m->m_current_lpf_q16[0],m->m_current_lpf_q16[1]}};
    foc_lpf2_run(&lf,MCCONF_FOC_CURRENT_FILTER_Q16,&raw,&filt);
    m->m_current_lpf_q16[0]=lf.state_q16[0];m->m_current_lpf_q16[1]=lf.state_q16[1];
    m->m_iq_q4=filt.q; m->m_id_q4=filt.d; m->m_current_in_counts=idc_counts;

    foc_dq_t v={m->m_vd,m->m_vq};
    const bool source_enabled = (enable != 0u) || mcpwm_foc_vesc_override_active(second);
    if (!source_enabled || m->m_fault!=FAULT_CODE_NONE || m->m_control_mode==CONTROL_MODE_NONE) {
        m->m_state=MC_STATE_OFF; reset_current_pi(m); m->m_speed_integrator=0; m->m_speed_prev_error=0; m->m_speed_sat_hold=0; reset_position_pid(m);
        m->m_iq_set_q4=0; m->m_iq_target_q4=0; m->m_iq_set_ramp_q16=0;
        m->m_id_set_q4=0; m->m_openloop_id_target_q4=0; m->m_openloop_id_ramp_q16=0;
        /* With the bridge undriven the phase-current ADC samples are not a
         * meaningful motor current. VESC 6.00 explicitly reports zero current
         * in this state; clearing the LPF also prevents stale 30-40 A dq values
         * from leaking into Realtime Data immediately after Hall detection. */
        m->m_i_alpha_q4=0; m->m_i_beta_q4=0; m->m_id_q4=0; m->m_iq_q4=0;
        m->m_current_in_counts=0;
        m->m_current_lpf_q16[0]=0; m->m_current_lpf_q16[1]=0;
        v.q=0;v.d=0;
    } else {
        m->m_state=MC_STATE_RUNNING;
        if (m->m_control_mode==CONTROL_MODE_DUTY) {
            /* Mode 1 was already proven good; retain direct-voltage behavior.
             * Mode-2 zero speed also lands here with duty=0, giving Vd=Vq=0. */
            v.q=(int16_t)(((int32_t)m->m_duty_set_permille*MCCONF_FOC_VOLTAGE_MAX)/1000); v.d=0;
        } else if (control_update) {
            /* The proven generated controller updates its regulators once every
             * three 16-kHz ADC frames (~5.333 kHz). */
            const int16_t v_closed=MCCONF_FOC_CLOSED_LOOP_VOLTAGE_MAX;

            if (m->m_control_mode==CONTROL_MODE_SPEED) {
                speed_setpoint_slew_step(m);
                const int32_t release_q16 = (int32_t)m->m_speed_release_rpm << 16;
                const int32_t set_abs_q16 = m->m_speed_set_ramp_q16 < 0 ?
                                             -m->m_speed_set_ramp_q16 : m->m_speed_set_ramp_q16;
                const bool stop_reached =
                    (m->m_speed_target_rpm_q16 == 0 && set_abs_q16 <= release_q16);
                if (stop_reached) {
                    /* VESC-style low-speed release: reset regulator state and
                     * make the bridge high impedance. Never hold 0 rpm against
                     * Hall chatter. */
                    m->m_speed_set_rpm = 0;
                    m->m_speed_set_ramp_q16 = 0;
                    m->m_speed_integrator = 0;
                    m->m_speed_sat_hold = 0;
                    reset_current_pi(m);
                    mcpwm_foc_release_motor(second);
                    v.q = 0;
                    v.d = 0;
                    goto control_done;
                }
            }

            m->m_id_set_q4 = (m->m_control_mode==CONTROL_MODE_OPENLOOP ||
                              m->m_control_mode==CONTROL_MODE_OPENLOOP_PHASE) ?
                              m->m_id_set_q4 : 0;
            const int16_t ed=(int16_t)CLAMP((int32_t)m->m_id_set_q4-m->m_id_q4,-32768,32767);
            v.d=pi_run_state(ed,m->m_kpd_q11,m->m_kid_q16,
                             v_closed,-v_closed,&m->m_id_integrator,&m->m_id_sat_hold);
            int16_t q_lim=voltage_circle_q_limit(v.d,v_closed);
            if (m->m_control_mode==CONTROL_MODE_SPEED && m->m_speed_target_rpm_q16==0 &&
                q_lim > MCCONF_SPEED_STOP_VOLTAGE_MAX) {
                q_lim = MCCONF_SPEED_STOP_VOLTAGE_MAX;
            }

            if (m->m_control_mode==CONTROL_MODE_SPEED) {
                /* VESC speed PID output is the active Iq request. Do not pass it
                 * through the slow command-current slew used by COMM_SET_CURRENT:
                 * that extra lag makes braking torque arrive after the speed has
                 * already crossed the target and creates a low-speed limit cycle. */
                m->m_iq_target_q4 = speed_pid_iq_target_step(m, second);
                m->m_iq_set_q4 = m->m_iq_target_q4;
                m->m_iq_set_ramp_q16 = (int32_t)m->m_iq_set_q4 << 16;
                const int16_t eq=(int16_t)CLAMP((int32_t)m->m_iq_set_q4-m->m_iq_q4,-32768,32767);
                v.q=pi_run_state(eq,m->m_kpq_q11,m->m_kiq_q16,
                                 q_lim,(int16_t)-q_lim,
                                 &m->m_iq_integrator,&m->m_iq_sat_hold);
            } else {
                if(m->m_control_mode==CONTROL_MODE_POS){
                    m->m_iq_target_q4=position_pid_iq_target_step(m,second);
                    /* VESC position PID writes Iq directly. The dedicated
                     * position current cap above makes a slower command slew
                     * unnecessary and avoids phase lag in velocity damping. */
                    m->m_iq_set_q4=m->m_iq_target_q4;
                    m->m_iq_set_ramp_q16=(int32_t)m->m_iq_set_q4<<16;
                } else if (m->m_control_mode==CONTROL_MODE_CURRENT ||
                    m->m_control_mode==CONTROL_MODE_CURRENT_BRAKE) {
                    iq_setpoint_slew_step(m);
                } else {
                    m->m_iq_target_q4=0;
                    m->m_iq_set_q4=0;
                    m->m_iq_set_ramp_q16=0;
                }
                const int16_t eq=(int16_t)CLAMP((int32_t)m->m_iq_set_q4-m->m_iq_q4,-32768,32767);
                v.q=pi_run_state(eq,m->m_kpq_q11,m->m_kiq_q16,
                                 q_lim,(int16_t)-q_lim,
                                 &m->m_iq_integrator,&m->m_iq_sat_hold);
            }
            /* Backup only; q_lim already enforces the voltage circle before
             * the Iq PI, so its anti-windup sees the real available headroom. */
            foc_vector_limit(&v,v_closed);
        }
    }
control_done:
    m->m_vd=v.d; m->m_vq=v.q;
    foc_abc_t pwm; foc_centered_svpwm(&v,m->m_phase,&pwm);
    m->m_pwm_a=pwm.a; m->m_pwm_b=pwm.b; m->m_pwm_c=pwm.c;
    int32_t maxabs=abs(pwm.a);if(abs(pwm.b)>maxabs)maxabs=abs(pwm.b);if(abs(pwm.c)>maxabs)maxabs=abs(pwm.c);
    m->m_duty_now_permille=(int16_t)((m->m_vq < 0) ? -CLAMP(maxabs,0,1000) : CLAMP(maxabs,0,1000));
    m->m_ccr_a=(uint16_t)CLAMP((int32_t)pwm.a+pwm_res/2,pwm_margin,pwm_res-pwm_margin);
    m->m_ccr_b=(uint16_t)CLAMP((int32_t)pwm.b+pwm_res/2,pwm_margin,pwm_res-pwm_margin);
    m->m_ccr_c=(uint16_t)CLAMP((int32_t)pwm.c+pwm_res/2,pwm_margin,pwm_res-pwm_margin);
    m->m_isr_count++;
}

void mcpwm_foc_adc_int_handler(void) {
    /* Source arbitration: direct VESC packets own each motor independently for
     * MCCONF_VESC_TIMEOUT_MS. While owned, the legacy dual command stream must
     * not overwrite the VESC setpoint on the next 16 kHz ISR. */
    if (s_vesc_override_ticks[0] != 0u) {
        if (--s_vesc_override_ticks[0] == 0u) mcpwm_foc_release_motor(false);
    } else {
        mcpwm_foc_set_mode_command(ctrlModReq,(int16_t)pwml,motorRunReq!=0u,svpwmOpenloopRpm,false);
    }
    if (s_vesc_override_ticks[1] != 0u) {
        if (--s_vesc_override_ticks[1] == 0u) mcpwm_foc_release_motor(true);
    } else {
        mcpwm_foc_set_mode_command(ctrlModReq,(int16_t)pwmr,motorRunReq!=0u,svpwmOpenloopRpm,true);
    }
    const bool control_update=(s_foc_control_div==0u);
    if(++s_foc_control_div>=MCCONF_FOC_CONTROL_DIV)s_foc_control_div=0u;
    motor_control_step(&m_motor_1,false,curL_phaA,curL_phaB,curL_DC,control_update);
    motor_control_step(&m_motor_2,true,curR_phaB,curR_phaC,curR_DC,control_update);
    foc_iqL_q4=m_motor_1.m_iq_q4;foc_idL_q4=m_motor_1.m_id_q4;
    foc_iqR_q4=m_motor_2.m_iq_q4;foc_idR_q4=m_motor_2.m_id_q4;
    LEFT_TIM->LEFT_TIM_U=m_motor_1.m_ccr_a;LEFT_TIM->LEFT_TIM_V=m_motor_1.m_ccr_b;LEFT_TIM->LEFT_TIM_W=m_motor_1.m_ccr_c;
    RIGHT_TIM->RIGHT_TIM_U=m_motor_2.m_ccr_a;RIGHT_TIM->RIGHT_TIM_V=m_motor_2.m_ccr_b;RIGHT_TIM->RIGHT_TIM_W=m_motor_2.m_ccr_c;
}

void mcpwm_foc_vesc_override_touch(bool second) {
    const uint32_t ticks = ((uint32_t)PWM_FREQ * MCCONF_VESC_TIMEOUT_MS) / 1000u;
    s_vesc_override_ticks[second ? 1u : 0u] = (uint16_t)(ticks > 0xffffu ? 0xffffu : ticks);
}
bool mcpwm_foc_vesc_override_active(bool second) { return s_vesc_override_ticks[second ? 1u : 0u] != 0u; }
bool mcpwm_foc_vesc_override_active_any(void) { return (s_vesc_override_ticks[0] != 0u) || (s_vesc_override_ticks[1] != 0u); }
void mcpwm_foc_vesc_override_clear(bool second) { s_vesc_override_ticks[second ? 1u : 0u] = 0u; }

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
        foc_isr_monitor_end(focIsrStartCycles);
        return;
    }

    if (buzzerTimer % 1000 == 0) {
        filtLowPass32(adc_buffer.batt1, BAT_FILT_COEF, &batVoltageFixdt);
        batVoltage = (int16_t)(batVoltageFixdt >> 16);
    }

    curL_phaA = (int16_t)(offsetrlA - adc_buffer.rlA);
    curL_phaB = (int16_t)(offsetrlB - adc_buffer.rlB);
    curL_DC   = (int16_t)(offsetdcl - adc_buffer.dcl);
    curR_phaB = (int16_t)(offsetrrB - adc_buffer.rrB);
    curR_phaC = (int16_t)(offsetrrC - adc_buffer.rrC);
    curR_DC   = (int16_t)(offsetdcr - adc_buffer.dcr);

    /* VESC ownership is per motor. A right-motor forwarded command must not
     * accidentally energize a stale left control mode (and vice versa). */
    const uint8_t leftSourceEnable=(enable!=0u)||mcpwm_foc_vesc_override_active(false);
    const uint8_t rightSourceEnable=(enable!=0u)||mcpwm_foc_vesc_override_active(true);
    const uint8_t leftDriveRequest=(leftSourceEnable!=0u)&&(m_motor_1.m_control_mode!=CONTROL_MODE_NONE);
    const uint8_t rightDriveRequest=(rightSourceEnable!=0u)&&(m_motor_2.m_control_mode!=CONTROL_MODE_NONE);
    const uint8_t leftBridgeWasOn=(LEFT_TIM->BDTR&TIM_BDTR_MOE)?1u:0u;
    const uint8_t rightBridgeWasOn=(RIGHT_TIM->BDTR&TIM_BDTR_MOE)?1u:0u;
    const int32_t curL_phaC=-(int32_t)curL_phaA-(int32_t)curL_phaB;
    const int32_t curR_phaA=-(int32_t)curR_phaB-(int32_t)curR_phaC;
    const uint8_t leftOpenloop = (m_motor_1.m_control_mode==CONTROL_MODE_OPENLOOP ||
                                  m_motor_1.m_control_mode==CONTROL_MODE_OPENLOOP_PHASE);
    const uint8_t rightOpenloop = (m_motor_2.m_control_mode==CONTROL_MODE_OPENLOOP ||
                                   m_motor_2.m_control_mode==CONTROL_MODE_OPENLOOP_PHASE);
    const int32_t leftPhaseLimit=leftOpenloop?((int32_t)SVPWM_PHASE_LIMIT_A*A2BIT_CONV):curPha_max;
    const int32_t rightPhaseLimit=rightOpenloop?((int32_t)SVPWM_PHASE_LIMIT_A*A2BIT_CONV):curPha_max;
    /* Phase over-current protection must apply to every powered mode. Hall
     * detection uses the mode-4 current-control power path, so its fixed-phase
     * submode gets the same stricter phase/DC limits even when the legacy
     * ctrlModReq is not SVPWM_MODE. */
    const uint8_t leftPhaseTrip=leftBridgeWasOn&&(ABS(curL_phaA)>leftPhaseLimit||ABS(curL_phaB)>leftPhaseLimit||ABS(curL_phaC)>leftPhaseLimit);
    const uint8_t rightPhaseTrip=rightBridgeWasOn&&(ABS(curR_phaB)>rightPhaseLimit||ABS(curR_phaC)>rightPhaseLimit||ABS(curR_phaA)>rightPhaseLimit);
    const int32_t leftDcLimit=leftOpenloop?((int32_t)SVPWM_DC_LIMIT_A*A2BIT_CONV):curDC_max;
    const int32_t rightDcLimit=rightOpenloop?((int32_t)SVPWM_DC_LIMIT_A*A2BIT_CONV):curDC_max;
    if(ABS(curL_DC)>leftDcLimit||leftPhaseTrip||leftDriveRequest==0u){LEFT_TIM->BDTR&=~TIM_BDTR_MOE;if(leftPhaseTrip)m_motor_1.m_current_trip_count++;}else LEFT_TIM->BDTR|=TIM_BDTR_MOE;
    if(ABS(curR_DC)>rightDcLimit||rightPhaseTrip||rightDriveRequest==0u){RIGHT_TIM->BDTR&=~TIM_BDTR_MOE;if(rightPhaseTrip)m_motor_2.m_current_trip_count++;}else RIGHT_TIM->BDTR|=TIM_BDTR_MOE;

    buzzerTimer++;
    if (buzzerFreq != 0 && (buzzerTimer / 5000) % (buzzerPattern + 1) == 0) {
        if (buzzerPrev == 0) { buzzerPrev=1; if(++buzzerIdx>(buzzerCount+2))buzzerIdx=1; }
        if (buzzerTimer % buzzerFreq == 0 && (buzzerIdx <= buzzerCount || buzzerCount == 0)) HAL_GPIO_TogglePin(BUZZER_PORT,BUZZER_PIN);
    } else if (buzzerPrev) { HAL_GPIO_WritePin(BUZZER_PORT,BUZZER_PIN,GPIO_PIN_RESET); buzzerPrev=0; }

    if (s_overrun) { m_motor_1.m_overrun_count++;m_motor_2.m_overrun_count++;foc_isr_monitor_end(focIsrStartCycles);return; }
    s_overrun=1;
    mcpwm_foc_adc_int_handler();

    /* Odom follows the already validated Hall transition accumulator. Right is
     * normalized to the same positive vehicle direction as VESC/CAN telemetry. */
    odom_l = modulo_i16((int16_t)(m_motor_1.m_position_counts % 9000), 9000);
    odom_r = modulo_i16((int16_t)((-m_motor_2.m_position_counts) % 9000), 9000);
    s_overrun=0;
    foc_isr_monitor_end(focIsrStartCycles);
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
    for (uint16_t k = 0u; k < 1000u; ++k) {
        const float align_i = current * (float)(k + 1u) / 1000.0f;
        mcpwm_foc_set_openloop_phase(align_i, 0.0f, second);
        mcpwm_foc_vesc_override_touch(second);
        HAL_Delay(1u);
    }

    for (uint8_t pass = 0u; pass < 6u; ++pass) {
        const bool reverse = pass >= 3u;
        for (uint16_t k = 0u; k < 360u; ++k) {
            const uint16_t deg = reverse ? (uint16_t)(359u - k) : k;
            mcpwm_foc_set_openloop_phase(current, (float)deg, second);
            mcpwm_foc_vesc_override_touch(second);
            HAL_Delay(5u);
            const uint8_t h = hall_read(second);
            if (h != 0u && h != 7u) {
                int16_t sn, cs;
                const uint16_t ph = (uint16_t)(((uint32_t)deg * 65536u) / 360u);
                foc_sin_cos_q15(ph, &sn, &cs);
                sum_s[h] += sn;
                sum_c[h] += cs;
                if (samples[h] < 0xffffu) samples[h]++;
            }
        }
    }
    mcpwm_foc_release_motor(second);
    mcpwm_foc_vesc_override_clear(second);

    uint8_t valid = 0u;
    for (uint8_t h = 1u; h <= 6u; ++h) {
        if (samples[h] <= 30u) continue;
        int64_t best_dot = INT64_MIN;
        uint8_t best = 255u;
        for (uint16_t a = 0u; a < 200u; ++a) {
            int16_t sn, cs;
            const uint16_t ph = (uint16_t)(((uint32_t)a * 65536u) / 200u);
            foc_sin_cos_q15(ph, &sn, &cs);
            const int64_t dot = sum_s[h] * sn + sum_c[h] * cs;
            if (dot > best_dot) { best_dot = dot; best = (uint8_t)a; }
        }
        table[h] = best;
        valid++;
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
    const int32_t id=m->m_id_q4;
    const int32_t iq=m->m_iq_q4;
    const uint32_t mag2=(uint32_t)(id*id)+(uint32_t)(iq*iq);
    int32_t mag=(int32_t)foc_isqrt_u32(mag2);
    /* VESC mcpwm_foc_get_tot_current_motor() is SIGN(i_bus) * |I_dq|.
     * EFeru DC-current ADC polarity is inverted, hence -m_current_in_counts. */
    if(m->m_current_in_counts>0)mag=-mag;
    return (float)mag/(float)FOC_CURRENT_Q4_PER_A;
}
float mcpwm_foc_get_tot_current_motor(bool s){return motor_current_vesc_a(mcpwm_foc_get_motor_const(s));}
float mcpwm_foc_get_tot_current_in_motor(bool s){return -(float)mcpwm_foc_get_motor_const(s)->m_current_in_counts/(float)A2BIT_CONV;}
float mcpwm_foc_get_rpm_motor(bool s){return (float)mcpwm_foc_get_motor_const(s)->m_rpm;}
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
float mcpwm_foc_get_id_motor(bool s){return q4_to_amp(mcpwm_foc_get_motor_const(s)->m_id_q4);}float mcpwm_foc_get_iq_motor(bool s){return q4_to_amp(mcpwm_foc_get_motor_const(s)->m_iq_q4);}
static float bus_voltage_now(void){return (float)(batVoltage*BAT_CALIB_REAL_VOLTAGE/BAT_CALIB_ADC)/100.0f;}
float mcpwm_foc_get_vd_motor(bool s){return (float)mcpwm_foc_get_motor_const(s)->m_vd*(bus_voltage_now()/(float)MCCONF_FOC_VOLTAGE_MAX);}
float mcpwm_foc_get_vq_motor(bool s){return (float)mcpwm_foc_get_motor_const(s)->m_vq*(bus_voltage_now()/(float)MCCONF_FOC_VOLTAGE_MAX);}
float mcpwm_foc_get_phase_motor(bool s){return (float)mcpwm_foc_get_motor_const(s)->m_phase*(360.0f/65536.0f);}
int32_t mcpwm_foc_get_position_counts(bool s){return mcpwm_foc_get_motor_const(s)->m_position_counts;}mc_state mcpwm_foc_get_state_motor(bool s){return mcpwm_foc_get_motor_const(s)->m_state;}mc_fault_code mcpwm_foc_get_fault_motor(bool s){return mcpwm_foc_get_motor_const(s)->m_fault;}

void mcpwm_foc_get_values(mc_values *v,bool second){
    if (!v) return;
    memset(v, 0, sizeof(*v));
    const mcpwm_foc_motor_t *m = mcpwm_foc_get_motor_const(second);
    v->v_in = bus_voltage_now();
    v->id = q4_to_amp(m->m_id_q4);
    v->iq = q4_to_amp(m->m_iq_q4);
    v->current_motor = mcpwm_foc_get_tot_current_motor(second);
    v->current_in = mcpwm_foc_get_tot_current_in_motor(second);
    /* VESC mc_values.rpm is ERPM, not mechanical RPM. Tachometer remains the
     * Hall-transition accumulator, while position follows stock FOC semantics:
     * normalized rotor electrical angle in degrees. */
    v->rpm=mcpwm_foc_get_erpm_motor(second);
    v->tachometer=m->m_position_counts;
    v->tachometer_abs=m->m_position_counts<0?-m->m_position_counts:m->m_position_counts;
    v->position=mcpwm_foc_get_phase_motor(second);
    v->duty_now=(float)m->m_duty_now_permille/1000.0f;v->fault_code=m->m_fault;v->vesc_id=second?2:1;v->vd=mcpwm_foc_get_vd_motor(second);v->vq=mcpwm_foc_get_vq_motor(second);
}
