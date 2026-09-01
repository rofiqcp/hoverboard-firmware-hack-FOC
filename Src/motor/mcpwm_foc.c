#include <string.h>
#include <stdlib.h>
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
static uint16_t wp_l_vorher = 0, wp_r_vorher = 0;
static volatile uint8_t s_overrun = 0;
static volatile uint16_t s_vesc_override_ticks[2] = {0u, 0u};
static uint8_t s_foc_control_div = 0u;

static const uint8_t s_hall_to_pos[8] = {0, 2, 0, 1, 4, 3, 5, 0};

static uint16_t motor_pole_pairs(bool second) {
    return second ? MCCONF_POLE_PAIRS_RIGHT : MCCONF_POLE_PAIRS_LEFT;
}

static int16_t erpm_to_mech_rpm(float erpm, bool second) {
    const float pp = (float)motor_pole_pairs(second);
    float mech = (pp > 0.0f) ? (erpm / pp) : erpm;
    if (mech > (float)MCCONF_MOTOR_RPM_MAX) mech = (float)MCCONF_MOTOR_RPM_MAX;
    if (mech < -(float)MCCONF_MOTOR_RPM_MAX) mech = -(float)MCCONF_MOTOR_RPM_MAX;
    return (int16_t)mech;
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
static int16_t up_or_down(int16_t before, int16_t after) {
    static const int8_t map[6] = {0, -1, -2, 0, 2, 1};
    return map[modulo_i16((int16_t)(before - after), 6)];
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
    c->si_motor_poles = (uint8_t)(2u * (second ? MCCONF_POLE_PAIRS_RIGHT : MCCONF_POLE_PAIRS_LEFT));
    for (int i=0;i<8;i++) c->foc_hall_table[i] = s_hall_to_pos[i];
}

static void motor_reset(mcpwm_foc_motor_t *m, bool second) {
    memset(m, 0, sizeof(*m));
    conf_defaults(&m->m_conf, second);
    m->m_state = MC_STATE_OFF;
    m->m_control_mode = CONTROL_MODE_NONE;
    m->m_fault = FAULT_CODE_NONE;
    m->m_hall_pos_prev = 0;
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

void mcpwm_foc_set_configuration(const mc_configuration *conf, bool second) {
    if (!conf) return;
    mcpwm_foc_get_motor(second)->m_conf = *conf;
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

static void set_control_mode(mcpwm_foc_motor_t *m, mc_control_mode mode) {
    if (m->m_control_mode != mode) {
        reset_current_pi(m);
        m->m_speed_integrator = 0;
        m->m_speed_sat_hold = 0;
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
    /* VESC COMM_SET_RPM is electrical RPM (ERPM). Keep the Hall estimator and
     * legacy mode-2 loop in mechanical RPM internally. */
    mcpwm_foc_motor_t *m=mcpwm_foc_get_motor(second);
    set_control_mode(m, CONTROL_MODE_SPEED);
    m->m_speed_set_rpm=erpm_to_mech_rpm(erpm, second);
}
void mcpwm_foc_set_current(float current, bool second) {
    mcpwm_foc_motor_t *m=mcpwm_foc_get_motor(second);
    set_control_mode(m, CONTROL_MODE_CURRENT);
    m->m_iq_target_q4=amp_to_q4(m,current);
    m->m_id_set_q4=0;
}
void mcpwm_foc_set_brake_current(float current, bool second) {
    mcpwm_foc_motor_t *m=mcpwm_foc_get_motor(second);
    set_control_mode(m, CONTROL_MODE_CURRENT_BRAKE);
    int16_t q=amp_to_q4(m,current<0?-current:current);
    if (m->m_rpm > MCCONF_TRQ_STOP_RPM_DEADBAND) m->m_iq_target_q4=-q;
    else if (m->m_rpm < -MCCONF_TRQ_STOP_RPM_DEADBAND) m->m_iq_target_q4=q;
    else m->m_iq_target_q4=0;
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
    m->m_openloop_id_ramp_q16=0; m->m_state=MC_STATE_OFF;
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
            if (abs(m->m_rpm)>MCCONF_TRQ_STOP_RPM_DEADBAND) {
                set_control_mode(m, CONTROL_MODE_CURRENT_BRAKE);
                int16_t b=trq_ca_to_q4(m,MCCONF_TRQ_STOP_BRAKE_CA);
                m->m_iq_target_q4=(m->m_rpm>0)?-b:b;
                m->m_id_set_q4=0;
            } else {
                set_control_mode(m, CONTROL_MODE_DUTY); m->m_duty_set_permille=0; m->m_iq_target_q4=0; m->m_iq_set_q4=0; m->m_id_set_q4=0;
            }
        } else {
            set_control_mode(m, CONTROL_MODE_CURRENT); m->m_iq_target_q4=trq_ca_to_q4(m,command); m->m_id_set_q4=0;
        }
    } else if (mode==SPD_MODE) {
        set_control_mode(m, CONTROL_MODE_SPEED); m->m_speed_set_rpm=command;
    } else if (mode==VLT_MODE) {
        set_control_mode(m, CONTROL_MODE_DUTY); m->m_duty_set_permille=(int16_t)CLAMP(command,-1000,1000);
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

static void hall_update(mcpwm_foc_motor_t *m, bool second) {
    const uint8_t h=hall_read(second);
    const bool valid=(h!=0u && h!=7u);
    const uint8_t pos=s_hall_to_pos[h&7u];
    m->m_hall_state=h;
    if (m->m_hall_ticks < 0xffffu) m->m_hall_ticks++;

    if (valid) {
        if (!m->m_hall_initialized) {
            m->m_hall_initialized=1u;
            m->m_hall_pos_prev=pos;
            m->m_hall_pos=pos;
            m->m_hall_ticks=0u;
            m->m_hall_direction=0;
            m->m_rpm=0;
        } else if (pos!=m->m_hall_pos_prev) {
            const int8_t delta=(int8_t)pos-(int8_t)m->m_hall_pos_prev;
            int8_t dir=0;
            if ((delta==1)||(delta==-5)) dir=1;
            else if ((delta==-1)||(delta==5)) dir=-1;

            if (dir!=0) {
                uint16_t period=m->m_hall_ticks;
                if (period==0u) period=1u;
                if (m->m_hall_direction==0 || m->m_hall_direction!=dir ||
                    m->m_hall_period_hist[0]==MCCONF_HALL_TIMEOUT_TICKS) {
                    for(int i=0;i<4;i++)m->m_hall_period_hist[i]=period;
                    m->m_hall_hist_pos=0u;
                } else {
                    m->m_hall_period_hist[m->m_hall_hist_pos++ & 3u]=period;
                }
                uint32_t sum=0u; for(int i=0;i<4;i++)sum+=m->m_hall_period_hist[i];
                m->m_hall_period=(uint16_t)(sum/4u); if(!m->m_hall_period)m->m_hall_period=1u;
                m->m_hall_ticks=0u;
                m->m_hall_direction=dir;
                m->m_hall_pos_prev=pos;
                m->m_hall_pos=pos;
            } else {
                /* Reject skipped/noisy Hall transitions. Do not corrupt direction
                 * or period with a non-adjacent sector jump. */
                m->m_hall_invalid_transition_count++;
            }
        } else {
            m->m_hall_pos=pos;
        }
    }

    if (!valid || !m->m_hall_initialized || m->m_hall_ticks>MCCONF_HALL_TIMEOUT_TICKS ||
        !m->m_hall_period || m->m_hall_direction==0) {
        m->m_rpm=0;
    } else {
        int32_t rpm=10667/(int32_t)m->m_hall_period;
        if(rpm>MCCONF_MOTOR_RPM_MAX)rpm=MCCONF_MOTOR_RPM_MAX;
        m->m_rpm=(int16_t)(rpm*m->m_hall_direction);
    }

    const int16_t abs_rpm=(int16_t)abs(m->m_rpm);
    if (abs_rpm>=MCCONF_HALL_INTERP_ON_RPM) m->m_hall_interp_active=1u;
    else if (abs_rpm<=MCCONF_HALL_INTERP_OFF_RPM) m->m_hall_interp_active=0u;

    /* Generated-controller angle convention:
     *   forward: (pos + fraction)*60deg + 30deg
     *   reverse: (pos + 1 - fraction)*60deg + 30deg
     * Interpolation is disabled at low speed (15/30-rpm hysteresis), matching
     * the proven model and avoiding a drifting electrical angle while stalled. */
    uint8_t base_pos=valid?pos:m->m_hall_pos;
    if (m->m_hall_direction<0) base_pos=(uint8_t)((base_pos+1u)%6u);
    const uint32_t sector=65536u/6u;
    uint32_t frac=0u;
    if (m->m_hall_interp_active && m->m_hall_period && m->m_hall_ticks<m->m_hall_period) {
        frac=((uint32_t)m->m_hall_ticks*sector)/m->m_hall_period;
    }
    int32_t phase=(int32_t)((uint32_t)base_pos*sector)+(65536/12);
    if (m->m_hall_interp_active) phase+=(m->m_hall_direction>=0)?(int32_t)frac:-(int32_t)frac;
    m->m_phase_hall=(uint16_t)phase;
}

static void openloop_update(mcpwm_foc_motor_t *m) {
    const int16_t target=m->m_speed_set_rpm;
    const int8_t requested_dir=(target>0)?1:(target<0?-1:0);
    const uint16_t absrpm=(uint16_t)(target<0?-target:target);

    /* Slew Id in Q16 so sub-count-per-ISR ramps remain exact. 4 A/s with
     * 800 Q4-count/A gives 0.2 Q4-count/ISR, which cannot be represented by a
     * plain integer step. */
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
    const uint32_t stepPerRpm=(uint32_t)(((uint64_t)MCCONF_POLE_PAIRS_LEFT*4294967296ULL)/(60ULL*PWM_FREQ));
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

static int16_t phase_current_counts_to_q4(int16_t counts) {
    int32_t q4=(int32_t)counts<<4;
    /* Exact generated input saturation before Clarke/Park. */
    if(q4>27200)q4=27200; else if(q4<-27200)q4=-27200;
    return (int16_t)q4;
}

static void motor_control_step(mcpwm_foc_motor_t *m, bool second, int16_t i0_counts,
                               int16_t i1_counts, int16_t idc_counts, bool control_update) {
    hall_update(m, second);
    if (m->m_control_mode==CONTROL_MODE_OPENLOOP || m->m_control_mode==CONTROL_MODE_OPENLOOP_PHASE) openloop_update(m);

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
        m->m_state=MC_STATE_OFF; reset_current_pi(m); m->m_speed_integrator=0; m->m_speed_sat_hold=0;
        m->m_iq_set_q4=0; m->m_iq_target_q4=0; m->m_iq_set_ramp_q16=0;
        m->m_id_set_q4=0; m->m_openloop_id_target_q4=0; m->m_openloop_id_ramp_q16=0;
        v.q=0;v.d=0;
    } else {
        m->m_state=MC_STATE_RUNNING;
        if (m->m_control_mode==CONTROL_MODE_DUTY) {
            /* Mode 1 was already proven good; retain direct-voltage behavior. */
            v.q=(int16_t)(((int32_t)m->m_duty_set_permille*MCCONF_FOC_VOLTAGE_MAX)/1000); v.d=0;
        } else if (control_update) {
            /* The proven generated controller updates its regulators once every
             * three 16-kHz ADC frames (~5.333 kHz). */
            const int16_t v_closed=MCCONF_FOC_CLOSED_LOOP_VOLTAGE_MAX;
            m->m_id_set_q4 = (m->m_control_mode==CONTROL_MODE_OPENLOOP ||
                              m->m_control_mode==CONTROL_MODE_OPENLOOP_PHASE) ?
                              m->m_id_set_q4 : 0;
            const int16_t ed=(int16_t)CLAMP((int32_t)m->m_id_set_q4-m->m_id_q4,-32768,32767);
            v.d=pi_run_state(ed,MCCONF_FOC_ID_KP_Q11,MCCONF_FOC_ID_KI_Q16,
                             v_closed,-v_closed,&m->m_id_integrator,&m->m_id_sat_hold);
            const int16_t q_lim=voltage_circle_q_limit(v.d,v_closed);

            if (m->m_control_mode==CONTROL_MODE_SPEED) {
                /* Bit-compatible architecture with the original EFeru model:
                 * speed PI drives Vq directly. Do not cascade the old speed
                 * gains into an Iq PI; those gains were never tuned for that. */
                const int32_t e_q4_raw=((int32_t)m->m_speed_set_rpm-(int32_t)m->m_rpm)<<4;
                const int16_t e_q4=(int16_t)CLAMP(e_q4_raw,-32768,32767);
                m->m_iq_target_q4=0;
                m->m_iq_set_q4=0;
                m->m_iq_set_ramp_q16=0;
                v.q=pi_run_state(e_q4,MCCONF_SPEED_KP_Q11,MCCONF_SPEED_KI_Q16,
                                 q_lim,(int16_t)-q_lim,
                                 &m->m_speed_integrator,&m->m_speed_sat_hold);
                /* Speed mode bypasses the Iq regulator, but the Id regulator
                 * still holds Id=0, so keep its integrator alive. */
                m->m_iq_integrator=0;
                m->m_iq_sat_hold=0;
            } else {
                if (m->m_control_mode==CONTROL_MODE_CURRENT ||
                    m->m_control_mode==CONTROL_MODE_CURRENT_BRAKE) {
                    iq_setpoint_slew_step(m);
                } else {
                    m->m_iq_target_q4=0;
                    m->m_iq_set_q4=0;
                    m->m_iq_set_ramp_q16=0;
                }
                const int16_t eq=(int16_t)CLAMP((int32_t)m->m_iq_set_q4-m->m_iq_q4,-32768,32767);
                v.q=pi_run_state(eq,MCCONF_FOC_CURRENT_KP_Q11,MCCONF_FOC_CURRENT_KI_Q16,
                                 q_lim,(int16_t)-q_lim,
                                 &m->m_iq_integrator,&m->m_iq_sat_hold);
            }
            /* Backup only; q_lim already enforces the voltage circle before
             * the Iq PI, so its anti-windup sees the real available headroom. */
            foc_vector_limit(&v,v_closed);
        }
    }
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

    const uint8_t svpwmMode=(ctrlModReq==SVPWM_MODE);
    const uint8_t sourceEnable=(enable!=0u)||mcpwm_foc_vesc_override_active_any();
    const uint8_t leftBridgeWasOn=(LEFT_TIM->BDTR&TIM_BDTR_MOE)?1u:0u;
    const uint8_t rightBridgeWasOn=(RIGHT_TIM->BDTR&TIM_BDTR_MOE)?1u:0u;
    const int32_t curL_phaC=-(int32_t)curL_phaA-(int32_t)curL_phaB;
    const int32_t curR_phaA=-(int32_t)curR_phaB-(int32_t)curR_phaC;
    const int32_t phaseLimit=svpwmMode?((int32_t)SVPWM_PHASE_LIMIT_A*A2BIT_CONV):curPha_max;
    /* Phase over-current protection must apply to every powered mode. V9 only
     * protected mode 4, so 15-A spikes in mode 3 could pass unnoticed. The
     * bridgeWasOn qualifier avoids re-tripping on the sample after a chop. */
    const uint8_t leftPhaseTrip=leftBridgeWasOn&&(ABS(curL_phaA)>phaseLimit||ABS(curL_phaB)>phaseLimit||ABS(curL_phaC)>phaseLimit);
    const uint8_t rightPhaseTrip=rightBridgeWasOn&&(ABS(curR_phaB)>phaseLimit||ABS(curR_phaC)>phaseLimit||ABS(curR_phaA)>phaseLimit);
    const int32_t dcLimit=svpwmMode?((int32_t)SVPWM_DC_LIMIT_A*A2BIT_CONV):curDC_max;
    if(ABS(curL_DC)>dcLimit||leftPhaseTrip||sourceEnable==0u){LEFT_TIM->BDTR&=~TIM_BDTR_MOE;if(leftPhaseTrip)m_motor_1.m_current_trip_count++;}else LEFT_TIM->BDTR|=TIM_BDTR_MOE;
    if(ABS(curR_DC)>dcLimit||rightPhaseTrip||sourceEnable==0u){RIGHT_TIM->BDTR&=~TIM_BDTR_MOE;if(rightPhaseTrip)m_motor_2.m_current_trip_count++;}else RIGHT_TIM->BDTR|=TIM_BDTR_MOE;

    buzzerTimer++;
    if (buzzerFreq != 0 && (buzzerTimer / 5000) % (buzzerPattern + 1) == 0) {
        if (buzzerPrev == 0) { buzzerPrev=1; if(++buzzerIdx>(buzzerCount+2))buzzerIdx=1; }
        if (buzzerTimer % buzzerFreq == 0 && (buzzerIdx <= buzzerCount || buzzerCount == 0)) HAL_GPIO_TogglePin(BUZZER_PORT,BUZZER_PIN);
    } else if (buzzerPrev) { HAL_GPIO_WritePin(BUZZER_PORT,BUZZER_PIN,GPIO_PIN_RESET); buzzerPrev=0; }

    if (s_overrun) { m_motor_1.m_overrun_count++;m_motor_2.m_overrun_count++;foc_isr_monitor_end(focIsrStartCycles);return; }
    s_overrun=1;
    mcpwm_foc_adc_int_handler();

    uint8_t hl=hall_read(false),hr=hall_read(true);
    int pl=s_hall_to_pos[hl&7u],pr=s_hall_to_pos[hr&7u];
    odom_l=modulo_i16((int16_t)(odom_l+up_or_down((int16_t)wp_l_vorher,(int16_t)pl)),9000);wp_l_vorher=(uint16_t)pl;
    odom_r=modulo_i16((int16_t)(odom_r-up_or_down((int16_t)wp_r_vorher,(int16_t)pr)),9000);wp_r_vorher=(uint16_t)pr;
    s_overrun=0;
    foc_isr_monitor_end(focIsrStartCycles);
}

bool mcpwm_foc_dc_cal_done(void){return offsetcount>=2000u;}
void mcpwm_foc_get_current_offsets(int16_t *p0,int16_t *p1,int16_t *dc,bool second){if(!second){if(p0)*p0=offsetrlA;if(p1)*p1=offsetrlB;if(dc)*dc=offsetdcl;}else{if(p0)*p0=offsetrrB;if(p1)*p1=offsetrrC;if(dc)*dc=offsetdcr;}}
uint32_t mcpwm_foc_get_isr_cycles(void){return foc_isr_cycles;}uint32_t mcpwm_foc_get_isr_cycles_max(void){return foc_isr_cycles_max;}

static float q4_to_amp(int16_t q){return (float)q/(float)FOC_CURRENT_Q4_PER_A;}
float mcpwm_foc_get_tot_current_motor(bool s){return q4_to_amp(mcpwm_foc_get_motor_const(s)->m_iq_q4);}float mcpwm_foc_get_tot_current_in_motor(bool s){return (float)mcpwm_foc_get_motor_const(s)->m_current_in_counts/(float)A2BIT_CONV;}
float mcpwm_foc_get_rpm_motor(bool s){return (float)mcpwm_foc_get_motor_const(s)->m_rpm;}
float mcpwm_foc_get_erpm_motor(bool s){return (float)mcpwm_foc_get_motor_const(s)->m_rpm*(float)motor_pole_pairs(s);}
float mcpwm_foc_get_duty_cycle_motor(bool s){return (float)mcpwm_foc_get_motor_const(s)->m_duty_now_permille/1000.0f;}
float mcpwm_foc_get_id_motor(bool s){return q4_to_amp(mcpwm_foc_get_motor_const(s)->m_id_q4);}float mcpwm_foc_get_iq_motor(bool s){return q4_to_amp(mcpwm_foc_get_motor_const(s)->m_iq_q4);}
static float bus_voltage_now(void){return (float)(batVoltage*BAT_CALIB_REAL_VOLTAGE/BAT_CALIB_ADC)/100.0f;}
float mcpwm_foc_get_vd_motor(bool s){return (float)mcpwm_foc_get_motor_const(s)->m_vd*(bus_voltage_now()/(float)MCCONF_FOC_VOLTAGE_MAX);}
float mcpwm_foc_get_vq_motor(bool s){return (float)mcpwm_foc_get_motor_const(s)->m_vq*(bus_voltage_now()/(float)MCCONF_FOC_VOLTAGE_MAX);}
float mcpwm_foc_get_phase_motor(bool s){return (float)mcpwm_foc_get_motor_const(s)->m_phase*(360.0f/65536.0f);}mc_state mcpwm_foc_get_state_motor(bool s){return mcpwm_foc_get_motor_const(s)->m_state;}mc_fault_code mcpwm_foc_get_fault_motor(bool s){return mcpwm_foc_get_motor_const(s)->m_fault;}

void mcpwm_foc_get_values(mc_values *v,bool second){
    if (!v) return;
    memset(v, 0, sizeof(*v));
    const mcpwm_foc_motor_t *m = mcpwm_foc_get_motor_const(second);
    v->v_in=bus_voltage_now();v->current_motor=q4_to_amp(m->m_iq_q4);v->current_in=-(float)m->m_current_in_counts/A2BIT_CONV;v->id=q4_to_amp(m->m_id_q4);v->iq=q4_to_amp(m->m_iq_q4);
    /* VESC mc_values.rpm is ERPM, not mechanical RPM. */
    v->rpm=mcpwm_foc_get_erpm_motor(second);v->duty_now=(float)m->m_duty_now_permille/1000.0f;v->fault_code=m->m_fault;v->vesc_id=second?2:1;v->vd=mcpwm_foc_get_vd_motor(second);v->vq=mcpwm_foc_get_vq_motor(second);
}
