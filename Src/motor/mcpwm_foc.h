#ifndef MCPWM_FOC_H_
#define MCPWM_FOC_H_

#include <stdint.h>
#include <stdbool.h>
#include "vesc/datatypes.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MCPWM_FOC_MOTOR_1 = 0,
    MCPWM_FOC_MOTOR_2 = 1,
    MCPWM_FOC_MOTOR_COUNT = 2
} mcpwm_foc_motor_id_t;

typedef struct {
    mc_configuration m_conf;
    mc_state m_state;
    mc_control_mode m_control_mode;
    mc_fault_code m_fault;

    /* VESC-style setpoints. Fixed-point values are authoritative in the ISR. */
    volatile int16_t m_iq_set_q4;       /* slewed/active Iq reference */
    volatile int16_t m_iq_target_q4;    /* requested Iq reference */
    volatile int16_t m_id_set_q4;
    volatile int16_t m_speed_set_rpm;       /* active/slewed mechanical RPM */
    volatile int16_t m_speed_target_rpm;    /* requested mechanical RPM, integer view */
    volatile int32_t m_speed_target_rpm_q16; /* authoritative requested mechanical RPM Q16 */
    volatile int16_t m_duty_set_permille;

    volatile uint16_t m_kpq_q11, m_kiq_q16;
    volatile uint16_t m_kpd_q11, m_kid_q16;
    volatile uint16_t m_kps_q11, m_kis_q16, m_kds_q11;
    volatile uint16_t m_kpp_q11, m_kip_q16, m_kdp_q11;
    volatile int32_t m_position_counts;
    volatile int32_t m_position_target_counts;
    volatile int32_t m_position_min_counts;
    volatile int32_t m_position_max_counts;
    volatile int16_t m_current_limit_q4;

    /* Current state, same Q4 current-count unit as the legacy generated FOC. */
    volatile int16_t m_i_alpha_q4;
    volatile int16_t m_i_beta_q4;
    volatile int16_t m_id_q4;
    volatile int16_t m_iq_q4;
    volatile int16_t m_vd;
    volatile int16_t m_vq;
    volatile int16_t m_current_in_counts;
    volatile int16_t m_rpm;
    volatile int16_t m_duty_now_permille;

    /* Electrical phase: 0..65535 = 0..360 degrees. */
    volatile uint16_t m_phase;
    volatile uint16_t m_phase_hall;
    volatile uint16_t m_phase_hall_target;
    volatile uint16_t m_phase_openloop;
    volatile uint8_t m_phase_override;

    /* Hall estimator and fixed point regulators. */
    uint8_t m_hall_state;
    uint8_t m_hall_pos;
    uint8_t m_hall_pos_prev;
    int8_t m_hall_direction;
    uint16_t m_hall_ticks;
    uint16_t m_hall_period;
    uint16_t m_hall_period_hist[4];
    uint8_t m_hall_hist_pos;
    uint8_t m_hall_initialized;
    uint8_t m_hall_interp_active;
    /* Raw Hall state whose current rejection has already been counted.
     * 0xff means no pending/rejected edge is latched. This prevents a single
     * rejected edge from incrementing the diagnostic counter at 16 kHz while
     * still allowing the same persistent edge to be re-evaluated after its
     * debounce/outlier hold time has elapsed. */
    uint8_t m_hall_reject_counted_state;
    uint32_t m_hall_invalid_transition_count;

    int32_t m_iq_integrator;
    int32_t m_iq_set_ramp_q16;
    int32_t m_id_integrator;
    /* Speed integrator is Iq(q4) Q16; previous error is ERPM Q16. */
    int32_t m_speed_integrator;
    int32_t m_speed_prev_error;
    int32_t m_position_integrator;
    int16_t m_position_prev_error;
    uint8_t m_position_sat_hold;
    int32_t m_speed_set_ramp_q16;
    uint16_t m_speed_ramp_rpm_s;
    uint16_t m_speed_release_rpm;
    uint8_t m_iq_sat_hold;
    uint8_t m_id_sat_hold;
    uint8_t m_speed_sat_hold;
    int32_t m_current_lpf_q16[2];

    uint32_t m_openloop_phase_acc_q32;
    int32_t m_openloop_speed_q16;
    uint16_t m_openloop_align_ticks;
    int8_t m_openloop_direction;
    uint8_t m_openloop_primed;
    int16_t m_openloop_id_target_q4;
    int32_t m_openloop_id_ramp_q16;

    /* PWM and diagnostics. */
    volatile int16_t m_pwm_a;
    volatile int16_t m_pwm_b;
    volatile int16_t m_pwm_c;
    volatile uint16_t m_ccr_a;
    volatile uint16_t m_ccr_b;
    volatile uint16_t m_ccr_c;
    volatile uint32_t m_isr_count;
    volatile uint32_t m_overrun_count;
    volatile uint32_t m_current_trip_count;
} mcpwm_foc_motor_t;

extern mcpwm_foc_motor_t m_motor_1;
extern mcpwm_foc_motor_t m_motor_2;

void mcpwm_foc_init(void);
mcpwm_foc_motor_t *mcpwm_foc_get_motor(bool is_second_motor);
const mcpwm_foc_motor_t *mcpwm_foc_get_motor_const(bool is_second_motor);

void mcpwm_foc_set_configuration(const mc_configuration *conf, bool is_second_motor);
const volatile mc_configuration *mcpwm_foc_get_configuration(bool is_second_motor);

void mcpwm_foc_set_duty(float duty, bool is_second_motor);
void mcpwm_foc_set_pid_speed(float rpm, bool is_second_motor);
void mcpwm_foc_set_current(float current, bool is_second_motor);
void mcpwm_foc_set_pid_pos(float position_deg, bool is_second_motor);
void mcpwm_foc_set_position_counts(int32_t position_counts, bool is_second_motor);
/* User-facing long-range position API. Left/right share the same sign convention;
 * right is mirrored only internally. Values are full signed int32 counts. */
void mcpwm_foc_set_position_user_counts(int32_t position_counts, bool is_second_motor);
void mcpwm_foc_set_position_user_limits(int32_t min_counts, int32_t max_counts, bool is_second_motor);
int32_t mcpwm_foc_get_position_user_counts(bool is_second_motor);
int32_t mcpwm_foc_get_position_target_user_counts(bool is_second_motor);
int32_t mcpwm_foc_get_position_min_user_counts(bool is_second_motor);
int32_t mcpwm_foc_get_position_max_user_counts(bool is_second_motor);
void mcpwm_foc_reset_position(bool is_second_motor);
void mcpwm_foc_set_brake_current(float current, bool is_second_motor);
void mcpwm_foc_set_openloop_current(float current, float rpm, bool is_second_motor);
void mcpwm_foc_set_openloop_phase(float current, float phase, bool is_second_motor);
void mcpwm_foc_release_motor(bool is_second_motor);
void mcpwm_foc_vesc_override_touch(bool is_second_motor);
bool mcpwm_foc_vesc_override_active(bool is_second_motor);
bool mcpwm_foc_vesc_override_active_any(void);
void mcpwm_foc_vesc_override_clear(bool is_second_motor);

/* Integer API used by the bare-metal command layer. */
void mcpwm_foc_set_mode_command(uint8_t mode, int16_t command, bool run_request,
                                uint16_t openloop_rpm, bool is_second_motor);

float mcpwm_foc_get_tot_current_motor(bool is_second_motor);
float mcpwm_foc_get_tot_current_in_motor(bool is_second_motor);
float mcpwm_foc_get_rpm_motor(bool is_second_motor); /* mechanical RPM */
float mcpwm_foc_get_erpm_motor(bool is_second_motor);  /* VESC electrical RPM */
float mcpwm_foc_get_duty_cycle_motor(bool is_second_motor);
float mcpwm_foc_get_id_motor(bool is_second_motor);
float mcpwm_foc_get_iq_motor(bool is_second_motor);
float mcpwm_foc_get_vd_motor(bool is_second_motor);
float mcpwm_foc_get_vq_motor(bool is_second_motor);
float mcpwm_foc_get_phase_motor(bool is_second_motor);
mc_state mcpwm_foc_get_state_motor(bool is_second_motor);
mc_fault_code mcpwm_foc_get_fault_motor(bool is_second_motor);
void mcpwm_foc_get_values(mc_values *values, bool is_second_motor);
int32_t mcpwm_foc_get_position_counts(bool is_second_motor);
void mcpwm_foc_sync_tuning_to_conf(bool is_second_motor);
void mcpwm_foc_get_default_configuration(mc_configuration *conf, bool is_second_motor);
/* VESC-compatible Hall FOC detection. Returns table[8] in 0..199 electrical-angle units. */
bool mcpwm_foc_detect_hall(float current, bool is_second_motor, uint8_t table[8]);

/* Hardware calibration / ISR diagnostics. */
bool mcpwm_foc_dc_cal_done(void);
void mcpwm_foc_get_current_offsets(int16_t *pha0, int16_t *pha1, int16_t *dc,
                                   bool is_second_motor);
uint32_t mcpwm_foc_get_isr_cycles(void);
uint32_t mcpwm_foc_get_isr_cycles_max(void);

/* Called from the original DMA1_Channel1_IRQHandler after ADC frame acquisition. */
void mcpwm_foc_adc_int_handler(void);

#ifdef __cplusplus
}
#endif

#endif
