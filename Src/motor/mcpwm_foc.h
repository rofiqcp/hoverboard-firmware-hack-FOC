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
    volatile uint32_t m_fault_recovery_ticks;

    /* VESC-style setpoints. Fixed-point values are authoritative in the ISR. */
    volatile int16_t m_iq_set_q4;       /* slewed/active Iq reference */
    volatile int16_t m_iq_target_q4;    /* requested Iq reference */
    volatile int16_t m_id_set_q4;
    volatile int16_t m_speed_set_rpm;       /* active/slewed mechanical RPM */
    volatile int16_t m_speed_target_rpm;    /* requested mechanical RPM, integer view */
    volatile int32_t m_speed_target_rpm_q16; /* authoritative requested mechanical RPM Q16 */
    volatile int16_t m_duty_set_permille;      /* requested VESC duty */
    int32_t m_duty_i_q15;
    uint32_t m_duty_kp_q12_per_permille;
    uint32_t m_duty_ki_q12_per_permille;
    uint8_t m_duty_pi_active;

    volatile uint16_t m_kpq_q11, m_kiq_q16;
    volatile uint16_t m_kpd_q11, m_kid_q16;
    /* VESC current PI coefficients in physical-voltage fixed point. Kp is
     * V/A Q16; Ki_dt is V/A per regulator step Q16. These make the ISR
     * implement vd_int += Ierr*Ki*dt; vd = vd_int + Ierr*Kp exactly. */
    uint32_t m_current_kpq_v_q16, m_current_kiq_dt_v_q16;
    uint32_t m_current_kpd_v_q16, m_current_kid_dt_v_q16;
    /* Precomputed VESC physical PI gains per Q4-current-count, Q8 scaling.
     * ISR uses multiply+shift only; no Cortex-M3 software 64-bit division. */
    uint32_t m_current_kpq_err_q8, m_current_kiq_err_q8;
    uint32_t m_current_kpd_err_q8, m_current_kid_err_q8;
    volatile uint16_t m_kps_q11, m_kis_q16, m_kds_q11;
    /* Precomputed speed-PID coefficients. Configuration may use float, but the
     * 16-kHz ISR executes multiply+shift only (no __aeabi_ldivmod). */
    uint32_t m_speed_kp_coeff_q16;
    uint32_t m_speed_ki_coeff_q16;
    uint32_t m_speed_kd_coeff_q8;
    volatile uint16_t m_kpp_q11, m_kip_q16, m_kdp_q11;
    /* Extension posisi multi-putaran proyek. Pada Hall satu count = satu edge
     * Hall; pada ABI satu count = satu quadrature count. Jangan gunakan field
     * ini sebagai tachometer wire VESC. */
    volatile int32_t m_position_counts;
    volatile uint32_t m_position_abs_counts;
    /* Tachometer standar VESC: selalu resolusi 60 derajat elektrik, terlepas
     * dari sumber rotor Hall atau encoder dan terlepas dari CPR encoder. */
    volatile int32_t m_tachometer;
    volatile uint32_t m_tachometer_abs;
    uint8_t m_tacho_step_last;
    volatile int32_t m_position_target_counts;
    volatile int32_t m_position_min_counts;
    volatile int32_t m_position_max_counts;
    /* LEFT steering calibration. span is signed user-direction count travel
     * from physical left stop (-30 deg) to right stop (+30 deg). */
    volatile int32_t m_steering_span_counts;
    volatile uint8_t m_steering_calibrated;
    volatile uint8_t m_steering_homed;
    /* Stock VESC COMM_SET_POS is single-turn electrical rotor position. Keep it
     * separate from this project's long-range Hall-count position extension. */
    volatile uint16_t m_pos_pid_set_phase;
    volatile uint8_t m_pos_pid_phase_mode;
    /* VESC p_pid_ang_div tracking. feedback_phase adalah m_pos_pid_now internal
     * sebelum mc_interface menerapkan inversion/direction/p_pid_offset. */
    volatile uint16_t m_pos_pid_feedback_phase;
    uint16_t m_pos_pid_raw_last;
    uint16_t m_pos_pid_div_accum;
    uint32_t m_pos_pid_ang_div_inv_q16; /* (1/p_pid_ang_div)*65536 */
    volatile int16_t m_current_limit_q4;      /* batas Iq motoring (+), Q4 */
    volatile int16_t m_current_limit_neg_q4;  /* magnitudo batas Iq regen/brake (-), Q4 */
    volatile uint16_t m_battery_cut_start_adc;/* awal derating baterai dalam hitungan ADC */
    volatile uint16_t m_battery_cut_end_adc;  /* arus motoring nol dalam hitungan ADC */
    volatile int16_t m_input_current_max_q4;   /* batas arus DC positif */
    volatile int16_t m_input_current_regen_q4; /* magnitudo batas arus DC regeneratif */
    volatile uint16_t m_vin_min_adc;           /* l_min_vin dalam hitungan ADC baterai */
    volatile uint16_t m_vin_max_adc;           /* l_max_vin dalam hitungan ADC baterai */
    volatile uint32_t m_watt_max_x10;          /* batas daya motoring, 0,1 W */
    volatile uint32_t m_watt_regen_x10;        /* magnitudo batas daya regeneratif, 0,1 W */
    volatile int16_t m_temp_fet_start_x10;     /* awal derating temperatur board/MOS, 0,1 C */
    volatile int16_t m_temp_fet_end_x10;       /* akhir derating / fault, 0,1 C */
    volatile uint32_t m_wrong_voltage_integrator;
    volatile int16_t m_abs_current_limit_counts; /* precomputed l_abs_current_max * A2BIT_CONV */
    volatile int16_t m_duty_limit_permille;      /* precomputed l_max_duty * 1000 */

    /* Current state, same Q4 current-count unit as the legacy generated FOC. */
    volatile int16_t m_i_alpha_q4;
    volatile int16_t m_i_beta_q4;
    volatile int16_t m_id_q4;
    volatile int16_t m_iq_q4;
    volatile int16_t m_vd;
    volatile int16_t m_vq;
    volatile int16_t m_current_in_counts;
    /* Set only when Clarke/Park has produced a new D/Q sample on this motor's
     * 5.33-kHz regulator slot. The outer ABS protection consumes this flag so
     * its three-sample qualification always means three distinct ADC samples. */
    volatile uint8_t m_dq_sample_fresh;
    /* Monitoring-only filtered currents. Upstream VESC keeps a separate current
     * filter for non-time-critical telemetry so the fast current controller is
     * not slowed down by display smoothing. */
    volatile int16_t m_id_telem_q4;
    volatile int16_t m_iq_telem_q4;
    volatile int16_t m_current_in_telem_counts;
    int32_t m_telem_current_lpf_q16[3];
    uint16_t m_telem_current_filter_q16;
    /* VESC-style read/reset telemetry averages. These are accumulated at the
     * 5.33-kHz control cadence and atomically consumed by COMM_GET_VALUES. */
    volatile int32_t m_telem_sum_id_q4;
    volatile int32_t m_telem_sum_iq_q4;
    volatile int32_t m_telem_sum_ibus_counts;
    volatile uint16_t m_telem_avg_samples;
    /* Short OFF->RUN sample blanking; fixed startup control offsets are never
     * modified here. */
    volatile uint16_t m_bridge_settle_ticks;
    volatile int16_t m_rpm;
    volatile int16_t m_duty_now_permille;
    volatile uint8_t m_driven_offset_calibrating;
    volatile uint8_t m_driven_offset_valid;
    volatile uint16_t m_driven_offset_samples;
    volatile int16_t m_driven_offset0, m_driven_offset1, m_driven_offsetdc;
    /* Separate zero-current ADC offsets while the bridge is high-impedance.
     * The low-side current amplifiers shift operating point between bridge-OFF
     * and centered-PWM states on this hoverboard hardware. */
    volatile int16_t m_off_offset0, m_off_offset1, m_off_offsetdc;
    volatile uint16_t m_off_offset_samples;
    volatile uint16_t m_off_settle_ticks;
    volatile uint8_t m_off_offset_valid;
    int32_t m_off_offset_sum0, m_off_offset_sum1, m_off_offset_sumdc;

    /* VESC energy counters since boot. Upstream exposes separate drawn and
     * charged Ah/Wh counters in COMM_GET_VALUES. Updated from the measured
     * DC-link current at the 5-ms housekeeping cadence, never from telemetry. */
    float m_amp_seconds;
    float m_amp_seconds_charged;
    float m_watt_seconds;
    float m_watt_seconds_charged;

    /* Electrical phase: 0..65535 = 0..360 degrees. */
    volatile uint16_t m_phase;
    volatile uint16_t m_phase_hall;
    volatile uint16_t m_phase_hall_target;
    volatile uint16_t m_phase_encoder;      /* corrected electrical ABI phase */
    volatile uint16_t m_encoder_mech_phase; /* raw mechanical ABI angle 0..360 */
    volatile uint16_t m_phase_openloop;
    volatile uint8_t m_phase_override;

    /* VESC ABI encoder runtime state. Only motor LEFT can own TIM4/PB6/PB7. */
    volatile uint32_t m_encoder_raw_count;
    uint32_t m_encoder_prev_count;
    uint32_t m_encoder_counts;
    uint32_t m_encoder_count_to_phase_q16;
    uint32_t m_encoder_ratio_q16;
    uint16_t m_encoder_offset_phase;
    int32_t m_encoder_delta_accum;
    uint16_t m_encoder_speed_ticks;
    uint16_t m_encoder_idle_ticks;
    volatile int32_t m_encoder_erpm_q16;
    volatile int32_t m_encoder_mech_rpm_q16;
    volatile uint8_t m_encoder_configured;
    volatile uint8_t m_encoder_synced;

    /* Hall estimator and fixed point regulators. */
    uint8_t m_hall_state;              /* debounced Hall state used by FOC */
    uint8_t m_hall_raw_state;          /* instantaneous GPIO sample */
    uint8_t m_hall_candidate_state;
    uint8_t m_hall_candidate_count;
    uint8_t m_hall_debounce_initialized;
    /* VESC m_hall_extra_samples adapted to the EFeru hard realtime path:
     * rolling 1+2N sample majority using one synchronized GPIO snapshot per
     * 16-kHz ADC frame. O(1) update avoids repeated GPIO loops inside ISR. */
    uint8_t m_hall_sample_history[41];
    uint8_t m_hall_sample_index;
    uint8_t m_hall_sample_count;
    uint8_t m_hall_sample_sum_u;
    uint8_t m_hall_sample_sum_v;
    uint8_t m_hall_sample_sum_w;
    uint8_t m_hall_filter_window;
    uint8_t m_hall_filter_delay_ticks;
    uint8_t m_hall_direction_stable_edges;
    uint8_t m_hall_pos;
    uint8_t m_hall_pos_prev;
    int8_t m_hall_direction;
    uint16_t m_hall_ticks;
    uint16_t m_hall_period;
    uint32_t m_hall_interp_step_q16; /* electrical phase units/tick in Q16 */
    uint16_t m_hall_rate_limit_step; /* Hall phase correction slew from latest edge */
    uint16_t m_hall_interp_erpm;     /* VESC foc_hall_interp_erpm, cached integer */
    uint16_t m_hall_interp_max_ticks;/* max(edge age,last period) for interpolation */
    uint16_t m_hall_rate_min_step;   /* VESC 1.5x minimum rate-limit step */
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
    /* Split Hall diagnostics: impossible state/angle sequence vs period filter.
     * hall_invalid_transition_count remains the true electrical-sequence error
     * counter; a legitimate acceleration/reversal must never inflate it. */
    uint32_t m_hall_period_reject_count;
    uint32_t m_hall_sequence_reject_count;
    uint8_t m_hall_last_reject_reason; /* 0 none, 1 period, 2 sequence */
    uint8_t m_hall_last_reject_from;
    uint8_t m_hall_last_reject_to;

    /* Detect-All R/L capture. Updated only on the motor's 5.333-kHz current
     * regulator slot. The main loop starts/stops and snapshots it with IRQs
     * masked, so no 64-bit accumulator can tear on Cortex-M3. */
    volatile uint8_t m_rl_capture_active;
    uint8_t m_rl_capture_have_prev;
    int16_t m_rl_capture_prev_id_q4;
    uint32_t m_rl_capture_n;
    int64_t m_rl_sum_di2;
    int64_t m_rl_sum_div;
    int64_t m_rl_sum_dii;
    int64_t m_rl_sum_di;

    int32_t m_iq_integrator;
    int32_t m_iq_set_ramp_q16;
    int32_t m_id_integrator;
    /* Speed integrator is Iq(q4) Q16; previous error is ERPM Q16. */
    int32_t m_speed_integrator;
    int32_t m_speed_prev_error;
    int32_t m_position_integrator;
    int16_t m_position_prev_error; /* retained for custom count diagnostics */
    int32_t m_position_prev_error_mdeg;
    uint16_t m_position_dt_ticks;
    int32_t m_position_d_filter_q15;
    int32_t m_position_d_proc_filter_q15;
    uint16_t m_position_prev_proc_phase;
    uint16_t m_position_proc_dt_ticks;
    uint16_t m_position_breakaway_ticks;
    uint16_t m_position_no_motion_ticks;
    uint8_t m_position_motion_seen;
    uint8_t m_position_step_braking; /* one Hall edge -> brake-to-stop before next sector */
    int8_t m_position_brake_direction; /* latched accepted-edge direction; never follows rebound */
    int32_t m_position_last_motion_count; /* only validated Hall edge movement arms tracking */
    uint16_t m_position_kd_filter_q16;
    /* Precomputed p_pid_kd_proc process-derivative coefficient. This preserves
     * VESC-scale sub-millith gain resolution without float math in the ISR. */
    uint32_t m_position_kd_proc_coeff_q16;
    uint16_t m_position_kd_proc_phase_coeff_q4;
    uint32_t m_position_gain_dec_mdeg; /* p_pid_gain_dec_angle/p_pid_ang_div */
    uint8_t m_position_sat_hold;
    int8_t m_position_drive_direction;
    uint16_t m_position_settle_ticks;
    int32_t m_speed_set_ramp_q16;
    uint16_t m_speed_ramp_rpm_s;
    uint16_t m_speed_release_rpm;
    uint8_t m_iq_sat_hold;
    uint8_t m_id_sat_hold;
    uint8_t m_speed_sat_hold;
    /* Brake current is stored as a magnitude. CONTROL_MODE_CURRENT_BRAKE
     * recomputes its sign from fresh Hall speed every control update, matching
     * VESC's -SIGN(speed)*abs(current) semantics without reverse run-away. */
    int16_t m_brake_current_q4;
    int16_t m_handbrake_current_q4;
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
    volatile uint32_t m_phase_trip_count;
    volatile uint32_t m_dc_trip_count;
    volatile uint8_t m_phase_overcurrent_streak;
    volatile uint8_t m_last_trip_source; /* bit0 phase, bit1 DC-link */
    volatile int16_t m_last_trip_phase0_counts;
    volatile int16_t m_last_trip_phase1_counts;
    volatile int16_t m_last_trip_phase2_counts;
    volatile int16_t m_last_trip_dc_counts;
    volatile int16_t m_last_trip_duty_permille;
} mcpwm_foc_motor_t;

/* Snapshot telemetry sudah diskalakan persis ke unit wire COMM_GET_VALUES.
 * Tujuannya menghindari puluhan operasi soft-float pada STM32F103 tanpa FPU
 * saat VESC Tool melakukan polling realtime 50 Hz. */
typedef struct {
    int32_t current_motor_x100;
    int32_t current_in_x100;
    int32_t id_x100;
    int32_t iq_x100;
    int16_t duty_x1000;
    int32_t erpm;
    int16_t vin_x10;
    int32_t ah_x10000;
    int32_t ah_charged_x10000;
    int32_t wh_x10000;
    int32_t wh_charged_x10000;
    int32_t tachometer;
    int32_t tachometer_abs;
    uint8_t fault;
    int32_t vd_x1000;
    int32_t vq_x1000;
} mcpwm_foc_values_scaled_t;

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
void mcpwm_foc_set_handbrake(float current, bool is_second_motor);
void mcpwm_foc_set_openloop_current(float current, float rpm, bool is_second_motor);
void mcpwm_foc_set_openloop_phase(float current, float phase, bool is_second_motor);
bool mcpwm_foc_encoder_startup_align(bool is_second_motor);
bool mcpwm_foc_encoder_is_synced(bool is_second_motor);
bool mcpwm_foc_encoder_detect(float current, bool is_second_motor, float *offset, float *ratio, bool *inverted);
void mcpwm_foc_release_motor(bool is_second_motor);
void mcpwm_foc_vesc_timeout_configure(bool is_second_motor, uint32_t timeout_ms, float brake_current);
void mcpwm_foc_vesc_override_touch(bool is_second_motor);
bool mcpwm_foc_vesc_override_active(bool is_second_motor);
bool mcpwm_foc_vesc_command_live(bool is_second_motor);
void mcpwm_foc_vesc_override_clear(bool is_second_motor);
void mcpwm_foc_energy_update(uint32_t now_ms);
void mcpwm_foc_set_board_temperature_x10(int16_t temperature_x10);

/* Integer API used by the bare-metal command layer. */
void mcpwm_foc_set_mode_command(uint8_t mode, int16_t command, bool run_request,
                                uint16_t openloop_rpm, bool is_second_motor);

float mcpwm_foc_get_tot_current_motor(bool is_second_motor);
float mcpwm_foc_get_tot_current_in_motor(bool is_second_motor);
float mcpwm_foc_get_erpm_motor(bool is_second_motor);  /* VESC electrical RPM */
float mcpwm_foc_get_motor_mechanical_rpm(bool is_second_motor);
float mcpwm_foc_get_output_rpm(bool is_second_motor); /* after si_gear_ratio */
uint16_t mcpwm_foc_get_pole_pairs(bool is_second_motor);
float mcpwm_foc_get_gear_ratio(bool is_second_motor);
float mcpwm_foc_get_duty_cycle_motor(bool is_second_motor);
float mcpwm_foc_get_id_motor(bool is_second_motor);
float mcpwm_foc_get_iq_motor(bool is_second_motor);
float mcpwm_foc_get_vd_motor(bool is_second_motor);
float mcpwm_foc_get_vq_motor(bool is_second_motor);
float mcpwm_foc_get_phase_motor(bool is_second_motor);
float mcpwm_foc_get_phase_encoder_motor(bool is_second_motor); /* corrected electrical */
float mcpwm_foc_get_encoder_position_motor(bool is_second_motor); /* raw mechanical ABI */
float mcpwm_foc_get_pid_pos_now_motor(bool is_second_motor);
float mcpwm_foc_get_pid_pos_set_motor(bool is_second_motor);
mc_state mcpwm_foc_get_state_motor(bool is_second_motor);
mc_fault_code mcpwm_foc_get_fault_motor(bool is_second_motor);
void mcpwm_foc_get_values(mc_values *values, bool is_second_motor);
void mcpwm_foc_get_values_scaled(mcpwm_foc_values_scaled_t *values, bool is_second_motor);
void mcpwm_foc_sync_tuning_to_conf(bool is_second_motor);
void mcpwm_foc_refresh_hall_interpolation(bool is_second_motor);
void mcpwm_foc_refresh_encoder_configuration(bool is_second_motor, bool reinitialize);
void mcpwm_foc_refresh_position_configuration(bool is_second_motor);
void mcpwm_foc_get_default_configuration(mc_configuration *conf, bool is_second_motor);
/* VESC-compatible Hall FOC detection. Returns table[8] in 0..199 electrical-angle units. */
bool mcpwm_foc_detect_hall(float current, bool is_second_motor, uint8_t table[8]);
bool mcpwm_foc_hall_table_sane(const uint8_t table[8]);

typedef struct {
    uint32_t samples;
    int64_t sum_di2;
    int64_t sum_div;
    int64_t sum_dii;
    int64_t sum_di;
} mcpwm_foc_rl_capture_t;
void mcpwm_foc_rl_capture_start(bool is_second_motor);
void mcpwm_foc_rl_capture_stop(bool is_second_motor);
void mcpwm_foc_rl_capture_get(bool is_second_motor, mcpwm_foc_rl_capture_t *out);

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


bool mcpwm_foc_steering_set_span(int32_t span_counts, bool homed);
bool mcpwm_foc_steering_rebase_left(void);
bool mcpwm_foc_steering_is_calibrated(void);
bool mcpwm_foc_steering_is_homed(void);
int32_t mcpwm_foc_steering_span_counts(void);
float mcpwm_foc_get_steering_deg(void);
bool mcpwm_foc_set_steering_deg(float deg);
#endif
