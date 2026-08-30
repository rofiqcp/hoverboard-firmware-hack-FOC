#ifndef MC_INTERFACE_H_
#define MC_INTERFACE_H_

#include <stdbool.h>
#include <stdint.h>
#include "motor/mcpwm_foc.h"
#include "motor/mcconf_default.h"

/* Runtime mode selectors. Optional side in the CLI means BOTH. */
extern volatile uint8_t m_sensor_mode_left;
extern volatile uint8_t m_sensor_mode_right;
extern volatile uint8_t m_comm_mode_left;
extern volatile uint8_t m_comm_mode_right;
extern volatile uint8_t m_control_mode_sel_left;
extern volatile uint8_t m_control_mode_sel_right;
extern volatile uint8_t m_live_stream_enabled;

/* Outer-loop tuning, still EEPROM friendly. */
extern int16_t s_pid_kp_left_x1000, s_pid_ki_left_x1000, s_pid_kd_left_x1000;
extern int16_t s_pid_kp_right_x1000, s_pid_ki_right_x1000, s_pid_kd_right_x1000;
extern int16_t s_pid_i_limit, s_pid_output_limit, s_pid_d_filter_x1000;
extern int16_t p_pid_kp_x1000, p_pid_ki_x1000, p_pid_kd_x1000;
extern int16_t p_pid_i_limit_rpm, p_pid_speed_limit_rpm, p_pid_deadband_counts;
extern int16_t p_pid_min_counts, p_pid_max_counts, p_pid_set_counts;
extern int16_t m_command_rate, m_command_filter;

/* Left encoder AB parameters/state. */
extern uint16_t m_encoder_counts;
extern uint16_t m_encoder_pole_pairs;
extern int16_t m_encoder_direction;
extern int16_t m_encoder_elec_trim_deg_x10;
extern int16_t m_encoder_sync_command;
extern uint16_t m_encoder_sync_sweep_ms;
extern uint16_t m_encoder_sync_settle_ms;
extern int16_t m_encoder_return_rpm;
extern int16_t m_encoder_return_tolerance_counts;
extern volatile int32_t m_encoder_position;
extern volatile int16_t m_encoder_rpm;
extern volatile int16_t m_position_speed_set;
extern volatile int16_t m_encoder_elec_angle_deg_x10;
extern volatile uint8_t m_encoder_sync_state;
extern volatile uint8_t m_encoder_sync_ok;

extern volatile int16_t m_speed_set_left_rpm;
extern volatile int16_t m_speed_set_right_rpm;
extern volatile int16_t m_speed_pid_output_left;
extern volatile int16_t m_speed_pid_output_right;
extern volatile int16_t m_speed_error_left_rpm;
extern volatile int16_t m_speed_error_right_rpm;

void mc_interface_init(void);

/* VESC-style application API. motor 1=Left, motor 2=Right. These functions
 * are optional for application code; USART3 pair commands continue to use
 * mc_interface_update(). A setter creates an explicit per-motor API override
 * until mc_interface_release_motor() is called for that motor. */
int mc_interface_motor_now(void);
void mc_interface_select_motor_thread(int motor);
mc_state mc_interface_get_state(void);
mc_control_mode mc_interface_get_control_mode(void);
void mc_interface_set_duty(float duty);
void mc_interface_set_current(float current);
void mc_interface_set_pid_speed(float rpm);
void mc_interface_set_pid_pos(float pos_counts);
void mc_interface_release_motor(void);
float mc_interface_get_rpm(void);
float mc_interface_get_tot_current(void);
float mc_interface_get_id(void);
float mc_interface_get_iq(void);
void mc_interface_reset_control(void);
void mc_interface_update(int16_t command_left, int16_t command_right,
                         int16_t rpm_left, int16_t rpm_right,
                         int16_t *motor_target_left, int16_t *motor_target_right,
                         uint8_t *inner_mode_left, uint8_t *inner_mode_right);

bool mc_interface_set_sensor_mode(uint8_t side, uint8_t mode); /* side: 0 both, 1 left, 2 right */
bool mc_interface_set_comm_mode(uint8_t side, uint8_t mode);
bool mc_interface_set_control_mode(uint8_t side, uint8_t mode);
void mc_interface_set_live(bool enabled);
uint16_t mc_interface_pack_mode_word(void);
int mc_interface_debug_command(const char *line);

void encoder_left_init(void);
void encoder_left_update(void);
int16_t encoder_left_mechanical_angle_x16(void);
uint8_t encoder_sync_active(void);
void encoder_sync_request(void);
void encoder_sync_service(uint8_t calibration_active);
int16_t encoder_sync_svpwm_command(void);
uint8_t encoder_sync_hold_vector(void);
uint16_t encoder_sync_hold_phase(void);
uint8_t encoder_left_synthetic_hall(void);

#endif
