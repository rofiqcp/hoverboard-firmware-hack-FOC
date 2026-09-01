#include <string.h>
#include "motor/mcpwm_foc.h"
#include "motor/mc_interface.h"

static int s_motor_selected = 1;

static bool selected_second(void) { return s_motor_selected == 2; }

void mc_interface_init(bool reset_conf) {
    (void)reset_conf;
    mcpwm_foc_init();
    s_motor_selected = 1;
}

int mc_interface_motor_now(void) { return s_motor_selected; }

void mc_interface_select_motor_thread(int motor) {
    if (motor == 1 || motor == 2) s_motor_selected = motor;
}

const volatile mc_configuration *mc_interface_get_configuration(void) {
    return mcpwm_foc_get_configuration(selected_second());
}

const volatile mc_configuration *mc_interface_get_configuration_motor(bool is_second_motor) {
    return mcpwm_foc_get_configuration(is_second_motor);
}

void mc_interface_set_configuration(mc_configuration *configuration) {
    if (configuration) mcpwm_foc_set_configuration(configuration, selected_second());
}

bool mc_interface_dccal_done(void) { return mcpwm_foc_dc_cal_done(); }
mc_fault_code mc_interface_get_fault(void) { return mcpwm_foc_get_fault_motor(selected_second()); }
mc_state mc_interface_get_state(void) { return mcpwm_foc_get_state_motor(selected_second()); }
mc_control_mode mc_interface_get_control_mode(void) { return mcpwm_foc_get_motor_const(selected_second())->m_control_mode; }
void mc_interface_set_duty(float dutyCycle) { mcpwm_foc_set_duty(dutyCycle, selected_second()); }
void mc_interface_set_pid_speed(float erpm) { mcpwm_foc_set_pid_speed(erpm, selected_second()); }
void mc_interface_set_current(float current) { mcpwm_foc_set_current(current, selected_second()); }
void mc_interface_set_brake_current(float current) { mcpwm_foc_set_brake_current(current, selected_second()); }
void mc_interface_set_openloop_current(float current, float rpm) { mcpwm_foc_set_openloop_current(current, rpm, selected_second()); }
void mc_interface_set_openloop_phase(float current, float phase) { mcpwm_foc_set_openloop_phase(current, phase, selected_second()); }
void mc_interface_release_motor(void) { mcpwm_foc_release_motor(selected_second()); }
float mc_interface_get_duty_cycle_now(void) { return mcpwm_foc_get_duty_cycle_motor(selected_second()); }
float mc_interface_get_rpm(void) { return mcpwm_foc_get_erpm_motor(selected_second()); }
float mc_interface_get_tot_current(void) { return mcpwm_foc_get_tot_current_motor(selected_second()); }
float mc_interface_get_tot_current_in(void) { return mcpwm_foc_get_tot_current_in_motor(selected_second()); }
float mc_interface_get_id(void) { return mcpwm_foc_get_id_motor(selected_second()); }
float mc_interface_get_iq(void) { return mcpwm_foc_get_iq_motor(selected_second()); }
float mc_interface_get_vd(void) { return mcpwm_foc_get_vd_motor(selected_second()); }
float mc_interface_get_vq(void) { return mcpwm_foc_get_vq_motor(selected_second()); }
float mc_interface_get_phase(void) { return mcpwm_foc_get_phase_motor(selected_second()); }
void mc_interface_get_values(mc_values *values) { mcpwm_foc_get_values(values, selected_second()); }

void mc_interface_set_mode_command_motor(uint8_t mode, int16_t command,
                                         bool run_request, uint16_t openloop_rpm,
                                         bool is_second_motor) {
    mcpwm_foc_set_mode_command(mode, command, run_request, openloop_rpm, is_second_motor);
}
void mc_interface_get_values_motor(mc_values *values, bool is_second_motor) { mcpwm_foc_get_values(values, is_second_motor); }
mc_fault_code mc_interface_get_fault_motor(bool is_second_motor) { return mcpwm_foc_get_fault_motor(is_second_motor); }
mc_state mc_interface_get_state_motor(bool is_second_motor) { return mcpwm_foc_get_state_motor(is_second_motor); }
