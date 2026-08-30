#include "motor/mc_interface.h"
#include "config.h"
#include "defines.h"
#include "setup.h"
#include "util.h"
#include "stm32f1xx_hal.h"
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

volatile uint8_t m_sensor_mode_left = MCCONF_DEFAULT_SENSOR_LEFT;
volatile uint8_t m_sensor_mode_right = MCCONF_DEFAULT_SENSOR_RIGHT;
volatile uint8_t m_comm_mode_left = MCCONF_DEFAULT_COMM_LEFT;
volatile uint8_t m_comm_mode_right = MCCONF_DEFAULT_COMM_RIGHT;
volatile uint8_t m_control_mode_sel_left = MCCONF_DEFAULT_CONTROL_LEFT;
volatile uint8_t m_control_mode_sel_right = MCCONF_DEFAULT_CONTROL_RIGHT;
volatile uint8_t m_live_stream_enabled = 0u;

int16_t s_pid_kp_left_x1000 = MCCONF_S_PID_KP_X1000;
int16_t s_pid_ki_left_x1000 = MCCONF_S_PID_KI_X1000;
int16_t s_pid_kd_left_x1000 = MCCONF_S_PID_KD_X1000;
int16_t s_pid_kp_right_x1000 = MCCONF_S_PID_KP_X1000;
int16_t s_pid_ki_right_x1000 = MCCONF_S_PID_KI_X1000;
int16_t s_pid_kd_right_x1000 = MCCONF_S_PID_KD_X1000;
int16_t s_pid_i_limit = MCCONF_S_PID_I_LIMIT;
int16_t s_pid_output_limit = MCCONF_S_PID_OUTPUT_LIMIT;
int16_t s_pid_d_filter_x1000 = MCCONF_S_PID_D_FILTER_X1000;
int16_t p_pid_kp_x1000 = MCCONF_P_PID_KP_X1000;
int16_t p_pid_ki_x1000 = MCCONF_P_PID_KI_X1000;
int16_t p_pid_kd_x1000 = MCCONF_P_PID_KD_X1000;
int16_t p_pid_i_limit_rpm = MCCONF_P_PID_I_LIMIT_RPM;
int16_t p_pid_speed_limit_rpm = MCCONF_P_PID_SPEED_LIMIT_RPM;
int16_t p_pid_deadband_counts = MCCONF_P_PID_DEADBAND_COUNTS;
int16_t p_pid_min_counts = -12000;
int16_t p_pid_max_counts = 12000;
int16_t p_pid_set_counts = 0;
int16_t m_command_rate = RATE;
int16_t m_command_filter = FILTER;

uint16_t m_encoder_counts = 4096;
uint16_t m_encoder_pole_pairs = SVPWM_POLE_PAIRS;
int16_t m_encoder_direction = 1;
int16_t m_encoder_elec_trim_deg_x10 = 0;
int16_t m_encoder_sync_command = 80;
uint16_t m_encoder_sync_sweep_ms = 1200;
uint16_t m_encoder_sync_settle_ms = 350;
int16_t m_encoder_return_rpm = 80;
int16_t m_encoder_return_tolerance_counts = 8;
volatile int32_t m_encoder_position = 0;
volatile int16_t m_encoder_rpm = 0;
volatile int16_t m_position_speed_set = 0;
volatile int16_t m_encoder_elec_angle_deg_x10 = 0;
volatile uint8_t m_encoder_sync_state = 0;
volatile uint8_t m_encoder_sync_ok = 0;
volatile int16_t m_speed_set_left_rpm = 0;
volatile int16_t m_speed_set_right_rpm = 0;
volatile int16_t m_speed_pid_output_left = 0;
volatile int16_t m_speed_pid_output_right = 0;
volatile int16_t m_speed_error_left_rpm = 0;
volatile int16_t m_speed_error_right_rpm = 0;

typedef struct {
  int32_t integral_x1000;
  int32_t d_filtered_x1000;
  int16_t prev_error;
} pid_state_t;

static pid_state_t m_speed_pid_left;
static pid_state_t m_speed_pid_right;
static pid_state_t m_position_pid;

/* Optional VESC-style direct API state. USART3 remains the normal command
 * source; an API setter overrides only the selected motor until release. */
static volatile uint8_t m_selected_motor = 1u;
static volatile uint8_t m_api_override_mask = 0u;
static volatile uint8_t m_api_control_left = MCCONF_CONTROL_CURRENT;
static volatile uint8_t m_api_control_right = MCCONF_CONTROL_CURRENT;
static volatile int16_t m_api_command_left = 0;
static volatile int16_t m_api_command_right = 0;

static uint16_t m_encoder_prev_hw;
static int32_t m_encoder_raw_extended;
static int32_t m_encoder_zero_raw;
static uint32_t m_encoder_last_ms;
static int32_t m_encoder_last_position;
static int16_t m_encoder_mech_offset_x16;
static int32_t m_sync_initial_position;
static uint32_t m_sync_started_ms;
static uint16_t m_sync_hold_phase;

static int16_t clamp_s16_local(int32_t v, int16_t lo, int16_t hi) {
  if (v > hi) return hi;
  if (v < lo) return lo;
  return (int16_t)v;
}

static void pid_reset(pid_state_t *pid) {
  memset(pid, 0, sizeof(*pid));
}

/* VESC-style engineering PID. State is command*x1000. The old implementation
 * multiplied the integral update by 1000 a second time, which caused rapid
 * wind-up and is the reason a 100-RPM request ran away to ~400-500 RPM. */
static int16_t pid_step(pid_state_t *pid, int16_t error,
                        int16_t kp_x1000, int16_t ki_x1000, int16_t kd_x1000,
                        int16_t i_limit, int16_t out_limit, int16_t d_filter_x1000) {
  int32_t candidate_i = pid->integral_x1000 + ((int32_t)ki_x1000 * (int32_t)error) / (int32_t)MAIN_LOOP_HZ;
  const int32_t i_lim_x1000 = (int32_t)i_limit * 1000;
  if (candidate_i > i_lim_x1000) candidate_i = i_lim_x1000;
  if (candidate_i < -i_lim_x1000) candidate_i = -i_lim_x1000;

  if (d_filter_x1000 < 0) d_filter_x1000 = 0;
  if (d_filter_x1000 > 1000) d_filter_x1000 = 1000;
  const int32_t d_raw_x1000 = (int32_t)kd_x1000 * ((int32_t)error - pid->prev_error) * (int32_t)MAIN_LOOP_HZ;
  const int32_t d_filtered = ((int32_t)d_filter_x1000 * pid->d_filtered_x1000 +
                              (int32_t)(1000 - d_filter_x1000) * d_raw_x1000) / 1000;
  const int32_t p_x1000 = (int32_t)kp_x1000 * (int32_t)error;
  int32_t raw_x1000 = p_x1000 + candidate_i + d_filtered;
  const int32_t max_x1000 = (int32_t)out_limit * 1000;
  bool saturated_high = raw_x1000 > max_x1000;
  bool saturated_low = raw_x1000 < -max_x1000;

  /* Conditional integration anti-windup: integrate while unsaturated or when
   * the error is driving a saturated output back toward the linear region. */
  if ((!saturated_high && !saturated_low) ||
      (saturated_high && error < 0) || (saturated_low && error > 0)) {
    pid->integral_x1000 = candidate_i;
  }
  pid->d_filtered_x1000 = d_filtered;
  pid->prev_error = error;

  raw_x1000 = p_x1000 + pid->integral_x1000 + pid->d_filtered_x1000;
  if (raw_x1000 > max_x1000) raw_x1000 = max_x1000;
  if (raw_x1000 < -max_x1000) raw_x1000 = -max_x1000;
  return (int16_t)(raw_x1000 / 1000);
}

static bool feedback_sensor_available(uint8_t side) {
  const uint8_t mode = side == 1u ? m_sensor_mode_left : m_sensor_mode_right;
  return mode == MCCONF_SENSOR_HALL || (side == 1u && mode == MCCONF_SENSOR_ENCODER_AB);
}

void mc_interface_reset_control(void) {
  pid_reset(&m_speed_pid_left);
  pid_reset(&m_speed_pid_right);
  pid_reset(&m_position_pid);
  m_position_speed_set = 0;
  m_speed_set_left_rpm = 0;
  m_speed_set_right_rpm = 0;
  m_speed_pid_output_left = 0;
  m_speed_pid_output_right = 0;
  m_speed_error_left_rpm = 0;
  m_speed_error_right_rpm = 0;
  /* STOP/control reset is authoritative even for application-API users. */
  m_api_command_left = 0;
  m_api_command_right = 0;
}

static mc_motor_type motor_type_from_comm(uint8_t comm) {
  if (comm == MCCONF_COMM_SIX_STEP) return MOTOR_TYPE_COMMUTATION;
  if (comm == MCCONF_COMM_SINE_PWM) return MOTOR_TYPE_SINE;
  return MOTOR_TYPE_FOC;
}

static void apply_configuration(void) {
  m_mcconf_1.sensor_mode = m_sensor_mode_left;
  m_mcconf_2.sensor_mode = m_sensor_mode_right;
  m_mcconf_1.comm_mode = m_comm_mode_left;
  m_mcconf_2.comm_mode = m_comm_mode_right;
  m_mcconf_1.motor_type = motor_type_from_comm(m_comm_mode_left);
  m_mcconf_2.motor_type = motor_type_from_comm(m_comm_mode_right);
  m_mcconf_1.foc_encoder_enable = (m_sensor_mode_left == MCCONF_SENSOR_ENCODER_AB);
  m_mcconf_2.foc_encoder_enable = 0u;
  m_mcconf_1.si_motor_pole_pairs = (uint8_t)(m_encoder_pole_pairs ? m_encoder_pole_pairs : SVPWM_POLE_PAIRS);
  m_mcconf_2.si_motor_pole_pairs = SVPWM_POLE_PAIRS;
}

static void left_encoder_hw_enable(bool enable) {
  GPIO_InitTypeDef gpio = {0};
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;

  if (enable) {
    /* Encoder AB owns PB6/PB7 only in sensor mode 3. */
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &gpio);
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_TIM_SET_COUNTER(&htim_encoder_left, 0u);
    HAL_TIM_Encoder_Start(&htim_encoder_left, TIM_CHANNEL_ALL);
    encoder_left_init();
  } else {
    HAL_TIM_Encoder_Stop(&htim_encoder_left, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(&htim_encoder_left, 0u);
    /* Restore the proven Hall electrical input state when encoder is unused. */
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &gpio);
    m_encoder_rpm = 0;
    m_encoder_position = 0;
    m_encoder_sync_state = 0;
    m_encoder_sync_ok = 0;
  }
}

static void sensor_reinit(void) {
  /* Mode changes are accepted only while stopped. Make GPIO/TIM4 ownership and
   * controller state atomic relative to the 16-kHz motor ISR so it can never
   * observe half of a sensor reconfiguration. */
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  left_encoder_hw_enable(m_sensor_mode_left == MCCONF_SENSOR_ENCODER_AB);
  mcpwm_foc_sensor_state_reset(0u);
  mcpwm_foc_sensor_state_reset(1u);
  apply_configuration();
  if (!primask) __enable_irq();
  mc_interface_reset_control();
  if (m_sensor_mode_left == MCCONF_SENSOR_ENCODER_AB) encoder_sync_request();
}

void mc_interface_init(void) {
  apply_configuration();
  sensor_reinit();
}

int mc_interface_motor_now(void) { return (int)m_selected_motor; }

void mc_interface_select_motor_thread(int motor) {
  if (motor == 1 || motor == 2) m_selected_motor = (uint8_t)motor;
}

static uint8_t selected_side(void) { return m_selected_motor == 2u ? 2u : 1u; }

static void api_set(uint8_t control, int16_t command) {
  const uint8_t side = selected_side();
  if (side == 1u) {
    m_api_control_left = control;
    m_api_command_left = command;
    m_api_override_mask |= 0x01u;
  } else {
    m_api_control_right = control;
    m_api_command_right = command;
    m_api_override_mask |= 0x02u;
  }
}

mc_state mc_interface_get_state(void) {
  return selected_side() == 1u ? m_motor_1.m_state : m_motor_2.m_state;
}

mc_control_mode mc_interface_get_control_mode(void) {
  return selected_side() == 1u ? m_motor_1.m_control_mode : m_motor_2.m_control_mode;
}

void mc_interface_set_duty(float duty) {
  if (duty > 1.0f) duty = 1.0f;
  if (duty < -1.0f) duty = -1.0f;
  api_set(MCCONF_CONTROL_PWM, (int16_t)(duty * 1000.0f));
}

void mc_interface_set_current(float current) {
  const mc_configuration *conf = selected_side() == 1u ? &m_mcconf_1 : &m_mcconf_2;
  const float max_current = (float)conf->l_current_max / ((float)A2BIT_CONV * 16.0f);
  if (max_current <= 0.0f) return;
  float rel = current / max_current;
  if (rel > 1.0f) rel = 1.0f;
  if (rel < -1.0f) rel = -1.0f;
  api_set(MCCONF_CONTROL_CURRENT, (int16_t)(rel * 1000.0f));
}

void mc_interface_set_pid_speed(float rpm) {
  const uint8_t side = selected_side();
  /* This port has no closed-loop sensorless observer yet. Do not silently use
   * the generated open-loop RPM estimate as speed feedback. */
  if (!feedback_sensor_available(side)) return;
  if (rpm > 32767.0f) rpm = 32767.0f;
  if (rpm < -32768.0f) rpm = -32768.0f;
  api_set(MCCONF_CONTROL_SPEED, (int16_t)rpm);
}

void mc_interface_set_pid_pos(float pos_counts) {
  if (selected_side() != 1u || m_sensor_mode_left != MCCONF_SENSOR_ENCODER_AB) return;
  if (pos_counts > (float)p_pid_max_counts) pos_counts = (float)p_pid_max_counts;
  if (pos_counts < (float)p_pid_min_counts) pos_counts = (float)p_pid_min_counts;
  p_pid_set_counts = (int16_t)pos_counts;
  api_set(MCCONF_CONTROL_POSITION, 0);
}

void mc_interface_release_motor(void) {
  const uint8_t side = selected_side();
  if (side == 1u) {
    m_api_override_mask &= (uint8_t)~0x01u;
    m_api_command_left = 0;
    pid_reset(&m_speed_pid_left);
  } else {
    m_api_override_mask &= (uint8_t)~0x02u;
    m_api_command_right = 0;
    pid_reset(&m_speed_pid_right);
  }
}

float mc_interface_get_rpm(void) {
  return selected_side() == 1u ? (float)m_sensor_rpm_left : (float)(-m_sensor_rpm_right);
}

float mc_interface_get_iq(void) {
  const int16_t q4 = selected_side() == 1u ? m_motor_1.m_output.iq : (int16_t)-m_motor_2.m_output.iq;
  return (float)q4 / ((float)A2BIT_CONV * 16.0f);
}

float mc_interface_get_id(void) {
  const int16_t q4 = selected_side() == 1u ? m_motor_1.m_output.id : m_motor_2.m_output.id;
  return (float)q4 / ((float)A2BIT_CONV * 16.0f);
}

float mc_interface_get_tot_current(void) { return mc_interface_get_iq(); }

bool mc_interface_set_sensor_mode(uint8_t side, uint8_t mode) {
  if (mode < MCCONF_SENSOR_OPENLOOP || mode > MCCONF_SENSOR_ENCODER_AB) return false;
  if ((side == 0u || side == 2u) && mode == MCCONF_SENSOR_ENCODER_AB) return false;
  if (side == 0u) {
    m_sensor_mode_left = mode;
    m_sensor_mode_right = mode;
  } else if (side == 1u) {
    m_sensor_mode_left = mode;
  } else if (side == 2u) {
    m_sensor_mode_right = mode;
  } else return false;
  sensor_reinit();
  return true;
}

bool mc_interface_set_comm_mode(uint8_t side, uint8_t mode) {
  if (mode < MCCONF_COMM_SIX_STEP || mode > MCCONF_COMM_SVPWM) return false;
  if (side == 0u || side == 1u) m_comm_mode_left = mode;
  if (side == 0u || side == 2u) m_comm_mode_right = mode;
  if (side > 2u) return false;
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  apply_configuration();
  mcpwm_foc_reset(&m_motor_1); m_motor_1.m_conf = &m_mcconf_1;
  mcpwm_foc_reset(&m_motor_2); m_motor_2.m_conf = &m_mcconf_2;
  if (!primask) __enable_irq();
  mc_interface_reset_control();
  return true;
}

bool mc_interface_set_control_mode(uint8_t side, uint8_t mode) {
  if (mode < MCCONF_CONTROL_PWM || mode > MCCONF_CONTROL_POSITION) return false;
  if (mode == MCCONF_CONTROL_SPEED) {
    if ((side == 0u || side == 1u) && !feedback_sensor_available(1u)) return false;
    if ((side == 0u || side == 2u) && !feedback_sensor_available(2u)) return false;
  }
  if (mode == MCCONF_CONTROL_POSITION) {
    /* Position feedback exists only on Left encoder AB. Requiring an explicit
     * Left side prevents `mode control 4` from silently changing Right to a
     * different control law. */
    if (side != 1u || m_sensor_mode_left != MCCONF_SENSOR_ENCODER_AB) return false;
  }
  if (side == 0u || side == 1u) { m_control_mode_sel_left = mode; m_api_override_mask &= (uint8_t)~0x01u; }
  if (side == 0u || side == 2u) { m_control_mode_sel_right = mode; m_api_override_mask &= (uint8_t)~0x02u; }
  if (side > 2u) return false;
  mc_interface_reset_control();
  return true;
}

void mc_interface_set_live(bool enabled) { m_live_stream_enabled = enabled ? 1u : 0u; }

static int16_t control_one(uint8_t side, uint8_t control_mode, int16_t command, int16_t rpm, uint8_t *inner) {
  if (control_mode == MCCONF_CONTROL_PWM) {
    *inner = CONTROL_MODE_DUTY;
    return command;
  }
  if (control_mode == MCCONF_CONTROL_CURRENT) {
    *inner = CONTROL_MODE_CURRENT;
    return command;
  }
  if (control_mode == MCCONF_CONTROL_SPEED) {
    *inner = CONTROL_MODE_CURRENT;
    if (command == 0) {
      if (side == 1u) { pid_reset(&m_speed_pid_left); m_speed_set_left_rpm = 0; m_speed_pid_output_left = 0; m_speed_error_left_rpm = 0; }
      else { pid_reset(&m_speed_pid_right); m_speed_set_right_rpm = 0; m_speed_pid_output_right = 0; m_speed_error_right_rpm = 0; }
      return 0;
    }
    /* New CLI semantics: start 100 100 in SPEED means exactly 100 mechanical RPM. */
    const int16_t speed_ref = command;
    const int16_t error = (int16_t)(speed_ref - rpm);
    int16_t output;
    if (side == 1u) {
      m_speed_set_left_rpm = speed_ref;
      m_speed_error_left_rpm = error;
      output = pid_step(&m_speed_pid_left, error, s_pid_kp_left_x1000, s_pid_ki_left_x1000,
                        s_pid_kd_left_x1000, s_pid_i_limit, s_pid_output_limit, s_pid_d_filter_x1000);
      m_speed_pid_output_left = output;
    } else {
      m_speed_set_right_rpm = speed_ref;
      m_speed_error_right_rpm = error;
      output = pid_step(&m_speed_pid_right, error, s_pid_kp_right_x1000, s_pid_ki_right_x1000,
                        s_pid_kd_right_x1000, s_pid_i_limit, s_pid_output_limit, s_pid_d_filter_x1000);
      m_speed_pid_output_right = output;
    }
    return output;
  }
  *inner = CONTROL_MODE_CURRENT;
  return 0;
}

void mc_interface_update(int16_t command_left, int16_t command_right,
                         int16_t rpm_left, int16_t rpm_right,
                         int16_t *motor_target_left, int16_t *motor_target_right,
                         uint8_t *inner_mode_left, uint8_t *inner_mode_right) {
  uint8_t control_left = m_control_mode_sel_left;
  uint8_t control_right = m_control_mode_sel_right;
  if (m_api_override_mask & 0x01u) { command_left = m_api_command_left; control_left = m_api_control_left; }
  if (m_api_override_mask & 0x02u) { command_right = m_api_command_right; control_right = m_api_control_right; }

  int16_t left = control_one(1u, control_left, command_left, rpm_left, inner_mode_left);
  int16_t right_host = control_one(2u, control_right, command_right, rpm_right, inner_mode_right);

  if (control_left == MCCONF_CONTROL_POSITION) {
    int32_t target = p_pid_set_counts;
    if (target < p_pid_min_counts) target = p_pid_min_counts;
    if (target > p_pid_max_counts) target = p_pid_max_counts;
    int32_t e32 = target - m_encoder_position;
    if (e32 > 32767) e32 = 32767;
    if (e32 < -32768) e32 = -32768;
    int16_t err = (int16_t)e32;
    if (abs(err) <= p_pid_deadband_counts) err = 0;
    m_position_speed_set = pid_step(&m_position_pid, err, p_pid_kp_x1000, p_pid_ki_x1000, p_pid_kd_x1000,
                                    p_pid_i_limit_rpm, p_pid_speed_limit_rpm, s_pid_d_filter_x1000);
    left = control_one(1u, MCCONF_CONTROL_SPEED, m_position_speed_set, rpm_left, inner_mode_left);
  } else {
    m_position_speed_set = 0;
  }

  *motor_target_left = left;
  *motor_target_right = (int16_t)-right_host; /* physical Right motor is mirrored */
}

uint16_t mc_interface_pack_mode_word(void) {
  return (uint16_t)((m_control_mode_sel_left & 0x7u) |
                    ((m_control_mode_sel_right & 0x7u) << 3) |
                    ((m_comm_mode_left & 0x3u) << 6) |
                    ((m_comm_mode_right & 0x3u) << 8) |
                    ((m_sensor_mode_left & 0x3u) << 10) |
                    ((m_sensor_mode_right & 0x3u) << 12));
}

static uint8_t parse_side(const char *s) {
  if (!s || !*s) return 0u;
  if (strcmp(s, "left") == 0 || strcmp(s, "l") == 0) return 1u;
  if (strcmp(s, "right") == 0 || strcmp(s, "r") == 0) return 2u;
  return 255u;
}

int mc_interface_debug_command(const char *line) {
  if (!line) return 0;
  char buf[80];
  size_t n = strlen(line); if (n >= sizeof(buf)) n = sizeof(buf) - 1u;
  for (size_t i = 0; i < n; ++i) buf[i] = (char)tolower((unsigned char)line[i]);
  buf[n] = 0;
  while (n && (buf[n-1] == '\r' || buf[n-1] == '\n' || buf[n-1] == ' ')) buf[--n] = 0;

  if (strncmp(buf, "live ", 5) == 0) {
    if (strcmp(buf + 5, "on") == 0) { mc_interface_set_live(true); printf("# LIVE ON\r\n"); return 1; }
    if (strcmp(buf + 5, "off") == 0) { mc_interface_set_live(false); printf("# LIVE OFF\r\n"); return 1; }
    printf("! LIVE expects ON or OFF\r\n"); return 1;
  }

  if (strncmp(buf, "mode ", 5) != 0) return 0;
  char *tok = strtok(buf + 5, " ");
  if (!tok) return 1;
  char *kind = tok;
  char *a = strtok(NULL, " ");
  char *b = strtok(NULL, " ");
  uint8_t side = 0u;
  char *value_s = a;
  if (a && (strcmp(a,"left") == 0 || strcmp(a,"right") == 0 || strcmp(a,"l") == 0 || strcmp(a,"r") == 0)) {
    side = parse_side(a); value_s = b;
  }
  if (!value_s) { printf("! MODE missing value\r\n"); return 1; }
  const int value = atoi(value_s);
  bool ok = false;
  if (strcmp(kind, "sensor") == 0) ok = mc_interface_set_sensor_mode(side, (uint8_t)value);
  else if (strcmp(kind, "comm") == 0) ok = mc_interface_set_comm_mode(side, (uint8_t)value);
  else if (strcmp(kind, "control") == 0 || strcmp(kind, "controll") == 0) ok = mc_interface_set_control_mode(side, (uint8_t)value);
  else { printf("! MODE type: sensor|comm|control\r\n"); return 1; }
  if (!ok) {
    printf("! MODE rejected (sensor: L 1/2/3 R 1/2; comm 1/2/3; control 1/2/3; control 4 requires explicit Left + encoder AB)\r\n");
  } else {
    printf("# MODE sensor L:%u R:%u comm L:%u R:%u control L:%u R:%u\r\n",
           m_sensor_mode_left, m_sensor_mode_right, m_comm_mode_left, m_comm_mode_right,
           m_control_mode_sel_left, m_control_mode_sel_right);
  }
  return 1;
}

void encoder_left_init(void) {
  m_encoder_prev_hw = (uint16_t)__HAL_TIM_GET_COUNTER(&htim_encoder_left);
  m_encoder_raw_extended = m_encoder_prev_hw;
  m_encoder_zero_raw = m_encoder_raw_extended;
  m_encoder_position = 0;
  m_encoder_last_position = 0;
  m_encoder_last_ms = HAL_GetTick();
  m_encoder_rpm = 0;
  m_encoder_sync_state = 0;
  m_encoder_sync_ok = 0;
}

void encoder_left_update(void) {
  if (m_sensor_mode_left != MCCONF_SENSOR_ENCODER_AB) { m_encoder_rpm = 0; return; }
  const uint16_t now = (uint16_t)__HAL_TIM_GET_COUNTER(&htim_encoder_left);
  const int16_t delta_hw = (int16_t)(now - m_encoder_prev_hw);
  m_encoder_prev_hw = now;
  m_encoder_raw_extended += (int32_t)delta_hw * (m_encoder_direction >= 0 ? 1 : -1);
  m_encoder_position = m_encoder_raw_extended - m_encoder_zero_raw;
  const uint32_t ms = HAL_GetTick();
  const uint32_t dt = ms - m_encoder_last_ms;
  if (dt >= 10u && m_encoder_counts > 0u) {
    const int32_t dp = m_encoder_position - m_encoder_last_position;
    int32_t rpm = (dp * 60000L) / ((int32_t)m_encoder_counts * (int32_t)dt);
    if (rpm > 32767) rpm = 32767;
    if (rpm < -32768) rpm = -32768;
    m_encoder_rpm = (int16_t)rpm;
    m_encoder_last_position = m_encoder_position;
    m_encoder_last_ms = ms;
  }
}

int16_t encoder_left_mechanical_angle_x16(void) {
  if (m_encoder_counts == 0u) return m_encoder_mech_offset_x16;
  int32_t mod = m_encoder_position % (int32_t)m_encoder_counts;
  if (mod < 0) mod += m_encoder_counts;
  int32_t mech = (mod * 5760L) / (int32_t)m_encoder_counts;
  mech += m_encoder_mech_offset_x16;
  if (m_encoder_pole_pairs == 0u) m_encoder_pole_pairs = 1u;
  mech += (((int32_t)m_encoder_elec_trim_deg_x10 * 16) / 10) / (int32_t)m_encoder_pole_pairs;
  while (mech >= 5760) mech -= 5760;
  while (mech < 0) mech += 5760;
  int32_t elec = mech * m_encoder_pole_pairs - 480;
  elec %= 5760; if (elec < 0) elec += 5760;
  m_encoder_elec_angle_deg_x10 = (int16_t)((elec * 10) / 16);
  return (int16_t)mech;
}

uint8_t encoder_left_synthetic_hall(void) {
  (void)encoder_left_mechanical_angle_x16();
  int32_t a = m_encoder_elec_angle_deg_x10;
  while (a < 0) a += 3600;
  while (a >= 3600) a -= 3600;
  static const uint8_t hall_by_sector[6] = {2u, 3u, 1u, 5u, 4u, 6u};
  return hall_by_sector[(uint8_t)(a / 600)];
}

uint8_t encoder_sync_active(void) { return (m_encoder_sync_state == 2u || m_encoder_sync_state == 3u); }
void encoder_sync_request(void) {
  if (m_sensor_mode_left != MCCONF_SENSOR_ENCODER_AB) return;
  m_encoder_sync_ok = 0; m_encoder_sync_state = 1u; m_sync_started_ms = HAL_GetTick();
  m_sync_initial_position = m_encoder_position; m_sync_hold_phase = 0u;
}

void encoder_sync_service(uint8_t calibration_active) {
  if (m_sensor_mode_left != MCCONF_SENSOR_ENCODER_AB) return;
  encoder_left_update();
  const uint32_t now = HAL_GetTick();
  if (m_encoder_sync_state == 0u || m_encoder_sync_state == 5u || m_encoder_sync_state == 6u) return;
  if (m_encoder_sync_state == 1u) { if (!calibration_active) { m_sync_started_ms = now; m_encoder_sync_state = 2u; } return; }
  if (m_encoder_sync_state == 2u) {
    if ((now - m_sync_started_ms) >= m_encoder_sync_sweep_ms) {
      int32_t moved = m_encoder_position - m_sync_initial_position;
      if (labs((long)moved) < 4L) { m_encoder_sync_state = 6u; return; }
      if (moved < 0) { m_encoder_direction = (int16_t)-m_encoder_direction; m_encoder_raw_extended = m_encoder_zero_raw - (m_encoder_raw_extended - m_encoder_zero_raw); m_encoder_position = -m_encoder_position; m_sync_initial_position = -m_sync_initial_position; }
      m_sync_started_ms = now; m_encoder_sync_state = 3u;
    }
    return;
  }
  if (m_encoder_sync_state == 3u && (now - m_sync_started_ms) >= m_encoder_sync_settle_ms) {
    const int32_t return_target = m_sync_initial_position - m_encoder_position;
    m_encoder_zero_raw = m_encoder_raw_extended; m_encoder_position = 0;
    if (m_encoder_pole_pairs == 0u) m_encoder_pole_pairs = 1u;
    m_encoder_mech_offset_x16 = (int16_t)(480 / (int32_t)m_encoder_pole_pairs);
    p_pid_set_counts = clamp_s16_local(return_target, p_pid_min_counts, p_pid_max_counts);
    mc_interface_reset_control(); m_encoder_sync_state = 4u; m_sync_started_ms = now; return;
  }
  if (m_encoder_sync_state == 4u) {
    if (labs((long)p_pid_set_counts - (long)m_encoder_position) <= m_encoder_return_tolerance_counts) { m_encoder_sync_ok = 1u; m_encoder_sync_state = 5u; }
    else if ((now - m_sync_started_ms) > 8000u) { m_encoder_sync_state = 6u; m_encoder_sync_ok = 0u; }
  }
}
int16_t encoder_sync_svpwm_command(void) { return m_encoder_sync_state == 2u ? m_encoder_sync_command : 0; }
uint8_t encoder_sync_hold_vector(void) { return m_encoder_sync_state == 3u; }
uint16_t encoder_sync_hold_phase(void) { return m_sync_hold_phase; }
