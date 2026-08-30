/*
 * Hoverboard dual-motor control core, refactored to VESC mcpwm_foc style.
 * LUT sin, fixed-point Clarke/Park, SVPWM, commutation, ADC calibration and the
 * validated DMA1 current-control ISR live in this single module.
 */
#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"
#include "util.h"
#include "motor/mcpwm_foc.h"
#include "motor/mc_interface.h"
#include <limits.h>
#include <stdbool.h>
#include <string.h>

/*
 * Static lookup/configuration data for the hand-written controller.
 * The three 181-point phase tables are retained from the proven firmware so
 * COMMUTATION and SINE algorithms keep their previous waveform behavior.
 */

const int8_t m_hall_to_sector[8] = {
  0, 2, 0, 1, 4, 3, 5, 0
};

const int8_t m_commutation_map[18] = {
  -1, 1, 0, -1, 0, 1, 0, -1, 1,
  1, -1, 0, 1, 0, -1, 0, 1, -1
};

const int16_t m_sine_phase_a_q14[181] = {
  -13091, -13634, -14126, -14565, -14953, -15289, -15577, -15816, -16009, -16159,
  -16269, -16340, -16377, -16383, -16362, -16317, -16253, -16172, -16079, -15977,
  -15870, -15762, -15656, -15555, -15461, -15377, -15306, -15248, -15206, -15180,
  -15172, -15180, -15206, -15248, -15306, -15377, -15461, -15555, -15656, -15762,
  -15870, -15977, -16079, -16172, -16253, -16317, -16362, -16383, -16377, -16340,
  -16269, -16159, -16009, -15816, -15577, -15289, -14953, -14565, -14126, -13634,
  -13091, -12496, -11849, -11154, -10411, -9623, -8791, -7921, -7014, -6075,
  -5107, -4115, -3104, -2077, -1041, 0, 1041, 2077, 3104, 4115,
  5107, 6075, 7014, 7921, 8791, 9623, 10411, 11154, 11849, 12496,
  13091, 13634, 14126, 14565, 14953, 15289, 15577, 15816, 16009, 16159,
  16269, 16340, 16377, 16383, 16362, 16317, 16253, 16172, 16079, 15977,
  15870, 15762, 15656, 15555, 15461, 15377, 15306, 15248, 15206, 15180,
  15172, 15180, 15206, 15248, 15306, 15377, 15461, 15555, 15656, 15762,
  15870, 15977, 16079, 16172, 16253, 16317, 16362, 16383, 16377, 16340,
  16269, 16159, 16009, 15816, 15577, 15289, 14953, 14565, 14126, 13634,
  13091, 12496, 11849, 11154, 10411, 9623, 8791, 7921, 7014, 6075,
  5107, 4115, 3104, 2077, 1041, 0, -1041, -2077, -3104, -4115,
  -5107, -6075, -7014, -7921, -8791, -9623, -10411, -11154, -11849, -12496,
  -13091
};

const int16_t m_sine_phase_b_q14[181] = {
  15172, 15180, 15206, 15248, 15306, 15377, 15461, 15555, 15656, 15762,
  15870, 15977, 16079, 16172, 16253, 16317, 16362, 16383, 16377, 16340,
  16269, 16159, 16009, 15816, 15577, 15289, 14953, 14565, 14126, 13634,
  13091, 12496, 11849, 11154, 10411, 9623, 8791, 7921, 7014, 6075,
  5107, 4115, 3104, 2077, 1041, 0, -1041, -2077, -3104, -4115,
  -5107, -6075, -7014, -7921, -8791, -9623, -10411, -11154, -11849, -12496,
  -13091, -13634, -14126, -14565, -14953, -15289, -15577, -15816, -16009, -16159,
  -16269, -16340, -16377, -16383, -16362, -16317, -16253, -16172, -16079, -15977,
  -15870, -15762, -15656, -15555, -15461, -15377, -15306, -15248, -15206, -15180,
  -15172, -15180, -15206, -15248, -15306, -15377, -15461, -15555, -15656, -15762,
  -15870, -15977, -16079, -16172, -16253, -16317, -16362, -16383, -16377, -16340,
  -16269, -16159, -16009, -15816, -15577, -15289, -14953, -14565, -14126, -13634,
  -13091, -12496, -11849, -11154, -10411, -9623, -8791, -7921, -7014, -6075,
  -5107, -4115, -3104, -2077, -1041, 0, 1041, 2077, 3104, 4115,
  5107, 6075, 7014, 7921, 8791, 9623, 10411, 11154, 11849, 12496,
  13091, 13634, 14126, 14565, 14953, 15289, 15577, 15816, 16009, 16159,
  16269, 16340, 16377, 16383, 16362, 16317, 16253, 16172, 16079, 15977,
  15870, 15762, 15656, 15555, 15461, 15377, 15306, 15248, 15206, 15180,
  15172
};

const int16_t m_sine_phase_c_q14[181] = {
  -13091, -12496, -11849, -11154, -10411, -9623, -8791, -7921, -7014, -6075,
  -5107, -4115, -3104, -2077, -1041, 0, 1041, 2077, 3104, 4115,
  5107, 6075, 7014, 7921, 8791, 9623, 10411, 11154, 11849, 12496,
  13091, 13634, 14126, 14565, 14953, 15289, 15577, 15816, 16009, 16159,
  16269, 16340, 16377, 16383, 16362, 16317, 16253, 16172, 16079, 15977,
  15870, 15762, 15656, 15555, 15461, 15377, 15306, 15248, 15206, 15180,
  15172, 15180, 15206, 15248, 15306, 15377, 15461, 15555, 15656, 15762,
  15870, 15977, 16079, 16172, 16253, 16317, 16362, 16383, 16377, 16340,
  16269, 16159, 16009, 15816, 15577, 15289, 14953, 14565, 14126, 13634,
  13091, 12496, 11849, 11154, 10411, 9623, 8791, 7921, 7014, 6075,
  5107, 4115, 3104, 2077, 1041, 0, -1041, -2077, -3104, -4115,
  -5107, -6075, -7014, -7921, -8791, -9623, -10411, -11154, -11849, -12496,
  -13091, -13634, -14126, -14565, -14953, -15289, -15577, -15816, -16009, -16159,
  -16269, -16340, -16377, -16383, -16362, -16317, -16253, -16172, -16079, -15977,
  -15870, -15762, -15656, -15555, -15461, -15377, -15306, -15248, -15206, -15180,
  -15172, -15180, -15206, -15248, -15306, -15377, -15461, -15555, -15656, -15762,
  -15870, -15977, -16079, -16172, -16253, -16317, -16362, -16383, -16377, -16340,
  -16269, -16159, -16009, -15816, -15577, -15289, -14953, -14565, -14126, -13634,
  -13091
};

const int16_t m_sin_q15[256] = {
  0, 804, 1608, 2410, 3212, 4011, 4808, 5602, 6393, 7179, 7962, 8739,
  9512, 10278, 11039, 11793, 12539, 13279, 14010, 14732, 15446, 16151, 16846, 17530,
  18204, 18868, 19519, 20159, 20787, 21403, 22005, 22594, 23170, 23731, 24279, 24811,
  25329, 25832, 26319, 26790, 27245, 27683, 28105, 28510, 28898, 29268, 29621, 29956,
  30273, 30571, 30852, 31113, 31356, 31580, 31785, 31971, 32137, 32285, 32412, 32521,
  32609, 32678, 32728, 32757, 32767, 32757, 32728, 32678, 32609, 32521, 32412, 32285,
  32137, 31971, 31785, 31580, 31356, 31113, 30852, 30571, 30273, 29956, 29621, 29268,
  28898, 28510, 28105, 27683, 27245, 26790, 26319, 25832, 25329, 24811, 24279, 23731,
  23170, 22594, 22005, 21403, 20787, 20159, 19519, 18868, 18204, 17530, 16846, 16151,
  15446, 14732, 14010, 13279, 12539, 11793, 11039, 10278, 9512, 8739, 7962, 7179,
  6393, 5602, 4808, 4011, 3212, 2410, 1608, 804, 0, -804, -1608, -2410,
  -3212, -4011, -4808, -5602, -6393, -7179, -7962, -8739, -9512, -10278, -11039, -11793,
  -12539, -13279, -14010, -14732, -15446, -16151, -16846, -17530, -18204, -18868, -19519, -20159,
  -20787, -21403, -22005, -22594, -23170, -23731, -24279, -24811, -25329, -25832, -26319, -26790,
  -27245, -27683, -28105, -28510, -28898, -29268, -29621, -29956, -30273, -30571, -30852, -31113,
  -31356, -31580, -31785, -31971, -32137, -32285, -32412, -32521, -32609, -32678, -32728, -32757,
  -32767, -32757, -32728, -32678, -32609, -32521, -32412, -32285, -32137, -31971, -31785, -31580,
  -31356, -31113, -30852, -30571, -30273, -29956, -29621, -29268, -28898, -28510, -28105, -27683,
  -27245, -26790, -26319, -25832, -25329, -24811, -24279, -23731, -23170, -22594, -22005, -21403,
  -20787, -20159, -19519, -18868, -18204, -17530, -16846, -16151, -15446, -14732, -14010, -13279,
  -12539, -11793, -11039, -10278, -9512, -8739, -7962, -7179, -6393, -5602, -4808, -4011,
  -3212, -2410, -1608, -804
};

mc_configuration m_mcconf_1 = {
  .l_current_max = (I_MOT_MAX * A2BIT_CONV) << 4,
  .l_max_rpm = N_MOT_MAX << 4,

  .foc_current_kp_q = 1229u,
  .foc_current_ki_q = 1229u,
  .foc_current_kp_d = 819u,
  .foc_current_ki_d = 737u,
  .foc_current_filter_const = 7864u,
  .foc_current_anti_windup = 768u,
  .foc_current_i_limit = 737u,

  .s_pid_kp = 4833u,
  .s_pid_ki = 251u,
  .s_pid_i_limit = 246u,

  .foc_comm_rpm_low = 15 << 4,
  .foc_comm_rpm_high = 30 << 4,
  .foc_fw_enable = FIELD_WEAK_ENA,
  .foc_fw_current_max = (FIELD_WEAK_MAX * A2BIT_CONV) << 4,
  .foc_fw_phase_advance_max = PHASE_ADV_MAX << 4,
  .foc_fw_rpm_start = FIELD_WEAK_HI << 4,
  .foc_fw_rpm_end = FIELD_WEAK_LO << 4,

  .si_motor_pole_pairs = SVPWM_POLE_PAIRS,
  .motor_type = MOTOR_TYPE_FOC,
  .foc_current_sample_map = 0u,
  .foc_encoder_enable = false,
  .m_diag_enable = DIAG_ENA
};

mc_configuration m_mcconf_2;
motor_all_state_t m_motor_1;
motor_all_state_t m_motor_2;


/* ========================= mcpwm_foc control ========================= */
#define FOC_VOLTAGE_MAX_RAW          14400
#define FOC_PHASE_Q16_60_DEG         10923u
#define FOC_PHASE_Q16_120_DEG        21845u
#define FOC_PHASE_Q16_240_DEG        43691u
#define FOC_ONE_BY_SQRT3_Q15         18919
#define FOC_SQRT3_BY_2_Q15           28378

static int16_t clamp_s16(int32_t value) {
  return foc_math_clamp_s16(value);
}

static int16_t clamp_s16_range(int32_t value, int16_t min_value, int16_t max_value) {
  return foc_math_clamp_s16_range(value, min_value, max_value);
}

static int16_t sin_q15(uint16_t phase) {
  return m_sin_q15[(uint8_t)(phase >> 8)];
}

static uint16_t phase_from_degrees_x16(int32_t degrees_x16) {
  return foc_math_phase_from_degrees_x16(degrees_x16);
}

static uint16_t hall_phase(motor_all_state_t *motor, uint8_t hall_code) {
  hall_state_t *hall = &motor->m_hall;
  const int8_t sector = m_hall_to_sector[hall_code & 7u];

  if (hall_code == 0u || hall_code == 7u || sector < 0 || sector > 5) {
    motor->m_speed_est_fast = 0;
    return motor->m_phase_now;
  }

  if (!hall->initialized) {
    hall->initialized = true;
    hall->sector = sector;
    hall->direction = 1;
    hall->ticks = 0u;
    hall->period = 0u;
    motor->m_speed_est_fast = 0;
  } else {
    if (hall->ticks < UINT16_MAX) hall->ticks++;

    if (sector != hall->sector) {
      const uint8_t diff = (uint8_t)((sector - hall->sector + 6) % 6);
      if (diff == 1u) hall->direction = 1;
      else if (diff == 5u) hall->direction = -1;

      if (hall->ticks > 0u && hall->direction != 0 && motor->m_conf->si_motor_pole_pairs > 0u) {
        hall->period = hall->ticks;
        const uint32_t rpm_mag = ((uint32_t)PWM_FREQ * 10u) /
                                 ((uint32_t)hall->period * (uint32_t)motor->m_conf->si_motor_pole_pairs);
        motor->m_speed_est_fast = (int16_t)(hall->direction > 0 ?
                                            (int32_t)rpm_mag : -(int32_t)rpm_mag);
      }

      hall->sector = sector;
      hall->ticks = 0u;
    } else if (hall->period > 0u && (uint32_t)hall->ticks > (uint32_t)hall->period * 2u) {
      motor->m_speed_est_fast = 0;
    }
  }

  uint32_t phase = (uint32_t)sector * FOC_PHASE_Q16_60_DEG;
  if (hall->direction < 0) phase += FOC_PHASE_Q16_60_DEG;

  if (hall->period > 0u) {
    uint32_t ticks = hall->ticks;
    if (ticks > hall->period) ticks = hall->period;
    const uint32_t interp = (ticks * FOC_PHASE_Q16_60_DEG) / hall->period;
    if (hall->direction < 0) phase -= interp;
    else phase += interp;
  }

  return (uint16_t)phase;
}

static uint16_t update_phase_and_speed(motor_all_state_t *motor) {
  const motor_input_t *input = &motor->m_input;
  uint16_t phase = motor->m_phase_now;

  if (motor->m_conf->sensor_mode == MCCONF_SENSOR_OPENLOOP) {
    phase = input->phase_openloop_q16;
  } else if (motor->m_conf->sensor_mode == MCCONF_SENSOR_ENCODER_AB) {
    int32_t electrical_x16 = (int32_t)input->phase_encoder_deg_x16 *
                             (int32_t)(motor->m_conf->si_motor_pole_pairs ? motor->m_conf->si_motor_pole_pairs : 1u);
    /* Preserve the established hoverboard encoder convention: -30 electrical degrees. */
    electrical_x16 -= 480;
    phase = phase_from_degrees_x16(electrical_x16);
  } else {
    const uint8_t hall_code = (uint8_t)((input->hall_a << 2) | (input->hall_b << 1) | input->hall_c);
    phase = hall_phase(motor, hall_code);
  }

  /* RPM is sourced from the selected physical sensor snapshot. Open-loop has
   * no physical feedback and therefore reports the generated estimate passed
   * by the ISR instead of inventing Hall feedback. */
  motor->m_speed_est_fast = input->rpm_sensor;
  motor->m_phase_now = phase;
  motor->m_output.phase_electrical_deg = (int16_t)(((uint32_t)phase * 360u) >> 16);
  motor->m_output.rpm = input->rpm_sensor;
  return phase;
}

static int16_t lowpass_q4(int16_t input, uint16_t coefficient, int32_t *state) {
  int32_t error = (int32_t)input - (*state >> 16);
  if (error > INT16_MAX) error = INT16_MAX;
  if (error < INT16_MIN) error = INT16_MIN;
  *state += (int32_t)coefficient * error;
  return clamp_s16(*state >> 16);
}

static void current_clarke_park(motor_all_state_t *motor, uint16_t phase) {
  const mc_configuration *conf = motor->m_conf;
  foc_state_t *state = &motor->m_motor_state;

  const int16_t s = sin_q15(phase);
  const int16_t c = sin_q15((uint16_t)(phase + 16384u));
  int16_t iq_raw = 0;
  int16_t id_raw = 0;
  foc_math_clarke_park_q4(motor->m_input.current_adc_1, motor->m_input.current_adc_2,
                          conf->foc_current_sample_map, s, c, &iq_raw, &id_raw);

  state->iq = lowpass_q4(iq_raw, conf->foc_current_filter_const, &state->iq_filter_state);
  state->id = lowpass_q4(id_raw, conf->foc_current_filter_const, &state->id_filter_state);
  motor->m_output.iq = state->iq;
  motor->m_output.id = state->id;
}

static void pi_reset(foc_pi_state_t *pi) {
  pi->integrator = 0;
  pi->saturated = false;
}

/* Keep the proven fixed-point coefficient scaling while using a compact
 * VESC-style current-controller state machine. */
static int16_t current_pi_step(foc_pi_state_t *pi, int16_t error,
                               uint16_t kp, uint16_t ki,
                               int16_t min_output, int16_t max_output) {
  const int32_t delta_i = (int32_t)error * (int32_t)ki;
  if (!pi->saturated) {
    if (delta_i > 0 && pi->integrator > INT32_MAX - delta_i) pi->integrator = INT32_MAX;
    else if (delta_i < 0 && pi->integrator < INT32_MIN - delta_i) pi->integrator = INT32_MIN;
    else pi->integrator += delta_i;
  }

  int32_t p_term = ((int32_t)error * (int32_t)kp) >> 11;
  p_term = clamp_s16(p_term);
  const int32_t i_term = pi->integrator >> 16;
  const int32_t raw_output = i_term + (p_term >> 1);
  const int16_t output = clamp_s16_range(raw_output, min_output, max_output);

  if (output != raw_output) {
    const int32_t error_sign = (error > 0) - (error < 0);
    const int32_t output_sign = (raw_output > 0) - (raw_output < 0);
    pi->saturated = error_sign != 0 && error_sign == output_sign;
  } else {
    pi->saturated = false;
  }

  return output;
}

static int16_t field_weakening_target(const motor_all_state_t *motor) {
  const mc_configuration *conf = motor->m_conf;
  if (!conf->foc_fw_enable) return 0;

  int32_t rpm = motor->m_speed_est_fast;
  if (rpm < 0) rpm = -rpm;
  const int32_t start_rpm = conf->foc_fw_rpm_end >> 4;
  const int32_t full_rpm = conf->foc_fw_rpm_start >> 4;
  if (rpm <= start_rpm || full_rpm <= start_rpm) return 0;

  int32_t fw = conf->foc_fw_current_max;
  if (rpm < full_rpm) fw = (fw * (rpm - start_rpm)) / (full_rpm - start_rpm);
  if (fw < 0) fw = -fw;
  return clamp_s16(-fw);
}

static void svm_from_dq(int16_t vd, int16_t vq, uint16_t phase,
                        int16_t *duty_a, int16_t *duty_b, int16_t *duty_c) {
  const int16_t s = sin_q15(phase);
  const int16_t c = sin_q15((uint16_t)(phase + 16384u));
  int32_t v_alpha = 0;
  int32_t v_beta = 0;
  foc_math_inv_park_q15(vd, vq, s, c, &v_alpha, &v_beta);

  int32_t a = v_alpha;
  int32_t b = -(v_alpha >> 1) + ((FOC_SQRT3_BY_2_Q15 * v_beta) >> 15);
  int32_t c_phase = -(v_alpha >> 1) - ((FOC_SQRT3_BY_2_Q15 * v_beta) >> 15);

  int32_t vmax = a;
  if (b > vmax) vmax = b;
  if (c_phase > vmax) vmax = c_phase;
  int32_t vmin = a;
  if (b < vmin) vmin = b;
  if (c_phase < vmin) vmin = c_phase;
  const int32_t common = (vmax + vmin) >> 1;

  a -= common;
  b -= common;
  c_phase -= common;

  *duty_a = clamp_s16_range(a >> 4, -1000, 1000);
  *duty_b = clamp_s16_range(b >> 4, -1000, 1000);
  *duty_c = clamp_s16_range(c_phase >> 4, -1000, 1000);
}

static void control_foc(motor_all_state_t *motor, uint16_t phase) {
  mc_configuration *conf = motor->m_conf;
  foc_state_t *state = &motor->m_motor_state;
  const int16_t setpoint = motor->m_input.control_setpoint;

  current_clarke_park(motor, phase);

  if (setpoint == 0) {
    state->iq_target = 0;
    state->id_target = 0;
    state->vq = 0;
    state->vd = 0;
    pi_reset(&motor->m_iq_pi);
    pi_reset(&motor->m_id_pi);
    motor->m_output.duty_a = 0;
    motor->m_output.duty_b = 0;
    motor->m_output.duty_c = 0;
    return;
  }

  if (motor->m_control_mode == CONTROL_MODE_DUTY || motor->m_control_mode == CONTROL_MODE_NONE) {
    const int32_t vq = ((int32_t)setpoint * FOC_VOLTAGE_MAX_RAW) / 1000;
    state->iq_target = 0;
    state->id_target = 0;
    state->vq = clamp_s16_range(vq, -FOC_VOLTAGE_MAX_RAW, FOC_VOLTAGE_MAX_RAW);
    state->vd = 0;
    pi_reset(&motor->m_iq_pi);
    pi_reset(&motor->m_id_pi);
  } else {
    int32_t iq_target = ((int32_t)setpoint * conf->l_current_max) / 1000;
    iq_target = clamp_s16_range(iq_target, (int16_t)-conf->l_current_max, conf->l_current_max);
    state->iq_target = (int16_t)iq_target;
    state->id_target = field_weakening_target(motor);

    const int16_t iq_error = clamp_s16((int32_t)state->iq_target - state->iq);
    const int16_t id_error = clamp_s16((int32_t)state->id_target - state->id);
    state->vq = current_pi_step(&motor->m_iq_pi, iq_error,
                                conf->foc_current_kp_q, conf->foc_current_ki_q,
                                -FOC_VOLTAGE_MAX_RAW, FOC_VOLTAGE_MAX_RAW);
    state->vd = current_pi_step(&motor->m_id_pi, id_error,
                                conf->foc_current_kp_d, conf->foc_current_ki_d,
                                -FOC_VOLTAGE_MAX_RAW, FOC_VOLTAGE_MAX_RAW);

    /* Circular voltage limit with d-axis priority, matching mcpwm_foc intent. */
    const int32_t vd_abs = state->vd < 0 ? -(int32_t)state->vd : state->vd;
    int32_t vq_limit = FOC_VOLTAGE_MAX_RAW - (vd_abs >> 1);
    if (vq_limit < 0) vq_limit = 0;
    state->vq = clamp_s16_range(state->vq, (int16_t)-vq_limit, (int16_t)vq_limit);
  }

  svm_from_dq(state->vd, state->vq, phase,
              &motor->m_output.duty_a, &motor->m_output.duty_b, &motor->m_output.duty_c);
}

static int16_t nonfoc_drive_command(motor_all_state_t *motor) {
  const int16_t setpoint = clamp_s16_range(motor->m_input.control_setpoint, -1000, 1000);
  if (motor->m_control_mode != CONTROL_MODE_CURRENT) return setpoint;
  if (setpoint == 0) {
    pi_reset(&motor->m_speed_pi);
    return 0;
  }

  /* Six-step and sine still honor CONTROL_CURRENT / outer SPEED PID. Estimate
   * phase-current magnitude from the two measured shunts and regulate waveform
   * amplitude. The sign remains the requested torque direction. */
  const int32_t i1 = motor->m_input.current_adc_1;
  const int32_t i2 = motor->m_input.current_adc_2;
  const int32_t i3 = -i1 - i2;
  int32_t mag = i1 < 0 ? -i1 : i1;
  const int32_t a2 = i2 < 0 ? -i2 : i2;
  const int32_t a3 = i3 < 0 ? -i3 : i3;
  if (a2 > mag) mag = a2;
  if (a3 > mag) mag = a3;
  const int32_t target_q4 = ((int32_t)(setpoint < 0 ? -setpoint : setpoint) * motor->m_conf->l_current_max) / 1000;
  const int16_t error_q4 = clamp_s16(target_q4 - (mag << 4));
  const int16_t voltage = current_pi_step(&motor->m_speed_pi, error_q4,
                                          motor->m_conf->foc_current_kp_q,
                                          motor->m_conf->foc_current_ki_q,
                                          0, FOC_VOLTAGE_MAX_RAW);
  int32_t amplitude = ((int32_t)voltage * 1000) / FOC_VOLTAGE_MAX_RAW;
  if (amplitude > 1000) amplitude = 1000;
  return (int16_t)(setpoint < 0 ? -amplitude : amplitude);
}

static uint8_t sine_table_index(uint16_t phase) {
  const uint32_t degree = ((uint32_t)phase * 360u) >> 16;
  uint32_t index = degree >> 1;
  if (index > 180u) index = 180u;
  return (uint8_t)index;
}

static void control_sine(motor_all_state_t *motor, uint16_t phase) {
  const int16_t command = nonfoc_drive_command(motor);
  if (command == 0) {
    motor->m_output.duty_a = motor->m_output.duty_b = motor->m_output.duty_c = 0;
    return;
  }

  const uint8_t index = sine_table_index(phase);
  motor->m_output.duty_a = clamp_s16_range(((int32_t)command * m_sine_phase_a_q14[index]) >> 14, -1000, 1000);
  motor->m_output.duty_b = clamp_s16_range(((int32_t)command * m_sine_phase_b_q14[index]) >> 14, -1000, 1000);
  motor->m_output.duty_c = clamp_s16_range(((int32_t)command * m_sine_phase_c_q14[index]) >> 14, -1000, 1000);
}

static void control_commutation(motor_all_state_t *motor) {
  const uint8_t hall_code = (uint8_t)((motor->m_input.hall_a << 2) |
                                      (motor->m_input.hall_b << 1) |
                                      motor->m_input.hall_c);
  const int8_t sector = m_hall_to_sector[hall_code & 7u];
  const int16_t command = nonfoc_drive_command(motor);

  if (command == 0 || hall_code == 0u || hall_code == 7u || sector < 0 || sector > 5) {
    motor->m_output.duty_a = motor->m_output.duty_b = motor->m_output.duty_c = 0;
    return;
  }

  const int8_t *map = &m_commutation_map[(uint8_t)sector * 3u];
  motor->m_output.duty_a = (int16_t)((int32_t)command * map[0]);
  motor->m_output.duty_b = (int16_t)((int32_t)command * map[1]);
  motor->m_output.duty_c = (int16_t)((int32_t)command * map[2]);
}

void mcpwm_foc_reset(motor_all_state_t *motor) {
  if (motor == NULL) return;

  motor->m_state = MC_STATE_OFF;
  motor->m_control_mode = CONTROL_MODE_NONE;
  memset(&motor->m_input, 0, sizeof(motor->m_input));
  memset(&motor->m_output, 0, sizeof(motor->m_output));
  memset(&motor->m_motor_state, 0, sizeof(motor->m_motor_state));
  memset(&motor->m_iq_pi, 0, sizeof(motor->m_iq_pi));
  memset(&motor->m_id_pi, 0, sizeof(motor->m_id_pi));
  memset(&motor->m_speed_pi, 0, sizeof(motor->m_speed_pi));
  memset(&motor->m_hall, 0, sizeof(motor->m_hall));
  motor->m_speed_est_fast = 0;
  motor->m_phase_now = 0u;
  motor->m_last_motor_type = 0xffu;
}

void mcpwm_foc_init_defaults(void) {
  /* Keep the proven fixed-point defaults and EEPROM unit conventions in the
   * motor module. EEPROM is loaded afterwards and may override these values. */
  m_mcconf_1.foc_encoder_enable = 0u;
  m_mcconf_1.sensor_mode = MCCONF_DEFAULT_SENSOR_LEFT;
  m_mcconf_1.comm_mode = MCCONF_DEFAULT_COMM_LEFT;
  m_mcconf_1.si_motor_pole_pairs = SVPWM_POLE_PAIRS;
  m_mcconf_1.foc_current_sample_map = 0u;
  m_mcconf_1.motor_type = MOTOR_TYPE_FOC;
  m_mcconf_1.m_diag_enable = DIAG_ENA;
  m_mcconf_1.l_current_max = (I_MOT_MAX * A2BIT_CONV) << 4;
  m_mcconf_1.l_max_rpm = N_MOT_MAX << 4;
  m_mcconf_1.foc_fw_enable = FIELD_WEAK_ENA;
  m_mcconf_1.foc_fw_current_max = (FIELD_WEAK_MAX * A2BIT_CONV) << 4;
  m_mcconf_1.foc_fw_phase_advance_max = PHASE_ADV_MAX << 4;
  m_mcconf_1.foc_fw_rpm_start = FIELD_WEAK_HI << 4;
  m_mcconf_1.foc_fw_rpm_end = FIELD_WEAK_LO << 4;

  m_mcconf_2 = m_mcconf_1;
  m_mcconf_2.sensor_mode = MCCONF_DEFAULT_SENSOR_RIGHT;
  m_mcconf_2.comm_mode = MCCONF_DEFAULT_COMM_RIGHT;
  m_mcconf_2.foc_encoder_enable = 0u;
  m_mcconf_2.foc_current_sample_map = 1u;

  mcpwm_foc_init(&m_mcconf_1, &m_mcconf_2);
}

void mcpwm_foc_init(mc_configuration *conf_m1, mc_configuration *conf_m2) {
  mcpwm_foc_reset(&m_motor_1);
  mcpwm_foc_reset(&m_motor_2);
  m_motor_1.m_conf = conf_m1;
  m_motor_2.m_conf = conf_m2;
}

void mcpwm_foc_sensor_state_reset(uint8_t is_second_motor) {
  motor_all_state_t *motor = is_second_motor ? &m_motor_2 : &m_motor_1;
  memset(&motor->m_hall, 0, sizeof(motor->m_hall));
  motor->m_speed_est_fast = 0;
  motor->m_phase_now = 0u;
  motor->m_output.rpm = 0;
  motor->m_output.phase_electrical_deg = 0;
}

void mcpwm_foc_control(motor_all_state_t *motor) {
  if (motor == NULL || motor->m_conf == NULL) return;

  mc_configuration *conf = motor->m_conf;
  motor->m_control_mode = motor->m_input.control_mode;
  const uint16_t phase = update_phase_and_speed(motor);

  const uint8_t hall_code = (uint8_t)((motor->m_input.hall_a << 2) |
                                      (motor->m_input.hall_b << 1) |
                                      motor->m_input.hall_c);
  motor->m_output.fault_code = 0u;
  if (conf->m_diag_enable && motor->m_input.enable &&
      conf->sensor_mode == MCCONF_SENSOR_HALL &&
      (hall_code == 0u || hall_code == 7u)) {
    motor->m_output.fault_code = (hall_code == 0u) ? 1u : 2u;
  }

  if (!motor->m_input.enable || motor->m_output.fault_code != 0u) {
    motor->m_state = MC_STATE_OFF;
    motor->m_output.duty_a = 0;
    motor->m_output.duty_b = 0;
    motor->m_output.duty_c = 0;
    motor->m_motor_state.iq_target = 0;
    motor->m_motor_state.id_target = 0;
    motor->m_motor_state.vq = 0;
    motor->m_motor_state.vd = 0;
    pi_reset(&motor->m_iq_pi);
    pi_reset(&motor->m_id_pi);
    return;
  }

  if (motor->m_last_motor_type != (uint8_t)conf->motor_type) {
    pi_reset(&motor->m_iq_pi);
    pi_reset(&motor->m_id_pi);
    pi_reset(&motor->m_speed_pi);
    motor->m_motor_state.iq_filter_state = 0;
    motor->m_motor_state.id_filter_state = 0;
    motor->m_last_motor_type = (uint8_t)conf->motor_type;
  }

  motor->m_state = MC_STATE_RUNNING;
  switch (conf->motor_type) {
    case MOTOR_TYPE_COMMUTATION:
      control_commutation(motor);
      break;
    case MOTOR_TYPE_SINE:
      control_sine(motor, phase);
      break;
    case MOTOR_TYPE_FOC:
    default:
      control_foc(motor, phase);
      break;
  }
}


/* ======================== hardware / ISR =========================== */
static int16_t m_pwm_margin;              /* This margin allows to have a window in the PWM signal for proper FOC Phase currents measurement */

extern volatile int16_t m_motor_target_left;
extern volatile int16_t m_motor_target_right;
extern volatile uint8_t m_control_mode_left;
extern volatile uint8_t m_control_mode_right;
static int16_t m_current_input_max  = (I_DC_MAX * A2BIT_CONV);
int16_t m_current_phase_a_left = 0, m_current_phase_b_left = 0, m_current_input_left = 0;
int16_t m_current_phase_b_right = 0, m_current_phase_c_right = 0, m_current_input_right = 0;

extern volatile adc_buf_t m_adc_buffer;

uint8_t m_buzzer_freq          = 0;
uint8_t m_buzzer_pattern       = 0;
uint8_t m_buzzer_count         = 0;
volatile uint32_t m_buzzer_timer = 0;
static uint8_t  m_buzzer_prev  = 0;
static uint8_t  m_buzzer_index   = 0;

uint8_t        m_motor_enable       = 0;        // initially motors are disabled for SAFETY

static const uint16_t m_pwm_top  = 64000000 / 2 / PWM_FREQ; // = 2000

volatile uint32_t m_foc_isr_cycles = 0;
volatile uint32_t m_foc_isr_cycles_max = 0;

/* Telemetry dq snapshots stay in the existing Q4 current-count convention
 * (16 * A2BIT_CONV units per ampere). Non-FOC modes use a measurement-only
 * Clarke/Park path so every control method exposes the same channels without
 * changing its control law. */
volatile int16_t m_foc_iq_left_q4 = 0;
volatile int16_t m_foc_iq_right_q4 = 0;
volatile int16_t m_foc_id_left_q4 = 0;
volatile int16_t m_foc_id_right_q4 = 0;
volatile uint16_t m_sensor_hall_left = 0;
volatile uint16_t m_sensor_hall_right = 0;
volatile int16_t m_sensor_rpm_left = 0;
volatile int16_t m_sensor_rpm_right = 0;
volatile uint8_t m_adc_current_valid = 0;
volatile uint8_t m_adc_current_valid_left = 0;
volatile uint8_t m_adc_current_valid_right = 0;

static inline void focIsrMonitorEnd(uint32_t start_cycles) {
  const uint32_t elapsed = DWT->CYCCNT - start_cycles;
  m_foc_isr_cycles = elapsed;
  if (elapsed > m_foc_isr_cycles_max) m_foc_isr_cycles_max = elapsed;
}

static uint16_t m_svpwm_phase_left = 0;
static uint16_t m_svpwm_phase_right = 0;

static inline int16_t svpwmSinQ15(uint16_t phase) {
  return m_sin_q15[(uint8_t)(phase >> 8)];
}

static inline uint16_t openloopPhaseAdvance(int16_t command, uint8_t pole_pairs, uint16_t *phase) {
  int32_t cmd = CLAMP((int32_t)command, -1000, 1000);
  uint32_t mag = (uint32_t)(cmd < 0 ? -cmd : cmd);
  if (pole_pairs == 0u) pole_pairs = 1u;
  if (mag == 0u) return *phase;
  const uint32_t step_at_max = ((uint32_t)N_MOT_MAX * (uint32_t)pole_pairs * 65536u) /
                               (60u * (uint32_t)PWM_FREQ);
  uint32_t step = (step_at_max * mag) / 1000u;
  if (step == 0u) step = 1u;
  *phase = (uint16_t)(cmd >= 0 ? (*phase + step) : (*phase - step));
  return *phase;
}

static inline uint8_t phaseToSyntheticHall(uint16_t phase) {
  static const uint8_t hall_by_sector[6] = {2u, 3u, 1u, 5u, 4u, 6u};
  const uint8_t sector = (uint8_t)(((uint32_t)phase * 6u) >> 16);
  return hall_by_sector[sector < 6u ? sector : 5u];
}

static inline int16_t openloopRpmEstimate(int16_t command) {
  return (int16_t)(((int32_t)CLAMP((int32_t)command, -1000, 1000) * N_MOT_MAX) / 1000);
}

static inline int16_t clampS16FromS32(int32_t x) {
  if (x > 32767) return 32767;
  if (x < -32768) return -32768;
  return (int16_t)x;
}

/* Measurement-only Clarke/Park for SVPWM/COM/SIN telemetry. Clarke equations
 * and Q4 scaling match the FOC core. theta is a Q16 electrical revolution:
 * 0..65535 => 0..360 deg. */
typedef struct {
  int32_t iq_state;
  int32_t id_state;
} dq_filter_state_t;

typedef struct {
  uint8_t initialized;
  int8_t sector;
  int8_t direction;
  uint16_t ticks;
  uint16_t period;
  int16_t rpm;
} hall_speed_state_t;

static dq_filter_state_t m_dq_filter_left = {0, 0};
static dq_filter_state_t m_dq_filter_right = {0, 0};
static hall_speed_state_t m_hall_speed_left = {0, 0, 0, 0, 0, 0};
static hall_speed_state_t m_hall_speed_right = {0, 0, 0, 0, 0, 0};
static uint8_t m_dq_last_runtime_mode = 0xffu;
static uint8_t m_sensor_mode_last_left = 0xffu;
static uint8_t m_sensor_mode_last_right = 0xffu;

static inline void dqMeasurementReset(void) {
  /* D/Q filters are mode-dependent, but physical sensor acquisition is not.
   * Never reset Hall edge timing merely because the control mode changed. */
  m_dq_filter_left.iq_state = m_dq_filter_left.id_state = 0;
  m_dq_filter_right.iq_state = m_dq_filter_right.id_state = 0;
}

static inline int16_t dqLowPassQ4(int16_t input, uint16_t coef, int32_t *state) {
  int32_t error = (int32_t)input - (*state >> 16);
  if (error > 32767) error = 32767;
  if (error < -32768) error = -32768;
  *state += (int32_t)coef * error;
  return clampS16FromS32(*state >> 16);
}

static inline uint16_t electricalDegreesToPhase(int16_t degrees) {
  int32_t d = degrees % 360;
  if (d < 0) d += 360;
  return (uint16_t)(((uint32_t)d * 65536u) / 360u);
}

static inline void measureCurrentDQ(int16_t current_1, int16_t current_2,
                                    uint8_t phase_pair_bc, uint16_t theta,
                                    uint8_t signed_by_command, int16_t command,
                                    uint16_t filter_coef, dq_filter_state_t *filter,
                                    volatile int16_t *iq_q4,
                                    volatile int16_t *id_q4) {
  int32_t i1 = (int32_t)current_1 << 4;
  int32_t i2 = (int32_t)current_2 << 4;
  int32_t alpha;
  int32_t beta;

  if (!phase_pair_bc) {
    const int32_t p1 = 18919 * i1;
    const int32_t p2 = 18919 * i2;
    alpha = i1;
    beta = (((p1 < 0 ? 32767 : 0) + p1) >> 15) +
           (((p2 < 0 ? 16383 : 0) + p2) >> 14);
  } else {
    const int32_t diff = i1 - i2;
    const int32_t pbc = 18919 * diff;
    alpha = -i1 - i2;
    beta = ((pbc < 0 ? 32767 : 0) + pbc) >> 15;
  }

  alpha = clampS16FromS32(alpha);
  beta = clampS16FromS32(beta);

  const int32_t sin_q15 = svpwmSinQ15(theta);
  const int32_t cos_q15 = svpwmSinQ15((uint16_t)(theta + 16384u));

  /* Same Park sign convention as the FOC core:
   *   iq = beta*cos(theta) - alpha*sin(theta)
   *   id = alpha*cos(theta) + beta*sin(theta) */
  int32_t iq = (beta * cos_q15 - alpha * sin_q15) >> 15;
  int32_t id = (alpha * cos_q15 + beta * sin_q15) >> 15;
  if (signed_by_command && command < 0) iq = -iq;

  const int16_t iq_raw = clampS16FromS32(iq);
  const int16_t id_raw = clampS16FromS32(id);
  *iq_q4 = dqLowPassQ4(iq_raw, filter_coef, &filter->iq_state);
  *id_q4 = dqLowPassQ4(id_raw, filter_coef, &filter->id_state);
}

static inline int16_t hallSpeedUpdate(uint8_t encoding, uint8_t pole_pairs, hall_speed_state_t *state) {
  const int8_t sector = m_hall_to_sector[encoding & 7u];
  if (sector < 0 || sector > 5 || pole_pairs == 0u) {
    state->rpm = 0;
    return 0;
  }

  if (!state->initialized) {
    state->initialized = 1u;
    state->sector = sector;
    state->ticks = 0u;
    state->period = 0u;
    state->direction = 0;
    state->rpm = 0;
    return 0;
  }

  if (state->ticks < 65535u) ++state->ticks;
  if (sector != state->sector) {
    const uint8_t diff = (uint8_t)((sector - state->sector + 6) % 6);
    if (diff == 1u) state->direction = 1;
    else if (diff == 5u) state->direction = -1;

    if (state->ticks > 0u && state->direction != 0) {
      state->period = state->ticks;
      const uint32_t rpm_mag = ((uint32_t)PWM_FREQ * 10u) /
                              ((uint32_t)state->period * (uint32_t)pole_pairs);
      state->rpm = (int16_t)(state->direction > 0 ? (int32_t)rpm_mag : -(int32_t)rpm_mag);
    }
    state->sector = sector;
    state->ticks = 0u;
  } else if (state->period > 0u && (uint32_t)state->ticks > (uint32_t)state->period * 2u) {
    state->rpm = 0;
  }
  return state->rpm;
}


static inline void svpwmOpenLoopStep(int16_t command, uint8_t pole_pairs, uint16_t *phase, int *u, int *v, int *w) {
  int32_t cmd = CLAMP((int32_t)command, -1000, 1000);
  uint32_t mag = (uint32_t)(cmd < 0 ? -cmd : cmd);
  if (mag == 0u) {
    *u = 0;
    *v = 0;
    *w = 0;
    return;
  }

  /* Q16 phase accumulator: 65536 counts = one electrical revolution.
   * command 1000 ~= N_MOT_MAX mechanical RPM. */
  if (pole_pairs == 0u) pole_pairs = 1u;
  const uint32_t phase_step_at_max =
      ((uint32_t)N_MOT_MAX * (uint32_t)pole_pairs * 65536u) /
      (60u * (uint32_t)PWM_FREQ);
  uint32_t phase_step = (phase_step_at_max * mag) / 1000u;
  if (phase_step == 0u) phase_step = 1u;
  *phase = (uint16_t)(cmd >= 0 ? (*phase + phase_step) : (*phase - phase_step));

  /* Three 120-degree sinusoidal references followed by common-mode injection.
   * Subtracting (max+min)/2 is the centered continuous-SVPWM equivalent. */
  int32_t a = svpwmSinQ15(*phase);
  int32_t b = svpwmSinQ15((uint16_t)(*phase + 21845u));
  int32_t c = svpwmSinQ15((uint16_t)(*phase + 43691u));
  const int32_t vmax = MAX3(a, b, c);
  const int32_t vmin = MIN3(a, b, c);
  const int32_t common = (vmax + vmin) / 2;
  a -= common;
  b -= common;
  c -= common;

  uint32_t modulation = SVPWM_MIN_MOD_PERMILLE +
      ((SVPWM_MAX_MOD_PERMILLE - SVPWM_MIN_MOD_PERMILLE) * mag) / 1000u;
  if (modulation > SVPWM_MAX_MOD_PERMILLE) modulation = SVPWM_MAX_MOD_PERMILLE;
  const int32_t amplitude_ticks = ((int32_t)(m_pwm_top / 2u) * (int32_t)modulation) / 1000;

  *u = (int)((a * amplitude_ticks) / 32767);
  *v = (int)((b * amplitude_ticks) / 32767);
  *w = (int)((c * amplitude_ticks) / 32767);
}

static inline void svpwmFixedVector(int16_t command, uint16_t phase, int *u, int *v, int *w) {
  uint32_t mag = (uint32_t)ABS(CLAMP((int32_t)command, -1000, 1000));
  if (mag == 0u) mag = 1u;
  int32_t a = svpwmSinQ15(phase);
  int32_t b = svpwmSinQ15((uint16_t)(phase + 21845u));
  int32_t c = svpwmSinQ15((uint16_t)(phase + 43691u));
  const int32_t vmax = MAX3(a, b, c);
  const int32_t vmin = MIN3(a, b, c);
  const int32_t common = (vmax + vmin) / 2;
  a -= common; b -= common; c -= common;
  uint32_t modulation = SVPWM_MIN_MOD_PERMILLE +
      ((SVPWM_MAX_MOD_PERMILLE - SVPWM_MIN_MOD_PERMILLE) * mag) / 1000u;
  if (modulation > SVPWM_MAX_MOD_PERMILLE) modulation = SVPWM_MAX_MOD_PERMILLE;
  const int32_t amplitude_ticks = ((int32_t)(m_pwm_top / 2u) * (int32_t)modulation) / 1000;
  *u = (int)((a * amplitude_ticks) / 32767);
  *v = (int)((b * amplitude_ticks) / 32767);
  *w = (int)((c * amplitude_ticks) / 32767);
}


static volatile uint16_t m_current_cal_count = 0u;
static int16_t m_current_offset_phase_a_left = 2000;
static int16_t m_current_offset_phase_b_left = 2000;
static int16_t m_current_offset_phase_b_right = 2000;
static int16_t m_current_offset_phase_c_right = 2000;
static int16_t m_current_offset_input_left = 2000;
static int16_t m_current_offset_input_right = 2000;

uint8_t currentCalibrationActive(void) {
  return m_current_cal_count < ADC_CALIBRATION_SAMPLES;
}

uint16_t currentCalibrationProgressPermille(void) {
  uint32_t count = m_current_cal_count;
  if (count > ADC_CALIBRATION_SAMPLES) count = ADC_CALIBRATION_SAMPLES;
  return (uint16_t)((count * 1000u) / ADC_CALIBRATION_SAMPLES);
}

/* Compatibility with the advanced main-loop API.  The proven calibration
 * path used by FULL_MODES_GUI does not need a post-calibration controller
 * reset.  Keeping these functions avoids changing any other advanced
 * feature or control-flow. */
uint8_t currentCalibrationResetPending(void) {
  return 0u;
}

void currentCalibrationFinalizeReset(void) {
  /* Intentionally empty. */
}

void currentCalibrationStart(void) {
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();

  /* Manual calibration must start from exactly the same state as boot.
   * At boot these six static offsets are initialized to 2000 and
   * m_current_cal_count starts at zero.  Do the identical initialization here. */
  m_motor_enable = 0;
  LEFT_TIM->BDTR &= ~TIM_BDTR_MOE;
  RIGHT_TIM->BDTR &= ~TIM_BDTR_MOE;
  m_current_offset_phase_a_left = 2000;
  m_current_offset_phase_b_left = 2000;
  m_current_offset_phase_b_right = 2000;
  m_current_offset_phase_c_right = 2000;
  m_current_offset_input_left = 2000;
  m_current_offset_input_right = 2000;
  m_current_cal_count = 0u;
  m_foc_iq_left_q4 = m_foc_iq_right_q4 = 0;
  m_foc_id_left_q4 = m_foc_id_right_q4 = 0;
  m_adc_current_valid = 0u;
  m_adc_current_valid_left = 0u;
  m_adc_current_valid_right = 0u;
  dqMeasurementReset();
  memset(&m_hall_speed_left, 0, sizeof(m_hall_speed_left));
  memset(&m_hall_speed_right, 0, sizeof(m_hall_speed_right));

  if (!primask) __enable_irq();
}

int16_t        m_input_voltage_adc       = (400 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE;
static int32_t m_input_voltage_filter  = (400 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE << 16;  // Fixed-point filter output initialized at 400 V*100/cell = 4 V/cell converted to fixed-point

int16_t m_odom_left = 0;
int16_t m_odom_right = 0;

static uint16_t m_hall_sector_prev_left = 0;
static uint16_t m_hall_sector_prev_right = 0;

int16_t modulo(int16_t m, int16_t rest_classes){
  return (((m % rest_classes) + rest_classes) %rest_classes);
}

int16_t up_or_down(int16_t vorher, int16_t nachher){
  static const int8_t up_down[6] = {0, -1, -2, 0, 2, 1};
  //uint16_t mod_diff =  (((vorher - nachher) % 6) + 6) % 6;
  
  return up_down[modulo(vorher-nachher, 6)];
}

// =================================
// DMA interrupt frequency =~ 16 kHz
// =================================

void DMA1_Channel1_IRQHandler(void) {
  const uint32_t isr_start_cycles = DWT->CYCCNT;

  DMA1->IFCR = DMA_IFCR_CTCIF1;
  // HAL_GPIO_WritePin(LED_PORT, LED_PIN, 1);
  // HAL_GPIO_TogglePin(LED_PORT, LED_PIN);

  if (m_current_cal_count < ADC_CALIBRATION_SAMPLES) {  // automatic boot or manual ADC offset calibration
    m_adc_current_valid = 0u;
    m_adc_current_valid_left = 0u;
    m_adc_current_valid_right = 0u;
    m_current_cal_count++;
    m_current_offset_phase_a_left = (m_adc_buffer.rlA + m_current_offset_phase_a_left) / 2;
    m_current_offset_phase_b_left = (m_adc_buffer.rlB + m_current_offset_phase_b_left) / 2;
    m_current_offset_phase_b_right = (m_adc_buffer.rrB + m_current_offset_phase_b_right) / 2;
    m_current_offset_phase_c_right = (m_adc_buffer.rrC + m_current_offset_phase_c_right) / 2;
    m_current_offset_input_left = (m_adc_buffer.dcl + m_current_offset_input_left) / 2;
    m_current_offset_input_right = (m_adc_buffer.dcr + m_current_offset_input_right) / 2;
    m_buzzer_timer++;  // keep main-loop telemetry/progress alive during calibration
    focIsrMonitorEnd(isr_start_cycles);
    return;
  }

  if (m_buzzer_timer % 1000 == 0) {  // Filter battery voltage at a slower sampling rate
    filtLowPass32(m_adc_buffer.batt1, BAT_FILT_COEF, &m_input_voltage_filter);
    m_input_voltage_adc = (int16_t)(m_input_voltage_filter >> 16);  // convert fixed-point to integer
  }

  // Get Left motor currents
  m_current_phase_a_left = (int16_t)(m_current_offset_phase_a_left - m_adc_buffer.rlA);
  m_current_phase_b_left = (int16_t)(m_current_offset_phase_b_left - m_adc_buffer.rlB);
  m_current_input_left   = (int16_t)(m_current_offset_input_left - m_adc_buffer.dcl);
  
  // Get Right motor currents
  m_current_phase_b_right = (int16_t)(m_current_offset_phase_b_right - m_adc_buffer.rrB);
  m_current_phase_c_right = (int16_t)(m_current_offset_phase_c_right - m_adc_buffer.rrC);
  m_current_input_right   = (int16_t)(m_current_offset_input_right - m_adc_buffer.dcr);

  /* Sensor selection is runtime-only. The unused source is not polled: TIM4 is
   * stopped outside Encoder AB mode, and Hall GPIO is ignored outside Hall mode. */
  if (m_sensor_mode_last_left != m_sensor_mode_left) {
    memset(&m_hall_speed_left, 0, sizeof(m_hall_speed_left));
    m_svpwm_phase_left = 0u;
    m_sensor_mode_last_left = m_sensor_mode_left;
  }
  if (m_sensor_mode_last_right != m_sensor_mode_right) {
    memset(&m_hall_speed_right, 0, sizeof(m_hall_speed_right));
    m_svpwm_phase_right = 0u;
    m_sensor_mode_last_right = m_sensor_mode_right;
  }
  uint8_t hall_code_left_sensor = 0u;
  if (m_sensor_mode_left == MCCONF_SENSOR_ENCODER_AB) {
    hall_code_left_sensor = encoder_left_synthetic_hall();
    m_sensor_hall_left = hall_code_left_sensor;
    m_sensor_rpm_left = m_encoder_rpm;
  } else if (m_sensor_mode_left == MCCONF_SENSOR_HALL) {
    const uint8_t hu = !(LEFT_HALL_U_PORT->IDR & LEFT_HALL_U_PIN);
    const uint8_t hv = !(LEFT_HALL_V_PORT->IDR & LEFT_HALL_V_PIN);
    const uint8_t hw = !(LEFT_HALL_W_PORT->IDR & LEFT_HALL_W_PIN);
    hall_code_left_sensor = (uint8_t)((hu << 2) | (hv << 1) | hw);
    m_sensor_hall_left = hall_code_left_sensor;
    m_sensor_rpm_left = hallSpeedUpdate(hall_code_left_sensor, m_mcconf_1.si_motor_pole_pairs, &m_hall_speed_left);
  } else {
    const uint16_t p = openloopPhaseAdvance(m_motor_target_left, m_mcconf_1.si_motor_pole_pairs, &m_svpwm_phase_left);
    hall_code_left_sensor = phaseToSyntheticHall(p);
    m_sensor_hall_left = 0u;
    m_sensor_rpm_left = openloopRpmEstimate(m_motor_target_left);
  }

  uint8_t hall_code_right_sensor = 0u;
  if (m_sensor_mode_right == MCCONF_SENSOR_HALL) {
    const uint8_t hu = !(RIGHT_HALL_U_PORT->IDR & RIGHT_HALL_U_PIN);
    const uint8_t hv = !(RIGHT_HALL_V_PORT->IDR & RIGHT_HALL_V_PIN);
    const uint8_t hw = !(RIGHT_HALL_W_PORT->IDR & RIGHT_HALL_W_PIN);
    hall_code_right_sensor = (uint8_t)((hu << 2) | (hv << 1) | hw);
    m_sensor_hall_right = hall_code_right_sensor;
    m_sensor_rpm_right = hallSpeedUpdate(hall_code_right_sensor, m_mcconf_2.si_motor_pole_pairs, &m_hall_speed_right);
  } else {
    const uint16_t p = openloopPhaseAdvance(m_motor_target_right, m_mcconf_2.si_motor_pole_pairs, &m_svpwm_phase_right);
    hall_code_right_sensor = phaseToSyntheticHall(p);
    m_sensor_hall_right = 0u;
    m_sensor_rpm_right = openloopRpmEstimate(m_motor_target_right);
  }

  const uint8_t sync_active = (uint8_t)(m_sensor_mode_left == MCCONF_SENSOR_ENCODER_AB && encoder_sync_active());
  const uint8_t drive_enable_left = (uint8_t)(m_motor_enable || sync_active || m_encoder_sync_state == 4u);
  const uint8_t drive_enable_right = m_motor_enable;

  if (m_comm_mode_left != m_dq_last_runtime_mode) {
    dqMeasurementReset();
    m_dq_last_runtime_mode = m_comm_mode_left;
  }

  /* Hardware-level phase-current chopping is a synchronous zero vector. It
   * never clears MOE, preserving the validated ~2k ADC current common mode. */
  const int32_t current_phase_c_left = -(int32_t)m_current_phase_a_left - (int32_t)m_current_phase_b_left;
  const int32_t current_phase_a_right = -(int32_t)m_current_phase_b_right - (int32_t)m_current_phase_c_right;
  const int32_t lim_left = m_mcconf_1.l_current_max >> 4;
  const int32_t lim_right = m_mcconf_2.l_current_max >> 4;
  const uint8_t phase_current_trip_left = (uint8_t)(ABS(m_current_phase_a_left) > lim_left ||
      ABS(m_current_phase_b_left) > lim_left || ABS(current_phase_c_left) > lim_left);
  const uint8_t phase_current_trip_right = (uint8_t)(ABS(m_current_phase_b_right) > lim_right ||
      ABS(m_current_phase_c_right) > lim_right || ABS(current_phase_a_right) > lim_right);

  const uint8_t hard_inhibit_left = (uint8_t)(ABS(m_current_input_left) > m_current_input_max || !drive_enable_left);
  const uint8_t hard_inhibit_right = (uint8_t)(ABS(m_current_input_right) > m_current_input_max || !drive_enable_right);
  if (hard_inhibit_left) LEFT_TIM->BDTR &= ~TIM_BDTR_MOE; else LEFT_TIM->BDTR |= TIM_BDTR_MOE;
  if (hard_inhibit_right) RIGHT_TIM->BDTR &= ~TIM_BDTR_MOE; else RIGHT_TIM->BDTR |= TIM_BDTR_MOE;
  m_adc_current_valid_left = (uint8_t)(!hard_inhibit_left);
  m_adc_current_valid_right = (uint8_t)(!hard_inhibit_right);
  m_adc_current_valid = (uint8_t)(m_adc_current_valid_left && m_adc_current_valid_right);

  m_buzzer_timer++;
  if (m_buzzer_freq != 0 && (m_buzzer_timer / 5000) % (m_buzzer_pattern + 1) == 0) {
    if (m_buzzer_prev == 0) {
      m_buzzer_prev = 1;
      if (++m_buzzer_index > (m_buzzer_count + 2)) m_buzzer_index = 1;
    }
    if (m_buzzer_timer % m_buzzer_freq == 0 && (m_buzzer_index <= m_buzzer_count || m_buzzer_count == 0)) {
      HAL_GPIO_TogglePin(BUZZER_PORT, BUZZER_PIN);
    }
  } else if (m_buzzer_prev) {
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
    m_buzzer_prev = 0;
  }

  /* FOC needs a current-sampling PWM window. Use it if either side is SVPWM. */
  m_pwm_margin = (m_comm_mode_left == MCCONF_COMM_SVPWM || m_comm_mode_right == MCCONF_COMM_SVPWM) ? 110 : 0;

  int duty_u_left = 0, duty_v_left = 0, duty_w_left = 0;
  int duty_u_right = 0, duty_v_right = 0, duty_w_right = 0;
  static bool m_isr_running = false;
  if (m_isr_running) { focIsrMonitorEnd(isr_start_cycles); return; }
  m_isr_running = true;

  /* LEFT: encoder sync has priority; otherwise the selected sensor+comm pair is
   * handled by the same VESC-style state object. */
  m_motor_1.m_input.enable = (uint8_t)(drive_enable_left && !m_motor_1.m_output.fault_code);
  m_motor_1.m_input.control_mode = (mc_control_mode)m_control_mode_left;
  m_motor_1.m_input.control_setpoint = m_motor_target_left;
  m_motor_1.m_input.current_adc_1 = m_current_phase_a_left;
  m_motor_1.m_input.current_adc_2 = m_current_phase_b_left;
  m_motor_1.m_input.current_input = m_current_input_left;
  m_motor_1.m_input.rpm_sensor = m_sensor_rpm_left;
  m_motor_1.m_input.phase_openloop_q16 = m_svpwm_phase_left;
  m_motor_1.m_input.phase_encoder_deg_x16 = encoder_left_mechanical_angle_x16();
  m_motor_1.m_input.hall_a = (uint8_t)((hall_code_left_sensor >> 2) & 1u);
  m_motor_1.m_input.hall_b = (uint8_t)((hall_code_left_sensor >> 1) & 1u);
  m_motor_1.m_input.hall_c = (uint8_t)(hall_code_left_sensor & 1u);

  if (sync_active) {
    m_motor_1.m_output.fault_code = 0u;
    if (encoder_sync_hold_vector()) {
      m_svpwm_phase_left = encoder_sync_hold_phase();
      svpwmFixedVector(m_encoder_sync_command, m_svpwm_phase_left, &duty_u_left, &duty_v_left, &duty_w_left);
    } else {
      svpwmOpenLoopStep(encoder_sync_svpwm_command(), (uint8_t)m_mcconf_1.si_motor_pole_pairs,
                        &m_svpwm_phase_left, &duty_u_left, &duty_v_left, &duty_w_left);
    }
  } else {
#ifdef MOTOR_LEFT_ENA
    mcpwm_foc_control(&m_motor_1);
#endif
    duty_u_left = m_motor_1.m_output.duty_a;
    duty_v_left = m_motor_1.m_output.duty_b;
    duty_w_left = m_motor_1.m_output.duty_c;
  }
  if (phase_current_trip_left) duty_u_left = duty_v_left = duty_w_left = 0;
  m_motor_1.m_output.rpm = m_sensor_rpm_left;
  if (m_comm_mode_left == MCCONF_COMM_SVPWM) {
    m_foc_iq_left_q4 = m_motor_1.m_output.iq; m_foc_id_left_q4 = m_motor_1.m_output.id;
  } else {
    measureCurrentDQ(m_current_phase_a_left, m_current_phase_b_left, 0u, m_motor_1.m_phase_now,
                     0u, 0, m_mcconf_1.foc_current_filter_const, &m_dq_filter_left,
                     &m_foc_iq_left_q4, &m_foc_id_left_q4);
  }
  const int hall_sector_left = m_hall_to_sector[hall_code_left_sensor & 7u];
  if (m_sensor_mode_left != MCCONF_SENSOR_OPENLOOP) {
    m_odom_left = modulo(m_odom_left + up_or_down(m_hall_sector_prev_left, hall_sector_left), 9000);
    m_hall_sector_prev_left = hall_sector_left;
  }
  LEFT_TIM->LEFT_TIM_U = (uint16_t)CLAMP(duty_u_left + m_pwm_top / 2, m_pwm_margin, m_pwm_top - m_pwm_margin);
  LEFT_TIM->LEFT_TIM_V = (uint16_t)CLAMP(duty_v_left + m_pwm_top / 2, m_pwm_margin, m_pwm_top - m_pwm_margin);
  LEFT_TIM->LEFT_TIM_W = (uint16_t)CLAMP(duty_w_left + m_pwm_top / 2, m_pwm_margin, m_pwm_top - m_pwm_margin);

  /* RIGHT */
  m_motor_2.m_input.enable = (uint8_t)(drive_enable_right && !m_motor_2.m_output.fault_code);
  m_motor_2.m_input.control_mode = (mc_control_mode)m_control_mode_right;
  m_motor_2.m_input.control_setpoint = m_motor_target_right;
  m_motor_2.m_input.current_adc_1 = m_current_phase_b_right;
  m_motor_2.m_input.current_adc_2 = m_current_phase_c_right;
  m_motor_2.m_input.current_input = m_current_input_right;
  m_motor_2.m_input.rpm_sensor = m_sensor_rpm_right;
  m_motor_2.m_input.phase_openloop_q16 = m_svpwm_phase_right;
  m_motor_2.m_input.phase_encoder_deg_x16 = 0;
  m_motor_2.m_input.hall_a = (uint8_t)((hall_code_right_sensor >> 2) & 1u);
  m_motor_2.m_input.hall_b = (uint8_t)((hall_code_right_sensor >> 1) & 1u);
  m_motor_2.m_input.hall_c = (uint8_t)(hall_code_right_sensor & 1u);
#ifdef MOTOR_RIGHT_ENA
  mcpwm_foc_control(&m_motor_2);
#endif
  duty_u_right = m_motor_2.m_output.duty_a;
  duty_v_right = m_motor_2.m_output.duty_b;
  duty_w_right = m_motor_2.m_output.duty_c;
  if (phase_current_trip_right) duty_u_right = duty_v_right = duty_w_right = 0;
  m_motor_2.m_output.rpm = m_sensor_rpm_right;
  if (m_comm_mode_right == MCCONF_COMM_SVPWM) {
    m_foc_iq_right_q4 = m_motor_2.m_output.iq; m_foc_id_right_q4 = m_motor_2.m_output.id;
  } else {
    measureCurrentDQ(m_current_phase_b_right, m_current_phase_c_right, 1u, m_motor_2.m_phase_now,
                     0u, 0, m_mcconf_2.foc_current_filter_const, &m_dq_filter_right,
                     &m_foc_iq_right_q4, &m_foc_id_right_q4);
  }
  const int hall_sector_right = m_hall_to_sector[hall_code_right_sensor & 7u];
  if (m_sensor_mode_right != MCCONF_SENSOR_OPENLOOP) {
    m_odom_right = modulo(m_odom_right - up_or_down(m_hall_sector_prev_right, hall_sector_right), 9000);
    m_hall_sector_prev_right = hall_sector_right;
  }
  RIGHT_TIM->RIGHT_TIM_U = (uint16_t)CLAMP(duty_u_right + m_pwm_top / 2, m_pwm_margin, m_pwm_top - m_pwm_margin);
  RIGHT_TIM->RIGHT_TIM_V = (uint16_t)CLAMP(duty_v_right + m_pwm_top / 2, m_pwm_margin, m_pwm_top - m_pwm_margin);
  RIGHT_TIM->RIGHT_TIM_W = (uint16_t)CLAMP(duty_w_right + m_pwm_top / 2, m_pwm_margin, m_pwm_top - m_pwm_margin);

  m_isr_running = false;
  focIsrMonitorEnd(isr_start_cycles);
}
