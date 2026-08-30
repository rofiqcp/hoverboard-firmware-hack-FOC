#include "advanced_control.h"
#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "BLDC_controller.h"
#include <stdlib.h>

extern P rtP_Left;
extern P rtP_Right;

int16_t spd_kp_l_x1000 = 200;
int16_t spd_ki_l_x1000 = 50;
int16_t spd_kd_l_x1000 = 0;
int16_t spd_kp_r_x1000 = 200;
int16_t spd_ki_r_x1000 = 50;
int16_t spd_kd_r_x1000 = 0;
int16_t spd_i_limit = 800;
int16_t spd_out_limit = 1000;
int16_t spd_d_filter_x1000 = 850;
int16_t pos_kp_x1000 = 20;
int16_t pos_ki_x1000 = 0;
int16_t pos_kd_x1000 = 0;
int16_t pos_i_limit_rpm = 150;
int16_t pos_speed_limit_rpm = 250;
int16_t pos_deadband_counts = 4;
int16_t pos_min_counts = -12000;
int16_t pos_max_counts = 12000;
int16_t pos_target_counts = 0;
int16_t cmd_rate_runtime = RATE;
int16_t cmd_filter_runtime = FILTER;

uint16_t enc_cpr = 4096;
uint16_t enc_pole_pairs = SVPWM_POLE_PAIRS;
int16_t enc_direction = 1;
int16_t enc_elec_trim_deg_x10 = 0;
int16_t enc_sync_cmd = 80;
uint16_t enc_sync_sweep_ms = 1200;
uint16_t enc_sync_settle_ms = 350;
int16_t enc_return_rpm = 80;
int16_t enc_return_tolerance_counts = 8;

volatile int32_t enc_position_counts = 0;
volatile int16_t enc_speed_rpm = 0;
volatile int16_t enc_position_speed_target_rpm = 0;
volatile int16_t enc_elec_angle_deg_x10 = 0;
volatile uint8_t enc_sync_state = 0;
volatile uint8_t enc_sync_ok = 0;

typedef struct {
  int32_t integral_x1000;
  int32_t d_filtered_x1000;
  int16_t prev_err;
} PidState;
static PidState spd_l = {0}, spd_r = {0}, pos_l = {0};

static int16_t clamp16(int32_t v, int16_t lo, int16_t hi) {
  if (v > hi) return hi;
  if (v < lo) return lo;
  return (int16_t)v;
}

static int16_t pidStep(PidState *s, int16_t err, int16_t kp, int16_t ki, int16_t kd,
                       int16_t iLimit, int16_t outLimit, int16_t dFilter) {
  const int32_t p = ((int32_t)kp * err) / 1000;
  s->integral_x1000 += ((int32_t)ki * err * 1000) / MAIN_LOOP_HZ;
  const int32_t iLimX1000 = (int32_t)iLimit * 1000;
  if (s->integral_x1000 > iLimX1000) s->integral_x1000 = iLimX1000;
  if (s->integral_x1000 < -iLimX1000) s->integral_x1000 = -iLimX1000;

  const int32_t dRawX1000 = (int32_t)kd * ((int32_t)err - s->prev_err) * MAIN_LOOP_HZ;
  s->prev_err = err;
  if (dFilter < 0) dFilter = 0;
  if (dFilter > 1000) dFilter = 1000;
  s->d_filtered_x1000 = ((int32_t)dFilter * s->d_filtered_x1000 +
                         (int32_t)(1000 - dFilter) * dRawX1000) / 1000;
  const int32_t out = p + s->integral_x1000 / 1000 + s->d_filtered_x1000 / 1000;
  return clamp16(out, (int16_t)-outLimit, outLimit);
}

void advancedControlReset(void) {
  spd_l.integral_x1000 = spd_l.d_filtered_x1000 = 0; spd_l.prev_err = 0;
  spd_r.integral_x1000 = spd_r.d_filtered_x1000 = 0; spd_r.prev_err = 0;
  pos_l.integral_x1000 = pos_l.d_filtered_x1000 = 0; pos_l.prev_err = 0;
  enc_position_speed_target_rpm = 0;
}

void advancedControlUpdate(uint8_t requestedMode, int16_t cmdL, int16_t cmdR,
                           int16_t rpmL, int16_t rpmR,
                           int16_t *motorTargetL, int16_t *motorTargetR,
                           uint8_t *generatedModeL, uint8_t *generatedModeR) {
  *motorTargetL = cmdL;
  *motorTargetR = -cmdR; /* generated Right convention is mirrored */
  uint8_t generated = requestedMode;
  if (requestedMode == COMMUTATION_MODE || requestedMode == SINE_MODE) generated = VLT_MODE;
  if (requestedMode == SVPWM_MODE || requestedMode == OPEN_MODE) generated = OPEN_MODE;
  *generatedModeL = generated;
  *generatedModeR = generated;

  if (requestedMode == SPD_MODE) {
    /* n_max is stored in generated fixed-point rpm x16. Use the live RAM
     * value so GUI tuning takes effect immediately without an EEPROM save. */
    int16_t maxRpmL = (int16_t)(rtP_Left.n_max >> 4);
    int16_t maxRpmR = (int16_t)(rtP_Right.n_max >> 4);
    if (maxRpmL < 1) maxRpmL = 1;
    if (maxRpmR < 1) maxRpmR = 1;
    const int16_t refL = (int16_t)(((int32_t)cmdL * maxRpmL) / 1000);
    const int16_t refR = (int16_t)(((int32_t)cmdR * maxRpmR) / 1000);
    *motorTargetL = pidStep(&spd_l, (int16_t)(refL - rpmL), spd_kp_l_x1000,
                            spd_ki_l_x1000, spd_kd_l_x1000, spd_i_limit,
                            spd_out_limit, spd_d_filter_x1000);
    const int16_t rightTorqueHost = pidStep(&spd_r, (int16_t)(refR - rpmR), spd_kp_r_x1000,
                            spd_ki_r_x1000, spd_kd_r_x1000, spd_i_limit,
                            spd_out_limit, spd_d_filter_x1000);
    *motorTargetR = (int16_t)-rightTorqueHost;
    *generatedModeL = TRQ_MODE;
    *generatedModeR = TRQ_MODE;
  }
#ifdef HW_PROFILE_ENC_HALL
  else if (requestedMode == POSITION_MODE) {
    int32_t target = pos_target_counts;
    if (target < pos_min_counts) target = pos_min_counts;
    if (target > pos_max_counts) target = pos_max_counts;
    int32_t e32 = target - enc_position_counts;
    if (e32 > 32767) e32 = 32767;
    if (e32 < -32768) e32 = -32768;
    int16_t err = (int16_t)e32;
    if (abs(err) <= pos_deadband_counts) err = 0;
    int16_t posOutLimit = pos_speed_limit_rpm;
    if (enc_sync_state == 4u && enc_return_rpm > 0 && enc_return_rpm < posOutLimit) posOutLimit = enc_return_rpm;
    enc_position_speed_target_rpm = pidStep(&pos_l, err, pos_kp_x1000, pos_ki_x1000,
                                             pos_kd_x1000, pos_i_limit_rpm,
                                             posOutLimit, spd_d_filter_x1000);
    *motorTargetL = pidStep(&spd_l, (int16_t)(enc_position_speed_target_rpm - rpmL),
                            spd_kp_l_x1000, spd_ki_l_x1000, spd_kd_l_x1000,
                            spd_i_limit, spd_out_limit, spd_d_filter_x1000);
    *generatedModeL = TRQ_MODE;
    /* Position mode is Left-only. Keep Right commanded zero. */
    *motorTargetR = 0;
    *generatedModeR = TRQ_MODE;
  }
#endif
  else {
    advancedControlReset();
  }
}

#ifdef HW_PROFILE_ENC_HALL
extern TIM_HandleTypeDef htim_encoder_left;
static uint16_t enc_prev_hw = 0;
static int32_t enc_raw_ext = 0;
static int32_t enc_zero_raw = 0;
static uint32_t enc_last_ms = 0;
static int32_t enc_last_pos = 0;
static int16_t enc_mech_offset_x16 = 0;
static int32_t sync_initial_rel = 0;
static uint32_t sync_started_ms = 0;
static uint16_t sync_hold_phase = 0;

void encoderLeftInit(void) {
  enc_prev_hw = (uint16_t)__HAL_TIM_GET_COUNTER(&htim_encoder_left);
  enc_raw_ext = enc_prev_hw;
  enc_zero_raw = enc_raw_ext;
  enc_position_counts = 0;
  enc_last_pos = 0;
  enc_last_ms = HAL_GetTick();
  enc_speed_rpm = 0;
  enc_sync_state = 0;
  enc_sync_ok = 0;
}

void encoderLeftUpdate(void) {
  const uint16_t now = (uint16_t)__HAL_TIM_GET_COUNTER(&htim_encoder_left);
  const int16_t deltaHw = (int16_t)(now - enc_prev_hw);
  enc_prev_hw = now;
  enc_raw_ext += (int32_t)deltaHw * (enc_direction >= 0 ? 1 : -1);
  enc_position_counts = enc_raw_ext - enc_zero_raw;
  const uint32_t ms = HAL_GetTick();
  const uint32_t dt = ms - enc_last_ms;
  if (dt >= 10u && enc_cpr > 0u) {
    const int32_t dp = enc_position_counts - enc_last_pos;
    int32_t rpm = (dp * 60000L) / ((int32_t)enc_cpr * (int32_t)dt);
    if (rpm > 32767) rpm = 32767;
    if (rpm < -32768) rpm = -32768;
    enc_speed_rpm = (int16_t)rpm;
    enc_last_pos = enc_position_counts;
    enc_last_ms = ms;
  }
}

int16_t encoderLeftMechanicalAngleX16(void) {
  if (enc_cpr == 0u) return enc_mech_offset_x16;
  int32_t mod = enc_position_counts % (int32_t)enc_cpr;
  if (mod < 0) mod += enc_cpr;
  int32_t mech = (mod * 5760L) / (int32_t)enc_cpr;
  mech += enc_mech_offset_x16;
  if (enc_pole_pairs == 0u) enc_pole_pairs = 1u;
  mech += (((int32_t)enc_elec_trim_deg_x10 * 16) / 10) / (int32_t)enc_pole_pairs;
  while (mech >= 5760) mech -= 5760;
  while (mech < 0) mech += 5760;
  /* For GUI: same electrical-angle equation used by generated controller. */
  int32_t elec = mech * enc_pole_pairs - 480;
  elec %= 5760; if (elec < 0) elec += 5760;
  enc_elec_angle_deg_x10 = (int16_t)((elec * 10) / 16);
  return (int16_t)mech;
}

uint8_t encoderSyncActive(void) { return (enc_sync_state == 2u || enc_sync_state == 3u); }
void encoderSyncRequest(void) {
  enc_sync_ok = 0;
  enc_sync_state = 1; /* wait for ADC calibration completion */
  sync_started_ms = HAL_GetTick();
  sync_initial_rel = enc_position_counts;
  sync_hold_phase = 0;
}

/* State: 1 wait calibration, 2 sweep, 3 hold/vector align, 4 return, 5 done, 6 failed. */
void encoderSyncService(uint8_t calibrationActive) {
  encoderLeftUpdate();
  const uint32_t now = HAL_GetTick();
  if (enc_sync_state == 0u || enc_sync_state == 5u || enc_sync_state == 6u) return;
  if (enc_sync_state == 1u) {
    if (!calibrationActive) { sync_started_ms = now; enc_sync_state = 2u; }
    return;
  }
  if (enc_sync_state == 2u) {
    if ((now - sync_started_ms) >= enc_sync_sweep_ms) {
      int32_t moved = enc_position_counts - sync_initial_rel;
      if (labs((long)moved) < 4L) { enc_sync_state = 6u; enc_sync_ok = 0u; return; }
      if (moved < 0) {
        /* Positive open-loop sweep defines positive mechanical direction. */
        enc_direction = (int16_t)-enc_direction;
        enc_raw_ext = enc_zero_raw - (enc_raw_ext - enc_zero_raw);
        enc_position_counts = -enc_position_counts;
        sync_initial_rel = -sync_initial_rel;
      }
      sync_hold_phase = 0u;
      sync_started_ms = now;
      enc_sync_state = 3u;
    }
    return;
  }
  if (enc_sync_state == 3u) {
    if ((now - sync_started_ms) >= enc_sync_settle_ms) {
      /* Define the held electrical vector as electrical zero + user trim. Since
       * generated FOC subtracts 30 deg internally, choose mech offset so the
       * generated electrical angle at software encoder zero equals trim. */
      const int32_t returnTarget = sync_initial_rel - enc_position_counts;
      enc_zero_raw = enc_raw_ext;
      enc_position_counts = 0;
      if (enc_pole_pairs == 0u) enc_pole_pairs = 1u;
      /* Generated FOC subtracts 30 degrees (=480 in deg*16), so at software
       * encoder zero feed +30/polePairs mechanical degrees. User trim is
       * applied separately in encoderLeftMechanicalAngleX16(). */
      enc_mech_offset_x16 = (int16_t)(480 / (int32_t)enc_pole_pairs);
      int32_t target = returnTarget;
      if (target > pos_max_counts) target = pos_max_counts;
      if (target < pos_min_counts) target = pos_min_counts;
      pos_target_counts = (int16_t)target;
      advancedControlReset();
      enc_sync_state = 4u;
      sync_started_ms = now;
    }
    return;
  }
  if (enc_sync_state == 4u) {
    if (labs((long)pos_target_counts - (long)enc_position_counts) <= enc_return_tolerance_counts) {
      enc_sync_ok = 1u;
      enc_sync_state = 5u;
    } else if ((now - sync_started_ms) > 8000u) {
      enc_sync_state = 6u;
      enc_sync_ok = 0u;
    }
  }
}

int16_t encoderSyncSvpwmCommand(void) {
  if (enc_sync_state == 2u) return enc_sync_cmd;
  return 0;
}
uint8_t encoderSyncHoldVector(void) { return enc_sync_state == 3u; }
uint16_t encoderSyncHoldPhase(void) { return sync_hold_phase; }
#endif
