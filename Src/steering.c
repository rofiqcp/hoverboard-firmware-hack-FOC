/**
  * Steering position controller - complete implementation.
  * 
  * Homing: move left until DC current threshold → capture pos_min,
  * move right until threshold → capture pos_max, compute pos_mid.
  * After homing: outer position PID → left speed command → SPD controller.
  * Right motor unchanged (normal speed control via pwmr).
  * 
  * Public state (flat globals, addressable by comms ADD_PARAM):
  *   steer_kp, steer_ki, steer_kd, steer_homing_spd,
  *   steer_homing_curr_thr, steer_homing_timeout_ms,
  *   steer_pos_min, steer_pos_max, steer_pos_mid,
  *   steer_homing_done, steer_failure_flag, steer_encoder_count
  *   target_normalized (public writable via steerCtrl_SetTarget)
  *
  * Public API: steerCtrl_Init/StartHoming/Update/GetOutput/SetTarget/SetK/SetHoming
  * Private state: static SteerState state, pid_integral, pid_prev_error, pid_output,
  *                homing_start_tick, debounce_count, homing_curr_thr, homing_timeout_ms
*/

#include "steering.h"
#include "defines.h"
#include "config.h"
#include "BLDC_controller.h"
#include "util.h"

// ===========================
// EXTERNS
// ===========================
extern uint8_t  ctrlModReq;         // BLDC control mode (SPD_MODE etc.)
extern volatile int pwml;             // left motor speed command → SPD controller
extern volatile int pwmr;             // right motor speed command (unchanged)
extern volatile int enable;          // motor enable flag

// ===========================
// PUBLIC STATE (addressable by comms ADD_PARAM)
// ===========================
int16_t  steer_kp               = STEER_KP_INIT;
int16_t  steer_ki               = STEER_KI_INIT;
int16_t  steer_kd               = STEER_KD_INIT;
int16_t  steer_homing_spd       = STEER_HOM_SPD_INIT;
int16_t  steer_homing_curr_thr  = STEER_HOM_CURR_INIT;  // A*100
int16_t  steer_homing_timeout_ms = STEER_HOM_TO_INIT;
int16_t  steer_pos_min          = 500;
int16_t  steer_pos_max          = 3500;
int16_t  steer_pos_mid          = 2000;
uint8_t  steer_homing_done      = 0;
uint8_t  steer_failure_flag     = 0;
int16_t  steer_encoder_count    = 0;     // current TIM4 encoder position
int16_t  steer_target_normalized = 0;     // -1000..1000 from USART steer

// ===========================
// PRIVATE STATE (not accessible outside this file)
// ===========================
typedef enum {
    STEER_DISABLED = 0,
    STEER_HOMING_START,
    STEER_HOMING_MOVE_LEFT,
    STEER_HOMING_LEFT_FOUND,
    STEER_HOMING_MOVE_RIGHT,
    STEER_HOMING_RIGHT_FOUND,
    STEER_HOMING_COMPLETE,
    STEER_HOMING_FAILED
} SteerState;

static SteerState state          = STEER_DISABLED;
static int16_t  pid_integral     = 0;
static int16_t  pid_prev_error  = 0;
static int16_t  pid_output      = 0;
static uint32_t homing_start_tick = 0;
static int16_t  debounce_count  = 0;
static int16_t  homing_curr_thr = 0;    // working copy of steer_homing_curr_thr
static uint16_t homing_timeout   = 0;    // working copy of steer_homing_timeout_ms

// ===========================
// CONSTANTS
// ===========================
#define STEER_HOMING_SAMPLES   5     // debounce: 5 × 5ms = 25ms
#define STEER_PID_OUT_MAX      ((int16_t)N_MOT_MAX)
#define STEER_INT_SCALE        1000  // kp/ki/kd *1000 scale factor
#define STEER_INT_WINDUP       50000 // integral anti-windup limit

// ===========================
// PUBLIC API
// ===========================

void steerCtrl_Init(void) {
    state              = STEER_DISABLED;
    steer_homing_done  = 0;
    steer_failure_flag = 0;
    pid_integral       = 0;
    pid_prev_error    = 0;
    pid_output        = 0;
    steer_target_normalized = 0;
    steer_pos_min     = 500;
    steer_pos_max     = 3500;
    steer_pos_mid     = 2000;
    steer_encoder_count = 0;
    beepShort(88);  // DEBUG: init ran
}

void steerCtrl_StartHoming(void) {
    if (state == STEER_DISABLED || state == STEER_HOMING_FAILED) {
        state               = STEER_HOMING_START;
        steer_homing_done  = 0;
        steer_failure_flag = 0;
        homing_start_tick  = HAL_GetTick();
        homing_curr_thr    = steer_homing_curr_thr;
        homing_timeout     = (uint16_t)steer_homing_timeout_ms;
        debounce_count     = 0;
        pid_integral       = 0;
        pid_prev_error     = 0;
        pid_output         = 0;
        ctrlModReq = SPD_MODE;   // left motor needs SPD mode during homing
        beepShort(1);
    }
}

int16_t steerCtrl_GetOutput(void) { return pid_output; }
int16_t steerCtrl_GetCenter(void)  { return steer_pos_mid; }
int16_t steerCtrl_GetMin(void)      { return steer_pos_min; }
int16_t steerCtrl_GetMax(void)      { return steer_pos_max; }
uint8_t steerCtrl_IsReady(void)     { return (state == STEER_HOMING_COMPLETE) ? 1U : 0U; }
uint8_t steerCtrl_HomingDone(void)  { return steer_homing_done; }
uint8_t steerCtrl_GetState(void)     { return (uint8_t)state; }

void steerCtrl_SetTarget(int16_t normalized_target) {
    steer_target_normalized = normalized_target;
}

void steerCtrl_SetK(int16_t kp, int16_t ki, int16_t kd) {
    steer_kp = kp;
    steer_ki = ki;
    steer_kd = kd;
}

void steerCtrl_SetHoming(int16_t hom_rpm, int16_t hom_curr_thr, int16_t hom_timeout) {
    steer_homing_spd        = hom_rpm;
    steer_homing_curr_thr   = hom_curr_thr;
    steer_homing_timeout_ms = hom_timeout;
}

// ===========================
// INTERNAL HELPERS
// ===========================

// Handle TIM4 16-bit encoder wrap-around for delta computation
static int16_t enc_delta(int16_t now, int16_t prev) {
    int16_t d = now - prev;
    if (d >  2048) d -= 4096;
    if (d < -2048) d += 4096;
    return d;
}

// Convert normalized target (-1000..1000) → encoder count within learned range
static int16_t target_to_count(void) {
    int32_t norm = steer_target_normalized;
    if (norm >  1000) norm =  1000;
    if (norm < -1000) norm = -1000;

    int32_t count = steer_pos_min
        + (((norm + 1000) * (steer_pos_max - steer_pos_min)) / 2000);

    if (count < steer_pos_min) count = steer_pos_min;
    if (count > steer_pos_max) count = steer_pos_max;
    return (int16_t)count;
}

// Discrete PID: u = kp*e + ki*∫e + kd*Δe
// kp/ki/kd stored as *1000 (e.g. 50 = 0.050)
static void pid_update(int16_t error) {
    int32_t integral = (int32_t)pid_integral + (int32_t)error;
    if (integral >  STEER_INT_WINDUP) integral =  STEER_INT_WINDUP;
    if (integral < -STEER_INT_WINDUP) integral = -STEER_INT_WINDUP;

    int16_t deriv = (int16_t)(error - pid_prev_error);

    int32_t u = ((int32_t)steer_kp * (int32_t)error)
              + (((int32_t)steer_ki * integral) / STEER_INT_SCALE)
              + ((int32_t)steer_kd * (int32_t)deriv);

    pid_output = (int16_t)(u / STEER_INT_SCALE);

    if (pid_output >  STEER_PID_OUT_MAX) pid_output =  STEER_PID_OUT_MAX;
    if (pid_output < -STEER_PID_OUT_MAX) pid_output = -STEER_PID_OUT_MAX;

    pid_integral    = (int16_t)integral;
    pid_prev_error = error;
}

// ===========================
// MAIN UPDATE (called from readCommand in main loop, 5 ms rate)
// enc_rpm: current encoder-derived RPM (unused, derived internally)
// left_curr100: left DC link current in A*100
// ===========================
void steerCtrl_Update(int16_t enc_count, int16_t enc_rpm, int16_t left_curr100) {
    (void)enc_rpm;  // unused; RPM derived from enc_count delta in caller if needed
    steer_encoder_count = enc_count;

    switch (state) {

        // ----------------------------------------------------------
        case STEER_DISABLED:
        case STEER_HOMING_FAILED:
            ctrlModReq = OPEN_MODE;
            pwml = 0;
            pid_integral = 0;
            pid_output   = 0;
            break;

        // ----------------------------------------------------------
        // HOMING: phase 0 - dwell then start moving left
        // ----------------------------------------------------------
        case STEER_HOMING_START: {
            beepShort(99);  // DEBUG
            pwml = 0;
            ctrlModReq = SPD_MODE;
            state = STEER_HOMING_MOVE_LEFT;
            beepShort(77);  // DEBUG
            homing_start_tick = HAL_GetTick();
            debounce_count = 0;
            break;
        }

        // ----------------------------------------------------------
        // HOMING: phase 1 - move left (negative RPM) until stall
        // ----------------------------------------------------------
        case STEER_HOMING_MOVE_LEFT: {
            pwml = -(steer_homing_spd);

            if ((HAL_GetTick() - homing_start_tick) > homing_timeout) {
                steer_failure_flag = 1;
                state = STEER_HOMING_FAILED;
                beepShort(11);  // DEBUG
                beepShort(11);  // DEBUG
                break;
            }

            if (left_curr100 >= homing_curr_thr) {
                debounce_count++;
                if (debounce_count >= STEER_HOMING_SAMPLES) {
                    steer_pos_min       = enc_count;
                    state               = STEER_HOMING_LEFT_FOUND;
                    homing_start_tick   = HAL_GetTick();
                    debounce_count      = 0;
                    beepShort(3);
                }
            } else {
                debounce_count = 0;
            }
            break;
        }

        // ----------------------------------------------------------
        // HOMING: phase 2 - dwell at left limit
        // ----------------------------------------------------------
        case STEER_HOMING_LEFT_FOUND: {
            pwml = 0;  // dwell
            if ((HAL_GetTick() - homing_start_tick) > 200) {
                state             = STEER_HOMING_MOVE_RIGHT;
                homing_start_tick = HAL_GetTick();
                debounce_count    = 0;
            }
            break;
        }

        // ----------------------------------------------------------
        // HOMING: phase 3 - move right until stall
        // ----------------------------------------------------------
        case STEER_HOMING_MOVE_RIGHT: {
            pwml = steer_homing_spd;

            if ((HAL_GetTick() - homing_start_tick) > (homing_timeout * 2)) {
                steer_failure_flag = 1;
                state = STEER_HOMING_FAILED;
                beepShort(10);
                break;
            }

            if (left_curr100 >= homing_curr_thr) {
                debounce_count++;
                if (debounce_count >= STEER_HOMING_SAMPLES) {
                    steer_pos_max    = enc_count;
                    state            = STEER_HOMING_RIGHT_FOUND;
                    homing_start_tick = HAL_GetTick();
                    beepShort(3);
                    break;
                }
            } else {
                debounce_count = 0;
            }
            break;
        }

        // ----------------------------------------------------------
        // HOMING: phase 4 - dwell at right limit, compute midpoint
        // ----------------------------------------------------------
        case STEER_HOMING_RIGHT_FOUND: {
            pwml = 0;  // dwell
            if ((HAL_GetTick() - homing_start_tick) > 200) {
                int16_t travel = steer_pos_max - steer_pos_min;
                if (travel < 300) {
                    // Travel too small: mechanical failure or sensor issue
                    steer_failure_flag = 1;
                    state = STEER_HOMING_FAILED;
                    beepShort(10);
                } else {
                    steer_pos_mid        = (steer_pos_min + steer_pos_max) / 2;
                    pid_integral         = 0;
                    pid_prev_error       = 0;
                    pid_output           = 0;
                    steer_homing_done    = 1;
                    state                = STEER_HOMING_COMPLETE;
                    steer_target_normalized = 0;  // auto-center on start
                    beepShort(2);
                }
            }
            break;
        }

        // ----------------------------------------------------------
        // POSITION CONTROL: after successful homing
        // ----------------------------------------------------------
        case STEER_HOMING_COMPLETE: {
            int16_t setpoint = target_to_count();
            int16_t error    = setpoint - enc_count;
            pid_update(error);
            ctrlModReq = SPD_MODE;
            pwml = pid_output;
            break;
        }
    }
}
