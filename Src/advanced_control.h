#ifndef ADVANCED_CONTROL_H
#define ADVANCED_CONTROL_H

#include <stdint.h>
#include "config.h"

/* Runtime-tunable outer loops. Coefficients are stored as x1000 engineering
 * values to keep USART/EEPROM 16-bit friendly while avoiding float in control. */
extern int16_t spd_kp_l_x1000, spd_ki_l_x1000, spd_kd_l_x1000;
extern int16_t spd_kp_r_x1000, spd_ki_r_x1000, spd_kd_r_x1000;
extern int16_t spd_i_limit, spd_out_limit, spd_d_filter_x1000;
extern int16_t pos_kp_x1000, pos_ki_x1000, pos_kd_x1000;
extern int16_t pos_i_limit_rpm, pos_speed_limit_rpm, pos_deadband_counts;
extern int16_t pos_min_counts, pos_max_counts, pos_target_counts;
extern int16_t cmd_rate_runtime, cmd_filter_runtime;

/* Encoder profile parameters (Left motor only in enc_hall). */
extern uint16_t enc_cpr;
extern uint16_t enc_pole_pairs;
extern int16_t enc_direction;
extern int16_t enc_elec_trim_deg_x10;
extern int16_t enc_sync_cmd;
extern uint16_t enc_sync_sweep_ms;
extern uint16_t enc_sync_settle_ms;
extern int16_t enc_return_rpm;
extern int16_t enc_return_tolerance_counts;

extern volatile int32_t enc_position_counts;
extern volatile int16_t enc_speed_rpm;
extern volatile int16_t enc_position_speed_target_rpm;
extern volatile int16_t enc_elec_angle_deg_x10;
extern volatile uint8_t enc_sync_state;
extern volatile uint8_t enc_sync_ok;

void advancedControlReset(void);
void advancedControlUpdate(uint8_t requestedMode, int16_t cmdL, int16_t cmdR,
                           int16_t rpmL, int16_t rpmR,
                           int16_t *motorTargetL, int16_t *motorTargetR,
                           uint8_t *generatedModeL, uint8_t *generatedModeR);

#ifdef HW_PROFILE_ENC_HALL
void encoderLeftInit(void);
void encoderLeftUpdate(void);
int16_t encoderLeftMechanicalAngleX16(void);
uint8_t encoderSyncActive(void);
void encoderSyncRequest(void);
void encoderSyncService(uint8_t calibrationActive);
int16_t encoderSyncSvpwmCommand(void);
uint8_t encoderSyncHoldVector(void);
uint16_t encoderSyncHoldPhase(void);
#else
static inline void encoderLeftInit(void) {}
static inline void encoderLeftUpdate(void) {}
static inline int16_t encoderLeftMechanicalAngleX16(void) { return 0; }
static inline uint8_t encoderSyncActive(void) { return 0; }
static inline void encoderSyncRequest(void) {}
static inline void encoderSyncService(uint8_t calibrationActive) {(void)calibrationActive;}
static inline int16_t encoderSyncSvpwmCommand(void) { return 0; }
static inline uint8_t encoderSyncHoldVector(void) { return 0; }
static inline uint16_t encoderSyncHoldPhase(void) { return 0; }
#endif

#endif
