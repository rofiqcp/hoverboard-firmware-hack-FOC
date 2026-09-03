#ifndef MCCONF_DEFAULT_H_
#define MCCONF_DEFAULT_H_

#include "config.h"
#include "vesc/datatypes.h"

/* VESC-style configuration names, values inherited from the proven fixed-point
 * EFeru/hoverboard controller. Runtime FOC math remains integer/fixed-point. */
#define MCCONF_L_CURRENT_MAX                 15.0f
#define MCCONF_L_CURRENT_MIN                -15.0f
#define MCCONF_L_IN_CURRENT_MAX              17.0f
#define MCCONF_L_IN_CURRENT_MIN             -17.0f
#define MCCONF_L_MAX_ERPM                 15000.0f
#define MCCONF_L_MIN_ERPM                -15000.0f
#define MCCONF_L_MIN_DUTY                     0.0f
#define MCCONF_L_MAX_DUTY                    0.95f
#define MCCONF_FAULT_STOP_TIME_MS             500u
#define MCCONF_FOC_SENSOR_MODE      FOC_SENSOR_MODE_HALL
#define MCCONF_FOC_CURRENT_KP_Q11           1229u
#define MCCONF_FOC_CURRENT_KI_Q16           1229u
#define MCCONF_FOC_ID_KP_Q11                 819u
#define MCCONF_FOC_ID_KI_Q16                 737u
#define MCCONF_FOC_CURRENT_FILTER_Q16        7864u
/* VESC speed PID uses normalized output/current scaling. Hardware step tests at
 * +/-750 ERPM selected Kp=0.002, Ki=0.002, Kd=0 for this Hall hoverboard: the
 * doubled Ki removed ~3.3% steady error without excessive current; Kd stays 0
 * because Hall-speed quantization makes a derivative term noisy. These integer
 * fields are persisted gain*1000, not direct Vq-controller coefficients. */
#define MCCONF_SPEED_KP_Q11                     2u
#define MCCONF_SPEED_KI_Q16                     2u
#define MCCONF_SPEED_KD_Q11                     0u
#define MCCONF_POSITION_KP_Q11                   8u
#define MCCONF_POSITION_KI_Q16                   0u
#define MCCONF_POSITION_KD_Q11                   0u
/* Hall-count position safety/tuning. Position PID follows VESC normalized
 * current-output architecture, but this low-resolution steering actuator gets a
 * hard current cap and velocity damping so one-sector commands cannot run away. */
#define MCCONF_POSITION_CURRENT_MAX_MA          600u
#define MCCONF_POSITION_SETTLE_CURRENT_MA        150u
#define MCCONF_POSITION_SETTLE_MS                120u
/* VESC-style speed-command ramp. VESC exposes this in ERPM/s; the ISR keeps
 * mechanical RPM fixed-point, so 1500 ERPM/s / 15 pole-pairs = 100 RPM/s. */
#define MCCONF_SPEED_RAMP_ERPMS_S             1500u
#define MCCONF_SPEED_RELEASE_ERPM               75u  /* 5 mechanical RPM @ 15 pole-pairs */
#define MCCONF_SPEED_STOP_VOLTAGE_MAX          4000   /* gentle stop ceiling; running limit remains 12800 */
#define MCCONF_FOC_VOLTAGE_MAX              14400
/* Closed-loop current/speed headroom for the hoverboard two-shunt ADC.
 * V9 logs stay clean below ~80% duty and become noisy around 83..90%.
 * Mode 1 keeps the proven 14400 direct-voltage ceiling. */
#define MCCONF_FOC_CLOSED_LOOP_VOLTAGE_MAX   12800
#define MCCONF_CURRENT_SLEW_A_PER_S              10u
#define MCCONF_MOTOR_CURRENT_MAX_Q4  (I_MOT_MAX * A2BIT_CONV * 16)
#define MCCONF_MOTOR_RPM_MAX                 N_MOT_MAX
#define MCCONF_POLE_PAIRS_LEFT               15u
#define MCCONF_POLE_PAIRS_RIGHT              15u
#define MCCONF_HALL_INTERP_ENABLE              1u
#define MCCONF_HALL_INTERP_ON_RPM              30
#define MCCONF_HALL_INTERP_OFF_RPM             15
#define MCCONF_FOC_CONTROL_DIV                  3u
/* Hall timeout must be longer than one Hall sector at low VESC ERPM.
 * At 50 ERPM: 60/(50*6)=0.2 s/edge => 3200 ISR ticks @16 kHz.
 * 8000 ticks (0.5 s) keeps valid low-speed Hall feedback down to ~20 ERPM. */
#define MCCONF_HALL_TIMEOUT_TICKS            8000u
/* Reject an impossible Hall edge that is >4x faster than the previous valid
 * sector period. This suppresses contact/boundary chatter near zero speed. */
#define MCCONF_HALL_PERIOD_OUTLIER_RATIO         4u
/* Require a new GPIO Hall code to persist for three 16-kHz samples (~125 us
 * from first to third sample). This filters switching-edge/metastability
 * glitches without materially shifting a 60-deg sector at steering speeds. */
#define MCCONF_HALL_DEBOUNCE_SAMPLES              3u
/* After start or a real direction reversal, accept five valid adjacent edges
 * before enabling the period-outlier test. This fully refreshes the four-edge
 * period history so acceleration from near-zero is not mistaken for chatter. */
#define MCCONF_HALL_PERIOD_FILTER_WARMUP_EDGES     5u
#define MCCONF_TRQ_STOP_RPM_DEADBAND  TRQ_STOP_RPM_DEADBAND
#define MCCONF_OPENLOOP_RPM_DEFAULT   SVPWM_OPENLOOP_RPM_DEFAULT
#define MCCONF_OPENLOOP_RPM_MAX       SVPWM_OPENLOOP_RPM_MAX
#define MCCONF_OPENLOOP_ACCEL_RPM_S   SVPWM_ACCEL_RPM_PER_S
#define MCCONF_OPENLOOP_ALIGN_MS       SVPWM_ALIGN_MS
#define MCCONF_OPENLOOP_ID_SLEW_A_S    SVPWM_ID_SLEW_A_PER_S
#define MCCONF_VESC_TIMEOUT_MS               500u

#endif
