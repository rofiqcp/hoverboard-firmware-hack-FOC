#ifndef MCCONF_DEFAULT_H_
#define MCCONF_DEFAULT_H_

#include "config.h"
#include "vesc/datatypes.h"

/* VESC-style configuration names, values inherited from the proven fixed-point
 * EFeru/hoverboard controller. Runtime FOC math remains integer/fixed-point. */
#define MCCONF_L_CURRENT_MAX                 15.0f
#define MCCONF_L_CURRENT_MIN                -15.0f
#define MCCONF_L_IN_CURRENT_MAX              15.0f
#define MCCONF_L_IN_CURRENT_MIN             -15.0f
/* Nilai konfigurasi VESC yang sebelumnya nol akibat memset. Battery cut
 * mengikuti batas baterai 10S pada firmware hardware masteran (3.50/3.37 V/cell). */
#define MCCONF_L_CURRENT_MAX_SCALE             1.0f
#define MCCONF_L_CURRENT_MIN_SCALE             1.0f
#define MCCONF_L_BATTERY_CUT_START            35.0f
#define MCCONF_L_BATTERY_CUT_END              33.7f
#define MCCONF_L_MIN_VIN                      30.0f
#define MCCONF_L_MAX_VIN                      50.0f
#define MCCONF_L_TEMP_FET_START                60.0f
#define MCCONF_L_TEMP_FET_END                  65.0f
#define MCCONF_L_TEMP_MOTOR_START              80.0f
#define MCCONF_L_TEMP_MOTOR_END               100.0f
#define MCCONF_L_WATT_MAX                1500000.0f
#define MCCONF_L_WATT_MIN               -1500000.0f
#define MCCONF_SI_WHEEL_DIAMETER              0.083f
#define MCCONF_L_MAX_ERPM                 15000.0f
#define MCCONF_L_MIN_ERPM                -15000.0f
#define MCCONF_L_MIN_DUTY                     0.0f
#define MCCONF_L_MAX_DUTY                    1.00f
#define MCCONF_FAULT_STOP_TIME_MS             500u
#define MCCONF_FOC_DUTY_DOWNRAMP_KP            20.0f
#define MCCONF_FOC_DUTY_DOWNRAMP_KI           400.0f
#define MCCONF_DUTY_RAMP_STEP_DEFAULT            0.02f /* VESC m_duty_ramp_step */
#define MCCONF_CC_MIN_CURRENT                     0.05f /* VESC-style release threshold */
#define MCCONF_DUTY_PI_BUS_NOMINAL_V            42.5f
#define MCCONF_FOC_SENSOR_MODE      FOC_SENSOR_MODE_HALL
#define MCCONF_FOC_CURRENT_KP_Q11           1229u
#define MCCONF_FOC_CURRENT_KI_Q16           1229u
#define MCCONF_FOC_ID_KP_Q11                 819u
#define MCCONF_FOC_ID_KI_Q16                 737u
#define MCCONF_FOC_CURRENT_FILTER_Q16        7864u /* alpha=0.12 at raw 16-kHz sample rate */
#define MCCONF_FOC_CURRENT_FILTER_CTRL_Q16  20874u /* 1-(1-0.12)^3 at 5.333-kHz D/Q update */
/* VESC default foc_current_filter_const is 0.1. This port keeps the proven
 * 0.12 fixed-point feedback filter above, while the standard runtime config
 * field controls a separate monitoring LPF exactly as upstream intends. */
#define MCCONF_FOC_TELEMETRY_FILTER_DEFAULT     0.10f
/* VESC speed PID uses normalized output/current scaling. Hardware step tests at
 * +/-750 ERPM selected Kp=0.002, Ki=0.002, Kd=0 for this Hall hoverboard: the
 * doubled Ki removed ~3.3% steady error without excessive current; Kd stays 0
 * because Hall-speed quantization makes a derivative term noisy. These integer
 * fields are persisted gain*1000, not direct Vq-controller coefficients. */
#define MCCONF_SPEED_GAIN_SCALE             100000u /* 1e-5 resolution; fits standard VESC speed gains in uint16 */
#define MCCONF_SPEED_KP_Q11                   200u /* 0.00200 */
#define MCCONF_SPEED_KI_Q16                   200u /* 0.00200 */
#define MCCONF_SPEED_KD_Q11                     0u
#define MCCONF_POSITION_KP_Q11                  25u /* 0.025: upstream VESC default position Kp */
#define MCCONF_POSITION_KI_Q16                   0u
#define MCCONF_POSITION_KD_Q11                   0u
#define MCCONF_POSITION_KD_FILTER_Q16         13107u /* 0.20, VESC default D filter */
/* Hall-count position safety/tuning. Position PID follows VESC normalized
 * current-output architecture, but this low-resolution steering actuator gets a
 * hardware-safe current cap and VESC process-D damping so one-sector commands cannot run away. */
#define MCCONF_POSITION_CURRENT_MAX_MA          600u /* custom count-position ceiling */
#define MCCONF_POSITION_RUN_CURRENT_MAX_MA      200u /* SET_POS tracking after first Hall motion */
#define MCCONF_POSITION_DAMP_CURRENT_MA         400u /* kinetic brake; below measured 0.6 A static breakaway */
#define MCCONF_POSITION_BREAKAWAY_CURRENT_MA    600u /* measured left-motor static breakaway */
#define MCCONF_POSITION_BREAKAWAY_KICK_MS       350u
#define MCCONF_POSITION_COUNT_BREAKAWAY_CURRENT_MA 600u /* minimum bounded torque for +/-1 Hall-count stiction */
#define MCCONF_POSITION_COUNT_BREAKAWAY_KICK_MS   350u /* short pulse; never continuous at target */
#define MCCONF_POSITION_COUNT_BREAKAWAY_DELAY_MS    20u /* let normal VESC PID act first */
#define MCCONF_POSITION_BREAKAWAY_REARM_MS      200u
#define MCCONF_POSITION_PHASE_DEADBAND_MDEG    30000u /* Hall-only resolution: +/- half one 60-deg sector */
#define MCCONF_POSITION_SETTLE_CURRENT_MA        150u
#define MCCONF_POSITION_CURRENT_SLEW_A_PER_S      20u /* faster drive->damp transition only in SET_POS */
#define MCCONF_POSITION_SETTLE_MS                120u
/* VESC-style speed-command ramp. VESC exposes this in ERPM/s; the ISR keeps
 * mechanical RPM fixed-point, so 1500 ERPM/s / 15 pole-pairs = 100 RPM/s. */
#define MCCONF_SPEED_RAMP_ERPMS_S             1500u
#define MCCONF_SPEED_RELEASE_ERPM               75u  /* 5 mechanical RPM @ 15 pole-pairs */
#define MCCONF_FOC_VOLTAGE_MAX              16000
#define MCCONF_FOC_CLOSED_LOOP_VOLTAGE_MAX   12800
#define MCCONF_FOC_DUTY_VOLTAGE_MAX          FOC_SVPWM_VECTOR_MAX
#define MCCONF_L_ABS_CURRENT_MAX               20.0f /* hard phase fault, above 15A control limit */
#define MCCONF_PWM_MARGIN_COUNTS          FOC_PWM_MARGIN_COUNTS
#define MCCONF_ABS_CURRENT_QUAL_SAMPLES           3u /* ~0.19 ms @16 kHz: reject transient D/Q spikes */
/* OFF->RUN ADC/gate-driver settling. Unlike the old per-start offset calibration,
 * this never learns a new offset. It only holds a zero vector for 8 PWM frames
 * (0.5 ms @16 kHz) so the first LOW-FET shunt sample belongs to the driven
 * operating point calibrated during the original 2000-sample startup window. */
#define MCCONF_BRIDGE_SETTLE_SAMPLES               80u
/* OFF/high-impedance telemetry uses its own frozen zero-current ADC baseline.
 * Remove a few ADC counts of amplifier noise without hiding real passive/regen
 * current changes when the wheel is back-driven manually. 4 counts = 0.08 A. */
#define MCCONF_OFF_TELEM_DEADBAND_COUNTS              4
#define MCCONF_OFF_TELEM_SETTLE_SAMPLES           16000u /* 1 s @16 kHz: high-Z shunt common-mode benar-benar stabil */
#define MCCONF_CURRENT_SLEW_A_PER_S              10u
#define MCCONF_MOTOR_CURRENT_MAX_Q4  (I_MOT_MAX * A2BIT_CONV * 16)
#define MCCONF_MOTOR_RPM_MAX                 N_MOT_MAX
#define MCCONF_POLE_PAIRS_LEFT               15u
#define MCCONF_POLE_PAIRS_RIGHT              15u
#define MCCONF_HALL_INTERP_ENABLE              1u
/* VESC mcconf_default.h: foc_hall_interp_erpm default = 500 ERPM.
 * Nilai runtime tetap berasal dari Motor Config dan diprecompute ke integer
 * sebelum masuk ISR; jangan ubah menjadi mechanical RPM karena pole-pair bisa
 * berbeda antar motor dan dapat diubah dari VESC Tool. */
#define MCCONF_FOC_HALL_INTERP_ERPM_DEFAULT    500u
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
#define MCCONF_HALL_PHASE_ADVANCE_TICKS       (MCCONF_HALL_DEBOUNCE_SAMPLES - 1u)
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
