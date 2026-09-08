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
/* Batas dinamis standar VESC. Nilai disiapkan di luar ISR lalu runtime hanya
 * memakai perbandingan, perkalian, dan shift integer agar ringan di F103. */
#define MCCONF_L_ERPM_START                    0.80f
#define MCCONF_L_DUTY_START                    1.00f
#define MCCONF_L_TEMP_ACCEL_DEC                0.15f
#define MCCONF_L_IN_CURRENT_MAP_START           0.90f
#define MCCONF_L_IN_CURRENT_MAP_FILTER          0.002f
/* Nilai konfigurasi VESC yang sebelumnya nol akibat memset. Battery cut
 * mengikuti batas baterai 10S pada firmware hardware masteran (3.50/3.37 V/cell). */
#define MCCONF_L_CURRENT_MAX_SCALE             1.0f
#define MCCONF_L_CURRENT_MIN_SCALE             1.0f
#define MCCONF_L_BATTERY_CUT_START            35.0f
#define MCCONF_L_BATTERY_CUT_END              33.7f
/* Derating regen dibuat sebelum hard over-voltage. Nilai ini masih aman untuk
 * bus 50 V dan dapat diubah dari VESC Tool sesuai pack baterai yang digunakan. */
#define MCCONF_L_BATTERY_REGEN_CUT_START       48.0f
#define MCCONF_L_BATTERY_REGEN_CUT_END         49.5f
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
/* ABI incremental encoder LEFT, VESC m_encoder_counts semantics.
 * 1024 PPR quadrature = 4096 counts/rev. PB6/PB7 are shared with LEFT Hall V/W,
 * therefore Hall and ABI are mutually exclusive sensor-port modes. */
#define MCCONF_ENCODER_COUNTS_DEFAULT             4096u
#define MCCONF_ENCODER_RATIO_MAX                  10000.0f
#define MCCONF_ENCODER_OFFSET_DEFAULT             0.0f
#define MCCONF_ENCODER_STARTUP_ALIGN_CURRENT_A    3.00f /* Detect-All/boot Id starts at 3 A */
#define MCCONF_ENCODER_STARTUP_ALIGN_STEP_A       1.00f /* VESC-style adaptive rise until motion */
#define MCCONF_ENCODER_STARTUP_ALIGN_MAX_A       15.00f /* hard board/config ceiling; never exceeded */
#define MCCONF_ENCODER_STARTUP_ALIGN_RAMP_MS       120u
#define MCCONF_ENCODER_STARTUP_ALIGN_HOLD_MS       120u
/* Physical LEFT steering envelope. VESC COMM_SET_POS is still the wire API,
 * but the user coordinate is signed mechanical degrees around center. 330 deg
 * from a legacy 0..360 UI is interpreted as -30 deg. */
#define MCCONF_STEERING_POS_MIN_DEG             (-30.0f)
#define MCCONF_STEERING_POS_MAX_DEG               30.0f
#define MCCONF_STEERING_POSITION_CURRENT_MAX_MA   5000u 
#define MCCONF_STEERING_SLEW_RATE_DEG_S          60u /* physical wheel slew; center-to-endpoint ~=0.5 s */
#define MCCONF_STEERING_POSITION_KP_MULTIPLIER        3u /* 0.025 VESC base -> 0.075 effective; proportional through ~13 deg before 3-A ceiling */
#define MCCONF_STEERING_BREAKAWAY_CURRENT_MA       3500u /* measured minimum to cross worst static steering stiction */
#define MCCONF_STEERING_BREAKAWAY_MAX_MS               0u /* disabled: standard position PID has enough 4-A authority */
#define MCCONF_STEERING_BREAKAWAY_DELAY_MS           150u
#define MCCONF_STEERING_BREAKAWAY_ERROR_MDEG        1000u /* no high-current assist inside +/-1 degree */
#define MCCONF_STEERING_BREAKAWAY_PROGRESS_COUNTS     32u /* require ~0.39 deg real rack motion; reject encoder/mechanical chatter */
#define MCCONF_STEERING_HOME_CURRENT_A             3.00f
#define MCCONF_STEERING_CAL_CURRENT_MAX_A         15.00f /* commissioning only; runtime steering stays capped separately */
#define MCCONF_STEERING_DETECT_CURRENT_START_A      3.00f
#define MCCONF_STEERING_DETECT_CURRENT_STEP_A       1.00f
#define MCCONF_STEERING_MOVE_PROBE_MS                450u
#define MCCONF_STEERING_STOP_CONFIRM_MS              300u
#define MCCONF_STEERING_STALL_MS                     350u
#define MCCONF_STEERING_SEEK_TIMEOUT_MS            20000u
#define MCCONF_STEERING_MIN_SPAN_COUNTS              32
#define MCCONF_STEERING_SAFE_SPAN_PERCENT             95u /* measured hard-stop span is preserved; runtime uses 95% for 2.5% margin each side */
#define MCCONF_STEERING_SETTLE_COUNTS                 6
#define MCCONF_ENCODER_SPEED_WINDOW_TICKS           320u /* 20 ms @16 kHz, 50-Hz speed estimator */
#define MCCONF_ENCODER_SPEED_TIMEOUT_TICKS         8000u /* 0.5 s -> zero */
#define MCCONF_FOC_CURRENT_KP_Q11           1229u
#define MCCONF_FOC_CURRENT_KI_Q16           1229u
#define MCCONF_FOC_ID_KP_Q11                 819u
#define MCCONF_FOC_ID_KI_Q16                 737u
/* VESC default foc_current_filter_const is 0.1. The current PI uses raw Park
 * feedback; this standard field controls monitoring/telemetry filtering only. */
#define MCCONF_FOC_TELEMETRY_FILTER_DEFAULT     0.10f
/* Upstream VESC 6.x PLL defaults. Runtime F103 mengubahnya menjadi koefisien
 * fixed-point pada saat konfigurasi berubah; tidak ada float di ADC ISR. */
#define MCCONF_FOC_PLL_KP_DEFAULT              2000.0f
#define MCCONF_FOC_PLL_KI_DEFAULT             30000.0f
/* VESC speed PID uses normalized output/current scaling. Hardware step tests at
 * +/-750 ERPM selected Kp=0.002, Ki=0.002, Kd=0 for this Hall hoverboard: the
 * doubled Ki removed ~3.3% steady error without excessive current; Kd stays 0
 * because Hall-speed quantization makes a derivative term noisy. These integer
 * fields are persisted gain*1000, not direct Vq-controller coefficients. */
#define MCCONF_SPEED_GAIN_SCALE             100000u /* 1e-5 resolution; fits standard VESC speed gains in uint16 */
#define MCCONF_SPEED_KP_Q11                   200u /* 0.00200 */
#define MCCONF_SPEED_KI_Q16                   200u /* 0.00200 */
#define MCCONF_SPEED_KD_Q11                     0u
#define MCCONF_SPEED_KD_FILTER_DEFAULT         0.20f
#define MCCONF_POSITION_KP_Q11                  25u /* 0.025: upstream VESC default position Kp */
#define MCCONF_POSITION_KI_Q16                   0u
#define MCCONF_POSITION_KD_Q11                   0u
#define MCCONF_POSITION_KD_FILTER_Q16         13107u /* 0.20, VESC default D filter */
/* Project-only multi-turn count-position safety. Standard COMM_SET_POS does
 * not use these limits; it follows VESC normalized PID and motor-current limits. */
#define MCCONF_POSITION_CURRENT_MAX_MA          600u /* custom count-position ceiling */
#define MCCONF_POSITION_DAMP_CURRENT_MA         400u /* kinetic brake; below measured 0.6 A static breakaway */
#define MCCONF_POSITION_COUNT_BREAKAWAY_CURRENT_MA 600u /* minimum bounded torque for +/-1 Hall-count stiction */
#define MCCONF_POSITION_COUNT_BREAKAWAY_KICK_MS   350u /* short pulse; never continuous at target */
#define MCCONF_POSITION_COUNT_BREAKAWAY_DELAY_MS    20u /* let normal VESC PID act first */
/* VESC-style speed-command ramp. VESC exposes this in ERPM/s; the ISR keeps
 * mechanical RPM fixed-point, so 1500 ERPM/s / 15 pole-pairs = 100 RPM/s. */
#define MCCONF_SPEED_RAMP_ERPMS_S             1500u
#define MCCONF_SPEED_RELEASE_ERPM               75u  /* 5 mechanical RPM @ 15 pole-pairs */
#define MCCONF_SPEED_BREAKAWAY_CURRENT_MA      1000u /* one-shot startup torque, bounded below normal 3 A limit */
#define MCCONF_SPEED_BREAKAWAY_MAX_MS            450u /* never hold breakaway torque on a blocked rotor */
#define MCCONF_SPEED_BREAKAWAY_EXIT_ERPM          150u /* first reliable Hall motion ends the startup kick */
#define MCCONF_FOC_VOLTAGE_MAX              16000
#define MCCONF_FOC_DUTY_VOLTAGE_MAX          FOC_SVPWM_VECTOR_MAX
#define MCCONF_L_ABS_CURRENT_MAX               20.0f /* hard phase fault, above 15A control limit */
#define MCCONF_PWM_MARGIN_COUNTS          FOC_PWM_MARGIN_COUNTS
#define MCCONF_ABS_CURRENT_QUAL_SAMPLES           3u /* ~0.19 ms @16 kHz: reject transient D/Q spikes */
/* Safety tambahan yang tetap ringan untuk Cortex-M3. Overspeed memakai Hall
 * period mentah agar fault tidak tertutup clamp telemetry 1000 mechanical RPM. */
#define MCCONF_ABS_OVERSPEED_MARGIN_PERCENT      110u /* hard fault 10% di atas soft ERPM limit */
#define MCCONF_ABS_OVERSPEED_QUAL_SAMPLES          8u /* 0,5 ms @16 kHz, menolak satu glitch timing */
/* Dua shunt fase FOC normalnya berpusat dekat ADC midscale. Toleransi sengaja
 * lebar agar pergeseran common-mode board hoverboard tidak memicu false fault. */
#define MCCONF_CURRENT_OFFSET_CENTER_ADC         2048
#define MCCONF_CURRENT_OFFSET_MAX_DEVIATION_ADC   900
#define MCCONF_CURRENT_OFFSET_MAX_PAIR_DELTA_ADC  900
/* LEFT ABI: batas delta dibuat jauh di atas kecepatan steering normal. Nilai
 * ini hanya menangkap loncatan counter/glitch; hard-stop calibration OPENLOOP
 * tetap diizinkan karena gerak aktualnya sangat lambat. */
#define MCCONF_ENCODER_FAULT_MAX_RPM             1500u
#define MCCONF_ENCODER_FAULT_DELTA_MARGIN_COUNTS    2u
#define MCCONF_ENCODER_STUCK_MIN_ERPM             100u
#define MCCONF_ENCODER_STUCK_CURRENT_MA           1000u
#define MCCONF_ENCODER_STUCK_TIMEOUT_TICKS       8000u /* 0,5 s @16 kHz */
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
#define MCCONF_MOTOR_CURRENT_MAX_Q4  (I_MOT_MAX * A2BIT_CONV * 16)
#define MCCONF_MOTOR_RPM_MAX                 N_MOT_MAX
#define MCCONF_POLE_PAIRS_LEFT               4u
#define MCCONF_POLE_PAIRS_RIGHT              15u
/* VESC mcconf_default.h: foc_hall_interp_erpm default = 500 ERPM.
 * Nilai runtime tetap berasal dari Motor Config dan diprecompute ke integer
 * sebelum masuk ISR; jangan ubah menjadi mechanical RPM karena pole-pair bisa
 * berbeda antar motor dan dapat diubah dari VESC Tool. */
#define MCCONF_FOC_HALL_INTERP_ERPM_DEFAULT    500u
/* Upstream VESC default: 3 extra samples => 7 instantaneous GPIO reads with majority vote. */
#define MCCONF_M_HALL_EXTRA_SAMPLES_DEFAULT       3u
#define MCCONF_FOC_CONTROL_DIV                  6u
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

#endif
