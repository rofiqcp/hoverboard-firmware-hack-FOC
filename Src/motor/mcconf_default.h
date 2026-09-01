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
#define MCCONF_FOC_SENSOR_MODE      FOC_SENSOR_MODE_HALL
#define MCCONF_FOC_CURRENT_KP_Q11           1229u
#define MCCONF_FOC_CURRENT_KI_Q16           1229u
#define MCCONF_FOC_ID_KP_Q11                 819u
#define MCCONF_FOC_ID_KI_Q16                 737u
#define MCCONF_FOC_CURRENT_FILTER_Q16        7864u
#define MCCONF_SPEED_KP_Q11                  4833u
#define MCCONF_SPEED_KI_Q16                   251u
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
#define MCCONF_HALL_TIMEOUT_TICKS            2000u
#define MCCONF_TRQ_STOP_BRAKE_CA      TRQ_STOP_BRAKE_CA
#define MCCONF_TRQ_STOP_RPM_DEADBAND  TRQ_STOP_RPM_DEADBAND
#define MCCONF_OPENLOOP_RPM_DEFAULT   SVPWM_OPENLOOP_RPM_DEFAULT
#define MCCONF_OPENLOOP_RPM_MAX       SVPWM_OPENLOOP_RPM_MAX
#define MCCONF_OPENLOOP_ACCEL_RPM_S   SVPWM_ACCEL_RPM_PER_S
#define MCCONF_OPENLOOP_ALIGN_MS       SVPWM_ALIGN_MS
#define MCCONF_OPENLOOP_ID_SLEW_A_S    SVPWM_ID_SLEW_A_PER_S
#define MCCONF_VESC_TIMEOUT_MS               500u

#endif
