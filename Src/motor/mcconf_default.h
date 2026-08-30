#ifndef MCCONF_DEFAULT_H_
#define MCCONF_DEFAULT_H_

/* Runtime selection values exposed by the USART3 CLI. */
#define MCCONF_SENSOR_OPENLOOP          1u
#define MCCONF_SENSOR_HALL              2u
#define MCCONF_SENSOR_ENCODER_AB        3u

#define MCCONF_COMM_SIX_STEP            1u
#define MCCONF_COMM_SINE_PWM            2u
#define MCCONF_COMM_SVPWM               3u

#define MCCONF_CONTROL_PWM               1u
#define MCCONF_CONTROL_CURRENT           2u
#define MCCONF_CONTROL_SPEED             3u
#define MCCONF_CONTROL_POSITION          4u

/* One firmware image: both motors start on Hall + SVPWM + current control. */
#define MCCONF_DEFAULT_SENSOR_LEFT       MCCONF_SENSOR_HALL
#define MCCONF_DEFAULT_SENSOR_RIGHT      MCCONF_SENSOR_HALL
#define MCCONF_DEFAULT_COMM_LEFT         MCCONF_COMM_SVPWM
#define MCCONF_DEFAULT_COMM_RIGHT        MCCONF_COMM_SVPWM
#define MCCONF_DEFAULT_CONTROL_LEFT      MCCONF_CONTROL_CURRENT
#define MCCONF_DEFAULT_CONTROL_RIGHT     MCCONF_CONTROL_CURRENT

/* VESC-style speed loop defaults. Engineering units are command/rpm x1000.
 * The integration update is Ki * error * dt exactly once; do not multiply dt
 * by another 1000. 100 RPM is intentionally allowed (no VESC 900-RPM floor). */
#define MCCONF_S_PID_KP_X1000            500
#define MCCONF_S_PID_KI_X1000            100
#define MCCONF_S_PID_KD_X1000            0
#define MCCONF_S_PID_I_LIMIT             400
#define MCCONF_S_PID_OUTPUT_LIMIT        700
#define MCCONF_S_PID_D_FILTER_X1000      850
#define MCCONF_S_PID_MIN_RPM             5

#define MCCONF_P_PID_KP_X1000            20
#define MCCONF_P_PID_KI_X1000            0
#define MCCONF_P_PID_KD_X1000            0
#define MCCONF_P_PID_I_LIMIT_RPM         150
#define MCCONF_P_PID_SPEED_LIMIT_RPM     250
#define MCCONF_P_PID_DEADBAND_COUNTS     4

#endif
