#ifndef CONFIG_H
#define CONFIG_H

#include "stm32f1xx_hal.h"

/* Fixed firmware profile: BOARD 0 + USART3 only. */
#ifndef VARIANT_USART
#define VARIANT_USART
#endif

#define PWM_FREQ                 16000
#define DEAD_TIME                48
#define DELAY_IN_MAIN_LOOP       5
#define A2BIT_CONV               50  /* EFeru ADC current scaling: 50 count/A */
#define ADC_CONV_TIME_7C5        20
#define ADC_CONV_CLOCK_CYCLES    ADC_CONV_TIME_7C5
#define ADC_CLOCK_DIV            4
#define ADC_TOTAL_CONV_TIME      (ADC_CLOCK_DIV * ADC_CONV_CLOCK_CYCLES)

#define BAT_FILT_COEF            655
#define BAT_CALIB_REAL_VOLTAGE   3970
#define BAT_CALIB_ADC            1492
#define BAT_CELLS                10
#define BAT_LVL2_ENABLE          0
#define BAT_LVL1_ENABLE          1
#define BAT_LVL2                 (360 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE
#define BAT_LVL1                 (350 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE
#define BAT_DEAD                 (337 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE

#define TEMP_FILT_COEF           655
#define TEMP_CAL_LOW_ADC         1655
#define TEMP_CAL_LOW_DEG_C       358
#define TEMP_CAL_HIGH_ADC        1588
#define TEMP_CAL_HIGH_DEG_C      489
#define TEMP_WARNING_ENABLE      0
#define TEMP_WARNING             600
#define TEMP_POWEROFF_ENABLE     0
#define TEMP_POWEROFF            650

#define COM_CTRL                 0
#define SIN_CTRL                 1
#define FOC_CTRL                 2
#define OPEN_MODE                0
#define VLT_MODE                 1
#define SPD_MODE                 2
#define TRQ_MODE                 3
#define SVPWM_MODE               4
#define MOTOR_LEFT_ENA
#define MOTOR_RIGHT_ENA
#define CTRL_TYP_SEL             FOC_CTRL
#define CTRL_MOD_REQ             SPD_MODE

/* Mode 4: VESC-style sensorless open-loop PHASE with closed current PI.
 *
 * Host command semantics are intentionally different from modes 1/2/3:
 *   mode 4: |cmd| = Id target in ampere, sign = rotation direction.
 *           Example: start 2,2 -> Id_ref = +2 A on both motors, Iq_ref = 0 A.
 * The electrical angle is generated internally (no Hall/encoder feedback) and
 * fed to the SAME generated Clarke/Park + Id/Iq PI + centered SVPWM path used
 * by normal FOC. Hall remains sampled only for telemetry/RPM diagnostics.
 *
 * This follows the VESC open-loop-phase convention: Id=current, Iq=0, with a
 * phase override. We add a slow phase rotation after alignment so the motor can
 * spin sensorlessly while the current loop regulates the requested Id. */
#define SVPWM_POLE_PAIRS                 15u
#define SVPWM_ALIGN_MS                  600u
#define SVPWM_ALIGN_PHASE             49152u   /* 3*pi/2 on uint16 electrical angle */
#define SVPWM_OPENLOOP_RPM_DEFAULT       10u   /* mechanical RPM, sign comes from command */
#define SVPWM_OPENLOOP_RPM_MAX          300u
#define SVPWM_ACCEL_RPM_PER_S            20u
#define SVPWM_ID_SLEW_A_PER_S             4u
#define SVPWM_MAX_ID_A                   6u   /* sensorless detect/open-loop safety ceiling */
#define SVPWM_PHASE_LIMIT_A               8u   /* fast phase-current chop threshold */
#define SVPWM_DC_LIMIT_A                  8u   /* DC-link chop threshold during mode 4 */

/* Torque/current mode uses direct centiampere command semantics:
 *   cmd 50 = 0.50 A, cmd 100 = 1.00 A, cmd 1500 = 15.00 A.
 * STOP braking is therefore also specified in centiamperes.
 * 10 mechanical RPM prevents Hall-boundary brake hunting near zero speed. */
#define TRQ_STOP_RPM_DEADBAND            10
#define DIAG_ENA                 1
#define I_MOT_MAX                15
#define I_DC_MAX                 17
#define N_MOT_MAX                1000
#define FIELD_WEAK_ENA           0
#define FIELD_WEAK_MAX           5
#define PHASE_ADV_MAX            25
#define FIELD_WEAK_HI            1000
#define FIELD_WEAK_LO            750

#define INACTIVITY_TIMEOUT       30
#define BEEPS_BACKWARD           0
#define RATE                     480
#define FILTER                   6553

/* Independent signed motor commands over USART3. Mode 3 needs +/-1500 cA
 * to represent the full +/-15 A range; modes 1/2 are still saturated by their
 * own generated-controller limits, while mode 4 clamps to +/-I_MOT_MAX A. */
#define PRI_INPUT1               2, -1500, 0, 1500, 0
#define PRI_INPUT2               2, -1500, 0, 1500, 0
#define INPUTS_NR                1
#define FLASH_WRITE_KEY          0x1002

/* One physical communication interface only: USART3 PB10/PB11. */
#define CONTROL_SERIAL_USART3    1
#define FEEDBACK_SERIAL_USART3
#define DEBUG_SERIAL_USART3
#define DEBUG_SERIAL_PROTOCOL
#define SERIAL_START_FRAME       0xABCD
#define SERIAL_BUFFER_SIZE       768
#define SERIAL_DEBUG_LINE_SIZE   96
#define SERIAL_TIMEOUT           160
#define USART3_BAUD              115200
#define USART3_WORDLENGTH        UART_WORDLENGTH_8B

#define SERIAL_STATUS_ENABLED    (1u << 0)
#define SERIAL_STATUS_TIMEOUT    (1u << 1)
#define SERIAL_STATUS_LEFT_FAULT (1u << 2)
#define SERIAL_STATUS_RIGHT_FAULT (1u << 3)

#endif
