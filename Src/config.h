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
#define A2BIT_CONV               50
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
#define MOTOR_LEFT_ENA
#define MOTOR_RIGHT_ENA
/* One image. Runtime sensor/comm/control selection lives in mc_interface. */
#define HW_PROFILE_ID            3u
#define ADC_CALIBRATION_SAMPLES  2000u
#define TELEMETRY_HZ             50u
#define MAIN_LOOP_HZ              (1000u / DELAY_IN_MAIN_LOOP)
#if (MAIN_LOOP_HZ % TELEMETRY_HZ) != 0
#error "TELEMETRY_HZ must divide MAIN_LOOP_HZ exactly"
#endif

/* Sensorless open-loop SVPWM (mode 4). Command keeps the same -1000..+1000 scale.
 * With N_MOT_MAX=1000 and 15 pole pairs, command 1000 corresponds to ~250 Hz electrical.
 * Modulation uses a small low-speed boost and then a linear V/f-style ramp. */
#define SVPWM_POLE_PAIRS         15u
#define SVPWM_MIN_MOD_PERMILLE   120u
#define SVPWM_MAX_MOD_PERMILLE   850u
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

/* Independent signed motor commands over USART3: cmdL, cmdR = -1000 .. +1000. */
#define PRI_INPUT1               2, -1000, 0, 1000, 0
#define PRI_INPUT2               2, -1000, 0, 1000, 0
#define INPUTS_NR                1
#define FLASH_WRITE_KEY          0x1002

/* One physical communication interface only: USART3 PB10/PB11. */
#define CONTROL_SERIAL_USART3    1
#define FEEDBACK_SERIAL_USART3
#define DEBUG_SERIAL_USART3
#define DEBUG_SERIAL_PROTOCOL
#define SERIAL_START_FRAME       0xABCD
#define SERIAL_BUFFER_SIZE       128
#define SERIAL_DEBUG_LINE_SIZE   96
#define SERIAL_TIMEOUT           160
#define USART3_BAUD              115200
#define USART3_WORDLENGTH        UART_WORDLENGTH_8B

#define SERIAL_STATUS_ENABLED    (1u << 0)
#define SERIAL_STATUS_TIMEOUT    (1u << 1)
#define SERIAL_STATUS_LEFT_FAULT (1u << 2)
#define SERIAL_STATUS_RIGHT_FAULT (1u << 3)
#define SERIAL_STATUS_CALIBRATING (1u << 4)
#define SERIAL_STATUS_ADC_CURRENT_VALID (1u << 5)
#define SERIAL_STATUS_ADC_LEFT_VALID    (1u << 6)
#define SERIAL_STATUS_ADC_RIGHT_VALID   (1u << 7)

#endif
