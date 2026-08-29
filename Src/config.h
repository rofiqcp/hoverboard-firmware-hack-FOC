// Define to prevent recursive inclusion
#ifndef CONFIG_H
#define CONFIG_H

#include "stm32f1xx_hal.h"

// ############################### DO-NOT-TOUCH SETTINGS ###############################
#define PWM_FREQ            16000     // PWM frequency in Hz / is also used for buzzer
#define DEAD_TIME              48     // PWM deadtime
#define DELAY_IN_MAIN_LOOP    5     // in ms. default 5. it is independent of all the timing critical stuff. do not touch if you do not know what you are doing.
#define TIMEOUT                20     // number of wrong / missing input commands before emergency off
#define A2BIT_CONV             50     // A to bit for current conversion on ADC. Example: 1 A = 50, 2 A = 100, etc
// #define PRINTF_FLOAT_SUPPORT          // [-] Uncomment this for printf to support float on Serial Debug. It will increase code size! Better to avoid it!

// ADC conversion time definitions
#define ADC_CONV_TIME_1C5       (14)  //Total ADC clock cycles / conversion = (  1.5+12.5)
#define ADC_CONV_TIME_7C5       (20)  //Total ADC clock cycles / conversion = (  7.5+12.5)
#define ADC_CONV_TIME_13C5      (26)  //Total ADC clock cycles / conversion = ( 13.5+12.5)
#define ADC_CONV_TIME_28C5      (41)  //Total ADC clock cycles / conversion = ( 28.5+12.5)
#define ADC_CONV_TIME_41C5      (54)  //Total ADC clock cycles / conversion = ( 41.5+12.5)
#define ADC_CONV_TIME_55C5      (68)  //Total ADC clock cycles / conversion = ( 55.5+12.5)
#define ADC_CONV_TIME_71C5      (84)  //Total ADC clock cycles / conversion = ( 71.5+12.5)
#define ADC_CONV_TIME_239C5     (252) //Total ADC clock cycles / conversion = (239.5+12.5)

// This settings influences the actual sample-time. Only use definitions above
// This parameter needs to be the same as the ADC conversion for Current Phase of the FIRST Motor in setup.c
#define ADC_CONV_CLOCK_CYCLES   (ADC_CONV_TIME_7C5)

// Set the configured ADC divider. This parameter needs to be the same ADC divider as PeriphClkInit.AdcClockSelection (see main.c)
#define ADC_CLOCK_DIV           (4)

// ADC Total conversion time: this will be used to offset TIM8 in advance of TIM1 to align the Phase current ADC measurement
// This parameter is used in setup.c
#define ADC_TOTAL_CONV_TIME     (ADC_CLOCK_DIV * ADC_CONV_CLOCK_CYCLES) // = ((SystemCoreClock / ADC_CLOCK_HZ) * ADC_CONV_CLOCK_CYCLES), where ADC_CLOCK_HZ = SystemCoreClock/ADC_CLOCK_DIV
// ########################### END OF  DO-NOT-TOUCH SETTINGS ############################

// ############################### BATTERY ###############################
/* Battery voltage calibration: connect power source.
 * see How to calibrate.
 * Write debug output value nr 5 to BAT_CALIB_ADC. make and flash firmware.
 * Then you can verify voltage on debug output value 6 (to get calibrated voltage multiplied by 100).
*/
#define BAT_FILT_COEF           655       // battery voltage filter coefficient in fixed-point. coef_fixedPoint = coef_floatingPoint * 2^16. In this case 655 = 0.01 * 2^16
#define BAT_CALIB_REAL_VOLTAGE  3970      // input voltage measured by multimeter (multiplied by 100). In this case 43.00 V * 100 = 4300
#define BAT_CALIB_ADC           1492      // adc-value measured by mainboard (value nr 5 on UART debug output)
#define BAT_CELLS               10        // battery number of cells. Normal Hoverboard battery: 10s
#define BAT_LVL2_ENABLE         0         // to beep or not to beep, 1 or 0
#define BAT_LVL1_ENABLE         1         // to beep or not to beep, 1 or 0
#define BAT_BLINK_INTERVAL      80        // battery led blink interval (80 loops * 5ms ~= 400ms)
#define BAT_LVL5                (390 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE    // Green blink:  no beep
#define BAT_LVL4                (380 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE    // Yellow:       no beep
#define BAT_LVL3                (370 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE    // Yellow blink: no beep 
#define BAT_LVL2                (360 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE    // Red:          gently beep at this voltage level. [V*100/cell]. In this case 3.60 V/cell
#define BAT_LVL1                (350 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE    // Red blink:    fast beep. Your battery is almost empty. Charge now! [V*100/cell]. In this case 3.50 V/cell
#define BAT_DEAD                (337 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE    // All leds off: undervoltage poweroff. (while not driving) [V*100/cell]. In this case 3.37 V/cell
// ######################## END OF BATTERY ###############################



// ############################### TEMPERATURE ###############################
/* Board overheat detection: the sensor is inside the STM/GD chip.
 * It is very inaccurate without calibration (up to 45°C). So only enable this funcion after calibration!
 * Let your board cool down.
 * see <How to calibrate.
 * Get the real temp of the chip by thermo cam or another temp-sensor taped on top of the chip and write it to TEMP_CAL_LOW_DEG_C.
 * Write debug output value 8 to TEMP_CAL_LOW_ADC. drive around to warm up the board. it should be at least 20°C warmer. repeat it for the HIGH-values.
 * Enable warning and/or poweroff and make and flash firmware.
*/
#define TEMP_FILT_COEF          655       // temperature filter coefficient in fixed-point. coef_fixedPoint = coef_floatingPoint * 2^16. In this case 655 = 0.01 * 2^16
#define TEMP_CAL_LOW_ADC        1655      // temperature 1: ADC value
#define TEMP_CAL_LOW_DEG_C      358       // temperature 1: measured temperature [°C * 10]. Here 35.8 °C
#define TEMP_CAL_HIGH_ADC       1588      // temperature 2: ADC value
#define TEMP_CAL_HIGH_DEG_C     489       // temperature 2: measured temperature [°C * 10]. Here 48.9 °C
#define TEMP_WARNING_ENABLE     0         // to beep or not to beep, 1 or 0, DO NOT ACTIVITE WITHOUT CALIBRATION!
#define TEMP_WARNING            600       // annoying fast beeps [°C * 10].  Here 60.0 °C
#define TEMP_POWEROFF_ENABLE    0         // to poweroff or not to poweroff, 1 or 0, DO NOT ACTIVITE WITHOUT CALIBRATION!
#define TEMP_POWEROFF           650       // overheat poweroff. (while not driving) [°C * 10]. Here 65.0 °C
// ######################## END OF TEMPERATURE ###############################



// ############################### MOTOR CONTROL #########################
/* GENERAL NOTES:
 * 1. The parameters here are over-writing the default motor parameters. For all the available parameters check BLDC_controller_data.c
 * 2. The parameters are represented in fixed point data type for a more efficient code execution
 * 3. For calibrating the fixed-point parameters use the Fixed-Point Viewer tool (see <https://github.com/EmanuelFeru/FixedPointViewer>)
 * 4. For more details regarding the parameters and the working principle of the controller please consult the Simulink model
 * 5. A webview was created, so Matlab/Simulink installation is not needed, unless you want to regenerate the code.
 * The webview is an html page that can be opened with browsers like: Microsoft Internet Explorer or Microsoft Edge
 *
 * NOTES Field Weakening / Phase Advance:
 * 1. The Field Weakening is a linear interpolation from 0 to FIELD_WEAK_MAX or PHASE_ADV_MAX (depeding if FOC or SIN is selected, respectively)
 * 2. The Field Weakening starts engaging at FIELD_WEAK_LO and reaches the maximum value at FIELD_WEAK_HI
 * 3. If you re-calibrate the Field Weakening please take all the safety measures! The motors can spin very fast!

   Inputs:
    - input1[inIdx].cmd and input2[inIdx].cmd: normalized input values. INPUT_MIN to INPUT_MAX
    - button1 and button2: digital input values. 0 or 1
    - adc_buffer.l_tx2 and adc_buffer.l_rx2: unfiltered ADC values (you do not need them). 0 to 4095
   Outputs:
    - cmdL and cmdR: normal driving INPUT_MIN to INPUT_MAX
*/
#define COM_CTRL        0               // [-] Commutation Control Type
#define SIN_CTRL        1               // [-] Sinusoidal Control Type
#define FOC_CTRL        2               // [-] Field Oriented Control (FOC) Type

#define OPEN_MODE       0               // [-] OPEN mode
#define VLT_MODE        1               // [-] VOLTAGE mode
#define SPD_MODE        2               // [-] SPEED mode
#define TRQ_MODE        3               // [-] TORQUE mode

// Enable/Disable Motor
#define MOTOR_LEFT_ENA                  // [-] Enable LEFT motor.  Comment-out if this motor is not needed to be operational
#define MOTOR_RIGHT_ENA                 // [-] Enable RIGHT motor. Comment-out if this motor is not needed to be operational

// Control selections
#define CTRL_TYP_SEL    FOC_CTRL        // [-] Control type selection: COM_CTRL, SIN_CTRL, FOC_CTRL (default)
#define CTRL_MOD_REQ    SPD_MODE        // [-] Control mode request: OPEN_MODE, VLT_MODE (default), SPD_MODE, TRQ_MODE. Note: SPD_MODE and TRQ_MODE are only available for CTRL_FOC!
#define DIAG_ENA        1               // [-] Motor Diagnostics enable flag: 0 = Disabled, 1 = Enabled (default)

// Limitation settings
#define I_MOT_MAX       15              // [A] Maximum single motor current limit
#define I_DC_MAX        17              // [A] Maximum stage2 DC Link current limit for Commutation and Sinusoidal types (This is the final current protection. Above this value, current chopping is applied. To avoid this make sure that I_DC_MAX = I_MOT_MAX + 2A)
#define N_MOT_MAX       1000            // [rpm] Maximum motor speed limit

// Field Weakening / Phase Advance
#define FIELD_WEAK_ENA  0               // [-] Field Weakening / Phase Advance enable flag: 0 = Disabled (default), 1 = Enabled
#define FIELD_WEAK_MAX  5               // [A] Maximum Field Weakening D axis current (only for FOC). Higher current results in higher maximum speed. Up to 10A has been tested using 10" wheels.
#define PHASE_ADV_MAX   25              // [deg] Maximum Phase Advance angle (only for SIN). Higher angle results in higher maximum speed.
#define FIELD_WEAK_HI   1000            // (1000, 1500] Input target High threshold for reaching maximum Field Weakening / Phase Advance. Do NOT set this higher than 1500.
#define FIELD_WEAK_LO   750             // ( 500, 1000] Input target Low threshold for starting Field Weakening / Phase Advance. Do NOT set this higher than 1000.

// Extra functionality
// #define STANDSTILL_HOLD_ENABLE          // [-] Flag to hold the position when standtill is reached. Only available and makes sense for VOLTAGE or TORQUE mode.
// #define ELECTRIC_BRAKE_ENABLE           // [-] Flag to enable electric brake and replace the motor "freewheel" with a constant braking when the input torque request is 0. Only available and makes sense for TORQUE mode.
// #define ELECTRIC_BRAKE_MAX    100       // (0, 500) Maximum electric brake to be applied when input torque request is 0 (pedal fully released).
// #define ELECTRIC_BRAKE_THRES  120       // (0, 500) Threshold below at which the electric brake starts engaging.
// ########################### END OF MOTOR CONTROL ########################



// ############################## DEFAULT SETTINGS ############################
// Default settings will be applied at the end of this config file if not set before
#define INACTIVITY_TIMEOUT        30       // Minutes of not driving until poweroff. it is not very precise.
#define BEEPS_BACKWARD            0       // 0 or 1
#define ADC_MARGIN                100     // ADC input margin applied on the raw ADC min and max to make sure the MIN and MAX values are reached even in the presence of noise
#define ADC_PROTECT_TIMEOUT       100     // ADC Protection: number of wrong / missing input commands before safety state is taken
#define ADC_PROTECT_THRESH        200     // ADC Protection threshold below/above the MIN/MAX ADC values
#define AUTO_CALIBRATION_ENA              // Enable/Disable input auto-calibration by holding power button pressed. Un-comment this if auto-calibration is not needed.

/* FILTER is in fixdt(0,16,16): VAL_fixedPoint = VAL_floatingPoint * 2^16. In this case 6553 = 0.1 * 2^16
 * Value of COEFFICIENT is in fixdt(1,16,14)
 * If VAL_floatingPoint >= 0, VAL_fixedPoint = VAL_floatingPoint * 2^14
 * If VAL_floatingPoint < 0,  VAL_fixedPoint = 2^16 + floor(VAL_floatingPoint * 2^14).
*/
// Value of RATE is in fixdt(1,16,4): VAL_fixedPoint = VAL_floatingPoint * 2^4. In this case 480 = 30 * 2^4
#define DEFAULT_RATE                480   // 30.0f [-] lower value == slower rate [0, 32767] = [0.0, 2047.9375]. Do NOT make rate negative (>32767)
#define DEFAULT_FILTER              6553  // Default for FILTER 0.1f [-] lower value == softer filter [0, 65535] = [0.0 - 1.0].
#define DEFAULT_SPEED_COEFFICIENT   16384 // Default for SPEED_COEFFICIENT 1.0f [-] higher value == stronger. [0, 65535] = [-2.0 - 2.0]. In this case 16384 = 1.0 * 2^14
#define DEFAULT_STEER_COEFFICIENT   8192  // Defualt for STEER_COEFFICIENT 0.5f [-] higher value == stronger. [0, 65535] = [-2.0 - 2.0]. In this case  8192 = 0.5 * 2^14. If you do not want any steering, set it to 0.
// ######################### END OF DEFAULT SETTINGS ##########################

// ############################### STEERING POSITION CONTROLLER ############################
#define STEER_KP_INIT         50    // 0.050 (fixed-point *1000)
#define STEER_KI_INIT          5    // 0.005 (fixed-point *1000)
#define STEER_KD_INIT          0    // 0.000 (fixed-point *1000)
#define STEER_HOM_SPD_INIT   150    // [rpm] homing move speed
#define STEER_HOM_CURR_INIT  100    // [A*100] homing current threshold (100 = 1.0A)
#define STEER_HOM_TO_INIT  10000    // [ms] homing total timeout
// ######################## END OF STEERING POSITION CONTROLLER ############################



// ############################## INPUT FORMAT ############################
/* ***_INPUT: TYPE, MIN, MID, MAX, DEADBAND
 * -----------------------------------------
 * TYPE:      0:Disabled, 1:Normal Pot, 2:Middle Resting Pot, 3:Auto-detect
 * MIN:       min ADC1-value while poti at minimum-position (0 - 4095)
 * MID:       mid ADC1-value while poti at mid-position (INPUT_MIN - INPUT_MAX)
 * MAX:       max ADC2-value while poti at maximum-position (0 - 4095)
 * DEADBAND:  how much of the center position is considered 'center' (100 = values -100 to 100 are considered 0)
 * 
 * Dual-inputs
 * PRI_INPUT: Primary   Input. These limits will be used for the input with priority 0
 * AUX_INPUT: Auxiliary Input. These limits will be used for the input with priority 1
 * -----------------------------------------
*/
 // ############################## END OF INPUT FORMAT ############################


// ############################### DEBUG SERIAL ###############################
/* Connect GND and RX of a 3.3v uart-usb adapter to the left (USART2) or right sensor board cable (USART3)
 * Be careful not to use the red wire of the cable. 15v will destroy everything.
 * enable DEBUG_SERIAL_USART3
 *
 *
 * DEBUG ASCII output is:
 * // "in1:345 in2:1337 cmdL:0 cmdR:0 BatADC:0 BatV:0 TempADC:0 Temp:0\r\n"
 *
 * in1:     (int16_t)input1[inIdx].raw);                                        raw input1: ADC1, UART, PWM, PPM, iBUS
 * in2:     (int16_t)input2[inIdx].raw);                                        raw input2: ADC2, UART, PWM, PPM, iBUS
 * cmdL:    (int16_t)cmdL);                                                     output command Left: [-1000, 1000]
 * cmdR:    (int16_t)cmdR);                                                     output command Right: [-1000, 1000]
 * BatADC:  (int16_t)adc_buffer.batt1);                                         Battery adc-value measured by mainboard
 * BatV:    (int16_t)(batVoltage * BAT_CALIB_REAL_VOLTAGE / BAT_CALIB_ADC));    Battery calibrated voltage multiplied by 100 for verifying battery voltage calibration
 * TempADC: (int16_t)board_temp_adcFilt);                                       for board temperature calibration
 * Temp:    (int16_t)board_temp_deg_c);                                         Temperature in celcius for verifying board temperature calibration
 *
*/

// #define DEBUG_SERIAL_USART3          // right sensor board cable, disable if I2C (nunchuk or lcd) is used!
// #define DEBUG_SERIAL_PROTOCOL        // uncomment this to send user commands to the board, change parameters and print specific signals (see comms.c for the user commands)
// ########################### END OF DEBUG SERIAL ############################

  // #define SIDEBOARD_SERIAL_USART2 0
  // #define CONTROL_SERIAL_USART2  0    // left sensor board cable, disable if ADC or PPM is used! For Arduino control check the hoverSerial.ino
  // #define FEEDBACK_SERIAL_USART2      // left sensor board cable, disable if ADC or PPM is used!

  // #define SIDEBOARD_SERIAL_USART3 0
  #define CONTROL_SERIAL_USART3  0    // primary input on right sensor board cable (USART3)
  #define FEEDBACK_SERIAL_USART3      // telemetry on right sensor board cable (USART3)
  // Extended feedback telemetry: append 32-bit real FOC ISR count + cycle stats to the frame.
  #define FEEDBACK_ISR_COUNTER
  // Host auto-detection / protocol markers consumed by tools/hoverserial.py
  #define FEEDBACK_EXTENDED_MAGIC 0x4953  // "IS"
  #define FEEDBACK_EXTENDED_VERSION 1
  #define INVERT_R_DIRECTION           // Invert rotation of right motor

  // #define DUAL_INPUTS                 //  UART*(Primary) + SIDEBOARD(Auxiliary). Uncomment this to use Dual-inputs
  #define PRI_INPUT1             3, -1000, 0, 1000, 0     // TYPE, MIN, MID, MAX, DEADBAND. See INPUT FORMAT section
  #define PRI_INPUT2             3, -1000, 0, 1000, 0     // TYPE, MIN, MID, MAX, DEADBAND. See INPUT FORMAT section
  #ifdef DUAL_INPUTS
    #define FLASH_WRITE_KEY      0x1102  // Flash memory writing key. Change this key to ignore the input calibrations from the flash memory and use the ones in config.h
   /// #define SIDEBOARD_SERIAL_USART2 1   // left sideboard
    /// #define SIDEBOARD_SERIAL_USART3 1   // right sideboard
    #define AUX_INPUT1           3, -1000, 0, 1000, 0     // TYPE, MIN, MID, MAX, DEADBAND. See INPUT FORMAT section
    #define AUX_INPUT2           3, -1000, 0, 1000, 0     // TYPE, MIN, MID, MAX, DEADBAND. See INPUT FORMAT section
  #else
    #define FLASH_WRITE_KEY      0x1002  // Flash memory writing key. Change this key to ignore the input calibrations from the flash memory and use the ones in config.h
  #endif

// ########################### UART SETIINGS ############################
#if defined(FEEDBACK_SERIAL_USART2) || defined(CONTROL_SERIAL_USART2) || defined(DEBUG_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART2) || \
    defined(FEEDBACK_SERIAL_USART3) || defined(CONTROL_SERIAL_USART3) || defined(DEBUG_SERIAL_USART3) || defined(SIDEBOARD_SERIAL_USART3)
  #define SERIAL_START_FRAME      0xABCD                  // [-] Start frame definition for serial commands
  #define SERIAL_BUFFER_SIZE      64                      // [bytes] Size of Serial Rx buffer. Make sure it is always larger than the structure size
  #define SERIAL_TIMEOUT          160                     // [-] Serial timeout duration for the received data. 160 ~= 0.8 sec. Calculation: 0.8 sec / 0.005 sec
#endif
#if defined(FEEDBACK_SERIAL_USART2) || defined(CONTROL_SERIAL_USART2) || defined(DEBUG_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART2)
  #ifndef USART2_BAUD
    #define USART2_BAUD           115200                  // UART2 baud rate (long wired cable)
  #endif
  #define USART2_WORDLENGTH       UART_WORDLENGTH_8B      // UART_WORDLENGTH_8B or UART_WORDLENGTH_9B
#endif
#if defined(FEEDBACK_SERIAL_USART3) || defined(CONTROL_SERIAL_USART3) || defined(DEBUG_SERIAL_USART3) || defined(SIDEBOARD_SERIAL_USART3)
  #ifndef USART3_BAUD
    #define USART3_BAUD           115200                  // UART3 baud rate (short wired cable)
  #endif
  #define USART3_WORDLENGTH       UART_WORDLENGTH_8B      // UART_WORDLENGTH_8B or UART_WORDLENGTH_9B
#endif
// ########################### UART SETIINGS ############################



// ############################### APPLY DEFAULT SETTINGS ###############################
#ifndef RATE
  #define RATE DEFAULT_RATE
#endif
#ifndef FILTER
  #define FILTER DEFAULT_FILTER
#endif
#ifndef SPEED_COEFFICIENT
  #define SPEED_COEFFICIENT DEFAULT_SPEED_COEFFICIENT
#endif
#ifndef STEER_COEFFICIENT
  #define STEER_COEFFICIENT DEFAULT_STEER_COEFFICIENT
#endif
#if defined(PRI_INPUT1) && defined(PRI_INPUT2) && defined(AUX_INPUT1) && defined(AUX_INPUT2)
  #define INPUTS_NR               2
#else
  #define INPUTS_NR               1
#endif
// ########################### END OF APPLY DEFAULT SETTING ############################



// General checks
#if defined(CONTROL_SERIAL_USART2) && defined(SIDEBOARD_SERIAL_USART2)
  #error CONTROL_SERIAL_USART2 and SIDEBOARD_SERIAL_USART2 not allowed, choose one.
#endif

#if defined(CONTROL_SERIAL_USART3) && defined(SIDEBOARD_SERIAL_USART3)
  #error CONTROL_SERIAL_USART3 and SIDEBOARD_SERIAL_USART3 not allowed, choose one.
#endif

#if defined(DEBUG_SERIAL_USART2) && defined(FEEDBACK_SERIAL_USART2)
  #error DEBUG_SERIAL_USART2 and FEEDBACK_SERIAL_USART2 not allowed, choose one.
#endif

#if defined(DEBUG_SERIAL_USART3) && defined(FEEDBACK_SERIAL_USART3)
  #error DEBUG_SERIAL_USART3 and FEEDBACK_SERIAL_USART3 not allowed, choose one.
#endif

#if defined(DEBUG_SERIAL_USART2) && defined(DEBUG_SERIAL_USART3)
  #error DEBUG_SERIAL_USART2 and DEBUG_SERIAL_USART3 not allowed, choose one.
#endif

#if defined(CONTROL_PPM_LEFT) && defined(CONTROL_PPM_RIGHT)
  #error CONTROL_PPM_LEFT and CONTROL_PPM_RIGHT not allowed, choose one.
#endif

#if defined(CONTROL_PWM_LEFT) && defined(CONTROL_PWM_RIGHT)
  #error CONTROL_PWM_LEFT and CONTROL_PWM_RIGHT not allowed, choose one.
#endif

#if defined(SUPPORT_BUTTONS_LEFT) && defined(SUPPORT_BUTTONS_RIGHT)
  #error SUPPORT_BUTTONS_LEFT and SUPPORT_BUTTONS_RIGHT not allowed, choose one.
#endif


// LEFT cable checks
#if defined(CONTROL_ADC) && (defined(CONTROL_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART2) || defined(FEEDBACK_SERIAL_USART2) || defined(DEBUG_SERIAL_USART2))
  #error CONTROL_ADC and SERIAL_USART2 not allowed. It is on the same cable.
#endif

#if defined(CONTROL_PPM_LEFT) && (defined(CONTROL_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART2) || defined(FEEDBACK_SERIAL_USART2) || defined(DEBUG_SERIAL_USART2))
  #error CONTROL_PPM_LEFT and SERIAL_USART2 not allowed. It is on the same cable.
#endif

#if defined(CONTROL_PWM_LEFT) && (defined(CONTROL_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART2) || defined(FEEDBACK_SERIAL_USART2) || defined(DEBUG_SERIAL_USART2))
  #error CONTROL_PWM_LEFT and SERIAL_USART2 not allowed. It is on the same cable.
#endif

#if defined(SUPPORT_BUTTONS_LEFT) && (defined(CONTROL_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART2) || defined(FEEDBACK_SERIAL_USART2) || defined(DEBUG_SERIAL_USART2))
  #error SUPPORT_BUTTONS_LEFT and SERIAL_USART2 not allowed. It is on the same cable.
#endif

#if defined(SUPPORT_BUTTONS_LEFT) && (defined(CONTROL_ADC) || defined(CONTROL_PPM_LEFT) || defined(CONTROL_PWM_LEFT))
  #error SUPPORT_BUTTONS_LEFT and (CONTROL_ADC or CONTROL_PPM_LEFT or CONTROL_PWM_LEFT) not allowed. It is on the same cable.
#endif

#if defined(CONTROL_ADC) && (defined(CONTROL_PPM_LEFT) || defined(CONTROL_PWM_LEFT))
  #error CONTROL_ADC and (CONTROL_PPM_LEFT or CONTROL_PWM_LEFT) not allowed. It is on the same cable.
#endif

#if defined(CONTROL_PPM_LEFT) && defined(CONTROL_PWM_LEFT)
  #error CONTROL_PPM_LEFT and CONTROL_PWM_LEFT not allowed. It is on the same cable.
#endif


// RIGHT cable checks
#if defined(CONTROL_NUNCHUK) && (defined(CONTROL_SERIAL_USART3) || defined(SIDEBOARD_SERIAL_USART3) || defined(FEEDBACK_SERIAL_USART3) || defined(DEBUG_SERIAL_USART3))
  #error CONTROL_NUNCHUK and SERIAL_USART3 not allowed. It is on the same cable.
#endif

#if defined(CONTROL_PPM_RIGHT) && (defined(CONTROL_SERIAL_USART3) || defined(SIDEBOARD_SERIAL_USART3) || defined(FEEDBACK_SERIAL_USART3) || defined(DEBUG_SERIAL_USART3))
  #error CONTROL_PPM_RIGHT and SERIAL_USART3 not allowed. It is on the same cable.
#endif

#if defined(CONTROL_PWM_RIGHT) && (defined(CONTROL_SERIAL_USART3) || defined(SIDEBOARD_SERIAL_USART3) || defined(FEEDBACK_SERIAL_USART3) || defined(DEBUG_SERIAL_USART3))
  #error CONTROL_PWM_RIGHT and SERIAL_USART3 not allowed. It is on the same cable.
#endif

#if defined(DEBUG_I2C_LCD) && (defined(CONTROL_SERIAL_USART3) || defined(SIDEBOARD_SERIAL_USART3) || defined(FEEDBACK_SERIAL_USART3) || defined(DEBUG_SERIAL_USART3))
  #error DEBUG_I2C_LCD and SERIAL_USART3 not allowed. It is on the same cable.
#endif

#if defined(SUPPORT_BUTTONS_RIGHT) && (defined(CONTROL_SERIAL_USART3) || defined(SIDEBOARD_SERIAL_USART3) || defined(FEEDBACK_SERIAL_USART3) || defined(DEBUG_SERIAL_USART3))
  #error SUPPORT_BUTTONS_RIGHT and SERIAL_USART3 not allowed. It is on the same cable.
#endif

#if defined(SUPPORT_BUTTONS_RIGHT) && (defined(CONTROL_NUNCHUK) || defined(CONTROL_PPM_RIGHT) || defined(CONTROL_PWM_RIGHT) || defined(DEBUG_I2C_LCD))
  #error SUPPORT_BUTTONS_RIGHT and (CONTROL_NUNCHUK or CONTROL_PPM_RIGHT or CONTROL_PWM_RIGHT or DEBUG_I2C_LCD) not allowed. It is on the same cable.
#endif

#if defined(CONTROL_NUNCHUK) && (defined(CONTROL_PPM_RIGHT) || defined(CONTROL_PWM_RIGHT) || defined(DEBUG_I2C_LCD))
  #error CONTROL_NUNCHUK and (CONTROL_PPM_RIGHT or CONTROL_PWM_RIGHT or DEBUG_I2C_LCD) not allowed. It is on the same cable.
#endif

#if defined(DEBUG_I2C_LCD) && (defined(CONTROL_PPM_RIGHT) || defined(CONTROL_PWM_RIGHT))
  #error DEBUG_I2C_LCD and (CONTROL_PPM_RIGHT or CONTROL_PWM_RIGHT) not allowed. It is on the same cable.
#endif

#if defined(CONTROL_PPM_RIGHT) && defined(CONTROL_PWM_RIGHT)
  #error CONTROL_PPM_RIGHT and CONTROL_PWM_RIGHT not allowed. It is on the same cable.
#endif


// Functional checks
#if (defined(CONTROL_PPM_LEFT) || defined(CONTROL_PPM_RIGHT)) && !defined(PPM_NUM_CHANNELS)
  #error Total number of PPM channels needs to be set
#endif
// ############################# END OF VALIDATE SETTINGS ############################

#endif

