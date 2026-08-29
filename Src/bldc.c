/*
* This file implements FOC motor control.
* This control method offers superior performanace
* compared to previous cummutation method. The new method features:
* ► reduced noise and vibrations
* ► smooth torque output
* ► improved motor efficiency -> lower energy consumption
*
* Copyright (C) 2019-2020 Emanuel FERU <aerdronix@gmail.com>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"
#include "util.h"

// Matlab includes and defines - from auto-code generation
// ###############################################################################
#include "BLDC_controller.h"           /* Model's header file */
#include "rtwtypes.h"

extern RT_MODEL *const rtM_Left;
extern RT_MODEL *const rtM_Right;

extern DW   rtDW_Left;                  /* Observable states */
extern ExtU rtU_Left;                   /* External inputs */
extern ExtY rtY_Left;                   /* External outputs */
extern P    rtP_Left;

extern DW   rtDW_Right;                 /* Observable states */
extern ExtU rtU_Right;                  /* External inputs */
extern ExtY rtY_Right;                  /* External outputs */
// ###############################################################################

static int16_t pwm_margin;              /* This margin allows to have a window in the PWM signal for proper FOC Phase currents measurement */

extern uint8_t ctrlModReq;
static int16_t curDC_max  = (I_DC_MAX * A2BIT_CONV);
static int16_t curPha_max = (I_MOT_MAX * A2BIT_CONV);
int16_t curL_phaA = 0, curL_phaB = 0, curL_DC = 0;
int16_t curR_phaB = 0, curR_phaC = 0, curR_DC = 0;

volatile int pwml = 0;
volatile int pwmr = 0;

extern volatile adc_buf_t adc_buffer;

uint8_t buzzerFreq          = 0;
uint8_t buzzerPattern       = 0;
uint8_t buzzerCount         = 0;
volatile uint32_t buzzerTimer = 0;
static uint8_t  buzzerPrev  = 0;
static uint8_t  buzzerIdx   = 0;

uint8_t        enable       = 0;        // initially motors are disabled for SAFETY
static uint8_t enableFin    = 0;

static const uint16_t pwm_res  = 64000000 / 2 / PWM_FREQ; // = 2000

volatile uint32_t foc_isr_cycles = 0;
volatile uint32_t foc_isr_cycles_max = 0;

static inline void focIsrMonitorEnd(uint32_t startCycles) {
  const uint32_t elapsed = DWT->CYCCNT - startCycles;
  foc_isr_cycles = elapsed;
  if (elapsed > foc_isr_cycles_max) foc_isr_cycles_max = elapsed;
}

/* 256-entry Q15 sine table used by sensorless open-loop SVPWM.
 * This avoids float/libm work in the 16 kHz motor ISR. */
static const int16_t svpwm_sin_q15[256] = {
       0,    804,   1608,   2410,   3212,   4011,   4808,   5602,   6393,   7179,   7962,   8739,   9512,  10278,  11039,  11793,
   12539,  13279,  14010,  14732,  15446,  16151,  16846,  17530,  18204,  18868,  19519,  20159,  20787,  21403,  22005,  22594,
   23170,  23731,  24279,  24811,  25329,  25832,  26319,  26790,  27245,  27683,  28105,  28510,  28898,  29268,  29621,  29956,
   30273,  30571,  30852,  31113,  31356,  31580,  31785,  31971,  32137,  32285,  32412,  32521,  32609,  32678,  32728,  32757,
   32767,  32757,  32728,  32678,  32609,  32521,  32412,  32285,  32137,  31971,  31785,  31580,  31356,  31113,  30852,  30571,
   30273,  29956,  29621,  29268,  28898,  28510,  28105,  27683,  27245,  26790,  26319,  25832,  25329,  24811,  24279,  23731,
   23170,  22594,  22005,  21403,  20787,  20159,  19519,  18868,  18204,  17530,  16846,  16151,  15446,  14732,  14010,  13279,
   12539,  11793,  11039,  10278,   9512,   8739,   7962,   7179,   6393,   5602,   4808,   4011,   3212,   2410,   1608,    804,
       0,   -804,  -1608,  -2410,  -3212,  -4011,  -4808,  -5602,  -6393,  -7179,  -7962,  -8739,  -9512, -10278, -11039, -11793,
  -12539, -13279, -14010, -14732, -15446, -16151, -16846, -17530, -18204, -18868, -19519, -20159, -20787, -21403, -22005, -22594,
  -23170, -23731, -24279, -24811, -25329, -25832, -26319, -26790, -27245, -27683, -28105, -28510, -28898, -29268, -29621, -29956,
  -30273, -30571, -30852, -31113, -31356, -31580, -31785, -31971, -32137, -32285, -32412, -32521, -32609, -32678, -32728, -32757,
  -32767, -32757, -32728, -32678, -32609, -32521, -32412, -32285, -32137, -31971, -31785, -31580, -31356, -31113, -30852, -30571,
  -30273, -29956, -29621, -29268, -28898, -28510, -28105, -27683, -27245, -26790, -26319, -25832, -25329, -24811, -24279, -23731,
  -23170, -22594, -22005, -21403, -20787, -20159, -19519, -18868, -18204, -17530, -16846, -16151, -15446, -14732, -14010, -13279,
  -12539, -11793, -11039, -10278,  -9512,  -8739,  -7962,  -7179,  -6393,  -5602,  -4808,  -4011,  -3212,  -2410,  -1608,   -804,
};

static uint16_t svpwm_phase_l = 0;
static uint16_t svpwm_phase_r = 0;

static inline int16_t svpwmSinQ15(uint16_t phase) {
  return svpwm_sin_q15[(uint8_t)(phase >> 8)];
}

static inline void svpwmOpenLoopStep(int16_t command, uint16_t *phase, int *u, int *v, int *w) {
  int32_t cmd = CLAMP((int32_t)command, -1000, 1000);
  uint32_t mag = (uint32_t)(cmd < 0 ? -cmd : cmd);
  if (mag == 0u) {
    *u = 0;
    *v = 0;
    *w = 0;
    return;
  }

  /* Q16 phase accumulator: 65536 counts = one electrical revolution.
   * command 1000 ~= N_MOT_MAX mechanical RPM. */
  const uint32_t phaseStepAtMax =
      ((uint32_t)N_MOT_MAX * (uint32_t)SVPWM_POLE_PAIRS * 65536u) /
      (60u * (uint32_t)PWM_FREQ);
  uint32_t phaseStep = (phaseStepAtMax * mag) / 1000u;
  if (phaseStep == 0u) phaseStep = 1u;
  *phase = (uint16_t)(cmd >= 0 ? (*phase + phaseStep) : (*phase - phaseStep));

  /* Three 120-degree sinusoidal references followed by common-mode injection.
   * Subtracting (max+min)/2 is the centered continuous-SVPWM equivalent. */
  int32_t a = svpwmSinQ15(*phase);
  int32_t b = svpwmSinQ15((uint16_t)(*phase + 21845u));
  int32_t c = svpwmSinQ15((uint16_t)(*phase + 43691u));
  const int32_t vmax = MAX3(a, b, c);
  const int32_t vmin = MIN3(a, b, c);
  const int32_t common = (vmax + vmin) / 2;
  a -= common;
  b -= common;
  c -= common;

  uint32_t modulation = SVPWM_MIN_MOD_PERMILLE +
      ((SVPWM_MAX_MOD_PERMILLE - SVPWM_MIN_MOD_PERMILLE) * mag) / 1000u;
  if (modulation > SVPWM_MAX_MOD_PERMILLE) modulation = SVPWM_MAX_MOD_PERMILLE;
  const int32_t amplitudeTicks = ((int32_t)(pwm_res / 2u) * (int32_t)modulation) / 1000;

  *u = (int)((a * amplitudeTicks) / 32767);
  *v = (int)((b * amplitudeTicks) / 32767);
  *w = (int)((c * amplitudeTicks) / 32767);
}

static uint16_t offsetcount = 0;
static int16_t offsetrlA    = 2000;
static int16_t offsetrlB    = 2000;
static int16_t offsetrrB    = 2000;
static int16_t offsetrrC    = 2000;
static int16_t offsetdcl    = 2000;
static int16_t offsetdcr    = 2000;

int16_t        batVoltage       = (400 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE;
static int32_t batVoltageFixdt  = (400 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE << 16;  // Fixed-point filter output initialized at 400 V*100/cell = 4 V/cell converted to fixed-point

int16_t odom_l = 0;
int16_t odom_r = 0;

static uint16_t wp_l_vorher = 0;
static uint16_t wp_r_vorher = 0;

int16_t modulo(int16_t m, int16_t rest_classes){
  return (((m % rest_classes) + rest_classes) %rest_classes);
}

int16_t up_or_down(int16_t vorher, int16_t nachher){
  uint16_t up_down[6] = {0,-1,-2,0,2,1};
  //uint16_t mod_diff =  (((vorher - nachher) % 6) + 6) % 6;
  
  return up_down[modulo(vorher-nachher, 6)];
}

// =================================
// DMA interrupt frequency =~ 16 kHz
// =================================
void DMA1_Channel1_IRQHandler(void) {
  const uint32_t focIsrStartCycles = DWT->CYCCNT;

  DMA1->IFCR = DMA_IFCR_CTCIF1;
  // HAL_GPIO_WritePin(LED_PORT, LED_PIN, 1);
  // HAL_GPIO_TogglePin(LED_PORT, LED_PIN);

  if(offsetcount < 2000) {  // calibrate ADC offsets
    offsetcount++;
    offsetrlA = (adc_buffer.rlA + offsetrlA) / 2;
    offsetrlB = (adc_buffer.rlB + offsetrlB) / 2;
    offsetrrB = (adc_buffer.rrB + offsetrrB) / 2;
    offsetrrC = (adc_buffer.rrC + offsetrrC) / 2;
    offsetdcl = (adc_buffer.dcl + offsetdcl) / 2;
    offsetdcr = (adc_buffer.dcr + offsetdcr) / 2;
    focIsrMonitorEnd(focIsrStartCycles);
    return;
  }

  if (buzzerTimer % 1000 == 0) {  // Filter battery voltage at a slower sampling rate
    filtLowPass32(adc_buffer.batt1, BAT_FILT_COEF, &batVoltageFixdt);
    batVoltage = (int16_t)(batVoltageFixdt >> 16);  // convert fixed-point to integer
  }

  // Get Left motor currents
  curL_phaA = (int16_t)(offsetrlA - adc_buffer.rlA);
  curL_phaB = (int16_t)(offsetrlB - adc_buffer.rlB);
  curL_DC   = (int16_t)(offsetdcl - adc_buffer.dcl);
  
  // Get Right motor currents
  curR_phaB = (int16_t)(offsetrrB - adc_buffer.rrB);
  curR_phaC = (int16_t)(offsetrrC - adc_buffer.rrC);
  curR_DC   = (int16_t)(offsetdcr - adc_buffer.dcr);

  const uint8_t svpwmMode = (ctrlModReq == SVPWM_MODE);

  // Hardware-level current chopping. The generated controller provides its own
  // phase-current limiting in modes 1..3; mode 4 bypasses that controller, so
  // reconstruct the third phase and enforce I_MOT_MAX here as well.
  const int32_t curL_phaC = -(int32_t)curL_phaA - (int32_t)curL_phaB;
  const int32_t curR_phaA = -(int32_t)curR_phaB - (int32_t)curR_phaC;
  const uint8_t leftPhaseTrip = svpwmMode &&
      (ABS(curL_phaA) > curPha_max || ABS(curL_phaB) > curPha_max || ABS(curL_phaC) > curPha_max);
  const uint8_t rightPhaseTrip = svpwmMode &&
      (ABS(curR_phaB) > curPha_max || ABS(curR_phaC) > curPha_max || ABS(curR_phaA) > curPha_max);
  const uint8_t leftSvpwmIdle = svpwmMode && (pwml == 0);
  const uint8_t rightSvpwmIdle = svpwmMode && (pwmr == 0);

  if (ABS(curL_DC) > curDC_max || leftPhaseTrip || leftSvpwmIdle || enable == 0) {
    LEFT_TIM->BDTR &= ~TIM_BDTR_MOE;
  } else {
    LEFT_TIM->BDTR |= TIM_BDTR_MOE;
  }

  if (ABS(curR_DC) > curDC_max || rightPhaseTrip || rightSvpwmIdle || enable == 0) {
    RIGHT_TIM->BDTR &= ~TIM_BDTR_MOE;
  } else {
    RIGHT_TIM->BDTR |= TIM_BDTR_MOE;
  }

  // Create square wave for buzzer
  buzzerTimer++;
  if (buzzerFreq != 0 && (buzzerTimer / 5000) % (buzzerPattern + 1) == 0) {
    if (buzzerPrev == 0) {
      buzzerPrev = 1;
      if (++buzzerIdx > (buzzerCount + 2)) {    // pause 2 periods
        buzzerIdx = 1;
      }
    }
    if (buzzerTimer % buzzerFreq == 0 && (buzzerIdx <= buzzerCount || buzzerCount == 0)) {
      HAL_GPIO_TogglePin(BUZZER_PORT, BUZZER_PIN);
    }
  } else if (buzzerPrev) {
      HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
      buzzerPrev = 0;
  }

  // FOC needs a current-sampling PWM window. Open-loop SVPWM stays below 85% modulation.
  pwm_margin = (!svpwmMode && rtP_Left.z_ctrlTypSel == FOC_CTRL) ? 110 : 0;

  // ############################### MOTOR CONTROL ###############################

  int ul = 0, vl = 0, wl = 0;
  int ur = 0, vr = 0, wr = 0;
  static boolean_T OverrunFlag = false;

  /* Check for overrun */
  if (OverrunFlag) {
    focIsrMonitorEnd(focIsrStartCycles);
    return;
  }
  OverrunFlag = true;

  /* In mode 4 the generated controller is intentionally bypassed: it only accepts
   * control modes 0..3. Current chopping/MOE safety above remains active. */
  enableFin = svpwmMode ? enable : (enable && !rtY_Left.z_errCode && !rtY_Right.z_errCode);

  // ========================= LEFT MOTOR ============================
  rtU_Left.b_motEna = enableFin;
  rtU_Left.i_phaAB = curL_phaA;
  rtU_Left.i_phaBC = curL_phaB;
  rtU_Left.i_DCLink = curL_DC;

  if (svpwmMode) {
    rtU_Left.z_ctrlModReq = OPEN_MODE;
    rtU_Left.r_inpTgt = 0;
    rtY_Left.z_errCode = 0;
    rtY_Left.n_mot = 0;       // no sensor => no measured RPM is claimed
    svpwmOpenLoopStep((int16_t)pwml, &svpwm_phase_l, &ul, &vl, &wl);
  } else {
    const uint8_t hall_ul = !(LEFT_HALL_U_PORT->IDR & LEFT_HALL_U_PIN);
    const uint8_t hall_vl = !(LEFT_HALL_V_PORT->IDR & LEFT_HALL_V_PIN);
    const uint8_t hall_wl = !(LEFT_HALL_W_PORT->IDR & LEFT_HALL_W_PIN);

    rtU_Left.z_ctrlModReq = ctrlModReq;
    rtU_Left.r_inpTgt = pwml;
    rtU_Left.b_hallA = hall_ul;
    rtU_Left.b_hallB = hall_vl;
    rtU_Left.b_hallC = hall_wl;

    #ifdef MOTOR_LEFT_ENA
    BLDC_controller_step(rtM_Left);
    #endif

    ul = rtY_Left.DC_phaA;
    vl = rtY_Left.DC_phaB;
    wl = rtY_Left.DC_phaC;

    const uint8_t encoding_l = (uint8_t)((hall_ul << 2) + (hall_vl << 1) + hall_wl);
    const int wheel_pos_l = rtConstP.vec_hallToPos_Value[encoding_l];
    odom_l = modulo(odom_l + up_or_down(wp_l_vorher, wheel_pos_l), 9000);
    wp_l_vorher = wheel_pos_l;
    svpwm_phase_l = 0;
  }

  LEFT_TIM->LEFT_TIM_U = (uint16_t)CLAMP(ul + pwm_res / 2, pwm_margin, pwm_res - pwm_margin);
  LEFT_TIM->LEFT_TIM_V = (uint16_t)CLAMP(vl + pwm_res / 2, pwm_margin, pwm_res - pwm_margin);
  LEFT_TIM->LEFT_TIM_W = (uint16_t)CLAMP(wl + pwm_res / 2, pwm_margin, pwm_res - pwm_margin);
  // =================================================================

  // ========================= RIGHT MOTOR ===========================
  rtU_Right.b_motEna = enableFin;
  rtU_Right.i_phaAB = curR_phaB;
  rtU_Right.i_phaBC = curR_phaC;
  rtU_Right.i_DCLink = curR_DC;

  if (svpwmMode) {
    rtU_Right.z_ctrlModReq = OPEN_MODE;
    rtU_Right.r_inpTgt = 0;
    rtY_Right.z_errCode = 0;
    rtY_Right.n_mot = 0;      // no sensor => no measured RPM is claimed
    svpwmOpenLoopStep((int16_t)pwmr, &svpwm_phase_r, &ur, &vr, &wr);
  } else {
    const uint8_t hall_ur = !(RIGHT_HALL_U_PORT->IDR & RIGHT_HALL_U_PIN);
    const uint8_t hall_vr = !(RIGHT_HALL_V_PORT->IDR & RIGHT_HALL_V_PIN);
    const uint8_t hall_wr = !(RIGHT_HALL_W_PORT->IDR & RIGHT_HALL_W_PIN);

    rtU_Right.z_ctrlModReq = ctrlModReq;
    rtU_Right.r_inpTgt = pwmr;
    rtU_Right.b_hallA = hall_ur;
    rtU_Right.b_hallB = hall_vr;
    rtU_Right.b_hallC = hall_wr;

    #ifdef MOTOR_RIGHT_ENA
    BLDC_controller_step(rtM_Right);
    #endif

    ur = rtY_Right.DC_phaA;
    vr = rtY_Right.DC_phaB;
    wr = rtY_Right.DC_phaC;

    const uint8_t encoding_r = (uint8_t)((hall_ur << 2) + (hall_vr << 1) + hall_wr);
    const int wheel_pos_r = rtConstP.vec_hallToPos_Value[encoding_r];
    odom_r = modulo(odom_r - up_or_down(wp_r_vorher, wheel_pos_r), 9000);
    wp_r_vorher = wheel_pos_r;
    svpwm_phase_r = 0;
  }

  RIGHT_TIM->RIGHT_TIM_U = (uint16_t)CLAMP(ur + pwm_res / 2, pwm_margin, pwm_res - pwm_margin);
  RIGHT_TIM->RIGHT_TIM_V = (uint16_t)CLAMP(vr + pwm_res / 2, pwm_margin, pwm_res - pwm_margin);
  RIGHT_TIM->RIGHT_TIM_W = (uint16_t)CLAMP(wr + pwm_res / 2, pwm_margin, pwm_res - pwm_margin);
  // =================================================================

  /* Indicate task complete */
  OverrunFlag = false;
  focIsrMonitorEnd(focIsrStartCycles);
 
 // ###############################################################################

}
