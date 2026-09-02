#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"
#include "util.h"
#include "motor/mcpwm_foc.h"
#include "motor/mcconf_default.h"
#include "motor/mc_interface.h"
#include "vesc/vesc_protocol.h"
#include "comms.h"

void SystemClock_Config(void);

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern UART_HandleTypeDef huart3;
extern volatile adc_buf_t adc_buffer;
extern InputStruct input1[];
extern InputStruct input2[];
extern int16_t speedAvg;
extern int16_t speedAvgAbs;
extern uint8_t timeoutFlgSerial;
extern uint8_t ctrlModReq;
extern volatile int pwml;
extern volatile int pwmr;
extern uint8_t enable;
extern int16_t batVoltage;
extern volatile uint32_t buzzerTimer;
extern volatile uint32_t foc_isr_cycles;
extern volatile int16_t foc_iqL_q4;
extern volatile int16_t foc_iqR_q4;
extern volatile int16_t foc_idL_q4;
extern volatile int16_t foc_idR_q4;

volatile uint32_t main_loop_counter = 0;
int16_t batVoltageCalib = 0;
int16_t board_temp_deg_c = 0;
int16_t left_dc_curr = 0;
int16_t right_dc_curr = 0;
int16_t dc_curr = 0;
int16_t cmdL = 0;
int16_t cmdR = 0;

typedef struct __attribute__((packed)) {
  uint16_t start;
  uint16_t version;              /* VESC-like telemetry payload version */
  int16_t cmdL, cmdR;
  int16_t rpmL, rpmR;
  int16_t dutyL_x1000, dutyR_x1000;
  int16_t currentMotorL_cA, currentMotorR_cA;
  int16_t currentInL_cA, currentInR_cA;
  int16_t idL_cA, idR_cA;
  int16_t iqL_cA, iqR_cA;
  int16_t vdL_cV, vdR_cV;
  int16_t vqL_cV, vqR_cV;
  int16_t vIn_x100;
  int16_t boardTemp_x10;
  uint16_t hallL, hallR;
  uint16_t faultL, faultR;
  uint16_t adc_dcl, adc_rla, adc_rlb;
  uint16_t adc_dcr, adc_rrb, adc_rrc;
  uint16_t status;
  uint32_t foc_isr_cycles;
  uint16_t checksum;
} SerialFeedback;

_Static_assert(sizeof(SerialFeedback) == 72u, "SerialFeedback V2 must stay 72 bytes");

static SerialFeedback feedback;
static int16_t cmdLRateFixdt = 0;
static int16_t cmdRRateFixdt = 0;
static int32_t cmdLFixdt = 0;
static int32_t cmdRFixdt = 0;
static uint32_t buzzerTimerPrev = 0;
static uint32_t inactivityTimeoutCounter = 0;
static uint32_t legacyTelemetryPrevMs = 0u;

static uint16_t feedbackChecksum(const SerialFeedback *f) {
  const uint8_t *p = (const uint8_t *)f;
  const size_t n = offsetof(SerialFeedback, checksum);
  uint16_t c = 0;
  for (size_t i = 0; i + 1 < n; i += 2) {
    c ^= (uint16_t)p[i] | ((uint16_t)p[i + 1] << 8);
  }
  return c;
}

static int16_t focCurrentQ4ToCentiAmp(int32_t currentQ4) {
  /* The fixed-point VESC-style FOC shifts phase-current inputs by 4 before Clarke/Park:
   *     rtb = rtU->i_phaXX << 4
   * and rtY.iq / rtY.id are exported without shifting back. Therefore dq is
   * Q4 current-count, i.e. 16 * A2BIT_CONV units per ampere. */
  const int32_t denom = (int32_t)A2BIT_CONV * 16;
  return (int16_t)(((int32_t)currentQ4 * 100) / denom);
}

static int16_t focVoltageToCentiVolt(int16_t v) {
  /* Report D/Q voltage against the measured DC bus, not a fixed 48-V nominal.
   * batVoltageCalib is centivolts. */
  return (int16_t)(((int32_t)v * (int32_t)batVoltageCalib) / MCCONF_FOC_VOLTAGE_MAX);
}

static uint16_t readHallLeft(void) {
  const uint16_t u = (LEFT_HALL_U_PORT->IDR & LEFT_HALL_U_PIN) ? 0u : 1u;
  const uint16_t v = (LEFT_HALL_V_PORT->IDR & LEFT_HALL_V_PIN) ? 0u : 1u;
  const uint16_t w = (LEFT_HALL_W_PORT->IDR & LEFT_HALL_W_PIN) ? 0u : 1u;
  return (uint16_t)((u << 2) | (v << 1) | w);
}

static uint16_t readHallRight(void) {
  const uint16_t u = (RIGHT_HALL_U_PORT->IDR & RIGHT_HALL_U_PIN) ? 0u : 1u;
  const uint16_t v = (RIGHT_HALL_V_PORT->IDR & RIGHT_HALL_V_PIN) ? 0u : 1u;
  const uint16_t w = (RIGHT_HALL_W_PORT->IDR & RIGHT_HALL_W_PIN) ? 0u : 1u;
  return (uint16_t)((u << 2) | (v << 1) | w);
}

static void cycleCounterInit(void) {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static uint8_t controllerFaultActive(void) {
  return (m_motor_1.m_fault != FAULT_CODE_NONE) || (m_motor_2.m_fault != FAULT_CODE_NONE);
}

int main(void) {
  HAL_Init();
  __HAL_RCC_AFIO_CLK_ENABLE();
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
  HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
  HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
  HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
  HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
  HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
  HAL_NVIC_SetPriority(SysTick_IRQn, 3, 0);

  SystemClock_Config();
  cycleCounterInit();

  __HAL_RCC_DMA1_CLK_DISABLE();
  MX_GPIO_Init();
  MX_TIM_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  BLDC_Init();

  HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, GPIO_PIN_SET);
  Input_Lim_Init();
  Input_Init();
  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start(&hadc2);

  poweronMelody();
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);

  int32_t boardTempAdcFixdt = adc_buffer.temp << 16;
  int16_t boardTempAdcFilt = adc_buffer.temp;
  while (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) HAL_Delay(10);

  while (1) {
    if ((buzzerTimer - buzzerTimerPrev) <= (16u * DELAY_IN_MAIN_LOOP)) continue;

    readCommand();
    vesc_protocol_process_pending();
    vesc_protocol_periodic();
    const bool vescLinkActive = vesc_protocol_link_active();
    calcAvgSpeed();

    if (!timeoutFlgSerial && enable == 0 && !controllerFaultActive() &&
        input1[0].cmd > -50 && input1[0].cmd < 50 && input2[0].cmd > -50 && input2[0].cmd < 50) {
      beepShort(6);
      beepShort(4);
      HAL_Delay(100);
      cmdLFixdt = 0;
      cmdRFixdt = 0;
      enable = 1;
      printf("-- Motors enabled --\r\n");
    }

    /* Left and right motor commands are independent. STOP also follows the
     * same rate limiter so the motors decelerate instead of dropping torque
     * abruptly. Snap only the final <=1 command-count residual to exact zero
     * after the rate-limiter target has already reached zero. */
    rateLimiter16(input1[0].cmd, RATE, &cmdLRateFixdt);
    rateLimiter16(input2[0].cmd, RATE, &cmdRRateFixdt);
    filtLowPass32(cmdLRateFixdt >> 4, FILTER, &cmdLFixdt);
    filtLowPass32(cmdRRateFixdt >> 4, FILTER, &cmdRFixdt);
    cmdL = (int16_t)(cmdLFixdt >> 16);
    cmdR = (int16_t)(cmdRFixdt >> 16);

    if (input1[0].cmd == 0 && cmdLRateFixdt == 0 && abs(cmdL) <= 1) {
      cmdLFixdt = 0;
      cmdL = 0;
    }
    if (input2[0].cmd == 0 && cmdRRateFixdt == 0 && abs(cmdR) <= 1) {
      cmdRFixdt = 0;
      cmdR = 0;
    }
    pwml = cmdL;
    pwmr = -cmdR;  // positive cmdR means forward wheel direction, matching positive cmdL


    filtLowPass32(adc_buffer.temp, TEMP_FILT_COEF, &boardTempAdcFixdt);
    boardTempAdcFilt = (int16_t)(boardTempAdcFixdt >> 16);
    board_temp_deg_c = (TEMP_CAL_HIGH_DEG_C - TEMP_CAL_LOW_DEG_C) *
                       (boardTempAdcFilt - TEMP_CAL_LOW_ADC) /
                       (TEMP_CAL_HIGH_ADC - TEMP_CAL_LOW_ADC) + TEMP_CAL_LOW_DEG_C;
    batVoltageCalib = batVoltage * BAT_CALIB_REAL_VOLTAGE / BAT_CALIB_ADC;
    left_dc_curr = -(m_motor_1.m_current_in_counts * 100) / A2BIT_CONV;
    right_dc_curr = -(m_motor_2.m_current_in_counts * 100) / A2BIT_CONV;
    dc_curr = left_dc_curr + right_dc_curr;

    if (!vescLinkActive && (main_loop_counter % 25u) == 0u) process_debug();

    /* Legacy 72-byte telemetry is automatic at 50 Hz when no VESC binary link
     * owns USART3. There is intentionally no user-controlled live telemetry switch anymore.
     * When VESC Tool is connected, unsolicited legacy bytes are suppressed and
     * VESC realtime data is served by its standard GET_VALUES polling. */
    const uint32_t telemetryNowMs = HAL_GetTick();
    if (!vescLinkActive && !timeoutFlgSerial &&
        (uint32_t)(telemetryNowMs - legacyTelemetryPrevMs) >= 20u &&
        huart3.hdmatx != NULL && __HAL_DMA_GET_COUNTER(huart3.hdmatx) == 0u) {
      legacyTelemetryPrevMs = telemetryNowMs;
      feedback.start = SERIAL_START_FRAME;
      feedback.version = 2u;
      feedback.cmdL = cmdL;
      feedback.cmdR = cmdR;
      feedback.rpmL = m_motor_1.m_rpm;
      feedback.rpmR = (int16_t)(-m_motor_2.m_rpm);
      feedback.dutyL_x1000 = m_motor_1.m_duty_now_permille;
      feedback.dutyR_x1000 = (int16_t)(-m_motor_2.m_duty_now_permille);
      feedback.currentMotorL_cA = focCurrentQ4ToCentiAmp(m_motor_1.m_iq_q4);
      feedback.currentMotorR_cA = focCurrentQ4ToCentiAmp(-((int32_t)m_motor_2.m_iq_q4));
      feedback.currentInL_cA = left_dc_curr;
      feedback.currentInR_cA = right_dc_curr;
      feedback.idL_cA = focCurrentQ4ToCentiAmp(m_motor_1.m_id_q4);
      feedback.idR_cA = focCurrentQ4ToCentiAmp(m_motor_2.m_id_q4);
      feedback.iqL_cA = focCurrentQ4ToCentiAmp(m_motor_1.m_iq_q4);
      feedback.iqR_cA = focCurrentQ4ToCentiAmp(-((int32_t)m_motor_2.m_iq_q4));
      feedback.vdL_cV = focVoltageToCentiVolt(m_motor_1.m_vd);
      feedback.vdR_cV = focVoltageToCentiVolt(m_motor_2.m_vd);
      feedback.vqL_cV = focVoltageToCentiVolt(m_motor_1.m_vq);
      feedback.vqR_cV = focVoltageToCentiVolt((int16_t)-m_motor_2.m_vq);
      feedback.vIn_x100 = batVoltageCalib;
      feedback.boardTemp_x10 = board_temp_deg_c;
      feedback.hallL = readHallLeft();
      feedback.hallR = readHallRight();
      feedback.faultL = (uint16_t)m_motor_1.m_fault;
      feedback.faultR = (uint16_t)m_motor_2.m_fault;
      feedback.adc_dcl = adc_buffer.dcl;
      feedback.adc_rla = adc_buffer.rlA;
      feedback.adc_rlb = adc_buffer.rlB;
      feedback.adc_dcr = adc_buffer.dcr;
      feedback.adc_rrb = adc_buffer.rrB;
      feedback.adc_rrc = adc_buffer.rrC;
      feedback.status = (enable ? SERIAL_STATUS_ENABLED : 0u) |
                        (timeoutFlgSerial ? SERIAL_STATUS_TIMEOUT : 0u) |
                        ((m_motor_1.m_fault != FAULT_CODE_NONE) ? SERIAL_STATUS_LEFT_FAULT : 0u) |
                        ((m_motor_2.m_fault != FAULT_CODE_NONE) ? SERIAL_STATUS_RIGHT_FAULT : 0u);
      feedback.foc_isr_cycles = foc_isr_cycles;
      feedback.checksum = feedbackChecksum(&feedback);
      HAL_UART_Transmit_DMA(&huart3, (uint8_t *)&feedback, sizeof(feedback));
    }

    poweroffPressCheck();

    if ((TEMP_POWEROFF_ENABLE && board_temp_deg_c >= TEMP_POWEROFF && speedAvgAbs < 20) ||
        (batVoltage < BAT_DEAD && speedAvgAbs < 20)) {
      poweroff();
    } else if (controllerFaultActive()) {
      enable = 0;
      beepCount(1, 24, 1);
    } else if (TEMP_WARNING_ENABLE && board_temp_deg_c >= TEMP_WARNING) {
      beepCount(5, 24, 1);
    } else if (BAT_LVL1_ENABLE && batVoltage < BAT_LVL1) {
      beepCount(0, 10, 6);
    } else if (BAT_LVL2_ENABLE && batVoltage < BAT_LVL2) {
      beepCount(0, 10, 30);
    } else {
      beepCount(0, 0, 0);
    }

    if (abs(cmdL) > 50 || abs(cmdR) > 50) inactivityTimeoutCounter = 0;
    else ++inactivityTimeoutCounter;
    if (inactivityTimeoutCounter > (INACTIVITY_TIMEOUT * 60u * 1000u) / (DELAY_IN_MAIN_LOOP + 1u)) poweroff();

    buzzerTimerPrev = buzzerTimer;
    ++main_loop_counter;
  }
}

void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL16;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType           = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource        = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider       = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider      = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider      = RCC_HCLK_DIV1;

  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  PeriphClkInit.PeriphClockSelection    = RCC_PERIPHCLK_ADC;
  // PeriphClkInit.AdcClockSelection    = RCC_ADCPCLK2_DIV8;  // 8 MHz
  PeriphClkInit.AdcClockSelection       = RCC_ADCPCLK2_DIV4;  // 16 MHz
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

  /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 3, 0);
}
