#include <stdio.h>
#include <stdlib.h>
#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"
#include "util.h"
#include "BLDC_controller.h"
#include "rtwtypes.h"
#include "comms.h"
#include "bldc.h"

void SystemClock_Config(void);

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern UART_HandleTypeDef huart3;
extern volatile adc_buf_t adc_buffer;
extern P rtP_Left;
extern P rtP_Right;
extern ExtY rtY_Left;
extern ExtY rtY_Right;
extern ExtU rtU_Left;
extern ExtU rtU_Right;
extern InputStruct input1[];
extern InputStruct input2[];
extern int16_t speedAvg;
extern int16_t speedAvgAbs;
extern uint8_t timeoutFlgSerial;
extern uint8_t ctrlModReq;
extern uint8_t ctrlModReqRaw;
extern volatile int pwml;
extern volatile int pwmr;
extern uint8_t enable;
extern int16_t batVoltage;
extern volatile uint32_t buzzerTimer;

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
  int16_t cmdL;
  int16_t cmdR;
  int16_t rpmL;
  int16_t rpmR;
  int16_t iqL_cA;
  int16_t iqR_cA;
  int16_t idL_cA;
  int16_t idR_cA;
  int16_t idcL_cA;
  int16_t idcR_cA;
  int16_t batVoltage_x100;
  int16_t boardTemp_x10;
  uint16_t hallL;
  uint16_t hallR;
  uint16_t adc_dcl;
  uint16_t adc_rla;
  uint16_t adc_rlb;
  uint16_t adc_dcr;
  uint16_t adc_rrb;
  uint16_t adc_rrc;
  uint16_t status;
  uint16_t mode;
  uint16_t calibration_permille;
  uint32_t telemetry_seq;
  uint32_t foc_isr_cycles;
  uint16_t checksum;
} SerialFeedback;

static SerialFeedback feedback;
static int16_t cmdLRateFixdt = 0;
static int16_t cmdRRateFixdt = 0;
static int32_t cmdLFixdt = 0;
static int32_t cmdRFixdt = 0;
static uint32_t buzzerTimerPrev = 0;
static uint32_t inactivityTimeoutCounter = 0;
static uint32_t telemetrySeq = 0;

static uint16_t feedbackChecksum(const SerialFeedback *f) {
  uint16_t c = f->start;
  c ^= (uint16_t)f->cmdL ^ (uint16_t)f->cmdR;
  c ^= (uint16_t)f->rpmL ^ (uint16_t)f->rpmR;
  c ^= (uint16_t)f->iqL_cA ^ (uint16_t)f->iqR_cA;
  c ^= (uint16_t)f->idL_cA ^ (uint16_t)f->idR_cA;
  c ^= (uint16_t)f->idcL_cA ^ (uint16_t)f->idcR_cA;
  c ^= (uint16_t)f->batVoltage_x100 ^ (uint16_t)f->boardTemp_x10;
  c ^= f->hallL ^ f->hallR;
  c ^= f->adc_dcl ^ f->adc_rla ^ f->adc_rlb ^ f->adc_dcr ^ f->adc_rrb ^ f->adc_rrc;
  c ^= f->status ^ f->mode ^ f->calibration_permille;
  c ^= (uint16_t)(f->telemetry_seq & 0xffffu) ^ (uint16_t)(f->telemetry_seq >> 16);
  c ^= (uint16_t)(f->foc_isr_cycles & 0xffffu) ^ (uint16_t)(f->foc_isr_cycles >> 16);
  return c;
}

static int16_t focCurrentQ4ToCentiAmp(int32_t currentQ4) {
  /* BLDC_controller.c shifts phase-current inputs by 4 before Clarke/Park:
   *     rtb = rtU->i_phaXX << 4
   * and rtY.iq / rtY.id are exported without shifting back. Therefore dq is
   * Q4 current-count, i.e. 16 * A2BIT_CONV units per ampere. */
  const int32_t denom = (int32_t)A2BIT_CONV * 16;
  return (int16_t)(((int32_t)currentQ4 * 100) / denom);
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
  return (ctrlModReq != SVPWM_MODE) && (rtY_Left.z_errCode || rtY_Right.z_errCode);
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
    if ((buzzerTimer - buzzerTimerPrev) < (16u * DELAY_IN_MAIN_LOOP)) continue;

    readCommand();
    calcAvgSpeed();

    if (!timeoutFlgSerial && enable == 0 && !currentCalibrationActive() && !controllerFaultActive() &&
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
    if (currentCalibrationActive()) {
      left_dc_curr = 0;
      right_dc_curr = 0;
    } else {
      left_dc_curr = -(rtU_Left.i_DCLink * 100) / A2BIT_CONV;
      right_dc_curr = -(rtU_Right.i_DCLink * 100) / A2BIT_CONV;
    }
    dc_curr = left_dc_curr + right_dc_curr;

    if ((main_loop_counter % 25u) == 0u) process_debug();

    /* Deterministic 50-Hz telemetry slot. telemetry_seq advances on every slot
     * so the host can detect a rare skipped packet if USART3 DMA is busy. */
    if ((main_loop_counter % (MAIN_LOOP_HZ / TELEMETRY_HZ)) == 0u) {
      const uint32_t scheduledTelemetrySeq = ++telemetrySeq;
      if (huart3.hdmatx != NULL && __HAL_DMA_GET_COUNTER(huart3.hdmatx) == 0u) {
      feedback.start = SERIAL_START_FRAME;
      /* Report the rate-limited command actually applied to each motor. */
      feedback.cmdL = cmdL;
      feedback.cmdR = cmdR;
      feedback.rpmL = (int16_t)rtY_Left.n_mot;
      /* Right motor is physically mirrored and internally driven with pwmr=-cmdR.
       * Normalize signed wheel-speed and torque-current to the same host convention
       * as cmdR: positive means forward for both wheels. */
      feedback.rpmR = (int16_t)(-rtY_Right.n_mot);
      feedback.iqL_cA = focCurrentQ4ToCentiAmp(foc_iqL_q4);
      feedback.iqR_cA = focCurrentQ4ToCentiAmp(-((int32_t)foc_iqR_q4));
      /* id is flux-axis current. Mirroring wheel direction does not invert the
       * physical d-axis sign, so idR is intentionally not negated. */
      feedback.idL_cA = focCurrentQ4ToCentiAmp(foc_idL_q4);
      feedback.idR_cA = focCurrentQ4ToCentiAmp(foc_idR_q4);
      feedback.idcL_cA = left_dc_curr;
      feedback.idcR_cA = right_dc_curr;
      feedback.batVoltage_x100 = batVoltageCalib;
      feedback.boardTemp_x10 = board_temp_deg_c;
      feedback.hallL = readHallLeft();
      feedback.hallR = readHallRight();
      feedback.adc_dcl = adc_buffer.dcl;
      feedback.adc_rla = adc_buffer.rlA;
      feedback.adc_rlb = adc_buffer.rlB;
      feedback.adc_dcr = adc_buffer.dcr;
      feedback.adc_rrb = adc_buffer.rrB;
      feedback.adc_rrc = adc_buffer.rrC;
      feedback.status = (enable ? SERIAL_STATUS_ENABLED : 0u) |
                        (timeoutFlgSerial ? SERIAL_STATUS_TIMEOUT : 0u) |
                        ((ctrlModReq != SVPWM_MODE && rtY_Left.z_errCode) ? SERIAL_STATUS_LEFT_FAULT : 0u) |
                        ((ctrlModReq != SVPWM_MODE && rtY_Right.z_errCode) ? SERIAL_STATUS_RIGHT_FAULT : 0u) |
                        (currentCalibrationActive() ? SERIAL_STATUS_CALIBRATING : 0u);
      feedback.mode = ctrlModReqRaw;
      feedback.calibration_permille = currentCalibrationProgressPermille();
      feedback.telemetry_seq = scheduledTelemetrySeq;
      feedback.foc_isr_cycles = foc_isr_cycles;
      feedback.checksum = feedbackChecksum(&feedback);
      HAL_UART_Transmit_DMA(&huart3, (uint8_t *)&feedback, sizeof(feedback));
      }
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
