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
extern volatile int pwml;
extern volatile int pwmr;
extern uint8_t enable;
extern int16_t batVoltage;
extern int16_t odom_r;
extern int16_t odom_l;
extern volatile uint32_t buzzerTimer;
extern volatile uint32_t foc_isr_cycles;
extern volatile uint32_t foc_isr_cycles_max;

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
  int16_t cmd1;
  int16_t cmd2;
  int16_t speedR_meas;
  int16_t speedL_meas;
  int16_t wheelR_cnt;
  int16_t wheelL_cnt;
  int16_t batVoltage;
  int16_t boardTemp;
  uint16_t status;
  uint32_t foc_isr_cycles;
  uint32_t foc_isr_cycles_max;
  uint16_t checksum;
} SerialFeedback;

static SerialFeedback feedback;
static int16_t speed = 0;
static int16_t steer = 0;
static int16_t steerRateFixdt = 0;
static int16_t speedRateFixdt = 0;
static int32_t steerFixdt = 0;
static int32_t speedFixdt = 0;
static uint32_t buzzerTimerPrev = 0;
static uint32_t inactivityTimeoutCounter = 0;

static uint16_t feedbackChecksum(const SerialFeedback *f) {
  uint16_t c = f->start ^ (uint16_t)f->cmd1 ^ (uint16_t)f->cmd2;
  c ^= (uint16_t)f->speedR_meas ^ (uint16_t)f->speedL_meas;
  c ^= (uint16_t)f->wheelR_cnt ^ (uint16_t)f->wheelL_cnt;
  c ^= (uint16_t)f->batVoltage ^ (uint16_t)f->boardTemp ^ f->status;
  c ^= (uint16_t)(f->foc_isr_cycles & 0xffffu) ^ (uint16_t)(f->foc_isr_cycles >> 16);
  c ^= (uint16_t)(f->foc_isr_cycles_max & 0xffffu) ^ (uint16_t)(f->foc_isr_cycles_max >> 16);
  return c;
}

static void cycleCounterInit(void) {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
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
    calcAvgSpeed();

    if (!timeoutFlgSerial && enable == 0 && !rtY_Left.z_errCode && !rtY_Right.z_errCode &&
        input1[0].cmd > -50 && input1[0].cmd < 50 && input2[0].cmd > -50 && input2[0].cmd < 50) {
      beepShort(6);
      beepShort(4);
      HAL_Delay(100);
      steerFixdt = 0;
      speedFixdt = 0;
      enable = 1;
      printf("-- Motors enabled --\r\n");
    }

    rateLimiter16(input1[0].cmd, RATE, &steerRateFixdt);
    rateLimiter16(input2[0].cmd, RATE, &speedRateFixdt);
    filtLowPass32(steerRateFixdt >> 4, FILTER, &steerFixdt);
    filtLowPass32(speedRateFixdt >> 4, FILTER, &speedFixdt);
    steer = (int16_t)(steerFixdt >> 16);
    speed = (int16_t)(speedFixdt >> 16);
    mixerFcn(speed << 4, steer << 4, &cmdR, &cmdL);
    pwmr = -cmdR;
    pwml = cmdL;

    filtLowPass32(adc_buffer.temp, TEMP_FILT_COEF, &boardTempAdcFixdt);
    boardTempAdcFilt = (int16_t)(boardTempAdcFixdt >> 16);
    board_temp_deg_c = (TEMP_CAL_HIGH_DEG_C - TEMP_CAL_LOW_DEG_C) *
                       (boardTempAdcFilt - TEMP_CAL_LOW_ADC) /
                       (TEMP_CAL_HIGH_ADC - TEMP_CAL_LOW_ADC) + TEMP_CAL_LOW_DEG_C;
    batVoltageCalib = batVoltage * BAT_CALIB_REAL_VOLTAGE / BAT_CALIB_ADC;
    left_dc_curr = -(rtU_Left.i_DCLink * 100) / A2BIT_CONV;
    right_dc_curr = -(rtU_Right.i_DCLink * 100) / A2BIT_CONV;
    dc_curr = left_dc_curr + right_dc_curr;

    if ((main_loop_counter % 25u) == 0u) process_debug();

    if ((main_loop_counter % 2u) == 0u && huart3.hdmatx != NULL && __HAL_DMA_GET_COUNTER(huart3.hdmatx) == 0u) {
      feedback.start = SERIAL_START_FRAME;
      feedback.cmd1 = input1[0].cmd;
      feedback.cmd2 = input2[0].cmd;
      feedback.speedR_meas = (int16_t)rtY_Right.n_mot;
      feedback.speedL_meas = (int16_t)rtY_Left.n_mot;
      feedback.wheelR_cnt = odom_r;
      feedback.wheelL_cnt = odom_l;
      feedback.batVoltage = batVoltageCalib;
      feedback.boardTemp = board_temp_deg_c;
      feedback.status = (enable ? SERIAL_STATUS_ENABLED : 0u) |
                        (timeoutFlgSerial ? SERIAL_STATUS_TIMEOUT : 0u) |
                        (rtY_Left.z_errCode ? SERIAL_STATUS_LEFT_FAULT : 0u) |
                        (rtY_Right.z_errCode ? SERIAL_STATUS_RIGHT_FAULT : 0u);
      feedback.foc_isr_cycles = foc_isr_cycles;
      feedback.foc_isr_cycles_max = foc_isr_cycles_max;
      feedback.checksum = feedbackChecksum(&feedback);
      HAL_UART_Transmit_DMA(&huart3, (uint8_t *)&feedback, sizeof(feedback));
    }

    poweroffPressCheck();

    if ((TEMP_POWEROFF_ENABLE && board_temp_deg_c >= TEMP_POWEROFF && speedAvgAbs < 20) ||
        (batVoltage < BAT_DEAD && speedAvgAbs < 20)) {
      poweroff();
    } else if (rtY_Left.z_errCode || rtY_Right.z_errCode) {
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
