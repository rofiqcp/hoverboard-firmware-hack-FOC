#include <stdio.h>
#include <stdlib.h>
#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"
#include "util.h"
#include "motor/mcpwm_foc.h"
#include "comms.h"
#include "motor/mc_interface.h"

void SystemClock_Config(void);

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern UART_HandleTypeDef huart3;
extern volatile adc_buf_t m_adc_buffer;
extern InputStruct input1[];
extern InputStruct input2[];
extern int16_t speedAvg;
extern int16_t speedAvgAbs;
extern uint8_t timeoutFlgSerial;
extern uint8_t m_motor_enable;
extern int16_t m_input_voltage_adc;
extern volatile uint32_t m_buzzer_timer;

volatile uint32_t main_loop_counter = 0;
int16_t batVoltageCalib = 0;
int16_t board_temp_deg_c = 0;
int16_t left_dc_curr = 0;
int16_t right_dc_curr = 0;
int16_t dc_curr = 0;
int16_t cmdL = 0;
int16_t cmdR = 0;
volatile int16_t m_motor_target_left = 0;
volatile int16_t m_motor_target_right = 0;
volatile uint8_t m_control_mode_left = CONTROL_MODE_DUTY;
volatile uint8_t m_control_mode_right = CONTROL_MODE_DUTY;

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
  int32_t encoder_position;
  int16_t position_target;
  int16_t position_speed_ref;
  int16_t encoder_elec_angle_x10;
  int16_t encoder_rpm;
  uint16_t encoder_sync_state;
  uint16_t hw_profile;
  uint32_t foc_isr_cycles_max;
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
  c ^= (uint16_t)(f->encoder_position & 0xffffu) ^ (uint16_t)((uint32_t)f->encoder_position >> 16);
  c ^= (uint16_t)f->position_target ^ (uint16_t)f->position_speed_ref;
  c ^= (uint16_t)f->encoder_elec_angle_x10 ^ (uint16_t)f->encoder_rpm;
  c ^= f->encoder_sync_state ^ f->hw_profile;
  c ^= (uint16_t)(f->foc_isr_cycles_max & 0xffffu) ^ (uint16_t)(f->foc_isr_cycles_max >> 16);
  return c;
}

static int16_t focCurrentQ4ToCentiAmp(int32_t currentQ4) {
  /* mcpwm_foc.c shifts phase-current inputs by 4 before Clarke/Park. D/Q
   * telemetry therefore remains Q4 current-count: 16 * A2BIT_CONV per ampere. */
  const int32_t denom = (int32_t)A2BIT_CONV * 16;
  return (int16_t)(((int32_t)currentQ4 * 100) / denom);
}

static uint16_t readHallLeft(void) {
  return m_sensor_hall_left;
}

static uint16_t readHallRight(void) {
  return m_sensor_hall_right;
}

static void cycleCounterInit(void) {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static uint8_t controllerFaultActive(void) {
  return (uint8_t)(m_motor_1.m_output.fault_code || m_motor_2.m_output.fault_code);
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
  mcpwm_foc_init_defaults();

  HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, GPIO_PIN_SET);
  Input_Lim_Init();
  Input_Init();
  loadAllParamVal();
  mc_interface_init();
  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start(&hadc2);

  poweronMelody();
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);

  int32_t boardTempAdcFixdt = m_adc_buffer.temp << 16;
  int16_t boardTempAdcFilt = m_adc_buffer.temp;
  while (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) HAL_Delay(10);

  while (1) {
    if ((m_buzzer_timer - buzzerTimerPrev) < (16u * DELAY_IN_MAIN_LOOP)) continue;

    if (currentCalibrationResetPending()) currentCalibrationFinalizeReset();
    if (m_sensor_mode_left == MCCONF_SENSOR_ENCODER_AB) encoder_sync_service(currentCalibrationActive());
    readCommand();
    calcAvgSpeed();

    const uint8_t profileReady = (uint8_t)(m_sensor_mode_left != MCCONF_SENSOR_ENCODER_AB ||
        (m_encoder_sync_state == 5u && m_encoder_sync_ok));
    const uint8_t positionMoveRequested = (uint8_t)(m_control_mode_sel_left == MCCONF_CONTROL_POSITION &&
        labs((long)p_pid_set_counts - (long)m_encoder_position) > p_pid_deadband_counts);
    const uint8_t normalStopRequested = (uint8_t)(!positionMoveRequested &&
                                                   input1[0].cmd == 0 && input2[0].cmd == 0);

    /* Hard inhibit conditions really disable the bridge. A normal user STOP
     * does not: it selects the neutral 50/50/50 zero vector so torque becomes
     * zero while phase-current ADC sampling stays in its valid operating point. */
    if (timeoutFlgSerial || !profileReady || currentCalibrationActive() || controllerFaultActive()) {
      m_motor_enable = 0u;
    } else if (m_motor_enable == 0u) {
      /* Match the proven baseline: after a valid serial frame the bridge is
       * armed even at zero command, but the commanded voltage/torque is zero. */
      m_motor_enable = 1u;
      printf("# MOTOR_ARM_NEUTRAL\r\n");
    }

    if (normalStopRequested) {
      /* STOP is authoritative and bypasses all command smoothing. This removes
       * the previous tail where the rate limiter / LPF could keep a non-zero
       * setpoint after the host had already requested zero. */
      cmdLRateFixdt = 0;
      cmdRRateFixdt = 0;
      cmdLFixdt = 0;
      cmdRFixdt = 0;
      cmdL = 0;
      cmdR = 0;
      mc_interface_reset_control();
    } else {
      rateLimiter16(input1[0].cmd, m_command_rate, &cmdLRateFixdt);
      rateLimiter16(input2[0].cmd, m_command_rate, &cmdRRateFixdt);
      filtLowPass32(cmdLRateFixdt >> 4, (uint16_t)m_command_filter, &cmdLFixdt);
      filtLowPass32(cmdRRateFixdt >> 4, (uint16_t)m_command_filter, &cmdRFixdt);
      cmdL = (int16_t)(cmdLFixdt >> 16);
      cmdR = (int16_t)(cmdRFixdt >> 16);
    }

    int16_t controlL = 0, controlR = 0;
    uint8_t genL = CONTROL_MODE_DUTY, genR = CONTROL_MODE_DUTY;
    if (m_sensor_mode_left == MCCONF_SENSOR_ENCODER_AB) encoder_left_update();
    mc_interface_update(cmdL, cmdR,
                        m_sensor_rpm_left, (int16_t)(-m_sensor_rpm_right),
                        &controlL, &controlR, &genL, &genR);

    /* The normal STOP path always wins over every outer-loop integrator. */
    if (normalStopRequested) {
      controlL = 0;
      controlR = 0;
      mc_interface_reset_control();
    }

    m_motor_target_left = controlL;
    m_motor_target_right = controlR;
    m_control_mode_left = genL;
    m_control_mode_right = genR;

    filtLowPass32(m_adc_buffer.temp, TEMP_FILT_COEF, &boardTempAdcFixdt);
    boardTempAdcFilt = (int16_t)(boardTempAdcFixdt >> 16);
    board_temp_deg_c = (TEMP_CAL_HIGH_DEG_C - TEMP_CAL_LOW_DEG_C) *
                       (boardTempAdcFilt - TEMP_CAL_LOW_ADC) /
                       (TEMP_CAL_HIGH_ADC - TEMP_CAL_LOW_ADC) + TEMP_CAL_LOW_DEG_C;
    batVoltageCalib = m_input_voltage_adc * BAT_CALIB_REAL_VOLTAGE / BAT_CALIB_ADC;
    if (currentCalibrationActive()) {
      left_dc_curr = 0;
      right_dc_curr = 0;
    } else {
      left_dc_curr = -(m_motor_1.m_input.current_input * 100) / A2BIT_CONV;
      right_dc_curr = -(m_motor_2.m_input.current_input * 100) / A2BIT_CONV;
    }
    dc_curr = left_dc_curr + right_dc_curr;

    if ((main_loop_counter % 25u) == 0u) process_debug();

    /* Deterministic 50-Hz telemetry slot. telemetry_seq advances on every slot
     * so the host can detect a rare skipped packet if USART3 DMA is busy. */
    if (m_live_stream_enabled && (main_loop_counter % (MAIN_LOOP_HZ / TELEMETRY_HZ)) == 0u) {
      const uint32_t scheduledTelemetrySeq = ++telemetrySeq;
      if (huart3.hdmatx != NULL && __HAL_DMA_GET_COUNTER(huart3.hdmatx) == 0u) {
      feedback.start = SERIAL_START_FRAME;
      /* Report the rate-limited command actually applied to each motor. */
      feedback.cmdL = cmdL;
      feedback.cmdR = cmdR;
      feedback.rpmL = m_sensor_rpm_left;
      /* Right motor is physically mirrored internally; host telemetry is normalized so positive means forward on both wheels. */
      feedback.rpmR = (int16_t)(-m_sensor_rpm_right);
      feedback.iqL_cA = focCurrentQ4ToCentiAmp(m_foc_iq_left_q4);
      feedback.iqR_cA = focCurrentQ4ToCentiAmp(-((int32_t)m_foc_iq_right_q4));
      /* id is flux-axis current. Mirroring wheel direction does not invert the
       * physical d-axis sign, so idR is intentionally not negated. */
      feedback.idL_cA = focCurrentQ4ToCentiAmp(m_foc_id_left_q4);
      feedback.idR_cA = focCurrentQ4ToCentiAmp(m_foc_id_right_q4);
      feedback.idcL_cA = left_dc_curr;
      feedback.idcR_cA = right_dc_curr;
      feedback.batVoltage_x100 = batVoltageCalib;
      feedback.boardTemp_x10 = board_temp_deg_c;
      feedback.hallL = readHallLeft();
      feedback.hallR = readHallRight();
      feedback.adc_dcl = m_adc_buffer.dcl;
      feedback.adc_rla = m_adc_buffer.rlA;
      feedback.adc_rlb = m_adc_buffer.rlB;
      feedback.adc_dcr = m_adc_buffer.dcr;
      feedback.adc_rrb = m_adc_buffer.rrB;
      feedback.adc_rrc = m_adc_buffer.rrC;
      feedback.status = (m_motor_enable ? SERIAL_STATUS_ENABLED : 0u) |
                        (timeoutFlgSerial ? SERIAL_STATUS_TIMEOUT : 0u) |
                        (m_motor_1.m_output.fault_code ? SERIAL_STATUS_LEFT_FAULT : 0u) |
                        (m_motor_2.m_output.fault_code ? SERIAL_STATUS_RIGHT_FAULT : 0u) |
                        (currentCalibrationActive() ? SERIAL_STATUS_CALIBRATING : 0u) |
                        (m_adc_current_valid ? SERIAL_STATUS_ADC_CURRENT_VALID : 0u) |
                        (m_adc_current_valid_left ? SERIAL_STATUS_ADC_LEFT_VALID : 0u) |
                        (m_adc_current_valid_right ? SERIAL_STATUS_ADC_RIGHT_VALID : 0u);
      feedback.mode = mc_interface_pack_mode_word();
      feedback.calibration_permille = currentCalibrationProgressPermille();
      feedback.telemetry_seq = scheduledTelemetrySeq;
      feedback.foc_isr_cycles = m_foc_isr_cycles;
      feedback.encoder_position = m_encoder_position;
      feedback.position_target = p_pid_set_counts;
      feedback.position_speed_ref = m_position_speed_set;
      feedback.encoder_elec_angle_x10 = m_encoder_elec_angle_deg_x10;
      feedback.encoder_rpm = m_encoder_rpm;
      feedback.encoder_sync_state = m_encoder_sync_state;
      feedback.hw_profile = HW_PROFILE_ID;
      feedback.foc_isr_cycles_max = m_foc_isr_cycles_max;
      feedback.checksum = feedbackChecksum(&feedback);
      HAL_UART_Transmit_DMA(&huart3, (uint8_t *)&feedback, sizeof(feedback));
      }
    }

    poweroffPressCheck();

    if ((TEMP_POWEROFF_ENABLE && board_temp_deg_c >= TEMP_POWEROFF && speedAvgAbs < 20) ||
        (m_input_voltage_adc < BAT_DEAD && speedAvgAbs < 20)) {
      poweroff();
    } else if (timeoutFlgSerial) {
      m_motor_enable = 0u;
      mc_interface_reset_control();
      beepCount(0, 0, 0);
    } else if (controllerFaultActive()) {
      m_motor_enable = 0;
      beepCount(1, 24, 1);
    } else if (TEMP_WARNING_ENABLE && board_temp_deg_c >= TEMP_WARNING) {
      beepCount(5, 24, 1);
    } else if (BAT_LVL1_ENABLE && m_input_voltage_adc < BAT_LVL1) {
      beepCount(0, 10, 6);
    } else if (BAT_LVL2_ENABLE && m_input_voltage_adc < BAT_LVL2) {
      beepCount(0, 10, 30);
    } else {
      beepCount(0, 0, 0);
    }

    if (abs(cmdL) > 50 || abs(cmdR) > 50) inactivityTimeoutCounter = 0;
    else ++inactivityTimeoutCounter;
    if (inactivityTimeoutCounter > (INACTIVITY_TIMEOUT * 60u * 1000u) / (DELAY_IN_MAIN_LOOP + 1u)) poweroff();

    buzzerTimerPrev = m_buzzer_timer;
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
