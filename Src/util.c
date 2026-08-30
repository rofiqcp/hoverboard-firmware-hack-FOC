#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"
#include "eeprom.h"
#include "util.h"
#include "BLDC_controller.h"
#include "rtwtypes.h"
#include "comms.h"
#include "advanced_control.h"

extern UART_HandleTypeDef huart3;
extern uint8_t buzzerCount;
extern uint8_t buzzerFreq;
extern uint8_t buzzerPattern;
extern uint8_t enable;

RT_MODEL rtM_Left_;
RT_MODEL rtM_Right_;
RT_MODEL *const rtM_Left = &rtM_Left_;
RT_MODEL *const rtM_Right = &rtM_Right_;

extern P rtP_Left;
DW rtDW_Left;
ExtU rtU_Left;
ExtY rtY_Left;
P rtP_Right;
DW rtDW_Right;
ExtU rtU_Right;
ExtY rtY_Right;

InputStruct input1[INPUTS_NR] = {{0, 0, 0, PRI_INPUT1}};
InputStruct input2[INPUTS_NR] = {{0, 0, 0, PRI_INPUT2}};

int16_t speedAvg = 0;
int16_t speedAvgAbs = 0;
uint8_t timeoutFlgSerial = 1;
uint8_t ctrlModReqRaw = CTRL_MOD_REQ;
uint8_t ctrlModReq = OPEN_MODE;

uint16_t VirtAddVarTab[NB_OF_VAR];

static int16_t inputMax = 1000;
static int16_t inputMin = -1000;
static uint8_t rxBuffer[SERIAL_BUFFER_SIZE];
static uint16_t serialTimeoutCount = SERIAL_TIMEOUT;
static SerialCommand serialCommand = {SERIAL_START_FRAME, 0, 0, 0};

static uint8_t frameBuffer[sizeof(SerialCommand)];
static uint8_t frameIndex = 0;
static uint8_t debugLine[SERIAL_DEBUG_LINE_SIZE];
static uint8_t debugIndex = 0;

#ifdef __GNUC__
int _write(int file, char *data, int len) {
  (void)file;
  if (len <= 0) return 0;
  /* Debug output and binary feedback share USART3. Wait for feedback DMA to finish. */
  while (huart3.gState != HAL_UART_STATE_READY) { }
  return (HAL_UART_Transmit(&huart3, (uint8_t *)data, (uint16_t)len, 1000) == HAL_OK) ? len : 0;
}
#endif

void BLDC_Init(void) {
#ifdef HW_PROFILE_ENC_HALL
  rtP_Left.b_angleMeasEna = 1;
  rtP_Left.n_polePairs = (uint8_t)enc_pole_pairs;
#else
  rtP_Left.b_angleMeasEna = 0;
#endif
  rtP_Left.z_selPhaCurMeasABC = 0;
  /* Runtime modes select COM/SIN/FOC inside the motor ISR. */
  rtP_Left.z_ctrlTypSel = FOC_CTRL;
  rtP_Left.b_diagEna = DIAG_ENA;
  rtP_Left.i_max = (I_MOT_MAX * A2BIT_CONV) << 4;
  rtP_Left.n_max = N_MOT_MAX << 4;
  rtP_Left.b_fieldWeakEna = FIELD_WEAK_ENA;
  rtP_Left.id_fieldWeakMax = (FIELD_WEAK_MAX * A2BIT_CONV) << 4;
  rtP_Left.a_phaAdvMax = PHASE_ADV_MAX << 4;
  rtP_Left.r_fieldWeakHi = FIELD_WEAK_HI << 4;
  rtP_Left.r_fieldWeakLo = FIELD_WEAK_LO << 4;

  rtP_Right = rtP_Left;
  rtP_Right.b_angleMeasEna = 0;
  rtP_Right.z_selPhaCurMeasABC = 1;

  rtM_Left->defaultParam = &rtP_Left;
  rtM_Left->dwork = &rtDW_Left;
  rtM_Left->inputs = &rtU_Left;
  rtM_Left->outputs = &rtY_Left;
  rtM_Right->defaultParam = &rtP_Right;
  rtM_Right->dwork = &rtDW_Right;
  rtM_Right->inputs = &rtU_Right;
  rtM_Right->outputs = &rtY_Right;

  BLDC_controller_initialize(rtM_Left);
  BLDC_controller_initialize(rtM_Right);
}

void Input_Lim_Init(void) {
  if (rtP_Left.b_fieldWeakEna || rtP_Right.b_fieldWeakEna) {
    inputMax = MAX(1000, FIELD_WEAK_HI);
    inputMin = MIN(-1000, -FIELD_WEAK_HI);
  } else {
    inputMax = 1000;
    inputMin = -1000;
  }
}

void UART_DisableRxErrors(UART_HandleTypeDef *huart) {
  CLEAR_BIT(huart->Instance->CR1, USART_CR1_PEIE);
  CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);
}

void Input_Init(void) {
  UART3_Init();
  HAL_UART_Receive_DMA(&huart3, rxBuffer, sizeof(rxBuffer));
  UART_DisableRxErrors(&huart3);

  for (uint16_t i = 0; i < NB_OF_VAR; ++i) VirtAddVarTab[i] = (uint16_t)(1000u + i);
  HAL_FLASH_Unlock();
  EE_Init();
  HAL_FLASH_Lock();
}

void poweronMelody(void) {
  buzzerCount = 0;
  for (int i = 8; i >= 0; --i) {
    buzzerFreq = (uint8_t)i;
    HAL_Delay(100);
  }
  buzzerFreq = 0;
}

void beepCount(uint8_t cnt, uint8_t freq, uint8_t pattern) {
  buzzerCount = cnt;
  buzzerFreq = freq;
  buzzerPattern = pattern;
}

void beepShort(uint8_t freq) {
  buzzerCount = 0;
  buzzerFreq = freq;
  HAL_Delay(100);
  buzzerFreq = 0;
}

void calcAvgSpeed(void) {
  speedAvg = (rtY_Left.n_mot - rtY_Right.n_mot) / 2;
  speedAvgAbs = abs(speedAvg);
}

static void serialAcceptCommand(const uint8_t *frame) {
  SerialCommand candidate;
  memcpy(&candidate, frame, sizeof(candidate));
  const uint16_t checksum = (uint16_t)(candidate.start ^ (uint16_t)candidate.cmdL ^ (uint16_t)candidate.cmdR);
  if (candidate.start == SERIAL_START_FRAME && candidate.checksum == checksum) {
    serialCommand = candidate;
    serialTimeoutCount = 0;
    timeoutFlgSerial = 0;
  }
}

static void debugAcceptByte(uint8_t byte) {
  if (byte == '\r' || byte == '\n') {
    if (debugIndex == 0) return;
    debugLine[debugIndex++] = '\n';
    handle_input(debugLine, debugIndex);
    debugIndex = 0;
    return;
  }
  if (byte < 0x20 || byte > 0x7e) {
    debugIndex = 0;
    return;
  }
  if (debugIndex < SERIAL_DEBUG_LINE_SIZE - 1) {
    debugLine[debugIndex++] = byte;
  } else {
    debugIndex = 0;
  }
}

static void serialAcceptByte(uint8_t byte) {
  const uint8_t startLo = (uint8_t)(SERIAL_START_FRAME & 0xffu);
  const uint8_t startHi = (uint8_t)(SERIAL_START_FRAME >> 8);

  if (frameIndex == 0) {
    if (byte == startLo) {
      frameBuffer[frameIndex++] = byte;
    } else {
      debugAcceptByte(byte);
    }
    return;
  }

  if (frameIndex == 1) {
    if (byte == startHi) {
      frameBuffer[frameIndex++] = byte;
    } else {
      debugAcceptByte(frameBuffer[0]);
      frameIndex = 0;
      serialAcceptByte(byte);
    }
    return;
  }

  frameBuffer[frameIndex++] = byte;
  if (frameIndex == sizeof(SerialCommand)) {
    serialAcceptCommand(frameBuffer);
    frameIndex = 0;
  }
}

void usart3_rx_check(void) {
  static uint32_t oldPos = 0;
  const uint32_t pos = sizeof(rxBuffer) - __HAL_DMA_GET_COUNTER(huart3.hdmarx);
  if (pos == oldPos) return;

  if (pos > oldPos) {
    for (uint32_t i = oldPos; i < pos; ++i) serialAcceptByte(rxBuffer[i]);
  } else {
    for (uint32_t i = oldPos; i < sizeof(rxBuffer); ++i) serialAcceptByte(rxBuffer[i]);
    for (uint32_t i = 0; i < pos; ++i) serialAcceptByte(rxBuffer[i]);
  }
  oldPos = (pos == sizeof(rxBuffer)) ? 0 : pos;
}

void readCommand(void) {
  input1[0].raw = CLAMP(serialCommand.cmdL, inputMin, inputMax);
  input2[0].raw = CLAMP(serialCommand.cmdR, inputMin, inputMax);
  input1[0].cmd = input1[0].raw;
  input2[0].cmd = input2[0].raw;

  if (serialTimeoutCount < SERIAL_TIMEOUT) {
    ++serialTimeoutCount;
  } else {
    timeoutFlgSerial = 1;
  }

  if (timeoutFlgSerial) {
    input1[0].cmd = 0;
    input2[0].cmd = 0;
    ctrlModReq = OPEN_MODE;
  } else {
    ctrlModReq = ctrlModReqRaw;
  }
}

void poweroff(void) {
  enable = 0;
  printf("-- Motors disabled --\r\n");
  buzzerCount = 0;
  buzzerPattern = 0;
  for (uint8_t i = 0; i < 8; ++i) {
    buzzerFreq = i;
    HAL_Delay(100);
  }
  buzzerFreq = 0;
  HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, GPIO_PIN_RESET);
  while (1) { }
}

void poweroffPressCheck(void) {
  if (!HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) return;
  uint16_t pressedMs = 0;
  enable = 0;
  while (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) {
    HAL_Delay(10);
    if (pressedMs < 60000) pressedMs += 10;
  }
  if (pressedMs >= 80) poweroff();
}

void filtLowPass32(int32_t u, uint16_t coef, int32_t *y) {
  int64_t tmp = ((int64_t)((u << 4) - (*y >> 12)) * coef) >> 4;
  tmp = CLAMP(tmp, -2147483648LL, 2147483647LL);
  *y = (int32_t)tmp + *y;
}

void rateLimiter16(int16_t u, int16_t rate, int16_t *y) {
  int16_t delta = (int16_t)((u << 4) - *y);
  if (delta > rate) delta = rate;
  if (delta < -rate) delta = (int16_t)-rate;
  *y = (int16_t)(*y + delta);
}

