#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"
#include "eeprom.h"
#include "util.h"
#include "motor/mcpwm_foc.h"
#include "motor/mc_interface.h"
#include "vesc/vesc_protocol.h"
#include "comms.h"

extern UART_HandleTypeDef huart3;
extern uint8_t buzzerCount;
extern uint8_t buzzerFreq;
extern uint8_t buzzerPattern;
extern uint8_t enable;

InputStruct input1[INPUTS_NR] = {{0, 0, 0, PRI_INPUT1}};
InputStruct input2[INPUTS_NR] = {{0, 0, 0, PRI_INPUT2}};

int16_t speedAvg = 0;
int16_t speedAvgAbs = 0;
uint8_t timeoutFlgSerial = 1;
uint8_t ctrlModReqRaw = CTRL_MOD_REQ;
uint8_t ctrlModReq = OPEN_MODE;

uint16_t VirtAddVarTab[NB_OF_VAR] = {1000, 1001, 1002};

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
  mc_interface_init(true);
}

void Input_Lim_Init(void) {
  inputMax = 1500;
  inputMin = -1500;
}

void UART_DisableRxErrors(UART_HandleTypeDef *huart) {
  CLEAR_BIT(huart->Instance->CR1, USART_CR1_PEIE);
  CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);
}

void Input_Init(void) {
  UART3_Init();
  vesc_protocol_init();
  HAL_UART_Receive_DMA(&huart3, rxBuffer, sizeof(rxBuffer));
  UART_DisableRxErrors(&huart3);

  uint16_t writeCheck = 0;
  uint16_t value = 0;
  HAL_FLASH_Unlock();
  EE_Init();
  if (EE_ReadVariable(VirtAddVarTab[0], &writeCheck) == 0 && writeCheck == FLASH_WRITE_KEY) {
    if (EE_ReadVariable(VirtAddVarTab[1], &value) == 0) {
      if (value >= 1 && value <= 40) {
        m_motor_1.m_conf.l_current_max = m_motor_2.m_conf.l_current_max = (float)value;
        m_motor_1.m_conf.l_current_min = m_motor_2.m_conf.l_current_min = -(float)value;
      }
    }
    if (EE_ReadVariable(VirtAddVarTab[2], &value) == 0) { (void)value; /* N_MOT_MAX kept compile-time in fixed speed PI. */ }
  }
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
  const int16_t rpmL = mcpwm_foc_get_motor_const(false)->m_rpm;
  const int16_t rpmR = mcpwm_foc_get_motor_const(true)->m_rpm;
  speedAvg = (rpmL - rpmR) / 2;
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

static void serialAcceptLegacyByte(uint8_t byte) {
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
      serialAcceptLegacyByte(byte);
    }
    return;
  }

  frameBuffer[frameIndex++] = byte;
  if (frameIndex == sizeof(SerialCommand)) {
    serialAcceptCommand(frameBuffer);
    frameIndex = 0;
  }
}

static void serialAcceptByte(uint8_t byte) {
  /* VESC packets use binary start markers 2/3/4. Never steal bytes that are
   * already inside the legacy 0xABCD command frame. Once a VESC packet has
   * started, all bytes go to its parser until the frame is complete/reset. */
  if (frameIndex == 0u && (vesc_protocol_rx_in_progress() || byte == 2u || byte == 3u || byte == 4u)) {
    (void)vesc_protocol_rx_byte(byte);
    return;
  }
  serialAcceptLegacyByte(byte);
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

