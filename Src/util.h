#ifndef UTIL_H
#define UTIL_H

#include <stdint.h>
#include "stm32f1xx_hal.h"

typedef struct {
  uint16_t start;
  int16_t cmdL;
  int16_t cmdR;
  uint16_t checksum;
} SerialCommand;

typedef struct {
  int16_t raw;
  int16_t cmd;
  uint8_t typ;
  uint8_t typDef;
  int16_t min;
  int16_t mid;
  int16_t max;
  int16_t dband;
} InputStruct;

void Input_Lim_Init(void);
void Input_Init(void);
void UART_DisableRxErrors(UART_HandleTypeDef *huart);

void poweronMelody(void);
void beepCount(uint8_t cnt, uint8_t freq, uint8_t pattern);
void beepShort(uint8_t freq);
void calcAvgSpeed(void);

void readCommand(void);
void usart3_rx_check(void);

void poweroff(void);
void poweroffPressCheck(void);

void filtLowPass32(int32_t u, uint16_t coef, int32_t *y);
void rateLimiter16(int16_t u, int16_t rate, int16_t *y);

#endif
