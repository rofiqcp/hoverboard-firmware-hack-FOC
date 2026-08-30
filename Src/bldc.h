#ifndef BLDC_RUNTIME_H
#define BLDC_RUNTIME_H

#include <stdint.h>

extern volatile uint32_t foc_isr_cycles;
extern volatile uint32_t foc_isr_cycles_max;
extern volatile int16_t foc_iqL_q4;
extern volatile int16_t foc_iqR_q4;
extern volatile int16_t foc_idL_q4;
extern volatile int16_t foc_idR_q4;
extern volatile int16_t offsetrlA, offsetrlB, offsetrrB, offsetrrC, offsetdcl, offsetdcr;

void currentCalibrationStart(void);
uint8_t currentCalibrationActive(void);
uint16_t currentCalibrationProgressPermille(void);
uint8_t currentCalibrationValid(void);
uint16_t currentCalibrationFailureCount(void);

#endif
