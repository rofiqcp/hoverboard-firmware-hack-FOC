#ifndef __EEPROM_H
#define __EEPROM_H

#include "stm32f1xx_hal.h"

/* STM32F103RCT6 = 256 KiB Flash. High-density F1 uses 2 KiB erase pages.
 * The linker reserves the final 4 KiB for EEPROM emulation. */
#define PAGE_SIZE               ((uint32_t)0x800u)
#define EEPROM_START_ADDRESS    ((uint32_t)0x0803F000u)
#define PAGE0_BASE_ADDRESS      EEPROM_START_ADDRESS
#define PAGE0_END_ADDRESS       ((uint32_t)(PAGE0_BASE_ADDRESS + PAGE_SIZE - 1u))
#define PAGE0_ID                PAGE0_BASE_ADDRESS
#define PAGE1_BASE_ADDRESS      ((uint32_t)0x0803F800u)
#define PAGE1_END_ADDRESS       ((uint32_t)(PAGE1_BASE_ADDRESS + PAGE_SIZE - 1u))
#define PAGE1_ID                PAGE1_BASE_ADDRESS

#define PAGE0                   ((uint16_t)0x0000)
#define PAGE1                   ((uint16_t)0x0001)
#define NO_VALID_PAGE           ((uint16_t)0x00AB)
#define ERASED                  ((uint16_t)0xFFFF)
#define RECEIVE_DATA            ((uint16_t)0xEEEE)
#define VALID_PAGE              ((uint16_t)0x0000)
#define READ_FROM_VALID_PAGE    ((uint8_t)0x00)
#define WRITE_IN_VALID_PAGE     ((uint8_t)0x01)
#define PAGE_FULL               ((uint8_t)0x80)

/* Address 0 is the transaction-valid key; remaining slots are persistent
 * runtime parameters. 64 slots leave headroom beyond the current address 60. */
#define NB_OF_VAR               ((uint8_t)64u)

uint16_t EE_Init(void);
uint16_t EE_ReadVariable(uint16_t VirtAddress, uint16_t* Data);
uint16_t EE_WriteVariable(uint16_t VirtAddress, uint16_t Data);

#endif
