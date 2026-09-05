#ifndef F103_BOOT_LAYOUT_H_
#define F103_BOOT_LAYOUT_H_

#include <stdint.h>

#define F103_FLASH_BASE_ADDR      0x08000000u
#define F103_FLASH_TOTAL_SIZE     0x00040000u /* 256 KiB */
#define F103_FLASH_PAGE_SIZE      0x00000800u /* 2 KiB for STM32F103xE */

/* Native VESC transport: F103 USART3 PB10/PB11 <-> F411 USART1 PB6/PB7.
 * APB1 is 32 MHz on this firmware; 2 Mbaud is an exact divider and is the
 * maximum rate for USART3 with 16x oversampling. It is accepted only after
 * hardware stress/CRC validation; 1 Mbaud is the fallback if link errors appear. Keep app, bootloader and host tools identical. */
#define F103_VESC_UART_BAUD       2000000u

#define F103_BOOT_BASE_ADDR       0x08000000u
#define F103_BOOT_SIZE            0x00002800u /* 10 KiB / 5 pages */
#define F103_APP_BASE_ADDR        0x08002800u
#define F103_APP_REGION_SIZE      0x0001E000u /* 120 KiB */
#define F103_STAGE_BASE_ADDR      0x08020800u
#define F103_STAGE_REGION_SIZE    0x0001E000u /* 120 KiB */
#define F103_META_BASE_ADDR       0x0803E800u
#define F103_META_REGION_SIZE     0x00000800u /* 2 KiB / 1 page */
#define F103_EEPROM_BASE_ADDR     0x0803F000u
#define F103_EEPROM_REGION_SIZE   0x00001000u /* 4 KiB / 2 pages */

#define F103_VESC_IMAGE_HEADER_SIZE 6u
#define F103_MAX_FW_IMAGE_SIZE      (F103_STAGE_REGION_SIZE - F103_VESC_IMAGE_HEADER_SIZE)

#define F103_UPDATE_META_MAGIC      0x56455343u /* 'VESC' */
#define F103_UPDATE_STATE_PENDING   0x50454E44u /* 'PEND' */
#define F103_UPDATE_STATE_RECOVERY  0x52454356u /* 'RECV' */
#define F103_UPDATE_META_VERSION    1u

typedef struct {
    uint32_t magic;
    uint32_t state;
    uint32_t size;
    uint32_t size_inv;
    uint16_t crc16;
    uint16_t crc16_inv;
    uint16_t version;
    uint16_t version_inv;
} f103_update_meta_t;

#endif
