#ifndef ENC_ABI_H_
#define ENC_ABI_H_

#include <stdbool.h>
#include <stdint.h>
#include "encoder/encoder_datatype.h"

bool enc_abi_init(ABI_config_t *cfg);
void enc_abi_deinit(ABI_config_t *cfg);
float enc_abi_read_deg(ABI_config_t *cfg);
uint32_t enc_abi_read_cnt(ABI_config_t *cfg);
void enc_abi_set_deg(ABI_config_t *cfg, float deg);
void enc_abi_pin_isr(ABI_config_t *cfg);

#endif /* ENC_ABI_H_ */
