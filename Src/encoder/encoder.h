#ifndef ENCODER_ENCODER_H_
#define ENCODER_ENCODER_H_

#include <stdbool.h>
#include <stdint.h>
#include "vesc/datatypes.h"
#include "encoder/encoder_datatype.h"
#include "encoder/enc_abi.h"

bool encoder_init(volatile mc_configuration *conf);
void encoder_update_config(volatile mc_configuration *conf);
void encoder_deinit(void);
float encoder_read_deg(void);
float encoder_read_deg_multiturn(void);
void encoder_set_deg(float deg);
encoder_type_t encoder_is_configured(void);
bool encoder_index_found(void);
void encoder_reset_multiturn(void);
void encoder_reset_errors(void);
float encoder_get_error_rate(void);
void encoder_check_faults(volatile mc_configuration *m_conf, bool is_second_motor);
void encoder_pin_isr(void);
void encoder_tim_isr(void);

uint32_t encoder_read_raw_count(void);
uint32_t encoder_get_counts(void);
#endif /* ENCODER_ENCODER_H_ */
