#ifndef ENCODER_ENCODER_H_
#define ENCODER_ENCODER_H_

#include <stdbool.h>
#include <stdint.h>
#include "vesc/datatypes.h"
#include "encoder/encoder_datatype.h"
#include "encoder/enc_abi.h"

/* Hardware contract: modul encoder ini hanya milik motor LEFT. LEFT dapat
 * dipilih Hall atau ABI; motor RIGHT selalu Hall dan tidak masuk API encoder. */
bool encoder_init(volatile mc_configuration *conf);
void encoder_update_config(volatile mc_configuration *conf);
void encoder_deinit(void);
float encoder_read_deg(void);
void encoder_set_deg(float deg);
encoder_type_t encoder_is_configured(void);
bool encoder_index_found(void);

uint32_t encoder_read_raw_count(void);
#endif /* ENCODER_ENCODER_H_ */
