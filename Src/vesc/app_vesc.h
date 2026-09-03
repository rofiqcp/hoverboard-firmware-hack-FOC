#ifndef APP_VESC_H_
#define APP_VESC_H_

#include <stdbool.h>
#include <stdint.h>
#include "vesc/datatypes.h"

void app_vesc_init(void);
void app_vesc_defaults(app_configuration *conf, uint8_t controller_id);
const app_configuration *app_vesc_get_configuration(bool second);
bool app_vesc_set_configuration(bool second, const app_configuration *conf);
void app_vesc_process(uint32_t now_ms);

float app_vesc_adc_decoded(bool second_channel);
float app_vesc_adc_voltage(bool second_channel);
bool app_vesc_adc_range_ok(void);

#endif
