#ifndef VESC_MCCONF_SERIAL_H_
#define VESC_MCCONF_SERIAL_H_
#include <stdint.h>
#include <stdbool.h>
#include "datatypes.h"
#define MCCONF_SIGNATURE 776184161u
#define APPCONF_SIGNATURE 486554156u
int32_t confgenerator_serialize_mcconf(uint8_t *buffer, const mc_configuration *conf);
int32_t confgenerator_serialize_appconf(uint8_t *buffer, const app_configuration *conf);
bool confgenerator_deserialize_mcconf(const uint8_t *buffer, mc_configuration *conf);
bool confgenerator_deserialize_appconf(const uint8_t *buffer, app_configuration *conf);
#endif
