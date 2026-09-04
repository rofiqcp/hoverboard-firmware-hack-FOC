#include "encoder/encoder.h"
#include "encoder/encoder_cfg.h"
#include <string.h>

static encoder_type_t m_encoder_type_now = ENCODER_TYPE_NONE;

bool encoder_init(volatile mc_configuration *conf) {
    if (!conf) return false;
    if (m_encoder_type_now != ENCODER_TYPE_NONE) encoder_deinit();

    if (conf->m_sensor_port_mode == SENSOR_PORT_MODE_ABI) {
        int32_t counts = conf->m_encoder_counts;
        if (counts < 4) counts = 4096;
        if (counts > 65536) counts = 65536;
        encoder_cfg_ABI.counts = (uint32_t)counts;
        if (!enc_abi_init(&encoder_cfg_ABI)) return false;
        m_encoder_type_now = ENCODER_TYPE_ABI;
        return true;
    }

    m_encoder_type_now = ENCODER_TYPE_NONE;
    return false;
}

void encoder_update_config(volatile mc_configuration *conf) {
    if (!conf || m_encoder_type_now != ENCODER_TYPE_ABI) return;
    uint32_t counts = conf->m_encoder_counts < 4 ? 4096u : (uint32_t)conf->m_encoder_counts;
    if (counts > 65536u) counts = 65536u;
    if (encoder_cfg_ABI.counts != counts) {
        encoder_cfg_ABI.counts = counts;
        encoder_cfg_ABI.timer->ARR = counts - 1u;
        if (encoder_cfg_ABI.timer->CNT >= counts) encoder_cfg_ABI.timer->CNT = 0u;
        /* VESC invalidates ABI sync whenever CPR changes. */
        memset(&encoder_cfg_ABI.state, 0, sizeof(encoder_cfg_ABI.state));
    }
}

void encoder_deinit(void) {
    if (m_encoder_type_now == ENCODER_TYPE_ABI) enc_abi_deinit(&encoder_cfg_ABI);
    m_encoder_type_now = ENCODER_TYPE_NONE;
}

float encoder_read_deg(void) {
    return m_encoder_type_now == ENCODER_TYPE_ABI ? enc_abi_read_deg(&encoder_cfg_ABI) : 0.0f;
}

float encoder_read_deg_multiturn(void) { return encoder_read_deg(); }

void encoder_set_deg(float deg) {
    if (m_encoder_type_now == ENCODER_TYPE_ABI) enc_abi_set_deg(&encoder_cfg_ABI, deg);
}

encoder_type_t encoder_is_configured(void) { return m_encoder_type_now; }

bool encoder_index_found(void) {
    return m_encoder_type_now == ENCODER_TYPE_ABI ? encoder_cfg_ABI.state.index_found : true;
}

void encoder_reset_multiturn(void) { }
void encoder_reset_errors(void) { encoder_cfg_ABI.state.bad_pulses = 0; }
float encoder_get_error_rate(void) { return -1.0f; }

void encoder_check_faults(volatile mc_configuration *m_conf, bool is_second_motor) {
    (void)m_conf;
    (void)is_second_motor;
    /* ABI A/B tidak mempunyai checksum/SPI fault. Plausibility arah dan
     * gerakan divalidasi oleh FOC/detect encoder, bukan fault transport. */
}

void encoder_pin_isr(void) { enc_abi_pin_isr(&encoder_cfg_ABI); }
void encoder_tim_isr(void) { }

uint32_t encoder_read_raw_count(void) {
    return m_encoder_type_now == ENCODER_TYPE_ABI ? enc_abi_read_cnt(&encoder_cfg_ABI) : 0u;
}

uint32_t encoder_get_counts(void) {
    return m_encoder_type_now == ENCODER_TYPE_ABI ? encoder_cfg_ABI.counts : 0u;
}
