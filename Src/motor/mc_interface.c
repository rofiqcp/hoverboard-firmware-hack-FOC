#include <string.h>
#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "config.h"
#include "eeprom.h"
#include "motor/mcpwm_foc.h"
#include "motor/mcconf_default.h"
#include "motor/mc_interface.h"

static int s_motor_selected = 1;

static bool selected_second(void) { return s_motor_selected == 2; }

void mc_interface_init(bool reset_conf) {
    (void)reset_conf;
    mcpwm_foc_init();
    s_motor_selected = 1;
}

int mc_interface_motor_now(void) { return s_motor_selected; }

void mc_interface_select_motor_thread(int motor) {
    if (motor == 1 || motor == 2) s_motor_selected = motor;
}

const volatile mc_configuration *mc_interface_get_configuration(void) {
    return mcpwm_foc_get_configuration(selected_second());
}

const volatile mc_configuration *mc_interface_get_configuration_motor(bool is_second_motor) {
    return mcpwm_foc_get_configuration(is_second_motor);
}

void mc_interface_set_configuration(mc_configuration *configuration) {
    if (configuration) mcpwm_foc_set_configuration(configuration, selected_second());
}

bool mc_interface_dccal_done(void) { return mcpwm_foc_dc_cal_done(); }
mc_fault_code mc_interface_get_fault(void) { return mcpwm_foc_get_fault_motor(selected_second()); }
mc_state mc_interface_get_state(void) { return mcpwm_foc_get_state_motor(selected_second()); }
mc_control_mode mc_interface_get_control_mode(void) { return mcpwm_foc_get_motor_const(selected_second())->m_control_mode; }
void mc_interface_set_duty(float dutyCycle) { mcpwm_foc_set_duty(dutyCycle, selected_second()); }
void mc_interface_set_pid_speed(float erpm) { mcpwm_foc_set_pid_speed(erpm, selected_second()); }
void mc_interface_set_pid_pos(float position_deg) { mcpwm_foc_set_pid_pos(position_deg, selected_second()); }
void mc_interface_set_current(float current) { mcpwm_foc_set_current(current, selected_second()); }
void mc_interface_set_brake_current(float current) { mcpwm_foc_set_brake_current(current, selected_second()); }
void mc_interface_set_openloop_current(float current, float rpm) { mcpwm_foc_set_openloop_current(current, rpm, selected_second()); }
void mc_interface_set_openloop_phase(float current, float phase) { mcpwm_foc_set_openloop_phase(current, phase, selected_second()); }
void mc_interface_release_motor(void) { mcpwm_foc_release_motor(selected_second()); }
float mc_interface_get_duty_cycle_now(void) { return mcpwm_foc_get_duty_cycle_motor(selected_second()); }
float mc_interface_get_rpm(void) { return mcpwm_foc_get_erpm_motor(selected_second()); }
float mc_interface_get_pid_pos_now(void){return mcpwm_foc_get_phase_motor(selected_second());}
float mc_interface_get_tot_current(void) { return mcpwm_foc_get_tot_current_motor(selected_second()); }
float mc_interface_get_tot_current_in(void) { return mcpwm_foc_get_tot_current_in_motor(selected_second()); }
float mc_interface_get_id(void) { return mcpwm_foc_get_id_motor(selected_second()); }
float mc_interface_get_iq(void) { return mcpwm_foc_get_iq_motor(selected_second()); }
float mc_interface_get_vd(void) { return mcpwm_foc_get_vd_motor(selected_second()); }
float mc_interface_get_vq(void) { return mcpwm_foc_get_vq_motor(selected_second()); }
float mc_interface_get_phase(void) { return mcpwm_foc_get_phase_motor(selected_second()); }
void mc_interface_get_values(mc_values *values) { mcpwm_foc_get_values(values, selected_second()); }

void mc_interface_set_mode_command_motor(uint8_t mode, int16_t command,
                                         bool run_request, uint16_t openloop_rpm,
                                         bool is_second_motor) {
    mcpwm_foc_set_mode_command(mode, command, run_request, openloop_rpm, is_second_motor);
}
void mc_interface_get_values_motor(mc_values *values, bool is_second_motor) { mcpwm_foc_get_values(values, is_second_motor); }
mc_fault_code mc_interface_get_fault_motor(bool is_second_motor) { return mcpwm_foc_get_fault_motor(is_second_motor); }
mc_state mc_interface_get_state_motor(bool is_second_motor) { return mcpwm_foc_get_state_motor(is_second_motor); }


/* EEPROM layout. Keep it compact: only VESC fields that are implemented by
 * this fixed-point dual-motor port are persisted. Standard GET_MCCONF still
 * serializes the complete VESC 6.00 wire structure. */
enum {
    EE_CFG_KEY = 0,
    EE_L_CUR_CA = 1, EE_R_CUR_CA = 2,
    EE_L_HALL0 = 3, EE_R_HALL0 = 11,
    EE_L_KPQ = 19, EE_L_KIQ, EE_L_KPD, EE_L_KID,
    EE_L_KPS, EE_L_KIS, EE_L_KDS, EE_L_KPP, EE_L_KIP, EE_L_KDP,
    EE_R_KPQ = 29, EE_R_KIQ, EE_R_KPD, EE_R_KID,
    EE_R_KPS, EE_R_KIS, EE_R_KDS, EE_R_KPP, EE_R_KIP, EE_R_KDP,
    EE_L_SPEED_RAMP10 = 39, EE_R_SPEED_RAMP10,
    EE_L_SPEED_REL = 41, EE_R_SPEED_REL,
    EE_L_CFG_SIGNATURE = 43, EE_R_CFG_SIGNATURE = 44
};
#define EE_CFG_SIGNATURE_VALUE 0x6011u
#define EE_CFG_SIGNATURE_V18   0x6010u
#define EE_CFG_SIGNATURE_V17   0x600Fu
#define EE_CFG_SIGNATURE_V16   0x600Eu

extern uint16_t VirtAddVarTab[NB_OF_VAR];

static bool ee_read_slot(uint8_t idx, uint16_t *v) {
    return idx < NB_OF_VAR && EE_ReadVariable(VirtAddVarTab[idx], v) == 0u;
}
static bool ee_write_slot(uint8_t idx, uint16_t v) {
    return idx < NB_OF_VAR && EE_WriteVariable(VirtAddVarTab[idx], v) == HAL_OK;
}

static bool hall_table_sane(const uint8_t t[8]) {
    if (t[0] != 255u || t[7] != 255u) return false;
    uint8_t valid = 0u;
    for (uint8_t h = 1u; h <= 6u; ++h) {
        if (t[h] < 200u) valid++;
    }
    return valid == 6u;
}

bool mc_interface_store_configuration_motor(bool second) {
    mcpwm_foc_motor_t *m = mcpwm_foc_get_motor(second);
    mcpwm_foc_sync_tuning_to_conf(second);
    const uint8_t hall_base = second ? EE_R_HALL0 : EE_L_HALL0;
    const uint8_t gain_base = second ? EE_R_KPQ : EE_L_KPQ;
    const uint8_t ramp_slot = second ? EE_R_SPEED_RAMP10 : EE_L_SPEED_RAMP10;
    const uint8_t rel_slot = second ? EE_R_SPEED_REL : EE_L_SPEED_REL;
    const uint8_t cur_slot = second ? EE_R_CUR_CA : EE_L_CUR_CA;
    const uint16_t pp = second ? MCCONF_POLE_PAIRS_RIGHT : MCCONF_POLE_PAIRS_LEFT;
    int32_t ca = (int32_t)(m->m_conf.l_current_max * 100.0f + 0.5f);
    if (ca < 1) ca = 1;
    if (ca > I_MOT_MAX * 100) ca = I_MOT_MAX * 100;
    uint32_t ramp_erpm_s = (uint32_t)m->m_speed_ramp_rpm_s * pp;
    uint16_t ramp10 = (uint16_t)((ramp_erpm_s + 5u) / 10u);
    uint16_t rel_erpm = (uint16_t)((uint32_t)m->m_speed_release_rpm * pp);
    const uint16_t gains[10] = {
        m->m_kpq_q11, m->m_kiq_q16, m->m_kpd_q11, m->m_kid_q16,
        m->m_kps_q11, m->m_kis_q16, m->m_kds_q11,
        m->m_kpp_q11, m->m_kip_q16, m->m_kdp_q11
    };

    bool ok = true;
    HAL_FLASH_Unlock();
    ok &= ee_write_slot(cur_slot, (uint16_t)ca);
    for (uint8_t i = 0u; i < 8u; ++i) ok &= ee_write_slot((uint8_t)(hall_base + i), m->m_conf.foc_hall_table[i]);
    for (uint8_t i = 0u; i < 10u; ++i) ok &= ee_write_slot((uint8_t)(gain_base + i), gains[i]);
    ok &= ee_write_slot(ramp_slot, ramp10);
    ok &= ee_write_slot(rel_slot, rel_erpm);
    /* Per-motor signature is written last, so an interrupted update of one
     * motor can never make the other motor's partial configuration look valid. */
    ok &= ee_write_slot(second ? EE_R_CFG_SIGNATURE : EE_L_CFG_SIGNATURE, EE_CFG_SIGNATURE_VALUE);
    ok &= ee_write_slot(EE_CFG_KEY, (uint16_t)FLASH_WRITE_KEY);
    HAL_FLASH_Lock();
    return ok;
}

bool mc_interface_load_configuration_motor(bool second) {
    uint16_t key = 0u, sig = 0u;
    const uint8_t sig_slot = second ? EE_R_CFG_SIGNATURE : EE_L_CFG_SIGNATURE;
    if (!ee_read_slot(EE_CFG_KEY, &key) || key != (uint16_t)FLASH_WRITE_KEY ||
        !ee_read_slot(sig_slot, &sig) ||
        (sig != EE_CFG_SIGNATURE_VALUE && sig != EE_CFG_SIGNATURE_V18 &&
         sig != EE_CFG_SIGNATURE_V17 && sig != EE_CFG_SIGNATURE_V16)) {
        return false;
    }
    const bool migrate_speed_pid = (sig == EE_CFG_SIGNATURE_V17 || sig == EE_CFG_SIGNATURE_V16);
    const bool migrate_position_pid = (sig != EE_CFG_SIGNATURE_VALUE);
    mcpwm_foc_motor_t *m = mcpwm_foc_get_motor(second);
    const uint8_t hall_base = second ? EE_R_HALL0 : EE_L_HALL0;
    const uint8_t gain_base = second ? EE_R_KPQ : EE_L_KPQ;
    const uint8_t ramp_slot = second ? EE_R_SPEED_RAMP10 : EE_L_SPEED_RAMP10;
    const uint8_t rel_slot = second ? EE_R_SPEED_REL : EE_L_SPEED_REL;
    const uint8_t cur_slot = second ? EE_R_CUR_CA : EE_L_CUR_CA;
    const uint16_t pp = second ? MCCONF_POLE_PAIRS_RIGHT : MCCONF_POLE_PAIRS_LEFT;
    uint16_t v = 0u;
    uint8_t hall[8];
    for (uint8_t i = 0u; i < 8u; ++i) {
        if (!ee_read_slot((uint8_t)(hall_base + i), &v)) return false;
        hall[i] = (uint8_t)v;
    }
    if (!hall_table_sane(hall)) return false;
    for (uint8_t i = 0u; i < 8u; ++i) m->m_conf.foc_hall_table[i] = hall[i];

    if (ee_read_slot(cur_slot, &v) && v >= 10u && v <= I_MOT_MAX * 100u) {
        m->m_conf.l_current_max = (float)v / 100.0f;
        m->m_conf.l_current_min = -m->m_conf.l_current_max;
        m->m_current_limit_q4 = (int16_t)((int32_t)v * A2BIT_CONV * 16 / 100);
    }
    volatile uint16_t *gain_dst[10] = {
        &m->m_kpq_q11, &m->m_kiq_q16, &m->m_kpd_q11, &m->m_kid_q16,
        &m->m_kps_q11, &m->m_kis_q16, &m->m_kds_q11,
        &m->m_kpp_q11, &m->m_kip_q16, &m->m_kdp_q11
    };
    for (uint8_t i = 0u; i < 10u; ++i) {
        if (ee_read_slot((uint8_t)(gain_base + i), &v)) {
            /* V16 used EFeru direct-Vq speed gains; V17 used the first
             * cascade defaults (0.004/0.004). Both are superseded by the
             * hardware-tested hoverboard cascade defaults. Preserve every
             * other persisted field and migrate only speed PID gains. */
            if (!(migrate_speed_pid && i >= 4u && i <= 6u) &&
                !(migrate_position_pid && i >= 7u && i <= 9u)) {
                *gain_dst[i] = v;
            }
        }
    }
    if (ee_read_slot(ramp_slot, &v) && v > 0u) {
        uint32_t erpm_s = (uint32_t)v * 10u;
        uint32_t mech = erpm_s / pp;
        if (mech < 1u) mech = 1u;
        if (mech > 5000u) mech = 5000u;
        m->m_speed_ramp_rpm_s = (uint16_t)mech;
    }
    if (ee_read_slot(rel_slot, &v) && v > 0u) {
        uint32_t mech = v / pp;
        if (mech < 1u) mech = 1u;
        if (mech > 100u) mech = 100u;
        m->m_speed_release_rpm = (uint16_t)mech;
    }
    mcpwm_foc_sync_tuning_to_conf(second);
    m->m_conf.s_pid_ramp_erpms_s = (float)((uint32_t)m->m_speed_ramp_rpm_s * pp);
    m->m_conf.s_pid_min_erpm = (float)((uint32_t)m->m_speed_release_rpm * pp);
    if (migrate_speed_pid || migrate_position_pid) {
        /* Rewrite only after a complete successful load; signature is written last. */
        (void)mc_interface_store_configuration_motor(second);
    }
    return true;
}

void mc_interface_restore_default_motor(bool second, bool store_to_eeprom) {
    mc_configuration c;
    mcpwm_foc_get_default_configuration(&c, second);
    mcpwm_foc_set_configuration(&c, second);
    if (store_to_eeprom) (void)mc_interface_store_configuration_motor(second);
}
