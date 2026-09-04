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
void mc_interface_set_handbrake(float current) { mcpwm_foc_set_handbrake(current, selected_second()); }
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
    EE_L_CFG_SIGNATURE = 43, EE_R_CFG_SIGNATURE = 44,
    EE_L_MOTOR_POLES = 45, EE_R_MOTOR_POLES = 46,
    EE_L_GEAR_X64 = 47, EE_R_GEAR_X64 = 48,

    /* 49..122 remain App Config forever. MC extension is appended after it so
     * existing EEPROM images keep their exact addresses. Six slots per motor. */
    EE_L_EXT_CURRENT_MIN_CA = 123, EE_L_EXT_ABS_CURRENT_CA, EE_L_EXT_MAX_DUTY_X10000,
    EE_L_EXT_POS_KD_FILTER_X10000, EE_L_EXT_DUTY_KP_X10, EE_L_EXT_DUTY_KI_X10,
    EE_R_EXT_CURRENT_MIN_CA = 129, EE_R_EXT_ABS_CURRENT_CA, EE_R_EXT_MAX_DUTY_X10000,
    EE_R_EXT_POS_KD_FILTER_X10000, EE_R_EXT_DUTY_KP_X10, EE_R_EXT_DUTY_KI_X10,
    /* Input-current and duty-ramp extension, appended without moving App Config. */
    EE_L_EXT_IN_CURRENT_MAX_CA = 135, EE_L_EXT_IN_CURRENT_MIN_CA, EE_L_EXT_DUTY_RAMP_X10000,
    EE_R_EXT_IN_CURRENT_MAX_CA = 138, EE_R_EXT_IN_CURRENT_MIN_CA, EE_R_EXT_DUTY_RAMP_X10000,
    /* Standard VESC current-controller release threshold. Appended only; no
     * historical EEPROM address is moved. Stored in centiamps. */
    EE_L_CC_MIN_CURRENT_CA = 141, EE_R_CC_MIN_CURRENT_CA = 142,

    /* V26+: VESC 6.00 fields that now have real runtime behavior. These slots
     * are append-only so every historical App/MC address remains unchanged. */
    EE_L_EXT2_BAT_CUT_START_CV = 143, EE_L_EXT2_BAT_CUT_END_CV,
    EE_L_EXT2_CUR_MAX_SCALE_X10000, EE_L_EXT2_CUR_MIN_SCALE_X10000,
    EE_L_EXT2_MIN_ERPM, EE_L_EXT2_MAX_ERPM, EE_L_EXT2_WHEEL_X10000,
    EE_R_EXT2_BAT_CUT_START_CV = 150, EE_R_EXT2_BAT_CUT_END_CV,
    EE_R_EXT2_CUR_MAX_SCALE_X10000, EE_R_EXT2_CUR_MIN_SCALE_X10000,
    EE_R_EXT2_MIN_ERPM, EE_R_EXT2_MAX_ERPM, EE_R_EXT2_WHEEL_X10000,
    EE_L_EXT2_BAT_META = 157, EE_L_EXT2_BAT_AH_X100,
    EE_R_EXT2_BAT_META = 159, EE_R_EXT2_BAT_AH_X100,
    /* V27+: resolusi persis field VESC 6.00 foc_current_filter_const (x10000). */
    EE_L_EXT3_TELEM_FILTER_X10000 = 161, EE_R_EXT3_TELEM_FILTER_X10000,
    /* V28+: safety field yang sekarang benar-benar dipakai runtime. Watt
     * disimpan 32-bit dalam 0,1 W agar round-trip VESC Tool tetap presisi. */
    EE_L_EXT4_VIN_MIN_CV = 163, EE_L_EXT4_VIN_MAX_CV,
    EE_L_EXT4_TEMP_START_X10, EE_L_EXT4_TEMP_END_X10,
    EE_L_EXT4_WATT_MAX_X10_LO, EE_L_EXT4_WATT_MAX_X10_HI,
    EE_L_EXT4_WATT_REGEN_X10_LO, EE_L_EXT4_WATT_REGEN_X10_HI,
    EE_R_EXT4_VIN_MIN_CV = 171, EE_R_EXT4_VIN_MAX_CV,
    EE_R_EXT4_TEMP_START_X10, EE_R_EXT4_TEMP_END_X10,
    EE_R_EXT4_WATT_MAX_X10_LO, EE_R_EXT4_WATT_MAX_X10_HI,
    EE_R_EXT4_WATT_REGEN_X10_LO, EE_R_EXT4_WATT_REGEN_X10_HI
};
#define EE_CFG_SIGNATURE_VALUE 0x601Bu
#define EE_CFG_SIGNATURE_V28   0x601Au /* before Vin/watt/temperature safety persistence */
#define EE_CFG_SIGNATURE_V27   0x6019u /* before exact x10000 telemetry-filter persistence */
#define EE_CFG_SIGNATURE_V26   0x6018u /* before battery-cut/current-scale/speed/wheel persistence */
#define EE_CFG_SIGNATURE_V25   0x6017u /* before cc_min_current persistence */
#define EE_CFG_SIGNATURE_V24   0x6016u /* before input-current/duty-ramp extension */
#define EE_CFG_SIGNATURE_V23   0x6015u /* before MC extension slots */
#define EE_CFG_SIGNATURE_V22   0x6014u /* position Kp/filter update, telemetry filter not packed yet */
#define EE_CFG_SIGNATURE_V21   0x6013u /* previous position default Kp=0.008 */
#define EE_CFG_SIGNATURE_V20   0x6012u /* previous speed gains used x1000 */
#define EE_CFG_SIGNATURE_V19   0x6011u
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
static bool ee_write_u32_pair(uint8_t lo_slot, uint32_t value) {
    return ee_write_slot(lo_slot, (uint16_t)(value & 0xffffu)) &&
           ee_write_slot((uint8_t)(lo_slot + 1u), (uint16_t)(value >> 16));
}
static bool ee_read_u32_pair(uint8_t lo_slot, uint32_t *value) {
    uint16_t lo=0u, hi=0u;
    if (!value || !ee_read_slot(lo_slot,&lo) || !ee_read_slot((uint8_t)(lo_slot+1u),&hi)) return false;
    *value=(uint32_t)lo | ((uint32_t)hi<<16);
    return true;
}

static bool hall_table_sane(const uint8_t t[8]) {
    if (t[0] != 255u || t[7] != 255u) return false;
    uint8_t sorted[6];
    for (uint8_t h = 1u; h <= 6u; ++h) {
        if (t[h] >= 200u) return false;
        sorted[h - 1u] = t[h];
    }
    for (uint8_t i = 0u; i < 5u; ++i) {
        for (uint8_t j = (uint8_t)(i + 1u); j < 6u; ++j) {
            if (sorted[j] < sorted[i]) { uint8_t x=sorted[i]; sorted[i]=sorted[j]; sorted[j]=x; }
        }
    }
    for (uint8_t i = 0u; i < 6u; ++i) {
        const uint16_t a=sorted[i];
        const uint16_t b=(i==5u)?(uint16_t)sorted[0]+200u:sorted[i+1u];
        const uint16_t gap=b-a;
        if (gap < 18u || gap > 48u) return false;
    }
    return true;
}

bool mc_interface_store_configuration_motor(bool second) {
    mcpwm_foc_motor_t *m = mcpwm_foc_get_motor(second);
    mcpwm_foc_sync_tuning_to_conf(second);
    const uint8_t hall_base = second ? EE_R_HALL0 : EE_L_HALL0;
    const uint8_t gain_base = second ? EE_R_KPQ : EE_L_KPQ;
    const uint8_t ramp_slot = second ? EE_R_SPEED_RAMP10 : EE_L_SPEED_RAMP10;
    const uint8_t rel_slot = second ? EE_R_SPEED_REL : EE_L_SPEED_REL;
    const uint8_t cur_slot = second ? EE_R_CUR_CA : EE_L_CUR_CA;
    const uint8_t pole_slot = second ? EE_R_MOTOR_POLES : EE_L_MOTOR_POLES;
    const uint8_t gear_slot = second ? EE_R_GEAR_X64 : EE_L_GEAR_X64;
    const uint8_t ext_base = second ? EE_R_EXT_CURRENT_MIN_CA : EE_L_EXT_CURRENT_MIN_CA;
    const uint8_t io_base = second ? EE_R_EXT_IN_CURRENT_MAX_CA : EE_L_EXT_IN_CURRENT_MAX_CA;
    const uint8_t ccmin_slot = second ? EE_R_CC_MIN_CURRENT_CA : EE_L_CC_MIN_CURRENT_CA;
    const uint8_t ext2_base = second ? EE_R_EXT2_BAT_CUT_START_CV : EE_L_EXT2_BAT_CUT_START_CV;
    const uint8_t bat_meta_slot = second ? EE_R_EXT2_BAT_META : EE_L_EXT2_BAT_META;
    const uint8_t bat_ah_slot = second ? EE_R_EXT2_BAT_AH_X100 : EE_L_EXT2_BAT_AH_X100;
    const uint8_t telem_filter_slot = second ? EE_R_EXT3_TELEM_FILTER_X10000 : EE_L_EXT3_TELEM_FILTER_X10000;
    const uint8_t safety_base = second ? EE_R_EXT4_VIN_MIN_CV : EE_L_EXT4_VIN_MIN_CV;
    const uint16_t pp = mcpwm_foc_get_pole_pairs(second);
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
    {
        /* Keep the original MC EEPROM layout: motor poles need only one byte,
         * so use the previously-unused high byte for the monitoring current LPF.
         * q8 gives ~0.004 resolution and avoids shifting the App Config slots. */
        float tf=m->m_conf.foc_current_filter_const;
        if (!(tf >= 0.001f && tf <= 1.0f)) tf=MCCONF_FOC_TELEMETRY_FILTER_DEFAULT;
        uint32_t fq=(uint32_t)(tf*255.0f+0.5f);
        if(fq<1u)fq=1u;
        if(fq>255u)fq=255u;
        const uint16_t packed=(uint16_t)((fq<<8)|(uint16_t)m->m_conf.si_motor_poles);
        ok &= ee_write_slot(pole_slot, packed);
        /* Slot baru menyimpan resolusi wire VESC 6.00 (1e-4). High-byte lama
         * tetap ditulis agar migrasi/downgrade image tidak kehilangan fallback. */
        uint32_t tf_x10000=(uint32_t)(tf*10000.0f+0.5f);
        if(tf_x10000<10u)tf_x10000=10u;
        if(tf_x10000>10000u)tf_x10000=10000u;
        ok &= ee_write_slot(telem_filter_slot,(uint16_t)tf_x10000);
    }
    {
        float gr = m->m_conf.si_gear_ratio;
        if (!(gr >= 0.01f && gr <= 1000.0f)) gr = 1.0f;
        uint32_t gx64 = (uint32_t)(gr * 64.0f + 0.5f);
        if (gx64 < 1u) gx64 = 1u;
        if (gx64 > 65535u) gx64 = 65535u;
        ok &= ee_write_slot(gear_slot, (uint16_t)gx64);
    }
    {
        /* Fields implemented at runtime but historically missing from EEPROM.
         * Fixed-point persistence is deliberate: deterministic across compiler
         * versions and enough resolution for VESC Tool round-trip. */
        int32_t cmin=(int32_t)(m->m_conf.l_current_min*100.0f + (m->m_conf.l_current_min>=0.0f?0.5f:-0.5f));
        if(cmin < -(int32_t)I_MOT_MAX*100) cmin=-(int32_t)I_MOT_MAX*100;
        if(cmin > -10) cmin=-10;
        uint32_t abs_ca=(uint32_t)(m->m_conf.l_abs_current_max*100.0f+0.5f);
        const uint32_t abs_ca_max=(uint32_t)(MCCONF_L_ABS_CURRENT_MAX*100.0f+0.5f);
        if(abs_ca<10u) abs_ca=10u;
        if(abs_ca>abs_ca_max) abs_ca=abs_ca_max;
        uint32_t duty=(uint32_t)(m->m_conf.l_max_duty*10000.0f+0.5f);
        if(duty<1u) duty=1u;
        if(duty>10000u) duty=10000u;
        uint32_t pdf=(uint32_t)(m->m_conf.p_pid_kd_filter*10000.0f+0.5f); if(pdf>10000u)pdf=10000u;
        uint32_t dkp=(uint32_t)(m->m_conf.foc_duty_dowmramp_kp*10.0f+0.5f); if(dkp>65535u)dkp=65535u;
        uint32_t dki=(uint32_t)(m->m_conf.foc_duty_dowmramp_ki*10.0f+0.5f); if(dki>65535u)dki=65535u;
        ok &= ee_write_slot(ext_base+0u,(uint16_t)(int16_t)cmin);
        ok &= ee_write_slot(ext_base+1u,(uint16_t)abs_ca);
        /* ext duty uses only 0..10000, so bit15 safely persists the standard
         * VESC l_slow_abs_current flag without consuming another EEPROM slot. */
        const uint16_t duty_flags=(uint16_t)duty | (m->m_conf.l_slow_abs_current?0x8000u:0u);
        ok &= ee_write_slot(ext_base+2u,duty_flags);
        ok &= ee_write_slot(ext_base+3u,(uint16_t)pdf);
        ok &= ee_write_slot(ext_base+4u,(uint16_t)dkp);
        ok &= ee_write_slot(ext_base+5u,(uint16_t)dki);
    }
    {
        int32_t imax=(int32_t)(m->m_conf.l_in_current_max*100.0f+0.5f);
        int32_t imin=(int32_t)(m->m_conf.l_in_current_min*100.0f-0.5f);
        if(imax<10)imax=10;
        if(imax>I_DC_MAX*100)imax=I_DC_MAX*100;
        if(imin>-10)imin=-10;
        if(imin<-(int32_t)I_DC_MAX*100)imin=-(int32_t)I_DC_MAX*100;
        int32_t dr=(int32_t)(m->m_conf.m_duty_ramp_step*10000.0f+0.5f);
        if(dr<1)dr=1;
        if(dr>2000)dr=2000;
        ok &= ee_write_slot(io_base+0u,(uint16_t)imax);
        ok &= ee_write_slot(io_base+1u,(uint16_t)(int16_t)imin);
        ok &= ee_write_slot(io_base+2u,(uint16_t)dr);
    }
    {
        float cc=m->m_conf.cc_min_current;
        if(!(cc>=0.001f && cc<=1.0f))cc=MCCONF_CC_MIN_CURRENT;
        uint32_t cca=(uint32_t)(cc*100.0f+0.5f);
        if(cca<1u)cca=1u;
        if(cca>100u)cca=100u;
        ok &= ee_write_slot(ccmin_slot,(uint16_t)cca);
    }
    {
        /* Persist only standard VESC fields that have real behavior on this
         * board. Fixed-point units are deterministic and fit one emulated word. */
        float bcs=m->m_conf.l_battery_cut_start;
        float bce=m->m_conf.l_battery_cut_end;
        if(!(bcs>bce && bce>=0.0f)){bcs=MCCONF_L_BATTERY_CUT_START;bce=MCCONF_L_BATTERY_CUT_END;}
        uint32_t bcs_cv=(uint32_t)(bcs*100.0f+0.5f); if(bcs_cv>8000u)bcs_cv=8000u;
        uint32_t bce_cv=(uint32_t)(bce*100.0f+0.5f); if(bce_cv>7999u)bce_cv=7999u;
        float smax=m->m_conf.l_current_max_scale; if(!(smax>=0.0f&&smax<=1.0f))smax=1.0f;
        float smin=m->m_conf.l_current_min_scale; if(!(smin>=0.0f&&smin<=1.0f))smin=1.0f;
        uint32_t sxmax=(uint32_t)(smax*10000.0f+0.5f); if(sxmax>10000u)sxmax=10000u;
        uint32_t sxmin=(uint32_t)(smin*10000.0f+0.5f); if(sxmin>10000u)sxmin=10000u;
        int32_t emin=(int32_t)(m->m_conf.l_min_erpm + (m->m_conf.l_min_erpm>=0.0f?0.5f:-0.5f));
        int32_t emax=(int32_t)(m->m_conf.l_max_erpm + (m->m_conf.l_max_erpm>=0.0f?0.5f:-0.5f));
        if(emin<(int32_t)MCCONF_L_MIN_ERPM)emin=(int32_t)MCCONF_L_MIN_ERPM;
        if(emin>-1)emin=-1;
        if(emax>(int32_t)MCCONF_L_MAX_ERPM)emax=(int32_t)MCCONF_L_MAX_ERPM;
        if(emax<1)emax=1;
        float wheel=m->m_conf.si_wheel_diameter;
        if(!(wheel>0.001f&&wheel<5.0f))wheel=MCCONF_SI_WHEEL_DIAMETER;
        uint32_t wx=(uint32_t)(wheel*10000.0f+0.5f); if(wx<10u)wx=10u; if(wx>50000u)wx=50000u;
        ok &= ee_write_slot(ext2_base+0u,(uint16_t)bcs_cv);
        ok &= ee_write_slot(ext2_base+1u,(uint16_t)bce_cv);
        ok &= ee_write_slot(ext2_base+2u,(uint16_t)sxmax);
        ok &= ee_write_slot(ext2_base+3u,(uint16_t)sxmin);
        ok &= ee_write_slot(ext2_base+4u,(uint16_t)(int16_t)emin);
        ok &= ee_write_slot(ext2_base+5u,(uint16_t)(int16_t)emax);
        ok &= ee_write_slot(ext2_base+6u,(uint16_t)wx);
        uint8_t cells=m->m_conf.si_battery_cells;
        if(cells<1u||cells>32u)cells=BAT_CELLS;
        uint8_t type=(uint8_t)m->m_conf.si_battery_type;
        if(type>(uint8_t)BATTERY_TYPE_LEAD_ACID)type=(uint8_t)BATTERY_TYPE_LIION_3_0__4_2;
        float ah=m->m_conf.si_battery_ah;
        if(!(ah>=0.0f&&ah<=655.35f))ah=0.0f;
        uint32_t ahx=(uint32_t)(ah*100.0f+0.5f); if(ahx>65535u)ahx=65535u;
        ok &= ee_write_slot(bat_meta_slot,(uint16_t)(((uint16_t)type<<8)|cells));
        ok &= ee_write_slot(bat_ah_slot,(uint16_t)ahx);
    }
    {
        /* Safety VESC 6.00 yang diimplementasikan runtime harus bertahan reboot. */
        float vmin=m->m_conf.l_min_vin, vmax=m->m_conf.l_max_vin;
        if(!(vmin>=5.0f && vmax>vmin && vmax<=80.0f)){vmin=MCCONF_L_MIN_VIN;vmax=MCCONF_L_MAX_VIN;}
        uint32_t vmin_cv=(uint32_t)(vmin*100.0f+0.5f), vmax_cv=(uint32_t)(vmax*100.0f+0.5f);
        float ts=m->m_conf.l_temp_fet_start, te=m->m_conf.l_temp_fet_end;
        if(!(te>ts && ts>=-40.0f && te<=180.0f)){ts=MCCONF_L_TEMP_FET_START;te=MCCONF_L_TEMP_FET_END;}
        int32_t ts10=(int32_t)(ts*10.0f+(ts>=0.0f?0.5f:-0.5f));
        int32_t te10=(int32_t)(te*10.0f+(te>=0.0f?0.5f:-0.5f));
        float wmax=m->m_conf.l_watt_max, wmin=m->m_conf.l_watt_min;
        if(!(wmax>0.0f&&wmax<=200000000.0f))wmax=MCCONF_L_WATT_MAX;
        if(!(wmin<0.0f&&wmin>=-200000000.0f))wmin=MCCONF_L_WATT_MIN;
        uint32_t wmax10=(uint32_t)(wmax*10.0f+0.5f);
        uint32_t wregen10=(uint32_t)(-wmin*10.0f+0.5f);
        ok &= ee_write_slot(safety_base+0u,(uint16_t)vmin_cv);
        ok &= ee_write_slot(safety_base+1u,(uint16_t)vmax_cv);
        ok &= ee_write_slot(safety_base+2u,(uint16_t)(int16_t)ts10);
        ok &= ee_write_slot(safety_base+3u,(uint16_t)(int16_t)te10);
        ok &= ee_write_u32_pair((uint8_t)(safety_base+4u),wmax10);
        ok &= ee_write_u32_pair((uint8_t)(safety_base+6u),wregen10);
    }
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
        (sig != EE_CFG_SIGNATURE_VALUE && sig != EE_CFG_SIGNATURE_V28 && sig != EE_CFG_SIGNATURE_V27 && sig != EE_CFG_SIGNATURE_V26 && sig != EE_CFG_SIGNATURE_V25 && sig != EE_CFG_SIGNATURE_V24 && sig != EE_CFG_SIGNATURE_V23 && sig != EE_CFG_SIGNATURE_V22 && sig != EE_CFG_SIGNATURE_V21 && sig != EE_CFG_SIGNATURE_V20 && sig != EE_CFG_SIGNATURE_V19 &&
         sig != EE_CFG_SIGNATURE_V18 && sig != EE_CFG_SIGNATURE_V17 && sig != EE_CFG_SIGNATURE_V16)) {
        return false;
    }
    const bool migrate_speed_pid_scale = (sig == EE_CFG_SIGNATURE_V20);
    const bool migrate_speed_pid = (sig == EE_CFG_SIGNATURE_V17 || sig == EE_CFG_SIGNATURE_V16);
    const bool migrate_position_pid = (sig == EE_CFG_SIGNATURE_V21 || sig == EE_CFG_SIGNATURE_V18 || sig == EE_CFG_SIGNATURE_V17 || sig == EE_CFG_SIGNATURE_V16);
    const bool migrate_telem_filter = (sig != EE_CFG_SIGNATURE_VALUE);
    const bool migrate_mc_extension = (sig != EE_CFG_SIGNATURE_VALUE);
    mcpwm_foc_motor_t *m = mcpwm_foc_get_motor(second);
    const uint8_t hall_base = second ? EE_R_HALL0 : EE_L_HALL0;
    const uint8_t gain_base = second ? EE_R_KPQ : EE_L_KPQ;
    const uint8_t ramp_slot = second ? EE_R_SPEED_RAMP10 : EE_L_SPEED_RAMP10;
    const uint8_t rel_slot = second ? EE_R_SPEED_REL : EE_L_SPEED_REL;
    const uint8_t cur_slot = second ? EE_R_CUR_CA : EE_L_CUR_CA;
    const uint8_t pole_slot = second ? EE_R_MOTOR_POLES : EE_L_MOTOR_POLES;
    const uint8_t gear_slot = second ? EE_R_GEAR_X64 : EE_L_GEAR_X64;
    const uint8_t ext_base = second ? EE_R_EXT_CURRENT_MIN_CA : EE_L_EXT_CURRENT_MIN_CA;
    const uint8_t io_base = second ? EE_R_EXT_IN_CURRENT_MAX_CA : EE_L_EXT_IN_CURRENT_MAX_CA;
    const uint8_t ccmin_slot = second ? EE_R_CC_MIN_CURRENT_CA : EE_L_CC_MIN_CURRENT_CA;
    const uint8_t ext2_base = second ? EE_R_EXT2_BAT_CUT_START_CV : EE_L_EXT2_BAT_CUT_START_CV;
    const uint8_t bat_meta_slot = second ? EE_R_EXT2_BAT_META : EE_L_EXT2_BAT_META;
    const uint8_t bat_ah_slot = second ? EE_R_EXT2_BAT_AH_X100 : EE_L_EXT2_BAT_AH_X100;
    const uint8_t telem_filter_slot = second ? EE_R_EXT3_TELEM_FILTER_X10000 : EE_L_EXT3_TELEM_FILTER_X10000;
    const uint8_t safety_base = second ? EE_R_EXT4_VIN_MIN_CV : EE_L_EXT4_VIN_MIN_CV;
    uint16_t v = 0u;
    uint8_t hall[8];
    for (uint8_t i = 0u; i < 8u; ++i) {
        if (!ee_read_slot((uint8_t)(hall_base + i), &v)) return false;
        hall[i] = (uint8_t)v;
    }
    if (!hall_table_sane(hall)) return false;
    for (uint8_t i = 0u; i < 8u; ++i) m->m_conf.foc_hall_table[i] = hall[i];

    if (sig == EE_CFG_SIGNATURE_VALUE || sig == EE_CFG_SIGNATURE_V28 || sig == EE_CFG_SIGNATURE_V27 || sig == EE_CFG_SIGNATURE_V26 || sig == EE_CFG_SIGNATURE_V25 || sig == EE_CFG_SIGNATURE_V24 || sig == EE_CFG_SIGNATURE_V23) {
        if (ee_read_slot(pole_slot, &v)) {
            const uint8_t poles=(uint8_t)(v & 0xffu);
            const uint8_t fq=(uint8_t)(v >> 8);
            if(poles>=2u && (poles&1u)==0u)m->m_conf.si_motor_poles=poles;
            if(fq>0u)m->m_conf.foc_current_filter_const=(float)fq/255.0f;
        }
        if (ee_read_slot(gear_slot, &v) && v > 0u) m->m_conf.si_gear_ratio = (float)v / 64.0f;
    } else if (sig == EE_CFG_SIGNATURE_V22 || sig == EE_CFG_SIGNATURE_V21) {
        /* V21/V22 stored plain poles in this slot and already had gear ratio. */
        if (ee_read_slot(pole_slot, &v) && v >= 2u && v <= 254u && (v & 1u) == 0u) m->m_conf.si_motor_poles=(uint8_t)v;
        if (ee_read_slot(gear_slot, &v) && v > 0u) m->m_conf.si_gear_ratio=(float)v/64.0f;
        m->m_conf.foc_current_filter_const=MCCONF_FOC_TELEMETRY_FILTER_DEFAULT;
    }
    if (sig == EE_CFG_SIGNATURE_VALUE || sig == EE_CFG_SIGNATURE_V28) {
        if (!ee_read_slot(telem_filter_slot, &v) || v < 10u || v > 10000u) return false;
        m->m_conf.foc_current_filter_const=(float)v/10000.0f;
    } else if (migrate_telem_filter) {
        /* Canonical-kan nilai legacy ke grid float16 VESC (scale 1e4).
         * buffer_append_float16 memakai truncation; nilai Q8 lama dapat berada
         * tepat di bawah batas integer akibat representasi float dan membuat
         * Write MC Config pertama bergeser 1 LSB. Dua iterasi cukup untuk
         * mencapai representasi yang serialize->deserialize stabil. */
        float tf = m->m_conf.foc_current_filter_const;
        for (uint8_t pass = 0u; pass < 3u; ++pass) {
            uint32_t q = (uint32_t)(tf * 10000.0f);
            if (q < 10u) q = 10u;
            if (q > 10000u) q = 10000u;
            const float next = (float)q / 10000.0f;
            const uint32_t q_next = (uint32_t)(next * 10000.0f);
            tf = next;
            if (q_next == q) break;
        }
        m->m_conf.foc_current_filter_const = tf;
    }
    const uint16_t pp = mcpwm_foc_get_pole_pairs(second);

    if (ee_read_slot(cur_slot, &v) && v >= 10u && v <= I_MOT_MAX * 100u) {
        m->m_conf.l_current_max = (float)v / 100.0f;
        m->m_conf.l_current_min = -m->m_conf.l_current_max;
        m->m_current_limit_q4 = (int16_t)((int32_t)v * A2BIT_CONV * 16 / 100);
    }
    if (sig == EE_CFG_SIGNATURE_VALUE || sig == EE_CFG_SIGNATURE_V28 || sig == EE_CFG_SIGNATURE_V27 || sig == EE_CFG_SIGNATURE_V26 || sig == EE_CFG_SIGNATURE_V25 || sig == EE_CFG_SIGNATURE_V24) {
        uint16_t e[6]; bool ext_ok=true;
        for(uint8_t i=0u;i<6u;++i) ext_ok &= ee_read_slot((uint8_t)(ext_base+i),&e[i]);
        if(ext_ok){
            const int16_t cmin=(int16_t)e[0];
            if(cmin<=-10 && cmin>=-(int16_t)(I_MOT_MAX*100))m->m_conf.l_current_min=(float)cmin/100.0f;
            if(e[1]>=10u && e[1]<=(uint16_t)(MCCONF_L_ABS_CURRENT_MAX*100.0f+0.5f))m->m_conf.l_abs_current_max=(float)e[1]/100.0f;
            {
                const uint16_t duty=(uint16_t)(e[2]&0x7fffu);
                m->m_conf.l_slow_abs_current=(e[2]&0x8000u)!=0u;
                if(duty>=1u && duty<=10000u)m->m_conf.l_max_duty=(float)duty/10000.0f;
            }
            if(e[3]<=10000u)m->m_conf.p_pid_kd_filter=(float)e[3]/10000.0f;
            if(e[4]>0u)m->m_conf.foc_duty_dowmramp_kp=(float)e[4]/10.0f;
            if(e[5]>0u)m->m_conf.foc_duty_dowmramp_ki=(float)e[5]/10.0f;
        }
    }
    if (sig == EE_CFG_SIGNATURE_VALUE || sig == EE_CFG_SIGNATURE_V28 || sig == EE_CFG_SIGNATURE_V27 || sig == EE_CFG_SIGNATURE_V26 || sig == EE_CFG_SIGNATURE_V25) {
        uint16_t x[3]; bool io_ok=true;
        for(uint8_t i=0u;i<3u;++i)io_ok &= ee_read_slot((uint8_t)(io_base+i),&x[i]);
        if(io_ok){
            const int16_t imin=(int16_t)x[1];
            if(x[0]>=10u && x[0]<=I_DC_MAX*100u)m->m_conf.l_in_current_max=(float)x[0]/100.0f;
            if(imin<=-10 && imin>=-(int16_t)(I_DC_MAX*100))m->m_conf.l_in_current_min=(float)imin/100.0f;
            if(x[2]>=1u && x[2]<=2000u)m->m_conf.m_duty_ramp_step=(float)x[2]/10000.0f;
        }
    }
    if ((sig == EE_CFG_SIGNATURE_VALUE || sig == EE_CFG_SIGNATURE_V28 || sig == EE_CFG_SIGNATURE_V27 || sig == EE_CFG_SIGNATURE_V26) && ee_read_slot(ccmin_slot,&v) && v>=1u && v<=100u) {
        m->m_conf.cc_min_current=(float)v/100.0f;
    } else if (sig != EE_CFG_SIGNATURE_VALUE) {
        m->m_conf.cc_min_current=MCCONF_CC_MIN_CURRENT;
    }
    if (sig == EE_CFG_SIGNATURE_VALUE || sig == EE_CFG_SIGNATURE_V28 || sig == EE_CFG_SIGNATURE_V27) {
        uint16_t x[7]; bool ext2_ok=true;
        for(uint8_t i=0u;i<7u;++i)ext2_ok &= ee_read_slot((uint8_t)(ext2_base+i),&x[i]);
        if(ext2_ok){
            const float bcs=(float)x[0]/100.0f, bce=(float)x[1]/100.0f;
            if(bcs>bce && bce>=0.0f && bcs<=80.0f){m->m_conf.l_battery_cut_start=bcs;m->m_conf.l_battery_cut_end=bce;}
            if(x[2]<=10000u)m->m_conf.l_current_max_scale=(float)x[2]/10000.0f;
            if(x[3]<=10000u)m->m_conf.l_current_min_scale=(float)x[3]/10000.0f;
            const int16_t emin=(int16_t)x[4], emax=(int16_t)x[5];
            if(emin<0 && emin>=(int16_t)MCCONF_L_MIN_ERPM)m->m_conf.l_min_erpm=(float)emin;
            if(emax>0 && emax<=(int16_t)MCCONF_L_MAX_ERPM)m->m_conf.l_max_erpm=(float)emax;
            if(x[6]>=10u && x[6]<=50000u)m->m_conf.si_wheel_diameter=(float)x[6]/10000.0f;
        }
        uint16_t meta=0u, ahx=0u;
        if(ee_read_slot(bat_meta_slot,&meta)){
            const uint8_t cells=(uint8_t)(meta&0xffu);
            const uint8_t type=(uint8_t)(meta>>8);
            if(cells>=1u&&cells<=32u)m->m_conf.si_battery_cells=cells;
            if(type<=(uint8_t)BATTERY_TYPE_LEAD_ACID)m->m_conf.si_battery_type=(BATTERY_TYPE)type;
        }
        if(ee_read_slot(bat_ah_slot,&ahx))m->m_conf.si_battery_ah=(float)ahx/100.0f;
    } else {
        /* Older signatures never stored these fields. Keep hardware-safe V26
         * defaults, then migrate them into the append-only slots below. */
        m->m_conf.l_battery_cut_start=MCCONF_L_BATTERY_CUT_START;
        m->m_conf.l_battery_cut_end=MCCONF_L_BATTERY_CUT_END;
        m->m_conf.l_current_max_scale=MCCONF_L_CURRENT_MAX_SCALE;
        m->m_conf.l_current_min_scale=MCCONF_L_CURRENT_MIN_SCALE;
        m->m_conf.l_min_erpm=MCCONF_L_MIN_ERPM;
        m->m_conf.l_max_erpm=MCCONF_L_MAX_ERPM;
        m->m_conf.si_wheel_diameter=MCCONF_SI_WHEEL_DIAMETER;
        m->m_conf.si_battery_type=BATTERY_TYPE_LIION_3_0__4_2;
        m->m_conf.si_battery_cells=BAT_CELLS;
        m->m_conf.si_battery_ah=0.0f;
    }
    if (sig == EE_CFG_SIGNATURE_VALUE) {
        uint16_t vmin_cv=0u,vmax_cv=0u,tsraw=0u,teraw=0u;
        uint32_t wmax10=0u,wregen10=0u;
        if(!ee_read_slot(safety_base+0u,&vmin_cv) || !ee_read_slot(safety_base+1u,&vmax_cv) ||
           !ee_read_slot(safety_base+2u,&tsraw) || !ee_read_slot(safety_base+3u,&teraw) ||
           !ee_read_u32_pair((uint8_t)(safety_base+4u),&wmax10) ||
           !ee_read_u32_pair((uint8_t)(safety_base+6u),&wregen10)) return false;
        const float vmin=(float)vmin_cv/100.0f, vmax=(float)vmax_cv/100.0f;
        if(vmin>=5.0f&&vmax>vmin&&vmax<=80.0f){m->m_conf.l_min_vin=vmin;m->m_conf.l_max_vin=vmax;}
        const int16_t ts10=(int16_t)tsraw, te10=(int16_t)teraw;
        if(te10>ts10&&ts10>=-400&&te10<=1800){m->m_conf.l_temp_fet_start=(float)ts10/10.0f;m->m_conf.l_temp_fet_end=(float)te10/10.0f;}
        if(wmax10>0u)m->m_conf.l_watt_max=(float)wmax10/10.0f;
        if(wregen10>0u)m->m_conf.l_watt_min=-(float)wregen10/10.0f;
    } else {
        m->m_conf.l_min_vin=MCCONF_L_MIN_VIN; m->m_conf.l_max_vin=MCCONF_L_MAX_VIN;
        m->m_conf.l_temp_fet_start=MCCONF_L_TEMP_FET_START; m->m_conf.l_temp_fet_end=MCCONF_L_TEMP_FET_END;
        m->m_conf.l_watt_max=MCCONF_L_WATT_MAX; m->m_conf.l_watt_min=MCCONF_L_WATT_MIN;
    }
    m->m_conf.m_sensor_port_mode=SENSOR_PORT_MODE_HALL;
    m->m_conf.m_motor_temp_sens_type=TEMP_SENSOR_DISABLED;
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
            if (migrate_speed_pid_scale && i >= 4u && i <= 6u) {
                /* V20 stored speed gains as gain*1000. New format stores
                 * gain*100000 so VESC Tool changes down to 1e-5 affect the
                 * live fixed-point regulator. */
                uint32_t sv=(uint32_t)v*100u;
                if(sv>65535u)sv=65535u;
                *gain_dst[i]=(uint16_t)sv;
            } else if (!(migrate_speed_pid && i >= 4u && i <= 6u) &&
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
    {
        float tf=m->m_conf.foc_current_filter_const;
        if (!(tf>=0.001f && tf<=1.0f)) tf=MCCONF_FOC_TELEMETRY_FILTER_DEFAULT;
        int32_t a=(int32_t)(tf*65535.0f+0.5f);
        if(a<1)a=1;
        if(a>65535)a=65535;
        m->m_telem_current_filter_q16=(uint16_t)a;
        m->m_conf.foc_current_filter_const=tf;
    }
    /* Rebuild every derived runtime coefficient that corresponds to persisted
     * VESC Tool fields. A value is not considered restored merely because
     * GET_MCCONF shows it; the ISR/controller must use it after reboot too. */
    {
        float cc=m->m_conf.cc_min_current;
        if(!(cc>=0.001f && cc<=1.0f))cc=MCCONF_CC_MIN_CURRENT;
        m->m_conf.cc_min_current=cc;
        float imax=m->m_conf.l_in_current_max;
        float imin=m->m_conf.l_in_current_min;
        if(!(imax>=0.1f) || imax>(float)I_DC_MAX)imax=MCCONF_L_IN_CURRENT_MAX;
        if(!(imin<=-0.1f) || imin<-(float)I_DC_MAX)imin=MCCONF_L_IN_CURRENT_MIN;
        m->m_conf.l_in_current_max=imax; m->m_conf.l_in_current_min=imin;
        float smx=m->m_conf.l_current_max_scale; if(!(smx>=0.0f&&smx<=1.0f))smx=MCCONF_L_CURRENT_MAX_SCALE;
        float smn=m->m_conf.l_current_min_scale; if(!(smn>=0.0f&&smn<=1.0f))smn=MCCONF_L_CURRENT_MIN_SCALE;
        m->m_conf.l_current_max_scale=smx; m->m_conf.l_current_min_scale=smn;
        int32_t lim_pos=(int32_t)(m->m_conf.l_current_max*smx*(float)(A2BIT_CONV*16)+0.5f);
        int32_t lim_neg=(int32_t)(-m->m_conf.l_current_min*smn*(float)(A2BIT_CONV*16)+0.5f);
        if (lim_pos < 1) {
            lim_pos = 1;
        }
        if (lim_pos > MCCONF_MOTOR_CURRENT_MAX_Q4) {
            lim_pos = MCCONF_MOTOR_CURRENT_MAX_Q4;
        }
        if (lim_neg < 1) {
            lim_neg = 1;
        }
        if (lim_neg > MCCONF_MOTOR_CURRENT_MAX_Q4) {
            lim_neg = MCCONF_MOTOR_CURRENT_MAX_Q4;
        }
        m->m_current_limit_q4=(int16_t)lim_pos; m->m_current_limit_neg_q4=(int16_t)lim_neg;
        float bcs=m->m_conf.l_battery_cut_start, bce=m->m_conf.l_battery_cut_end;
        if(!(bcs>bce&&bce>=0.0f)){bcs=MCCONF_L_BATTERY_CUT_START;bce=MCCONF_L_BATTERY_CUT_END;}
        m->m_conf.l_battery_cut_start=bcs; m->m_conf.l_battery_cut_end=bce;
        int32_t bsa=(int32_t)(bcs*100.0f*(float)BAT_CALIB_ADC/(float)BAT_CALIB_REAL_VOLTAGE+0.5f);
        int32_t bea=(int32_t)(bce*100.0f*(float)BAT_CALIB_ADC/(float)BAT_CALIB_REAL_VOLTAGE+0.5f);
        if (bsa < 1) {
            bsa = 1;
        }
        if (bsa > 4095) {
            bsa = 4095;
        }
        if (bea < 0) {
            bea = 0;
        }
        if (bea > 4094) {
            bea = 4094;
        }
        m->m_battery_cut_start_adc=(uint16_t)bsa; m->m_battery_cut_end_adc=(uint16_t)bea;
        float vmin=m->m_conf.l_min_vin, vmax=m->m_conf.l_max_vin;
        if(!(vmin>=5.0f&&vmax>vmin&&vmax<=80.0f)){vmin=MCCONF_L_MIN_VIN;vmax=MCCONF_L_MAX_VIN;}
        m->m_conf.l_min_vin=vmin; m->m_conf.l_max_vin=vmax;
        m->m_vin_min_adc=(uint16_t)(vmin*100.0f*(float)BAT_CALIB_ADC/(float)BAT_CALIB_REAL_VOLTAGE+0.5f);
        m->m_vin_max_adc=(uint16_t)(vmax*100.0f*(float)BAT_CALIB_ADC/(float)BAT_CALIB_REAL_VOLTAGE+0.5f);
        float wmax=m->m_conf.l_watt_max, wmin=m->m_conf.l_watt_min;
        if(!(wmax>0.0f&&wmax<=200000000.0f))wmax=MCCONF_L_WATT_MAX;
        if(!(wmin<0.0f&&wmin>=-200000000.0f))wmin=MCCONF_L_WATT_MIN;
        m->m_conf.l_watt_max=wmax; m->m_conf.l_watt_min=wmin;
        m->m_watt_max_x10=(uint32_t)(wmax*10.0f+0.5f);
        m->m_watt_regen_x10=(uint32_t)(-wmin*10.0f+0.5f);
        float ts=m->m_conf.l_temp_fet_start, te=m->m_conf.l_temp_fet_end;
        if(!(te>ts&&ts>=-40.0f&&te<=180.0f)){ts=MCCONF_L_TEMP_FET_START;te=MCCONF_L_TEMP_FET_END;}
        m->m_conf.l_temp_fet_start=ts; m->m_conf.l_temp_fet_end=te;
        m->m_temp_fet_start_x10=(int16_t)(ts*10.0f+(ts>=0.0f?0.5f:-0.5f));
        m->m_temp_fet_end_x10=(int16_t)(te*10.0f+(te>=0.0f?0.5f:-0.5f));
        m->m_wrong_voltage_integrator=0u;
        m->m_conf.m_motor_temp_sens_type=TEMP_SENSOR_DISABLED;
        int32_t imax_q4=(int32_t)(imax*(float)(A2BIT_CONV*16)+0.5f);
        int32_t iregen_q4=(int32_t)(-imin*(float)(A2BIT_CONV*16)+0.5f);
        const int32_t idc_q4_max=I_DC_MAX*A2BIT_CONV*16;
        if(imax_q4<1)imax_q4=1;
        if(imax_q4>idc_q4_max)imax_q4=idc_q4_max;
        if(iregen_q4<1)iregen_q4=1;
        if(iregen_q4>idc_q4_max)iregen_q4=idc_q4_max;
        m->m_input_current_max_q4=(int16_t)imax_q4;
        m->m_input_current_regen_q4=(int16_t)iregen_q4;
        float dr=m->m_conf.m_duty_ramp_step;
        if(!(dr>=0.0001f && dr<=0.20f))dr=MCCONF_DUTY_RAMP_STEP_DEFAULT;
        m->m_conf.m_duty_ramp_step=dr;
        int32_t drp=(int32_t)(dr*1000.0f+0.5f);
        if(drp<1)drp=1;
        if(drp>200)drp=200;
        m->m_duty_ramp_step_permille=(uint16_t)drp;
        float abs_i=m->m_conf.l_abs_current_max;
        float commanded_abs=m->m_conf.l_current_max;
        if(-m->m_conf.l_current_min>commanded_abs)commanded_abs=-m->m_conf.l_current_min;
        if(!(abs_i>=commanded_abs) || abs_i>MCCONF_L_ABS_CURRENT_MAX) abs_i=MCCONF_L_ABS_CURRENT_MAX;
        m->m_conf.l_abs_current_max=abs_i;
        int32_t ac=(int32_t)(abs_i*(float)A2BIT_CONV+0.5f);
        if(ac<1) ac=1;
        if(ac>32767) ac=32767;
        m->m_abs_current_limit_counts=(int16_t)ac;

        float md=m->m_conf.l_max_duty;
        if(!(md>0.0f) || md>MCCONF_L_MAX_DUTY)md=MCCONF_L_MAX_DUTY;
        m->m_conf.l_max_duty=md;
        int32_t dp=(int32_t)(md*1000.0f+0.5f);
        if(dp<1) dp=1;
        if(dp>1000) dp=1000;
        m->m_duty_limit_permille=(int16_t)dp;

        float pdf=m->m_conf.p_pid_kd_filter;
        if(!(pdf>=0.0f && pdf<=1.0f))pdf=(float)MCCONF_POSITION_KD_FILTER_Q16/65536.0f;
        m->m_conf.p_pid_kd_filter=pdf;
        int32_t pf=(int32_t)(pdf*65535.0f+0.5f);
        if(pf<0) pf=0;
        if(pf>65535) pf=65535;
        m->m_position_kd_filter_q16=(uint16_t)pf;

        float dkp=m->m_conf.foc_duty_dowmramp_kp; if(!(dkp>0.0f))dkp=MCCONF_FOC_DUTY_DOWNRAMP_KP;
        float dki=m->m_conf.foc_duty_dowmramp_ki; if(!(dki>0.0f))dki=MCCONF_FOC_DUTY_DOWNRAMP_KI;
        m->m_conf.foc_duty_dowmramp_kp=dkp; m->m_conf.foc_duty_dowmramp_ki=dki;
        const float kscale=(32768.0f*4096.0f)/(1000.0f*MCCONF_DUTY_PI_BUS_NOMINAL_V);
        const float dt=(float)MCCONF_FOC_CONTROL_DIV/(float)PWM_FREQ;
        m->m_duty_kp_q12_per_permille=(uint32_t)(dkp*kscale+0.5f);
        m->m_duty_ki_q12_per_permille=(uint32_t)(dki*dt*kscale+0.5f);
    }
    if (migrate_speed_pid_scale || migrate_speed_pid || migrate_position_pid || migrate_telem_filter || migrate_mc_extension) {
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
