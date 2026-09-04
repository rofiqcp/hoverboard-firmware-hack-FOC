#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "stm32f1xx_hal.h"
#include "config.h"
#include "defines.h"
#include "eeprom.h"
#include "motor/mcpwm_foc.h"
#include "motor/mcconf_default.h"
#include "motor/mc_interface.h"

GPIO_TypeDef _GPIOA={0},_GPIOB={0},_GPIOC={0};
TIM_TypeDef _TIM1={0},_TIM8={0};
DMA_TypeDef _DMA1={0};
DWT_Type _DWT={0};
CoreDebug_Type _CoreDebug={0};
volatile adc_buf_t adc_buffer={0};
uint8_t ctrlModReq=VLT_MODE;
uint16_t VirtAddVarTab[NB_OF_VAR];

static uint16_t ee_value[NB_OF_VAR];
static uint8_t ee_valid[NB_OF_VAR];

static int slot_from_addr(uint16_t addr){
    for(unsigned i=0;i<NB_OF_VAR;i++) if(VirtAddVarTab[i]==addr) return (int)i;
    return -1;
}
uint16_t EE_ReadVariable(uint16_t VirtAddress, uint16_t* Data){
    int i=slot_from_addr(VirtAddress);
    if(i<0 || !ee_valid[i]) return 1u;
    *Data=ee_value[i];
    return 0u;
}
uint16_t EE_WriteVariable(uint16_t VirtAddress, uint16_t Data){
    int i=slot_from_addr(VirtAddress);
    if(i<0) return 1u;
    ee_value[i]=Data; ee_valid[i]=1u;
    return HAL_OK;
}
uint16_t EE_Init(void){return 0u;}
uint16_t EE_Format(void){memset(ee_valid,0,sizeof(ee_valid));return 0u;}

void filtLowPass32(int16_t u, uint16_t coef, int32_t *y){
    int32_t err=(int32_t)u-(*y>>16);
    if(err>32767)err=32767; else if(err<-32768)err=-32768;
    *y+=(int32_t)coef*err;
}

static int fail(const char *s){fprintf(stderr,"FAIL %s\n",s);return 1;}
static void fill_table(uint8_t t[8], uint8_t base){
    t[0]=255u; t[7]=255u;
    for(int i=1;i<=6;i++) t[i]=(uint8_t)(base+(i-1)*33u);
}
static int same8(const uint8_t a[8],const uint8_t b[8]){return memcmp(a,b,8)==0;}

int main(void){
    for(unsigned i=0;i<NB_OF_VAR;i++) VirtAddVarTab[i]=(uint16_t)(1000u+i);
    memset(ee_valid,0,sizeof(ee_valid));
    mcpwm_foc_init();

    /* ABS current must cover the magnitude of BOTH current directions. A
     * braking limit larger than the positive motoring limit must not slip past
     * validation and cause a guaranteed ABS fault during regen. */
    {
        mc_configuration bad=m_motor_1.m_conf;
        bad.l_current_max=1.0f; bad.l_current_min=-12.0f; bad.l_abs_current_max=10.0f;
        mcpwm_foc_set_configuration(&bad,false);
        if(fabsf(m_motor_1.m_conf.l_abs_current_max-MCCONF_L_ABS_CURRENT_MAX)>0.001f)
            return fail("ABS current must cover negative current magnitude");
        if(m_motor_1.m_abs_current_limit_counts!=(int16_t)(MCCONF_L_ABS_CURRENT_MAX*A2BIT_CONV+0.5f))
            return fail("ABS runtime count scaling after bidirectional validation");
        bad.l_abs_current_max=13.0f;
        mcpwm_foc_set_configuration(&bad,false);
        if(fabsf(m_motor_1.m_conf.l_abs_current_max-13.0f)>0.001f ||
           m_motor_1.m_abs_current_limit_counts!=13*A2BIT_CONV)
            return fail("valid ABS current setting must stay authoritative");
    }

    uint8_t hl[8],hr[8]; fill_table(hl,5u); fill_table(hr,11u);
    mc_configuration cl=m_motor_1.m_conf, cr=m_motor_2.m_conf;
    memcpy(cl.foc_hall_table,hl,8); memcpy(cr.foc_hall_table,hr,8);
    cl.l_current_max=12.34f; cl.l_current_min=-7.65f; cl.l_abs_current_max=18.25f; cl.l_max_duty=0.9134f; cl.l_slow_abs_current=true;
    cr.l_current_max=9.87f; cr.l_current_min=-6.54f; cr.l_abs_current_max=17.75f; cr.l_max_duty=0.8765f; cr.l_slow_abs_current=false;
    cl.l_in_current_max=14.50f; cl.l_in_current_min=-13.50f; cl.m_duty_ramp_step=0.0312f; cl.cc_min_current=0.17f;
    cr.l_in_current_max=12.25f; cr.l_in_current_min=-11.75f; cr.m_duty_ramp_step=0.0175f; cr.cc_min_current=0.23f;
    cl.l_current_max_scale=0.80f; cl.l_current_min_scale=0.60f;
    cr.l_current_max_scale=0.70f; cr.l_current_min_scale=0.50f;
    cl.l_battery_cut_start=37.20f; cl.l_battery_cut_end=33.10f;
    cr.l_battery_cut_start=36.80f; cr.l_battery_cut_end=32.90f;
    cl.l_min_vin=28.50f; cl.l_max_vin=52.30f; cl.l_watt_max=1234.5f; cl.l_watt_min=-987.6f; cl.l_temp_fet_start=61.2f; cl.l_temp_fet_end=72.3f;
    cr.l_min_vin=29.20f; cr.l_max_vin=51.70f; cr.l_watt_max=1111.1f; cr.l_watt_min=-888.8f; cr.l_temp_fet_start=59.4f; cr.l_temp_fet_end=70.5f;
    cl.l_min_erpm=-12000.0f; cl.l_max_erpm=10000.0f; cl.si_wheel_diameter=0.2450f;
    cr.l_min_erpm=-9000.0f; cr.l_max_erpm=11000.0f; cr.si_wheel_diameter=0.3100f;
    cl.p_pid_kd_filter=0.37f; cr.p_pid_kd_filter=0.63f;
    cl.foc_duty_dowmramp_kp=23.4f; cl.foc_duty_dowmramp_ki=456.7f;
    cr.foc_duty_dowmramp_kp=17.8f; cr.foc_duty_dowmramp_ki=321.2f;
    cl.si_motor_poles=20u; cl.si_gear_ratio=5.25f; cl.foc_current_filter_const=0.0731f; cl.foc_hall_interp_erpm=620.0f;
    cl.m_sensor_port_mode=SENSOR_PORT_MODE_ABI; cl.foc_sensor_mode=FOC_SENSOR_MODE_ENCODER;
    cl.m_encoder_counts=4096; cl.foc_encoder_offset=17.25f; cl.foc_encoder_ratio=10.0f; cl.foc_encoder_inverted=true;
    cr.si_motor_poles=14u; cr.si_gear_ratio=1.0f; cr.foc_current_filter_const=0.2197f; cr.foc_hall_interp_erpm=900.0f;
    mcpwm_foc_set_configuration(&cl,false);
    mcpwm_foc_set_configuration(&cr,true);
    m_motor_1.m_kpq_q11=1111u; m_motor_1.m_kiq_q16=2222u;
    m_motor_1.m_kpd_q11=333u;  m_motor_1.m_kid_q16=444u;
    m_motor_1.m_kps_q11=555u;  m_motor_1.m_kis_q16=666u; m_motor_1.m_kds_q11=77u;
    m_motor_1.m_kpp_q11=888u;  m_motor_1.m_kip_q16=999u; m_motor_1.m_kdp_q11=111u;
    m_motor_1.m_speed_ramp_rpm_s=123u; m_motor_1.m_speed_release_rpm=7u;
    m_motor_2.m_kpq_q11=1212u; m_motor_2.m_kiq_q16=2323u;
    m_motor_2.m_kpd_q11=343u;  m_motor_2.m_kid_q16=454u;
    m_motor_2.m_kps_q11=565u;  m_motor_2.m_kis_q16=676u; m_motor_2.m_kds_q11=87u;
    m_motor_2.m_kpp_q11=898u;  m_motor_2.m_kip_q16=909u; m_motor_2.m_kdp_q11=121u;
    m_motor_2.m_speed_ramp_rpm_s=234u; m_motor_2.m_speed_release_rpm=8u;
    if(!mc_interface_store_configuration_motor(false)) return fail("store left");
    if(!mc_interface_store_configuration_motor(true)) return fail("store right");

    mcpwm_foc_init();
    if(!mc_interface_load_configuration_motor(false)) return fail("load left");
    if(!mc_interface_load_configuration_motor(true)) return fail("load right");
    if(!same8(m_motor_1.m_conf.foc_hall_table,hl)) return fail("left Hall persistence");
    if(!same8(m_motor_2.m_conf.foc_hall_table,hr)) return fail("right Hall persistence");
    if(fabsf(m_motor_1.m_conf.l_current_max-12.34f)>0.011f) return fail("left current persistence");
    if(fabsf(m_motor_2.m_conf.l_current_max-9.87f)>0.011f) return fail("right current persistence");
    if(fabsf(m_motor_1.m_conf.l_current_min+7.65f)>0.011f || fabsf(m_motor_2.m_conf.l_current_min+6.54f)>0.011f) return fail("current min persistence");
    if(fabsf(m_motor_1.m_conf.l_in_current_max-14.50f)>0.011f || fabsf(m_motor_2.m_conf.l_in_current_max-12.25f)>0.011f) return fail("input current max persistence");
    if(fabsf(m_motor_1.m_conf.l_in_current_min+13.50f)>0.011f || fabsf(m_motor_2.m_conf.l_in_current_min+11.75f)>0.011f) return fail("input current min persistence");
    if(fabsf(m_motor_1.m_conf.m_duty_ramp_step-0.0312f)>0.00011f || fabsf(m_motor_2.m_conf.m_duty_ramp_step-0.0175f)>0.00011f) return fail("duty ramp persistence");
    if(fabsf(m_motor_1.m_conf.cc_min_current-0.17f)>0.011f || fabsf(m_motor_2.m_conf.cc_min_current-0.23f)>0.011f) return fail("cc_min_current persistence");
    if(fabsf(m_motor_1.m_conf.l_current_max_scale-0.80f)>0.00011f ||
       fabsf(m_motor_1.m_conf.l_current_min_scale-0.60f)>0.00011f ||
       fabsf(m_motor_2.m_conf.l_current_max_scale-0.70f)>0.00011f ||
       fabsf(m_motor_2.m_conf.l_current_min_scale-0.50f)>0.00011f) return fail("current scale persistence");
    if(fabsf(m_motor_1.m_conf.l_battery_cut_start-1.0f*37.20f)>0.011f ||
       fabsf(m_motor_1.m_conf.l_battery_cut_end-33.10f)>0.011f ||
       fabsf(m_motor_2.m_conf.l_battery_cut_start-36.80f)>0.011f ||
       fabsf(m_motor_2.m_conf.l_battery_cut_end-32.90f)>0.011f) return fail("battery cut persistence");
    if(fabsf(m_motor_1.m_conf.l_min_vin-28.50f)>0.011f || fabsf(m_motor_1.m_conf.l_max_vin-52.30f)>0.011f ||
       fabsf(m_motor_2.m_conf.l_min_vin-29.20f)>0.011f || fabsf(m_motor_2.m_conf.l_max_vin-51.70f)>0.011f) return fail("Vin safety persistence");
    if(fabsf(m_motor_1.m_conf.l_watt_max-1234.5f)>0.11f || fabsf(m_motor_1.m_conf.l_watt_min+987.6f)>0.11f ||
       fabsf(m_motor_2.m_conf.l_watt_max-1111.1f)>0.11f || fabsf(m_motor_2.m_conf.l_watt_min+888.8f)>0.11f) return fail("watt safety persistence");
    if(fabsf(m_motor_1.m_conf.l_temp_fet_start-61.2f)>0.051f || fabsf(m_motor_1.m_conf.l_temp_fet_end-72.3f)>0.051f ||
       fabsf(m_motor_2.m_conf.l_temp_fet_start-59.4f)>0.051f || fabsf(m_motor_2.m_conf.l_temp_fet_end-70.5f)>0.051f) return fail("FET temperature persistence");
    if(m_motor_1.m_watt_max_x10!=12345u || m_motor_1.m_watt_regen_x10!=9876u ||
       m_motor_2.m_watt_max_x10!=11111u || m_motor_2.m_watt_regen_x10!=8888u) return fail("watt runtime restore");
    if(m_motor_1.m_temp_fet_start_x10!=612 || m_motor_1.m_temp_fet_end_x10!=723 ||
       m_motor_2.m_temp_fet_start_x10!=594 || m_motor_2.m_temp_fet_end_x10!=705) return fail("temperature runtime restore");
    if(abs((int)m_motor_1.m_vin_min_adc-(int)(28.50f*100.0f*BAT_CALIB_ADC/BAT_CALIB_REAL_VOLTAGE+0.5f))>1 ||
       abs((int)m_motor_2.m_vin_max_adc-(int)(51.70f*100.0f*BAT_CALIB_ADC/BAT_CALIB_REAL_VOLTAGE+0.5f))>1) return fail("Vin ADC runtime restore");
    if(fabsf(m_motor_1.m_conf.l_min_erpm+12000.0f)>0.5f || fabsf(m_motor_1.m_conf.l_max_erpm-10000.0f)>0.5f ||
       fabsf(m_motor_2.m_conf.l_min_erpm+9000.0f)>0.5f || fabsf(m_motor_2.m_conf.l_max_erpm-11000.0f)>0.5f) return fail("ERPM persistence/clamp");
    if(fabsf(m_motor_1.m_conf.si_wheel_diameter-0.2450f)>0.00011f ||
       fabsf(m_motor_2.m_conf.si_wheel_diameter-0.3100f)>0.00011f) return fail("wheel diameter persistence");
    if(abs((int)m_motor_1.m_current_limit_q4-7898)>1 || abs((int)m_motor_1.m_current_limit_neg_q4-3672)>1 ||
       abs((int)m_motor_2.m_current_limit_q4-5527)>1 || abs((int)m_motor_2.m_current_limit_neg_q4-2616)>1)
        return fail("scaled bidirectional current runtime restore");
    {
        const int left_start=(int)(37.20f*100.0f*(float)BAT_CALIB_ADC/(float)BAT_CALIB_REAL_VOLTAGE+0.5f);
        const int left_end=(int)(33.10f*100.0f*(float)BAT_CALIB_ADC/(float)BAT_CALIB_REAL_VOLTAGE+0.5f);
        if(abs((int)m_motor_1.m_battery_cut_start_adc-left_start)>1 ||
           abs((int)m_motor_1.m_battery_cut_end_adc-left_end)>1)
            return fail("battery cut ADC runtime restore");
    }
    if(m_motor_1.m_input_current_max_q4!=11600 || m_motor_1.m_input_current_regen_q4!=10800 ||
       m_motor_2.m_input_current_max_q4!=9800 || m_motor_2.m_input_current_regen_q4!=9400) return fail("input current runtime restore");
    if(m_motor_1.m_duty_ramp_step_permille!=31u || m_motor_2.m_duty_ramp_step_permille!=18u) return fail("duty ramp runtime restore");
    if(fabsf(m_motor_1.m_conf.l_abs_current_max-18.25f)>0.011f || fabsf(m_motor_2.m_conf.l_abs_current_max-17.75f)>0.011f) return fail("abs current persistence");
    if(!m_motor_1.m_conf.l_slow_abs_current || m_motor_2.m_conf.l_slow_abs_current) return fail("slow ABS current persistence");
    if(fabsf(m_motor_1.m_conf.l_max_duty-0.9134f)>0.00011f || fabsf(m_motor_2.m_conf.l_max_duty-0.8765f)>0.00011f) return fail("max duty persistence");
    if(fabsf(m_motor_1.m_conf.p_pid_kd_filter-0.37f)>0.00011f || fabsf(m_motor_2.m_conf.p_pid_kd_filter-0.63f)>0.00011f) return fail("position D filter persistence");
    if(fabsf(m_motor_1.m_conf.foc_duty_dowmramp_kp-23.4f)>0.051f || fabsf(m_motor_2.m_conf.foc_duty_dowmramp_kp-17.8f)>0.051f) return fail("duty downramp Kp persistence");
    if(fabsf(m_motor_1.m_conf.foc_duty_dowmramp_ki-456.7f)>0.051f || fabsf(m_motor_2.m_conf.foc_duty_dowmramp_ki-321.2f)>0.051f) return fail("duty downramp Ki persistence");
    if(m_motor_1.m_abs_current_limit_counts<912 || m_motor_1.m_abs_current_limit_counts>914 ||
       m_motor_2.m_abs_current_limit_counts<887 || m_motor_2.m_abs_current_limit_counts>889) return fail("absolute current runtime restore");
    if(m_motor_1.m_duty_limit_permille!=913 || m_motor_2.m_duty_limit_permille!=877) return fail("max duty runtime restore");
    if(abs((int)m_motor_1.m_position_kd_filter_q16-(int)(0.37f*65535.0f+0.5f))>2 ||
       abs((int)m_motor_2.m_position_kd_filter_q16-(int)(0.63f*65535.0f+0.5f))>2) return fail("position D filter runtime restore");
    if(m_motor_1.m_duty_kp_q12_per_permille==0u || m_motor_1.m_duty_ki_q12_per_permille==0u ||
       m_motor_2.m_duty_kp_q12_per_permille==0u || m_motor_2.m_duty_ki_q12_per_permille==0u) return fail("duty PI runtime restore");
    if(m_motor_1.m_conf.si_motor_poles!=20u || fabsf(m_motor_1.m_conf.si_gear_ratio-5.25f)>0.02f) return fail("left poles/gear persistence");
    if(m_motor_2.m_conf.si_motor_poles!=14u || fabsf(m_motor_2.m_conf.si_gear_ratio-1.0f)>0.02f) return fail("right poles/gear persistence");
    if(fabsf(m_motor_1.m_conf.foc_current_filter_const-0.0731f)>0.00011f) return fail("left telemetry filter persistence");
    if(fabsf(m_motor_2.m_conf.foc_current_filter_const-0.2197f)>0.00011f) return fail("right telemetry filter persistence");
    if(fabsf(m_motor_1.m_conf.foc_hall_interp_erpm-620.0f)>0.5f || fabsf(m_motor_2.m_conf.foc_hall_interp_erpm-900.0f)>0.5f)
        return fail("Hall interpolation ERPM persistence");
    if(m_motor_1.m_conf.m_sensor_port_mode!=SENSOR_PORT_MODE_ABI || m_motor_1.m_conf.foc_sensor_mode!=FOC_SENSOR_MODE_ENCODER ||
       m_motor_1.m_conf.m_encoder_counts!=4096 || !m_motor_1.m_conf.foc_encoder_inverted ||
       fabsf(m_motor_1.m_conf.foc_encoder_offset-17.25f)>0.011f || fabsf(m_motor_1.m_conf.foc_encoder_ratio-10.0f)>0.0002f)
        return fail("LEFT ABI encoder persistence");
    if(!m_motor_1.m_encoder_configured || m_motor_1.m_encoder_synced) return fail("LEFT ABI runtime init/sync lifecycle");
    if(m_motor_2.m_conf.m_sensor_port_mode!=SENSOR_PORT_MODE_HALL || m_motor_2.m_conf.foc_sensor_mode!=FOC_SENSOR_MODE_HALL)
        return fail("RIGHT must remain Hall-only");
    if(m_motor_1.m_hall_interp_erpm!=620u || m_motor_2.m_hall_interp_erpm!=900u ||
       m_motor_1.m_hall_interp_max_ticks==0u || m_motor_2.m_hall_interp_max_ticks==0u ||
       m_motor_1.m_hall_rate_min_step==0u || m_motor_2.m_hall_rate_min_step==0u)
        return fail("Hall interpolation runtime coefficient restore");
    if(m_motor_1.m_kpq_q11!=1111u || m_motor_1.m_kiq_q16!=2222u || m_motor_1.m_kdp_q11!=111u) return fail("left gains persistence");
    if(m_motor_2.m_kpq_q11!=1212u || m_motor_2.m_kiq_q16!=2323u || m_motor_2.m_kdp_q11!=121u) return fail("right gains persistence");
    if(m_motor_1.m_speed_ramp_rpm_s!=123u || m_motor_1.m_speed_release_rpm!=7u) return fail("left speed persistence");
    if(m_motor_2.m_speed_ramp_rpm_s!=234u || m_motor_2.m_speed_release_rpm!=8u) return fail("right speed persistence");

    /* V30 (0x601C) sudah punya Hall interpolation, tetapi belum menyimpan ABI.
     * Migrasi tidak boleh merusak nilai Hall interpolation lama. */
    ee_value[43]=0x601Cu; ee_value[44]=0x601Cu;
    for(unsigned i=181u;i<=185u;i++)ee_valid[i]=0u;
    mcpwm_foc_init();
    if(!mc_interface_load_configuration_motor(false) || !mc_interface_load_configuration_motor(true))
        return fail("V30 migration load");
    if(fabsf(m_motor_1.m_conf.foc_hall_interp_erpm-620.0f)>0.5f || fabsf(m_motor_2.m_conf.foc_hall_interp_erpm-900.0f)>0.5f)
        return fail("V30 Hall interpolation preservation");
    if(m_motor_1.m_conf.m_sensor_port_mode!=SENSOR_PORT_MODE_HALL || m_motor_1.m_conf.m_encoder_counts!=4096)
        return fail("V30 encoder default migration");
    if(ee_value[43]!=0x601Du || ee_value[44]!=0x601Du || !ee_valid[181] || !ee_valid[185])
        return fail("V30 encoder schema rewrite");

    /* V29 (0x601B) sudah punya seluruh safety persistence tetapi belum punya
     * slot foc_hall_interp_erpm. Migrasi harus memakai default upstream 500
     * ERPM, membangun koefisien runtime, lalu menulis schema/slot baru. */
    ee_value[43]=0x601Bu; ee_value[44]=0x601Bu;
    ee_valid[179]=ee_valid[180]=0u;
    mcpwm_foc_init();
    if(!mc_interface_load_configuration_motor(false) || !mc_interface_load_configuration_motor(true))
        return fail("V29 migration load");
    if(fabsf(m_motor_1.m_conf.foc_hall_interp_erpm-500.0f)>0.5f ||
       fabsf(m_motor_2.m_conf.foc_hall_interp_erpm-500.0f)>0.5f ||
       m_motor_1.m_hall_interp_erpm!=500u || m_motor_2.m_hall_interp_erpm!=500u)
        return fail("V29 Hall interpolation migration default/runtime");
    if(ee_value[43]!=0x601Du || ee_value[44]!=0x601Du || !ee_valid[179] || !ee_valid[180] ||
       ee_value[179]!=500u || ee_value[180]!=500u || !ee_valid[181] || !ee_valid[182] ||
       !ee_valid[183] || !ee_valid[184] || !ee_valid[185])
        return fail("V29 migration Hall/encoder slots/signature");

    /* Simulasikan upgrade image V28 (0x601A), yaitu firmware yang sudah punya
     * filter x10000 tetapi belum punya slot Vin/watt/temperatur. Field lama
     * harus tetap utuh, safety baru memakai default aman, lalu schema ditulis
     * ulang atomik menjadi 0x601D. */
    ee_value[43]=0x601Au; ee_value[44]=0x601Au;
    for(unsigned i=163u;i<179u;i++)ee_valid[i]=0u;
    mcpwm_foc_init();
    if(!mc_interface_load_configuration_motor(false) || !mc_interface_load_configuration_motor(true))
        return fail("V28 migration load");
    if(fabsf(m_motor_1.m_conf.l_battery_cut_start-37.20f)>0.011f ||
       fabsf(m_motor_2.m_conf.l_battery_cut_end-32.90f)>0.011f ||
       fabsf(m_motor_1.m_conf.foc_current_filter_const-0.0731f)>0.00011f ||
       fabsf(m_motor_2.m_conf.si_wheel_diameter-0.3100f)>0.00011f)
        return fail("V28 old-field preservation");
    if(fabsf(m_motor_1.m_conf.l_min_vin-MCCONF_L_MIN_VIN)>0.011f ||
       fabsf(m_motor_2.m_conf.l_temp_fet_end-MCCONF_L_TEMP_FET_END)>0.051f)
        return fail("V28 safety default migration");
    if(ee_value[43]!=0x601Du || ee_value[44]!=0x601Du)
        return fail("V28 migration signature");
    for(unsigned i=163u;i<179u;i++)if(!ee_valid[i])return fail("V28 migration safety slots");
    if(!ee_valid[179] || !ee_valid[180] || ee_value[179]!=500u || ee_value[180]!=500u ||
       !ee_valid[181] || !ee_valid[185]) return fail("V28 migration Hall/encoder slots");

    /* V27 (0x6019) juga harus tetap diterima. Hilangkan slot filter dan safety
     * untuk meniru image lama, lalu pastikan keduanya dibuat kembali. */
    ee_value[43]=0x6019u; ee_value[44]=0x6019u;
    ee_valid[161]=ee_valid[162]=0u;
    for(unsigned i=163u;i<179u;i++)ee_valid[i]=0u;
    mcpwm_foc_init();
    if(!mc_interface_load_configuration_motor(false) || !mc_interface_load_configuration_motor(true))
        return fail("V27 migration load");
    if(ee_value[43]!=0x601Du || ee_value[44]!=0x601Du || !ee_valid[161] || !ee_valid[162] ||
       !ee_valid[179] || !ee_valid[180] || !ee_valid[181] || !ee_valid[185])
        return fail("V27 migration rewrite/signature");

    /* Independent signatures: corrupt only right signature; left remains valid. */
    ee_value[44]=0u;
    mcpwm_foc_init();
    if(!mc_interface_load_configuration_motor(false)) return fail("left must survive right signature corruption");
    if(mc_interface_load_configuration_motor(true)) return fail("right corrupted signature must reject");

    printf("EEPROM_DUAL_PERSISTENCE_PASS leftI=%.2f rightI=%.2f leftHall=%u rightHall=%u leftRamp=%u rightRamp=%u poles=%u/%u gear=%.2f/%.2f\n",
           m_motor_1.m_conf.l_current_max,9.87f,m_motor_1.m_conf.foc_hall_table[1],hr[1],
           m_motor_1.m_speed_ramp_rpm_s,234u,m_motor_1.m_conf.si_motor_poles,m_motor_2.m_conf.si_motor_poles,
           m_motor_1.m_conf.si_gear_ratio,m_motor_2.m_conf.si_gear_ratio);
    return 0;
}
