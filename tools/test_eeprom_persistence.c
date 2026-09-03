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

    uint8_t hl[8],hr[8]; fill_table(hl,5u); fill_table(hr,11u);
    mc_configuration cl=m_motor_1.m_conf, cr=m_motor_2.m_conf;
    memcpy(cl.foc_hall_table,hl,8); memcpy(cr.foc_hall_table,hr,8);
    cl.l_current_max=12.34f; cl.l_current_min=-7.65f; cl.l_abs_current_max=18.25f; cl.l_max_duty=0.9134f;
    cr.l_current_max=9.87f; cr.l_current_min=-6.54f; cr.l_abs_current_max=17.75f; cr.l_max_duty=0.8765f;
    cl.p_pid_kd_filter=0.37f; cr.p_pid_kd_filter=0.63f;
    cl.foc_duty_dowmramp_kp=23.4f; cl.foc_duty_dowmramp_ki=456.7f;
    cr.foc_duty_dowmramp_kp=17.8f; cr.foc_duty_dowmramp_ki=321.2f;
    cl.si_motor_poles=20u; cl.si_gear_ratio=5.25f; cl.foc_current_filter_const=0.07f;
    cr.si_motor_poles=14u; cr.si_gear_ratio=1.0f; cr.foc_current_filter_const=0.21f;
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
    if(fabsf(m_motor_1.m_conf.l_abs_current_max-18.25f)>0.011f || fabsf(m_motor_2.m_conf.l_abs_current_max-17.75f)>0.011f) return fail("abs current persistence");
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
    if(fabsf(m_motor_1.m_conf.foc_current_filter_const-0.07f)>0.005f) return fail("left telemetry filter persistence");
    if(fabsf(m_motor_2.m_conf.foc_current_filter_const-0.21f)>0.005f) return fail("right telemetry filter persistence");
    if(m_motor_1.m_kpq_q11!=1111u || m_motor_1.m_kiq_q16!=2222u || m_motor_1.m_kdp_q11!=111u) return fail("left gains persistence");
    if(m_motor_2.m_kpq_q11!=1212u || m_motor_2.m_kiq_q16!=2323u || m_motor_2.m_kdp_q11!=121u) return fail("right gains persistence");
    if(m_motor_1.m_speed_ramp_rpm_s!=123u || m_motor_1.m_speed_release_rpm!=7u) return fail("left speed persistence");
    if(m_motor_2.m_speed_ramp_rpm_s!=234u || m_motor_2.m_speed_release_rpm!=8u) return fail("right speed persistence");

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
