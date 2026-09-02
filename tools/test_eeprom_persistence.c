#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
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
    cl.l_current_max=12.34f; cl.l_current_min=-12.34f;
    cr.l_current_max=9.87f; cr.l_current_min=-9.87f;
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
    if(m_motor_1.m_kpq_q11!=1111u || m_motor_1.m_kiq_q16!=2222u || m_motor_1.m_kdp_q11!=111u) return fail("left gains persistence");
    if(m_motor_2.m_kpq_q11!=1212u || m_motor_2.m_kiq_q16!=2323u || m_motor_2.m_kdp_q11!=121u) return fail("right gains persistence");
    if(m_motor_1.m_speed_ramp_rpm_s!=123u || m_motor_1.m_speed_release_rpm!=7u) return fail("left speed persistence");
    if(m_motor_2.m_speed_ramp_rpm_s!=234u || m_motor_2.m_speed_release_rpm!=8u) return fail("right speed persistence");

    /* Independent signatures: corrupt only right signature; left remains valid. */
    ee_value[44]=0u;
    mcpwm_foc_init();
    if(!mc_interface_load_configuration_motor(false)) return fail("left must survive right signature corruption");
    if(mc_interface_load_configuration_motor(true)) return fail("right corrupted signature must reject");

    printf("EEPROM_DUAL_PERSISTENCE_PASS leftI=%.2f rightI=%.2f leftHall=%u rightHall=%u leftRamp=%u rightRamp=%u\n",
           m_motor_1.m_conf.l_current_max,9.87f,m_motor_1.m_conf.foc_hall_table[1],hr[1],
           m_motor_1.m_speed_ramp_rpm_s,234u);
    return 0;
}
