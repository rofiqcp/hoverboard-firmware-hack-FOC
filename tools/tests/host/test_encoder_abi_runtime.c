#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "stm32f1xx_hal.h"
#include "config.h"
#include "defines.h"
#include "motor/mcpwm_foc.h"
#include "motor/mcconf_default.h"
#include "encoder/encoder.h"
#include "encoder/encoder_cfg.h"

GPIO_TypeDef _GPIOA={0},_GPIOB={0},_GPIOC={0};
TIM_TypeDef _TIM1={0},_TIM8={0};
DMA_TypeDef _DMA1={0};
DWT_Type _DWT={0};
CoreDebug_Type _CoreDebug={0};
volatile adc_buf_t adc_buffer={0};
uint8_t ctrlModReq=VLT_MODE;
extern uint8_t enable;

void filtLowPass32(int16_t u, uint16_t coef, int32_t *y) {
    int32_t err=(int32_t)u-(*y>>16);
    if(err>32767)err=32767; else if(err<-32768)err=-32768;
    *y+=(int32_t)coef*err;
}

static int fail(const char *s){fprintf(stderr,"FAIL %s\n",s);return 1;}
static bool nearf(float a,float b,float tol){return fabsf(a-b)<=tol;}
static mc_configuration abi_conf(void){
    mc_configuration c=m_motor_1.m_conf;
    c.m_sensor_port_mode=SENSOR_PORT_MODE_ABI;
    c.foc_sensor_mode=FOC_SENSOR_MODE_ENCODER_AB;
    c.m_encoder_counts=4096;
    c.foc_encoder_ratio=15.0f;
    c.foc_encoder_offset=10.0f;
    c.foc_encoder_inverted=false;
    return c;
}

int main(void){
    mcpwm_foc_init();
    mc_configuration c=abi_conf();
    mcpwm_foc_set_configuration(&c,false);
    if(encoder_is_configured()!=ENCODER_TYPE_ABI)return fail("ABI not configured");
    if(encoder_index_found())return fail("ABI must start unsynced without I");
    if(m_motor_1.m_encoder_synced)return fail("FOC sync must start false");

    encoder_set_deg(90.0f);
    if(!encoder_index_found())return fail("software electrical sync not latched");
    m_motor_1.m_encoder_synced=1u;
    mcpwm_foc_adc_int_handler();
    if(!nearf(mcpwm_foc_get_encoder_position_motor(false),90.0f,0.11f))
        return fail("mechanical encoder position");
    if(!nearf(mcpwm_foc_get_phase_encoder_motor(false),260.0f,0.25f))
        return fail("electrical phase ratio-offset");
    if(m_motor_1.m_phase!=m_motor_1.m_phase_encoder)
        return fail("FOC phase must select synced encoder");
    mc_values v;
    mcpwm_foc_get_values(&v,false);
    if(!nearf(v.position,90.0f,0.11f))return fail("GET_VALUES position must be mechanical");
    if(abs(v.tachometer)>3)return fail("tachometer must not expose 1024 raw ABI counts");

    c.foc_encoder_inverted=true;
    mcpwm_foc_set_configuration(&c,false);
    if(m_motor_1.m_encoder_synced)return fail("calibration change must clear sync");
    encoder_set_deg(90.0f); m_motor_1.m_encoder_synced=1u;
    mcpwm_foc_adc_int_handler();
    if(!nearf(mcpwm_foc_get_phase_encoder_motor(false),80.0f,0.25f))
        return fail("electrical inversion semantics");

    c.m_encoder_counts=2048;
    mcpwm_foc_set_configuration(&c,false);
    if(encoder_index_found())return fail("CPR change must invalidate ABI sync");
    if(m_motor_1.m_encoder_synced)return fail("CPR change FOC sync");
    c=abi_conf(); c.foc_encoder_offset=0.0f;
    mcpwm_foc_set_configuration(&c,false);
    encoder_set_deg(0.0f); m_motor_1.m_encoder_synced=1u;
    enable=1u;

    m_motor_1.m_position_counts=0;
    mcpwm_foc_set_position_counts(1,false);
    mcpwm_foc_vesc_override_touch(false);
    for(int i=0;i<6;i++)mcpwm_foc_adc_int_handler();
    if(abs(m_motor_1.m_iq_target_q4)>8)
        return fail("one ABI count incorrectly scaled as Hall sector");
    mcpwm_foc_release_motor(false);
    m_motor_1.m_tachometer=0; m_motor_1.m_tachometer_abs=0u;
    m_motor_1.m_tacho_step_last=3u;
    encoder_set_deg(0.0f); m_motor_1.m_encoder_synced=1u;
    mcpwm_foc_adc_int_handler();
    /* Jalankan >2 window agar estimator 50-Hz sudah steady-state meskipun
     * subtest sebelumnya meninggalkan partial window. */
    for(int i=0;i<700;i++){
        encoder_cfg_ABI.timer->CNT=(encoder_cfg_ABI.timer->CNT+1u)%4096u;
        mcpwm_foc_adc_int_handler();
    }
    const float erpm=mcpwm_foc_get_erpm_motor(false);
    if(!nearf(erpm,3515.625f,12.0f))return fail("ABI ERPM scaling");
    if(m_motor_1.m_tachometer<14 || m_motor_1.m_tachometer>16)
        return fail("VESC tachometer must be 6 counts/electrical revolution");
    mcpwm_foc_get_values(&v,false);
    if(!nearf(v.position,61.5234f,0.20f))return fail("RT Data mechanical encoder position");
    if(v.tachometer!=m_motor_1.m_tachometer)return fail("RT Data VESC tachometer source");

    m_motor_1.m_encoder_mech_rpm_q16=50*65536;
    m_motor_1.m_encoder_idle_ticks=0u; m_motor_1.m_encoder_synced=1u;
    mcpwm_foc_set_brake_current(1.0f,false);
    if(m_motor_1.m_brake_direction!=1)return fail("ABI brake direction source");

    printf("ENCODER_ABI_RUNTIME_PASS mech=%.3f elec=%.3f erpm=%.1f tacho=%ld poscnt=%ld\n",
           mcpwm_foc_get_encoder_position_motor(false),
           mcpwm_foc_get_phase_encoder_motor(false),erpm,
           (long)v.tachometer,(long)m_motor_1.m_position_counts);
    return 0;
}
