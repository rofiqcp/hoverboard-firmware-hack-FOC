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

static void host_set_deg_synced(float deg){
    encoder_set_deg(deg);
    mcpwm_foc_refresh_encoder_configuration(false,false);
    m_motor_1.m_encoder_synced=1u;
}

int main(void){
    mcpwm_foc_init();
    mc_configuration c=abi_conf();
    mcpwm_foc_set_configuration(&c,false);
    if(encoder_is_configured()!=ENCODER_TYPE_ABI)return fail("ABI not configured");
    if(encoder_index_found())return fail("ABI must start unsynced without I");
    if(m_motor_1.m_encoder_synced)return fail("FOC sync must start false");

    host_set_deg_synced(90.0f);
    if(!encoder_index_found())return fail("software electrical sync not latched");
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
    host_set_deg_synced(90.0f);
    mcpwm_foc_adc_int_handler();
    if(!nearf(mcpwm_foc_get_phase_encoder_motor(false),80.0f,0.25f))
        return fail("electrical inversion semantics");
    /* Corrected VESC phase runs opposite raw ABI when inverted=true, therefore
     * ERPM/brake direction must invert too, not only the angle. */
    host_set_deg_synced(0.0f);
    m_motor_1.m_encoder_speed_ticks=0u; m_motor_1.m_encoder_delta_accum=0;
    m_motor_1.m_encoder_prev_count=encoder_cfg_ABI.timer->CNT;
    for(int i=0;i<400;i++){
        encoder_cfg_ABI.timer->CNT=(encoder_cfg_ABI.timer->CNT+1u)%4096u;
        mcpwm_foc_adc_int_handler();
    }
    if(mcpwm_foc_get_erpm_motor(false)>-3500.0f)return fail("inverted ABI ERPM sign");

    c.m_encoder_counts=2048;
    mcpwm_foc_set_configuration(&c,false);
    if(encoder_index_found())return fail("CPR change must invalidate ABI sync");
    if(m_motor_1.m_encoder_synced)return fail("CPR change FOC sync");
    c=abi_conf(); c.foc_encoder_offset=0.0f;
    mcpwm_foc_set_configuration(&c,false);
    /* Unsynced ABI may be readable for diagnostics, but must not run PI/state
     * or replace the active FOC phase before electrical alignment. */
    const uint16_t phase_before=m_motor_1.m_phase;
    mcpwm_foc_set_current(1.0f,false); mcpwm_foc_vesc_override_touch(false); enable=1u;
    encoder_cfg_ABI.timer->CNT=777u; mcpwm_foc_adc_int_handler();
    if(m_motor_1.m_state!=MC_STATE_OFF)return fail("unsynced ABI control state must stay OFF");
    if(m_motor_1.m_phase!=phase_before)return fail("unsynced ABI must not become active FOC phase");
    mcpwm_foc_release_motor(false);

    host_set_deg_synced(0.0f);

    /* VESC p_pid_ang_div=2: one physical encoder revolution advances PID
     * position only 180 deg while preserving continuity through 360->0. */
    c.p_pid_ang_div=2.0f; mcpwm_foc_set_configuration(&c,false);
    host_set_deg_synced(350.0f); mcpwm_foc_adc_int_handler();
    if(!nearf(mcpwm_foc_get_pid_pos_now_motor(false),355.0f,0.35f))return fail("p_pid_ang_div reverse wrap");
    host_set_deg_synced(10.0f); mcpwm_foc_adc_int_handler();
    if(!nearf(mcpwm_foc_get_pid_pos_now_motor(false),5.0f,0.35f))return fail("p_pid_ang_div forward wrap");
    c.p_pid_ang_div=1.0f; mcpwm_foc_set_configuration(&c,false);
    host_set_deg_synced(0.0f);

    m_motor_1.m_position_counts=0;
    mcpwm_foc_set_position_counts(1,false);
    mcpwm_foc_vesc_override_touch(false);
    for(int i=0;i<6;i++)mcpwm_foc_adc_int_handler();
    if(abs(m_motor_1.m_iq_target_q4)>8)
        return fail("one ABI count incorrectly scaled as Hall sector");
    mcpwm_foc_release_motor(false);

    /* VESC p_pid_gain_dec_angle scales all position gains proportionally near
     * target. With Kp=.02 and 20 deg error, gain_dec=40 deg must halve Iq. */
    c=abi_conf(); c.foc_encoder_offset=0.0f; c.p_pid_kp=0.020f; c.p_pid_ki=0.0f;
    c.p_pid_kd=0.0f; c.p_pid_kd_proc=0.0f; c.p_pid_gain_dec_angle=0.0f;
    mcpwm_foc_set_configuration(&c,false); host_set_deg_synced(0.0f);
    mcpwm_foc_set_pid_pos(20.0f,false); mcpwm_foc_vesc_override_touch(false);
    for(int i=0;i<6;i++)mcpwm_foc_adc_int_handler();
    const int iq_full=abs(m_motor_1.m_iq_target_q4);
    mcpwm_foc_release_motor(false);
    c.p_pid_gain_dec_angle=40.0f; mcpwm_foc_set_configuration(&c,false); host_set_deg_synced(0.0f);
    mcpwm_foc_set_pid_pos(20.0f,false); mcpwm_foc_vesc_override_touch(false);
    for(int i=0;i<6;i++)mcpwm_foc_adc_int_handler();
    const int iq_half=abs(m_motor_1.m_iq_target_q4);
    if(iq_full<100 || iq_half<40 || iq_half>iq_full*3/5 || iq_half<iq_full*2/5)
        return fail("p_pid_gain_dec_angle VESC proportional scaling");
    mcpwm_foc_release_motor(false);
    c=abi_conf(); c.foc_encoder_offset=0.0f; mcpwm_foc_set_configuration(&c,false); host_set_deg_synced(0.0f);

    m_motor_1.m_tachometer=0; m_motor_1.m_tachometer_abs=0u;
    m_motor_1.m_tacho_step_last=3u;
    host_set_deg_synced(0.0f);
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
