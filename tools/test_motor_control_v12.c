#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "stm32f1xx_hal.h"
#include "config.h"
#include "defines.h"
#include "motor/mcpwm_foc.h"
#include "motor/mcconf_default.h"
#include "motor/foc_math.h"

GPIO_TypeDef _GPIOA={0},_GPIOB={0},_GPIOC={0};
TIM_TypeDef _TIM1={0},_TIM8={0};
DMA_TypeDef _DMA1={0};
DWT_Type _DWT={0};
CoreDebug_Type _CoreDebug={0};
volatile adc_buf_t adc_buffer={0};
uint8_t ctrlModReq=VLT_MODE;
extern uint8_t enable;
extern volatile uint8_t motorRunReq;
extern volatile int pwml;
extern volatile int pwmr;

void DMA1_Channel1_IRQHandler(void);

void filtLowPass32(int16_t u, uint16_t coef, int32_t *y) {
    int32_t err=(int32_t)u-(*y>>16);
    if(err>32767)err=32767; else if(err<-32768)err=-32768;
    *y+=(int32_t)coef*err;
}

static int fail(const char *s){fprintf(stderr,"FAIL %s\n",s);return 1;}
static void set_hall(GPIO_TypeDef *port,uint16_t pu,uint16_t pv,uint16_t pw,uint8_t h){
    port->IDR|=(uint32_t)(pu|pv|pw);
    if(h&4u)port->IDR&=~(uint32_t)pu;
    if(h&2u)port->IDR&=~(uint32_t)pv;
    if(h&1u)port->IDR&=~(uint32_t)pw;
}
static void set_halls(uint8_t l,uint8_t r){
    set_hall(GPIOB,LEFT_HALL_U_PIN,LEFT_HALL_V_PIN,LEFT_HALL_W_PIN,l);
    set_hall(GPIOC,RIGHT_HALL_U_PIN,RIGHT_HALL_V_PIN,RIGHT_HALL_W_PIN,r);
}

int main(void){
    mcpwm_foc_init(); enable=1u; motorRunReq=1u; set_halls(3u,3u);

    /* MODE 1: 100 = 10% direct voltage while running. Explicit stop releases
     * the motor instead of holding a zero-voltage vector. */
    ctrlModReq=VLT_MODE; pwml=100; pwmr=-100;
    mcpwm_foc_adc_int_handler();
    if(m_motor_1.m_vq!=1440)return fail("mode1 100 permille voltage scaling");
    if(m_motor_1.m_vd!=0)return fail("mode1 Vd must be zero");
    if(m_motor_2.m_vq!=-1440)return fail("mode1 right internal mirror sign");
    mcpwm_foc_set_mode_command(VLT_MODE,0,false,SVPWM_OPENLOOP_RPM_DEFAULT,false);
    if(m_motor_1.m_control_mode!=CONTROL_MODE_NONE)return fail("mode1 STOP must release/free-run");

    /* Verify free-run is electrical high impedance, not merely Vq=0: after the
     * control mode is released the corresponding timer MOE must turn off. */
    mcpwm_foc_init(); enable=1u; motorRunReq=1u; ctrlModReq=VLT_MODE; pwml=100; pwmr=-100;
    adc_buffer.rlA=adc_buffer.rlB=adc_buffer.rrB=adc_buffer.rrC=2000;
    adc_buffer.dcl=adc_buffer.dcr=2000; adc_buffer.batt1=2000;
    for(int i=0;i<2004;i++)DMA1_Channel1_IRQHandler();
    if((LEFT_TIM->BDTR & TIM_BDTR_MOE)==0u || (RIGHT_TIM->BDTR & TIM_BDTR_MOE)==0u)
        return fail("mode1 RUN must enable both bridges");
    pwml=0; pwmr=0; motorRunReq=0u;
    DMA1_Channel1_IRQHandler();
    DMA1_Channel1_IRQHandler();
    if((LEFT_TIM->BDTR & TIM_BDTR_MOE)!=0u || (RIGHT_TIM->BDTR & TIM_BDTR_MOE)!=0u)
        return fail("free-run release must disable MOE/high impedance");

    /* MODE 2: legacy command is mechanical RPM. Active speed setpoint must ramp
     * at 100 mech RPM/s (=1500 ERPM/s at 15 pole pairs), not step. */
    mcpwm_foc_init(); enable=1u; motorRunReq=1u; set_halls(3u,3u);
    ctrlModReq=SPD_MODE; pwml=50; pwmr=-50;
    mcpwm_foc_adc_int_handler();
    if(m_motor_1.m_speed_target_rpm!=50)return fail("mode2 target must be 50 mechanical RPM");
    if(m_motor_1.m_speed_set_rpm!=0)return fail("mode2 active speed must start ramped from measured speed");
    if(m_motor_1.m_iq_set_q4!=0)return fail("mode2 must not create Iq reference");
    for(int i=0;i<8500;i++)mcpwm_foc_adc_int_handler();
    if(m_motor_1.m_speed_set_rpm!=50)return fail("mode2 speed ramp must reach 50 RPM");
    if(m_motor_1.m_control_mode!=CONTROL_MODE_SPEED)return fail("mode2 must remain speed control while running");

    /* Other speed commands use the same physical mechanical-RPM scale and the
     * same ramp. Check 100 RPM and a direction reversal to -50 RPM. */
    pwml=100; pwmr=-100;
    for(int i=0;i<8500;i++)mcpwm_foc_adc_int_handler();
    if(m_motor_1.m_speed_target_rpm!=100 || m_motor_1.m_speed_set_rpm!=100)
        return fail("mode2 100 RPM scaling/ramp");
    pwml=-50; pwmr=50;
    for(int i=0;i<25000;i++)mcpwm_foc_adc_int_handler();
    if(m_motor_1.m_speed_target_rpm!=-50 || m_motor_1.m_speed_set_rpm!=-50)
        return fail("mode2 reverse -50 RPM ramp");
    if(m_motor_1.m_control_mode!=CONTROL_MODE_SPEED)return fail("mode2 reverse must stay speed mode");

    /* STOP: target becomes zero, active setpoint ramps down gradually. It must
     * NOT release on the first zero command. Stop Vq is gently limited. */
    pwml=0; pwmr=0;
    mcpwm_foc_adc_int_handler();
    if(m_motor_1.m_speed_target_rpm!=0)return fail("mode2 STOP target zero");
    if(m_motor_1.m_control_mode!=CONTROL_MODE_SPEED)return fail("mode2 STOP must ramp before release");
    if(abs(m_motor_1.m_speed_set_rpm)<=5)return fail("mode2 STOP ramp must not jump to zero");
    for(int i=0;i<3500;i++)mcpwm_foc_adc_int_handler();
    if(m_motor_1.m_control_mode!=CONTROL_MODE_SPEED)return fail("mode2 STOP released too early");
    if(abs(m_motor_1.m_vq)>MCCONF_SPEED_STOP_VOLTAGE_MAX)return fail("mode2 STOP Vq ceiling");
    for(int i=0;i<5000;i++)mcpwm_foc_adc_int_handler();
    if(m_motor_1.m_control_mode!=CONTROL_MODE_NONE)return fail("mode2 STOP must release near zero");
    if(m_motor_1.m_speed_integrator!=0 || m_motor_1.m_iq_integrator!=0 || m_motor_1.m_id_integrator!=0)
        return fail("mode2 release must reset all PI integrators");
    if(m_motor_1.m_speed_set_rpm!=0 || m_motor_1.m_speed_target_rpm!=0)
        return fail("mode2 release must zero speed states");

    /* VESC boundary: 750 ERPM = 50 mechanical RPM. VESC SET_RPM 0 while speed
     * is active must request a ramp, not immediate active braking/reversal. */
    mcpwm_foc_init(); enable=1u; set_halls(3u,3u);
    mcpwm_foc_set_pid_speed(750.0f,false);
    mcpwm_foc_vesc_override_touch(false);
    if(m_motor_1.m_speed_target_rpm!=50)return fail("VESC ERPM target conversion");
    if(m_motor_1.m_control_mode!=CONTROL_MODE_SPEED)return fail("VESC speed mode entry");
    for(int i=0;i<8500;i++){ if((i%1000)==0)mcpwm_foc_vesc_override_touch(false); mcpwm_foc_adc_int_handler(); }
    if(m_motor_1.m_speed_set_rpm!=50)return fail("VESC speed ramp reach target");
    mcpwm_foc_set_pid_speed(0.0f,false);
    mcpwm_foc_vesc_override_touch(false);
    if(m_motor_1.m_control_mode!=CONTROL_MODE_SPEED)return fail("VESC zero ERPM must ramp before release");
    if(m_motor_1.m_speed_target_rpm!=0)return fail("VESC zero ERPM target");
    for(int i=0;i<8500;i++){ if((i%1000)==0)mcpwm_foc_vesc_override_touch(false); mcpwm_foc_adc_int_handler(); }
    if(m_motor_1.m_control_mode!=CONTROL_MODE_NONE)return fail("VESC zero ERPM release");
    mcpwm_foc_set_pid_speed(750.0f,false);
    mcpwm_foc_vesc_override_touch(false);
    m_motor_1.m_rpm=50;
    mc_values vals;
    mcpwm_foc_get_values(&vals,false);
    if(fabsf(vals.rpm-750.0f)>0.1f)return fail("mc_values.rpm must be ERPM");

    /* MODE 3 scaling remains exact: 50 cA = 0.50 A, 1500 cA = 15 A. */
    mcpwm_foc_init(); set_halls(3u,3u); enable=1u; motorRunReq=1u;
    ctrlModReq=TRQ_MODE; pwml=1500; pwmr=-1500;
    mcpwm_foc_adc_int_handler();
    if(m_motor_1.m_iq_target_q4!=12000)return fail("TRQ 1500cA = 15A scaling");
    if(m_motor_2.m_iq_target_q4!=-12000)return fail("TRQ right 15A internal mirror scaling");

    mcpwm_foc_init(); set_halls(3u,3u); enable=1u; motorRunReq=1u;
    ctrlModReq=TRQ_MODE; pwml=50; pwmr=-50;
    for(int i=0;i<1000;i++)mcpwm_foc_adc_int_handler();
    if(m_motor_1.m_iq_target_q4!=400)return fail("TRQ 50cA target scaling");
    if(m_motor_1.m_iq_set_q4!=400)return fail("TRQ slew must reach 0.50A");
    if(m_motor_1.m_id_set_q4!=0)return fail("TRQ Id must remain zero");
    if(m_motor_1.m_vq<=0)return fail("TRQ PI must generate positive Vq");

    /* MODE 3 STOP: no current-brake state. Ramp Iq reference to zero and then
     * release/high-impedance so the wheel keeps free-running. */
    motorRunReq=0u; pwml=0; pwmr=0;
    mcpwm_foc_adc_int_handler();
    if(m_motor_1.m_control_mode==CONTROL_MODE_CURRENT_BRAKE)return fail("TRQ STOP must not brake");
    if(m_motor_1.m_iq_target_q4!=0)return fail("TRQ STOP target must be zero");
    if(m_motor_1.m_control_mode!=CONTROL_MODE_CURRENT)return fail("TRQ STOP must slew current before release");
    for(int i=0;i<1200;i++)mcpwm_foc_adc_int_handler();
    if(m_motor_1.m_control_mode!=CONTROL_MODE_NONE)return fail("TRQ STOP must release after Iq ramp");
    if(m_motor_1.m_iq_set_q4!=0 || m_motor_1.m_iq_target_q4!=0)return fail("TRQ release current state");
    for(int i=0;i<20;i++)mcpwm_foc_adc_int_handler();
    if(m_motor_1.m_control_mode!=CONTROL_MODE_NONE)return fail("TRQ STOP must stay released");

    /* MODE 4 unchanged: sensorless Id current, Iq=0, 2 -> 2 A and 6 A safety clamp. */
    mcpwm_foc_init(); set_halls(3u,3u); enable=1u; motorRunReq=1u;
    ctrlModReq=SVPWM_MODE; pwml=2; pwmr=-2;
    mcpwm_foc_adc_int_handler();
    if(m_motor_1.m_id_set_q4>=1600)return fail("mode4 Id must slew, not step");
    for(uint32_t i=0u;i<((uint32_t)PWM_FREQ*SVPWM_ALIGN_MS/1000u)+32u;i++)mcpwm_foc_adc_int_handler();
    if(m_motor_1.m_openloop_id_target_q4!=1600)return fail("mode4 2A target");
    if(m_motor_1.m_id_set_q4!=1600)return fail("mode4 Id ramp reaches 2A");
    if(m_motor_1.m_iq_set_q4!=0)return fail("mode4 Iq target zero");
    if(m_motor_1.m_phase!=m_motor_1.m_phase_openloop)return fail("mode4 synthetic phase offset");
    ctrlModReq=SVPWM_MODE; pwml=10; pwmr=-10;
    mcpwm_foc_adc_int_handler();
    if(m_motor_1.m_openloop_id_target_q4!=SVPWM_MAX_ID_A*FOC_CURRENT_Q4_PER_A)return fail("mode4 Id safety clamp");

    /* Reverse Hall convention remains the generated-controller pos+1 rule. */
    mcpwm_foc_init(); enable=1u; ctrlModReq=VLT_MODE; pwml=1;pwmr=-1;set_halls(3u,3u);
    for(int i=0;i<100;i++)mcpwm_foc_adc_int_handler();
    set_halls(2u,2u);
    for(uint8_t i=0u;i<MCCONF_HALL_DEBOUNCE_SAMPLES;i++)mcpwm_foc_adc_int_handler();
    if(m_motor_2.m_hall_direction!=-1)return fail("right reverse Hall direction");
    for(uint32_t i=0u;i<MCCONF_HALL_TIMEOUT_TICKS+100u;i++)mcpwm_foc_adc_int_handler();
    if(m_motor_2.m_hall_interp_active!=0u)return fail("Hall interpolation low-speed disable");

    printf("MOTOR_CONTROL_V12_PASS speed_ramp=%uRPM/s release=%uRPM trq50=0.50A erpm=%.0f mode4=2A\n",
           m_motor_1.m_speed_ramp_rpm_s,m_motor_1.m_speed_release_rpm,vals.rpm);
    return 0;
}
