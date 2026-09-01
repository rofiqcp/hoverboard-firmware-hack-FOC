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


    /* Mode 1: direct-voltage command remains permille. 100 -> 10% of the
     * proven voltage-vector ceiling; Id is not regulated in this mode. */
    ctrlModReq=VLT_MODE; pwml=100; pwmr=-100;
    mcpwm_foc_adc_int_handler();
    if(m_motor_1.m_vq!=1440)return fail("mode1 100 permille voltage scaling");
    if(m_motor_1.m_vd!=0)return fail("mode1 Vd must be zero");
    if(m_motor_2.m_vq!=-1440)return fail("mode1 right internal mirror sign");

    mcpwm_foc_init(); enable=1u; motorRunReq=1u; set_halls(3u,3u);

    /* Mode 2: legacy target remains mechanical RPM. The proven generated
     * architecture is speed PI -> Vq directly (not speed PI -> Iq PI). */
    ctrlModReq=SPD_MODE; pwml=50; pwmr=-50;
    mcpwm_foc_adc_int_handler();
    if(m_motor_1.m_iq_set_q4!=0)return fail("speed mode must not create Iq reference");
    if(m_motor_1.m_vq!=946)return fail("speed PI first Vq does not match generated controller");
    const int16_t vq0=m_motor_1.m_vq;
    mcpwm_foc_adc_int_handler();
    mcpwm_foc_adc_int_handler();
    if(m_motor_1.m_vq!=vq0)return fail("speed Vq must hold between 1-of-3 control slots");
    mcpwm_foc_adc_int_handler();
    if(m_motor_1.m_vq==vq0)return fail("speed PI must update on next control slot");

    /* VESC API is standardized to ERPM. 750 ERPM / 15 pole pairs = 50 mech RPM. */
    mcpwm_foc_init(); set_halls(3u,3u);
    mcpwm_foc_set_pid_speed(750.0f,false);
    if(m_motor_1.m_speed_set_rpm!=50)return fail("VESC ERPM to mechanical RPM conversion");
    m_motor_1.m_rpm=50;
    mc_values vals;
    mcpwm_foc_get_values(&vals,false);
    if(fabsf(vals.rpm-750.0f)>0.1f)return fail("mc_values.rpm must be ERPM");

    /* Full standardized torque scaling: 1500 cA = 15.00 A and clamps there. */
    mcpwm_foc_init(); set_halls(3u,3u); enable=1u; motorRunReq=1u;
    ctrlModReq=TRQ_MODE; pwml=1500; pwmr=-1500;
    mcpwm_foc_adc_int_handler();
    if(m_motor_1.m_iq_target_q4!=12000)return fail("TRQ 1500cA = 15A scaling");
    if(m_motor_2.m_iq_target_q4!=-12000)return fail("TRQ right 15A internal mirror scaling");

    /* Mode 3: 50 cA = 0.50 A target. Active Iq is slewed at 10 A/s, then the
     * current PI must produce Vq. */
    mcpwm_foc_init(); set_halls(3u,3u); enable=1u; motorRunReq=1u;
    ctrlModReq=TRQ_MODE; pwml=50; pwmr=-50;
    mcpwm_foc_adc_int_handler();
    if(m_motor_1.m_iq_target_q4!=400)return fail("TRQ 50cA target scaling");
    if(m_motor_1.m_iq_set_q4>=400)return fail("TRQ target must slew, not step");
    for(int i=0;i<1000;i++)mcpwm_foc_adc_int_handler();
    if(m_motor_1.m_iq_set_q4!=400)return fail("TRQ slew must reach 0.50A");
    if(m_motor_1.m_id_set_q4!=0)return fail("TRQ Id must remain zero");
    if(m_motor_1.m_vq<=0)return fail("TRQ PI must generate positive Vq");
    if(abs(m_motor_1.m_vq)>MCCONF_FOC_CLOSED_LOOP_VOLTAGE_MAX)return fail("TRQ closed-loop voltage ceiling");

    /* Mode 4 remains sensorless Id control: 2 means 2 A Id, Iq target zero. */
    ctrlModReq=SVPWM_MODE; pwml=2; pwmr=-2;
    mcpwm_foc_adc_int_handler();
    if(m_motor_1.m_id_set_q4>=1600)return fail("mode4 Id must slew, not step");
    for(uint32_t i=0u;i<((uint32_t)PWM_FREQ*SVPWM_ALIGN_MS/1000u)+32u;i++)mcpwm_foc_adc_int_handler();
    if(m_motor_1.m_openloop_id_target_q4!=1600)return fail("mode4 2A target");
    if(m_motor_1.m_id_set_q4!=1600)return fail("mode4 Id ramp reaches 2A");
    if(m_motor_1.m_iq_set_q4!=0)return fail("mode4 Iq target zero");
    if(m_motor_1.m_phase!=m_motor_1.m_phase_openloop)return fail("mode4 synthetic phase offset");
    if(abs(m_motor_1.m_vq)>MCCONF_FOC_CLOSED_LOOP_VOLTAGE_MAX ||
       abs(m_motor_1.m_vd)>MCCONF_FOC_CLOSED_LOOP_VOLTAGE_MAX)return fail("mode4 closed-loop voltage ceiling");

    /* Mode 4 safety semantics: current command is whole ampere Id and is
     * clamped to 6 A while Iq remains zero. */
    ctrlModReq=SVPWM_MODE; pwml=10; pwmr=-10;
    mcpwm_foc_adc_int_handler();
    if(m_motor_1.m_openloop_id_target_q4!=SVPWM_MAX_ID_A*FOC_CURRENT_Q4_PER_A)return fail("mode4 Id safety clamp");
    if(m_motor_1.m_iq_target_q4!=0)return fail("mode4 requested Iq must stay zero");

    /* TRQ stop: while rotating it commands opposing brake current, then
     * returns to neutral duty when Hall RPM reaches the deadband. */
    m_motor_1.m_rpm=100;
    mcpwm_foc_set_mode_command(TRQ_MODE,0,false,SVPWM_OPENLOOP_RPM_DEFAULT,false);
    if(m_motor_1.m_control_mode!=CONTROL_MODE_CURRENT_BRAKE || m_motor_1.m_iq_target_q4>=0)return fail("TRQ stop brake polarity");
    m_motor_1.m_rpm=0;
    mcpwm_foc_set_mode_command(TRQ_MODE,0,false,SVPWM_OPENLOOP_RPM_DEFAULT,false);
    if(m_motor_1.m_control_mode!=CONTROL_MODE_DUTY || m_motor_1.m_duty_set_permille!=0)return fail("TRQ stop neutral state");

    /* Reverse Hall convention remains the proven generated pos+1 rule. */
    mcpwm_foc_init(); enable=1u; ctrlModReq=VLT_MODE; pwml=0;pwmr=0;set_halls(3u,3u);
    for(int i=0;i<100;i++)mcpwm_foc_adc_int_handler();
    set_halls(2u,2u);
    mcpwm_foc_adc_int_handler();
    if(m_motor_2.m_hall_direction!=-1)return fail("right reverse Hall direction");
    for(int i=0;i<2100;i++)mcpwm_foc_adc_int_handler();
    if(m_motor_2.m_hall_interp_active!=0u)return fail("Hall interpolation low-speed disable");
    const int32_t err=(int32_t)(uint16_t)(m_motor_2.m_phase_hall-16384u);
    if(abs(err)>4 && abs(err-65536)>4)return fail("reverse Hall phase convention");

    printf("MOTOR_CONTROL_V10_PASS speed_vq=%d trq_target_q4=%d trq_active_q4=%d erpm=%.0f mode4_id_q4=%d\n",
           vq0,400,m_motor_1.m_iq_set_q4,vals.rpm,1600);
    return 0;
}
