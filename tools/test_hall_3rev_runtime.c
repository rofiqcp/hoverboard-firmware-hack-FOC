#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f1xx_hal.h"
#include "config.h"
#include "defines.h"
#include "motor/mcpwm_foc.h"
#include "motor/mcconf_default.h"

GPIO_TypeDef _GPIOA={0},_GPIOB={0},_GPIOC={0};
TIM_TypeDef _TIM1={0},_TIM8={0};
DMA_TypeDef _DMA1={0};
DWT_Type _DWT={0};
CoreDebug_Type _CoreDebug={0};
volatile adc_buf_t adc_buffer={0};
uint8_t ctrlModReq=VLT_MODE;
extern uint8_t enable;
extern volatile uint8_t motorRunReq;
extern volatile int pwml,pwmr;
void filtLowPass32(int16_t u,uint16_t coef,int32_t*y){int32_t e=(int32_t)u-(*y>>16);if(e>32767)e=32767;if(e<-32768)e=-32768;*y+=(int32_t)coef*e;}
void HAL_Delay(uint32_t ms){(void)ms;}

static void set_hall(GPIO_TypeDef *p,uint16_t pu,uint16_t pv,uint16_t pw,uint8_t h){
    p->IDR|=(uint32_t)(pu|pv|pw);
    if(h&4u)p->IDR&=~(uint32_t)pu;
    if(h&2u)p->IDR&=~(uint32_t)pv;
    if(h&1u)p->IDR&=~(uint32_t)pw;
}
static void set_halls(uint8_t l,uint8_t r){
    set_hall(GPIOB,LEFT_HALL_U_PIN,LEFT_HALL_V_PIN,LEFT_HALL_W_PIN,l);
    set_hall(GPIOC,RIGHT_HALL_U_PIN,RIGHT_HALL_V_PIN,RIGHT_HALL_W_PIN,r);
}
static int phase_diff(uint16_t a,uint16_t b){int32_t d=(int16_t)(a-b); if(d<0)d=-d; return (int)d;}
static uint16_t phase_from_table(uint8_t a){return (uint16_t)(((uint32_t)a*65536u)/200u);}

static int run_motor(int second,const uint8_t table[8]){
    const uint8_t seq[6]={2,3,1,5,4,6};
    mc_configuration c = second ? m_motor_2.m_conf : m_motor_1.m_conf;
    memcpy(c.foc_hall_table,table,8);
    mcpwm_foc_set_configuration(&c,second!=0);
    enable=1u; motorRunReq=1u; ctrlModReq=VLT_MODE; pwml=1; pwmr=-1;
    uint8_t li=2,ri=2;
    if(second)ri=seq[0]; else li=seq[0];
    set_halls(li,ri);
    for(int i=0;i<20;i++)mcpwm_foc_adc_int_handler();
    mcpwm_foc_motor_t *m=second?&m_motor_2:&m_motor_1;
    if(m->m_hall_state!=seq[0] || !m->m_hall_initialized){printf("FAIL init motor%d state=%u\n",second+1,m->m_hall_state);return 1;}
    const int edges=270; /* 3 mechanical revolutions @15 pole pairs: 3*15*6 */
    const int hold=213;  /* ~750 ERPM: 16000*10/750 = 213.3 ISR ticks/edge */
    int idx=0; uint16_t prev_phase=m->m_phase; int max_step=0,max_target_center_err=0,max_phase_target_err=0;
    for(int e=0;e<edges;e++){
        idx=(idx+1)%6; uint8_t h=seq[idx];
        for(int t=0;t<hold;t++){
            uint8_t raw=h;
            if(e==90 && t==80) raw=7u; /* deliberate one-sample glitch */
            if(second)ri=raw; else li=raw;
            set_halls(li,ri); mcpwm_foc_adc_int_handler();
            int st=phase_diff(m->m_phase,prev_phase); if(st>max_step)max_step=st; prev_phase=m->m_phase;
            if(m->m_hall_state>=1 && m->m_hall_state<=6){
                uint16_t cen=phase_from_table(table[m->m_hall_state]); int te=phase_diff(m->m_phase_hall_target,cen); int pe=phase_diff(m->m_phase,m->m_phase_hall_target); if(te>max_target_center_err)max_target_center_err=te; if(pe>max_phase_target_err)max_phase_target_err=pe;
                if(te>8000 || pe>16000){printf("FAIL mapping motor%d e=%d t=%d state=%u target-center=%d phase-target=%d\n",second+1,e,t,m->m_hall_state,te,pe);return 2;}
            }
        }
    }
    if(m->m_position_counts!=edges){printf("FAIL forward count motor%d got=%ld\n",second+1,(long)m->m_position_counts);return 3;}
    if(m->m_hall_sequence_reject_count||m->m_hall_period_reject_count){printf("FAIL forward reject motor%d seq=%lu period=%lu\n",second+1,(unsigned long)m->m_hall_sequence_reject_count,(unsigned long)m->m_hall_period_reject_count);return 4;}
    for(int e=0;e<edges;e++){
        idx=(idx+5)%6; uint8_t h=seq[idx];
        for(int t=0;t<hold;t++){
            uint8_t raw=h;
            if(e==120 && t==100) raw=0u; /* deliberate one-sample glitch */
            if(second)ri=raw; else li=raw;
            set_halls(li,ri); mcpwm_foc_adc_int_handler();
            int st=phase_diff(m->m_phase,prev_phase); if(st>max_step)max_step=st; prev_phase=m->m_phase;
            if(m->m_hall_state>=1 && m->m_hall_state<=6){
                uint16_t cen=phase_from_table(table[m->m_hall_state]); int te=phase_diff(m->m_phase_hall_target,cen); int pe=phase_diff(m->m_phase,m->m_phase_hall_target); if(te>max_target_center_err)max_target_center_err=te; if(pe>max_phase_target_err)max_phase_target_err=pe;
                if(te>8000 || pe>16000){printf("FAIL reverse mapping motor%d e=%d t=%d state=%u target-center=%d phase-target=%d\n",second+1,e,t,m->m_hall_state,te,pe);return 5;}
            }
        }
    }
    if(m->m_position_counts!=0){printf("FAIL reverse count motor%d got=%ld\n",second+1,(long)m->m_position_counts);return 6;}
    if(m->m_hall_sequence_reject_count||m->m_hall_period_reject_count){printf("FAIL reverse reject motor%d seq=%lu period=%lu\n",second+1,(unsigned long)m->m_hall_sequence_reject_count,(unsigned long)m->m_hall_period_reject_count);return 7;}
    printf("MOTOR%d_3MECHREV_PASS edges_fwd=%d edges_rev=%d seq_reject=%lu period_reject=%lu max_phase_step=%.3fdeg max_target_center_error=%.3fdeg max_phase_target_error=%.3fdeg final_dir=%d\n",
        second+1,edges,edges,(unsigned long)m->m_hall_sequence_reject_count,(unsigned long)m->m_hall_period_reject_count,
        max_step*360.0/65536.0,max_target_center_err*360.0/65536.0,max_phase_target_err*360.0/65536.0,m->m_hall_direction);
    return 0;
}
int main(void){
    const uint8_t L[8]={255,107,32,72,161,134,195,255};
    const uint8_t R[8]={255,101,28,61,174,143,199,255};
    mcpwm_foc_init();
    int a=run_motor(0,L);
    mcpwm_foc_init();
    int b=run_motor(1,R);
    if(a||b)return 1;
    puts("HALL_3_MECHANICAL_REVOLUTIONS_BOTH_DIRECTIONS_PASS");
    return 0;
}
