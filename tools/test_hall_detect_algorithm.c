#include <stdint.h>
#include <stdbool.h>
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

void filtLowPass32(int16_t u, uint16_t coef, int32_t *y){
    int32_t err=(int32_t)u-(*y>>16);
    if(err>32767)err=32767; else if(err<-32768)err=-32768;
    *y+=(int32_t)coef*err;
}

static void set_hall(GPIO_TypeDef *port,uint16_t pu,uint16_t pv,uint16_t pw,uint8_t h){
    port->IDR|=(uint32_t)(pu|pv|pw);
    if(h&4u)port->IDR&=~(uint32_t)pu;
    if(h&2u)port->IDR&=~(uint32_t)pv;
    if(h&1u)port->IDR&=~(uint32_t)pw;
}
static void hold_left_hall(uint8_t h,uint16_t ticks){
    set_hall(GPIOB,LEFT_HALL_U_PIN,LEFT_HALL_V_PIN,LEFT_HALL_W_PIN,h);
    for(uint16_t i=0u;i<ticks;++i)mcpwm_foc_adc_int_handler();
}

/* Deliberately non-default Hall/phase permutations. Index is 60-degree
 * electrical sector in applied board phase coordinates. */
static const uint8_t left_raw_for_sector[6]  ={5u,1u,3u,2u,6u,4u};
static const uint8_t right_raw_for_sector[6] ={2u,6u,4u,5u,1u,3u};

static uint8_t raw_for_phase(uint16_t phase,const uint8_t map[6]){
    /* sector center 0,60,...; boundaries +/-30 deg */
    uint32_t a200=((uint32_t)phase*200u+32768u)/65536u;
    if(a200>=200u)a200-=200u;
    uint32_t sector=((a200+17u)/33u)%6u;
    return map[sector];
}

void HAL_Delay(uint32_t ms){
    /* Hardware keeps the 16-kHz FOC ISR running during the blocking detector.
     * Simulate that here. This regression specifically catches the V14 bug
     * where CONTROL_MODE_OPENLOOP_PHASE was overwritten by openloop_update(). */
    for(uint32_t t=0;t<ms;t++){
        for(uint8_t isr=0;isr<16u;isr++){
            uint8_t hl=raw_for_phase(m_motor_1.m_phase_openloop,left_raw_for_sector);
            uint8_t hr=raw_for_phase(m_motor_2.m_phase_openloop,right_raw_for_sector);
            set_hall(GPIOB,LEFT_HALL_U_PIN,LEFT_HALL_V_PIN,LEFT_HALL_W_PIN,hl);
            set_hall(GPIOC,RIGHT_HALL_U_PIN,RIGHT_HALL_V_PIN,RIGHT_HALL_W_PIN,hr);
            mcpwm_foc_adc_int_handler();
        }
    }
}

static int circular_diff200(uint8_t a,uint8_t b){
    int d=(int)a-(int)b;
    while(d>100)d-=200;
    while(d<-100)d+=200;
    return d<0?-d:d;
}
static int fail(const char*s){fprintf(stderr,"FAIL %s\n",s);return 1;}

static int validate(const uint8_t t[8],const uint8_t map[6],const char *which){
    if(t[0]!=255u||t[7]!=255u)return fail("invalid-state markers");
    for(uint8_t sec=0;sec<6u;sec++){
        const uint8_t raw=map[sec];
        const uint8_t expected=(uint8_t)((sec*200u+3u)/6u);
        if(t[raw]>=200u || circular_diff200(t[raw],expected)>4){
            fprintf(stderr,"FAIL %s raw=%u got=%u expected~%u\n",which,raw,t[raw],expected);
            return 1;
        }
    }
    return 0;
}

int main(void){
    uint8_t tl[8],tr[8];
    mcpwm_foc_init();
    HAL_Delay(1u);
    if(!mcpwm_foc_detect_hall(1.0f,false,tl))return fail("left detector returned false");
    if(validate(tl,left_raw_for_sector,"left"))return 1;
    if(!mcpwm_foc_detect_hall(1.0f,true,tr))return fail("right detector returned false");
    if(validate(tr,right_raw_for_sector,"right"))return 1;
    for(int i=0;i<8;i++){
        if((uint8_t)m_motor_1.m_conf.foc_hall_table[i]!=tl[i])return fail("left active table not applied");
        if((uint8_t)m_motor_2.m_conf.foc_hall_table[i]!=tr[i])return fail("right active table not applied");
    }

    /* Reset estimator state after the synthetic dual detector sweeps. During
     * right detection the test ISR also services left, so without this reset
     * left legitimately carries interpolation/rate-limit history. Re-apply the
     * just-detected tables exactly as a clean boot/config load would. */
    mc_configuration cl=m_motor_1.m_conf, cr=m_motor_2.m_conf;
    mcpwm_foc_init();
    mcpwm_foc_set_configuration(&cl,false);
    mcpwm_foc_set_configuration(&cr,true);

    /* VESC Hall FOC format is 0..199 -> 0..360 electrical degrees; 255 is
     * invalid. Prove the detected center is the phase used by closed-loop FOC. */
    const uint8_t h0=left_raw_for_sector[2];
    set_hall(GPIOB,LEFT_HALL_U_PIN,LEFT_HALL_V_PIN,LEFT_HALL_W_PIN,h0);
    for(uint8_t n=0u;n<(uint8_t)(MCCONF_HALL_DEBOUNCE_SAMPLES+1u);++n)mcpwm_foc_adc_int_handler();
    const uint16_t expected_phase=(uint16_t)(((uint32_t)tl[h0]*65536u)/200u);
    if(m_motor_1.m_phase_hall!=expected_phase)return fail("Hall table 0..199 electrical-angle mapping");
    mcpwm_foc_set_current(0.2f,false); mcpwm_foc_vesc_override_touch(false);
    mcpwm_foc_adc_int_handler();
    if(m_motor_1.m_phase!=m_motor_1.m_phase_hall)return fail("closed-loop FOC must use corrected Hall phase");

    /* One asynchronous wrong sample must not alter the debounced state/count. */
    const int32_t pos_before=m_motor_1.m_position_counts;
    const uint8_t glitch=left_raw_for_sector[4];
    set_hall(GPIOB,LEFT_HALL_U_PIN,LEFT_HALL_V_PIN,LEFT_HALL_W_PIN,glitch);
    mcpwm_foc_adc_int_handler();
    set_hall(GPIOB,LEFT_HALL_U_PIN,LEFT_HALL_V_PIN,LEFT_HALL_W_PIN,h0);
    mcpwm_foc_adc_int_handler();
    if(m_motor_1.m_hall_state!=h0 || m_motor_1.m_position_counts!=pos_before)return fail("single-sample Hall glitch debounce");

    /* A real adjacent code held for the debounce window must be accepted. */
    const uint8_t h1=left_raw_for_sector[3];
    set_hall(GPIOB,LEFT_HALL_U_PIN,LEFT_HALL_V_PIN,LEFT_HALL_W_PIN,h1);
    for(uint8_t n=0u;n<MCCONF_HALL_DEBOUNCE_SAMPLES;++n)mcpwm_foc_adc_int_handler();
    if(m_motor_1.m_hall_state!=h1)return fail("stable Hall transition acceptance");
    if(abs((int)(m_motor_1.m_position_counts-pos_before))!=1)return fail("stable Hall transition position count");

    /* A malformed Hall table arriving through SET_MCCONF must never replace
     * the last-known-good angle map used by active FOC. */
    uint8_t good_table[8]; memcpy(good_table,m_motor_1.m_conf.foc_hall_table,8);
    mc_configuration bad=m_motor_1.m_conf;
    bad.foc_hall_table[0]=255u; bad.foc_hall_table[7]=255u;
    bad.foc_hall_table[1]=10u; bad.foc_hall_table[2]=11u;
    bad.foc_hall_table[3]=12u; bad.foc_hall_table[4]=13u;
    bad.foc_hall_table[5]=14u; bad.foc_hall_table[6]=15u;
    mcpwm_foc_set_configuration(&bad,false);
    if(memcmp(good_table,m_motor_1.m_conf.foc_hall_table,8)!=0)return fail("malformed runtime Hall table must be rejected");

    /* Reversal/noise regression at ~700-750 ERPM-equivalent edge timing.
     * Sequence follows detected table phase order. One deliberate one-ISR
     * skipped-state glitch must be invisible after debounce. A real reversal
     * must reset period history and cannot be counted as a bad Hall sequence. */
    mcpwm_foc_init(); mcpwm_foc_set_configuration(&cl,false);
    const uint8_t seq[6]={5u,1u,3u,2u,6u,4u};
    hold_left_hall(seq[0],240u);
    for(uint8_t lap=0u;lap<2u;++lap){
        for(uint8_t k=1u;k<=6u;++k){
            const uint8_t h=seq[k%6u];
            if(lap==1u && k==3u){
                const uint8_t glitch=seq[(k+2u)%6u];
                set_hall(GPIOB,LEFT_HALL_U_PIN,LEFT_HALL_V_PIN,LEFT_HALL_W_PIN,glitch);
                mcpwm_foc_adc_int_handler();
            }
            hold_left_hall(h,220u);
        }
    }
    if(m_motor_1.m_hall_direction!=1)return fail("forward Hall direction after debounce");
    if(m_motor_1.m_hall_sequence_reject_count!=0u)return fail("forward/glitch sequence reject");
    if(m_motor_1.m_hall_period_reject_count!=0u)return fail("forward/glitch period reject");
    /* Reverse from current seq[0] by walking the same six states backward. */
    for(uint8_t k=1u;k<=12u;++k){
        const uint8_t idx=(uint8_t)((6u-(k%6u))%6u);
        hold_left_hall(seq[idx],220u);
    }
    if(m_motor_1.m_hall_direction!=-1)return fail("reverse Hall direction after warmup");
    if(m_motor_1.m_hall_sequence_reject_count!=0u)return fail("reversal sequence reject");
    if(m_motor_1.m_hall_period_reject_count!=0u)return fail("reversal period reject");
    printf("HALL_DETECT_ALGORITHM_PASS left=[%u,%u,%u,%u,%u,%u] right=[%u,%u,%u,%u,%u,%u]\n",
           tl[1],tl[2],tl[3],tl[4],tl[5],tl[6],tr[1],tr[2],tr[3],tr[4],tr[5],tr[6]);
    return 0;
}
