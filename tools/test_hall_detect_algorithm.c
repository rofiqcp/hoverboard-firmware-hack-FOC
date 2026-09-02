#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include "stm32f1xx_hal.h"
#include "config.h"
#include "defines.h"
#include "motor/mcpwm_foc.h"

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
    printf("HALL_DETECT_ALGORITHM_PASS left=[%u,%u,%u,%u,%u,%u] right=[%u,%u,%u,%u,%u,%u]\n",
           tl[1],tl[2],tl[3],tl[4],tl[5],tl[6],tr[1],tr[2],tr[3],tr[4],tr[5],tr[6]);
    return 0;
}
