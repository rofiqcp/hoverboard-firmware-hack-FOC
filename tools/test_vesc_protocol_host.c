#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "stm32f1xx_hal.h"
#include "vesc/datatypes.h"
#include "vesc/buffer.h"
#include "vesc/crc.h"
#include "vesc/vesc_protocol.h"
#include "motor/mc_interface.h"
#include "motor/mcpwm_foc.h"

UART_HandleTypeDef huart3 = {0};
int16_t board_temp_deg_c = 31;
static uint32_t tick_ms = 1000u;
static uint8_t tx_capture[1024];
static uint16_t tx_capture_len = 0u;
static int selected_motor = 1;
static float set_current[2];
static float set_rpm[2];
static float set_duty[2];
static unsigned touch_count[2];
static mc_configuration confs[2];

uint32_t HAL_GetTick(void) { return tick_ms; }
int HAL_UART_Transmit(UART_HandleTypeDef *h, uint8_t *d, uint16_t n, uint32_t t) {
    (void)h; (void)t;
    if (n > sizeof(tx_capture)) return 1;
    memcpy(tx_capture,d,n); tx_capture_len=n; return HAL_OK;
}
void __disable_irq(void) {}
void __enable_irq(void) {}

void mc_interface_select_motor_thread(int motor) { selected_motor=motor; }
const volatile mc_configuration *mc_interface_get_configuration_motor(bool second) { return &confs[second?1:0]; }
void mc_interface_set_configuration(mc_configuration *configuration) { confs[selected_motor==2?1:0]=*configuration; }
void mc_interface_get_values_motor(mc_values *v, bool second) {
    memset(v,0,sizeof(*v));
    v->v_in=48.1f; v->current_motor=second?-2.5f:1.5f; v->current_in=second?1.1f:0.8f;
    v->id=second?0.2f:0.1f; v->iq=second?-2.5f:1.5f; v->rpm=second?-321.0f:123.0f;
    v->duty_now=second?-0.22f:0.11f; v->fault_code=FAULT_CODE_NONE; v->vd=1.2f; v->vq=second?-5.0f:4.0f;
}
void mc_interface_set_current(float c) { set_current[selected_motor==2?1:0]=c; }
void mc_interface_set_brake_current(float c) { set_current[selected_motor==2?1:0]=c; }
void mc_interface_set_pid_speed(float r) { set_rpm[selected_motor==2?1:0]=r; }
void mc_interface_set_duty(float d) { set_duty[selected_motor==2?1:0]=d; }
void mcpwm_foc_vesc_override_touch(bool second) { touch_count[second?1:0]++; }

static uint16_t make_frame(const uint8_t *payload, uint16_t len, uint8_t *out) {
    uint16_t i=0u;
    if (len<=255u) { out[i++]=2u; out[i++]=(uint8_t)len; }
    else { out[i++]=3u; out[i++]=(uint8_t)(len>>8); out[i++]=(uint8_t)len; }
    memcpy(out+i,payload,len); i=(uint16_t)(i+len);
    uint16_t crc=vesc_crc16(payload,len); out[i++]=(uint8_t)(crc>>8); out[i++]=(uint8_t)crc; out[i++]=3u;
    return i;
}
static bool decode_tx(uint8_t *payload, uint16_t *len) {
    if (tx_capture_len<5u) return false;
    uint16_t h=0u,n=0u;
    if(tx_capture[0]==2u){h=2u;n=tx_capture[1];}
    else if(tx_capture[0]==3u){h=3u;n=(uint16_t)(((uint16_t)tx_capture[1]<<8)|tx_capture[2]);}
    else return false;
    if(tx_capture_len!=(uint16_t)(h+n+3u) || tx_capture[h+n+2u]!=3u) return false;
    uint16_t crc=(uint16_t)(((uint16_t)tx_capture[h+n]<<8)|tx_capture[h+n+1u]);
    if(crc!=vesc_crc16(tx_capture+h,n)) return false;
    memcpy(payload,tx_capture+h,n);*len=n;return true;
}
static bool transact(const uint8_t *p,uint16_t n,uint8_t *reply,uint16_t *rn){
    uint8_t f[800];uint16_t fn=make_frame(p,n,f);tx_capture_len=0u;
    for(uint16_t i=0;i<fn;i++) (void)vesc_protocol_rx_byte(f[i]);
    vesc_protocol_process_pending();
    if(tx_capture_len==0u){*rn=0u;return true;}
    return decode_tx(reply,rn);
}
static int fail(const char *s){fprintf(stderr,"FAIL: %s\n",s);return 1;}
int main(void){
    memset(confs,0,sizeof(confs)); confs[0].motor_type=confs[1].motor_type=MOTOR_TYPE_FOC;
    confs[0].l_current_max=confs[1].l_current_max=15.0f; confs[0].l_current_min=confs[1].l_current_min=-15.0f;
    vesc_protocol_init();
    uint8_t r[800];uint16_t rn=0u;
    uint8_t fw[]={COMM_FW_VERSION}; if(!transact(fw,sizeof(fw),r,&rn)||rn<4u||r[0]!=COMM_FW_VERSION||r[1]!=7u||r[2]!=1u)return fail("local fw");
    uint8_t ping[]={COMM_PING_CAN}; if(!transact(ping,sizeof(ping),r,&rn)||rn!=2u||r[0]!=COMM_PING_CAN||r[1]!=2u)return fail("ping id2");
    uint8_t fwr[]={COMM_FORWARD_CAN,2u,COMM_FW_VERSION}; if(!transact(fwr,sizeof(fwr),r,&rn)||rn<4u||r[0]!=COMM_FW_VERSION)return fail("right fw");
    uint8_t cur[7]={COMM_FORWARD_CAN,2u,COMM_SET_CURRENT,0,0,0,0}; int32_t k=3; buffer_append_int32(cur,2500,&k);
    if(!transact(cur,sizeof(cur),r,&rn)||rn!=0u)return fail("right current reply");
    if(fabsf(set_current[1]+2.5f)>0.001f||touch_count[1]==0u)return fail("right current sign/ownership");
    uint8_t rpm[5]={COMM_SET_RPM,0,0,0,0}; k=1;buffer_append_int32(rpm,300,&k);if(!transact(rpm,sizeof(rpm),r,&rn))return fail("local rpm frame");
    if(fabsf(set_rpm[0]-300.0f)>0.001f||touch_count[0]==0u)return fail("local rpm");
    printf("VESC_PROTOCOL_HOST_PASS fw=7.01 can=2 rightIqInternal=%.2f localRpm=%.0f\n",set_current[1],set_rpm[0]);
    return 0;
}
