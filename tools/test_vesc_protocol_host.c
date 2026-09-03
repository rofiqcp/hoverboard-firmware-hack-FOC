#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "stm32f1xx_hal.h"
#include "vesc/datatypes.h"
#include "vesc/buffer.h"
#include "vesc/crc.h"
#include "vesc/vesc_protocol.h"
#include "vesc/mcconf_serial.h"
#include "motor/mc_interface.h"
#include "motor/mcpwm_foc.h"
#include "defines.h"

UART_HandleTypeDef huart3 = {0};
volatile adc_buf_t adc_buffer = {0};
int16_t board_temp_deg_c = 31;
static uint32_t tick_ms = 1000u;
static uint8_t tx_capture[1024];
static uint16_t tx_capture_len = 0u;
static int selected_motor = 1;
static float set_current[2];
static float set_rpm[2];
static float set_duty[2];
static float set_pos[2];
static unsigned touch_count[2];
static unsigned store_count[2];
static mc_configuration confs[2];
static mcpwm_foc_motor_t diag_motors[2];
static int32_t pos_user[2] = {0,0};
static int32_t pos_target_user[2] = {0,0};
static int32_t pos_min_user[2] = {INT32_MIN,INT32_MIN};
static int32_t pos_max_user[2] = {INT32_MAX,INT32_MAX};

uint32_t HAL_GetTick(void) { return tick_ms; }
int HAL_UART_Transmit(UART_HandleTypeDef *h, uint8_t *d, uint16_t n, uint32_t t) {
    (void)h; (void)t;
    if (n > sizeof(tx_capture)) return 1;
    memcpy(tx_capture,d,n); tx_capture_len=n; return HAL_OK;
}
int HAL_UART_Transmit_DMA(UART_HandleTypeDef *h, uint8_t *d, uint16_t n) {
    (void)h;
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
    v->v_in=48.1f; v->current_motor=second?2.5f:1.5f; v->current_in=second?1.1f:0.8f;
    v->id=second?0.2f:0.1f; v->iq=second?-2.5f:1.5f; v->rpm=second?-321.0f:123.0f;
    v->duty_now=second?-0.22f:0.11f; v->fault_code=FAULT_CODE_NONE; v->vd=1.2f; v->vq=second?-5.0f:4.0f;
    v->position=second?342.0f:12.5f; v->tachometer=second?-45:31; v->tachometer_abs=second?45:31;
}
void mc_interface_set_current(float c) { set_current[selected_motor==2?1:0]=c; }
void mc_interface_set_brake_current(float c) { set_current[selected_motor==2?1:0]=c; }
void mc_interface_set_handbrake(float c) { set_current[selected_motor==2?1:0]=c; }
void mc_interface_set_pid_speed(float r) { set_rpm[selected_motor==2?1:0]=r; }
void mc_interface_set_pid_pos(float p) { set_pos[selected_motor==2?1:0]=p; }
void mcpwm_foc_sync_tuning_to_conf(bool second) { (void)second; }
void mcpwm_foc_get_default_configuration(mc_configuration *c, bool second) {
    (void)second; memset(c,0,sizeof(*c)); c->motor_type=MOTOR_TYPE_FOC; c->l_current_max=15.0f; c->l_current_min=-15.0f;
    c->foc_sensor_mode=FOC_SENSOR_MODE_HALL; c->si_motor_poles=30u;
    { const uint8_t t[8]={255u,83u,17u,50u,150u,117u,183u,255u}; for(int i=0;i<8;i++) c->foc_hall_table[i]=(int8_t)t[i]; }
}
void mc_interface_set_duty(float d) { set_duty[selected_motor==2?1:0]=d; }
mc_fault_code mc_interface_get_fault_motor(bool second) { (void)second; return FAULT_CODE_NONE; }
void mcpwm_foc_vesc_timeout_configure(bool second, uint32_t timeout_ms, float brake_current) {
    (void)second; (void)timeout_ms; (void)brake_current;
}
void mcpwm_foc_vesc_override_touch(bool second) { touch_count[second?1:0]++; }
bool mcpwm_foc_vesc_override_active(bool second) { return touch_count[second?1:0] != 0u; }
mcpwm_foc_motor_t *mcpwm_foc_get_motor(bool second) { return &diag_motors[second?1:0]; }
const mcpwm_foc_motor_t *mcpwm_foc_get_motor_const(bool second) { return &diag_motors[second?1:0]; }
void mcpwm_foc_set_openloop_phase(float current, float phase, bool second) {
    (void)phase; diag_motors[second?1:0].m_openloop_id_target_q4=(int16_t)lroundf(fabsf(current)*800.0f);
    diag_motors[second?1:0].m_control_mode=CONTROL_MODE_OPENLOOP_PHASE;
}
void mc_interface_release_motor(void) { diag_motors[selected_motor==2?1:0].m_control_mode=CONTROL_MODE_NONE; }
float mcpwm_foc_get_phase_motor(bool second) { return (float)diag_motors[second?1:0].m_phase * (360.0f / 65536.0f); }
uint32_t mcpwm_foc_get_isr_cycles(void) { return 1234u; }
uint32_t mcpwm_foc_get_isr_cycles_max(void) { return 2345u; }
float mcpwm_foc_get_erpm_motor(bool second) { return second ? -50.0f : 50.0f; }
void mcpwm_foc_get_current_offsets(int16_t *p0,int16_t *p1,int16_t *dc,bool second){if(p0)*p0=second?2003:1998;if(p1)*p1=second?1997:2001;if(dc)*dc=second?2002:1999;}
uint16_t mcpwm_foc_get_pole_pairs(bool second){return (uint16_t)((confs[second?1:0].si_motor_poles>=2?confs[second?1:0].si_motor_poles:30u)/2u);}
float mcpwm_foc_get_gear_ratio(bool second){float g=confs[second?1:0].si_gear_ratio;return g>0.0f?g:1.0f;}
float mcpwm_foc_get_motor_mechanical_rpm(bool second){return mcpwm_foc_get_erpm_motor(second)/(float)mcpwm_foc_get_pole_pairs(second);}
float mcpwm_foc_get_output_rpm(bool second){return mcpwm_foc_get_motor_mechanical_rpm(second)/mcpwm_foc_get_gear_ratio(second);}
void mcpwm_foc_set_position_user_counts(int32_t v, bool second) {
    const int j=second?1:0;
    if(v<pos_min_user[j])v=pos_min_user[j];
    if(v>pos_max_user[j])v=pos_max_user[j];
    pos_target_user[j]=v;
}
void mcpwm_foc_set_position_user_limits(int32_t lo,int32_t hi,bool second) {
    const int j=second?1:0; pos_min_user[j]=lo;pos_max_user[j]=hi;
    if(pos_target_user[j]<lo)pos_target_user[j]=lo;
    if(pos_target_user[j]>hi)pos_target_user[j]=hi;
}
int32_t mcpwm_foc_get_position_user_counts(bool second){return pos_user[second?1:0];}
int32_t mcpwm_foc_get_position_target_user_counts(bool second){return pos_target_user[second?1:0];}
int32_t mcpwm_foc_get_position_min_user_counts(bool second){return pos_min_user[second?1:0];}
int32_t mcpwm_foc_get_position_max_user_counts(bool second){return pos_max_user[second?1:0];}
void mcpwm_foc_reset_position(bool second){const int j=second?1:0;pos_user[j]=0;pos_target_user[j]=0;}
bool mc_interface_store_configuration_motor(bool second) { store_count[second?1:0]++; return true; }
bool mcpwm_foc_detect_hall(float current, bool second, uint8_t table[8]) {
    (void)current;
    static const uint8_t t[8]={255u,83u,17u,50u,150u,117u,183u,255u};
    for (int i=0;i<8;i++) table[i]=t[i];
    for (int i=0;i<8;i++) confs[second?1:0].foc_hall_table[i]=(int8_t)t[i];
    return true;
}

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
static void enqueue_only(const uint8_t *p,uint16_t n){
    uint8_t f[800];const uint16_t fn=make_frame(p,n,f);
    for(uint16_t i=0;i<fn;i++) (void)vesc_protocol_rx_byte(f[i]);
}
static int fail(const char *s){fprintf(stderr,"FAIL: %s\n",s);return 1;}
static int nearf32(float a,float b,float eps){return fabsf(a-b)<=eps;}
static int check_values_reply(const uint8_t *r,uint16_t rn,bool second){
    if(rn<60u || r[0]!=COMM_GET_VALUES) return fail(second?"right values header":"local values header");
    int32_t i=1;
    (void)buffer_get_float16(r,1e1f,&i); /* temp mos */
    (void)buffer_get_float16(r,1e1f,&i); /* temp motor */
    const float im=buffer_get_float32(r,1e2f,&i);
    const float iin=buffer_get_float32(r,1e2f,&i);
    const float id=buffer_get_float32(r,1e2f,&i);
    const float iq=buffer_get_float32(r,1e2f,&i);
    const float duty=buffer_get_float16(r,1e3f,&i);
    const float erpm=buffer_get_float32(r,1e0f,&i);
    const float vin=buffer_get_float16(r,1e1f,&i);
    (void)buffer_get_float32(r,1e4f,&i); (void)buffer_get_float32(r,1e4f,&i);
    (void)buffer_get_float32(r,1e4f,&i); (void)buffer_get_float32(r,1e4f,&i);
    (void)buffer_get_int32(r,&i); (void)buffer_get_int32(r,&i);
    const uint8_t fault=r[i++];
    const float pos=buffer_get_float32(r,1e6f,&i);
    const uint8_t idvesc=r[i++];
    (void)buffer_get_float16(r,1e1f,&i); (void)buffer_get_float16(r,1e1f,&i); (void)buffer_get_float16(r,1e1f,&i);
    const float vd=buffer_get_float32(r,1e3f,&i);
    const float vq=buffer_get_float32(r,1e3f,&i);
    if (i < rn) (void)r[i++]; /* timeout/kill */
    if(!nearf32(im,second?2.5f:1.5f,0.011f)) return fail(second?"right Imotor":"local Imotor");
    if(!nearf32(iin,second?1.1f:0.8f,0.011f)) return fail(second?"right Iin":"local Iin");
    if(!nearf32(id,second?0.2f:0.1f,0.011f)) return fail(second?"right Id":"local Id");
    if(!nearf32(iq,second?2.5f:1.5f,0.011f)) return fail(second?"right Iq normalize":"local Iq");
    if(!nearf32(duty,second?0.22f:0.11f,0.002f)) return fail(second?"right duty normalize":"local duty");
    if(!nearf32(erpm,second?321.0f:123.0f,0.5f)) return fail(second?"right ERPM normalize":"local ERPM");
    if(!nearf32(vin,48.1f,0.11f)) return fail("Vin");
    if(fault!=FAULT_CODE_NONE || idvesc!=(second?2u:1u)) return fail(second?"right fault/id":"local fault/id");
    if(!nearf32(pos,second?18.0f:12.5f,0.001f)) return fail(second?"right position normalize":"local position");
    if(!nearf32(vd,1.2f,0.002f)) return fail(second?"right Vd":"local Vd");
    if(!nearf32(vq,second?5.0f:4.0f,0.002f)) return fail(second?"right Vq normalize":"local Vq");
    return 0;
}
int main(void){
    memset(confs,0,sizeof(confs)); memset(diag_motors,0,sizeof(diag_motors));
    confs[0].motor_type=confs[1].motor_type=MOTOR_TYPE_FOC;
    diag_motors[0].m_iq_target_q4=2400; diag_motors[0].m_iq_set_q4=1600; diag_motors[0].m_iq_q4=800;
    diag_motors[0].m_hall_state=5u; diag_motors[0].m_control_mode=CONTROL_MODE_CURRENT; diag_motors[0].m_state=MC_STATE_RUNNING;
    diag_motors[1].m_iq_target_q4=-2400; diag_motors[1].m_iq_set_q4=-1600; diag_motors[1].m_iq_q4=-800;
    diag_motors[1].m_hall_state=3u; diag_motors[1].m_control_mode=CONTROL_MODE_CURRENT; diag_motors[1].m_state=MC_STATE_RUNNING;
    confs[0].l_current_max=confs[1].l_current_max=15.0f; confs[0].l_current_min=confs[1].l_current_min=-15.0f;
    vesc_protocol_init();
    uint8_t r[800];uint16_t rn=0u; int32_t k=0;
    uint8_t fw[]={COMM_FW_VERSION}; if(!transact(fw,sizeof(fw),r,&rn)||rn<4u||r[0]!=COMM_FW_VERSION||r[1]!=6u||r[2]!=0u)return fail("local fw");
    if(strcmp((const char *)&r[3],"motor_left")!=0)return fail("local hardware name");
    uint8_t ping[]={COMM_PING_CAN}; if(!transact(ping,sizeof(ping),r,&rn)||rn!=2u||r[0]!=COMM_PING_CAN||r[1]!=2u)return fail("ping id2");
    uint8_t fwr[]={COMM_FORWARD_CAN,2u,COMM_FW_VERSION}; if(!transact(fwr,sizeof(fwr),r,&rn)||rn<4u||r[0]!=COMM_FW_VERSION)return fail("right fw");
    if(strcmp((const char *)&r[3],"motor_right")!=0)return fail("right hardware name");

    /* Four valid request frames can arrive before the 5-ms main loop runs.
     * The RX FIFO must preserve all of them rather than V15's one-slot drop. */
    {
        uint8_t qd[5]={COMM_SET_DUTY,0,0,0,0}; int32_t qi=1; buffer_append_int32(qd,5000,&qi);
        uint8_t qc[5]={COMM_SET_CURRENT,0,0,0,0}; qi=1; buffer_append_int32(qc,3000,&qi);
        uint8_t qr[5]={COMM_SET_RPM,0,0,0,0}; qi=1; buffer_append_int32(qr,50,&qi);
        uint8_t qp[5]={COMM_SET_POS,0,0,0,0}; qi=1; buffer_append_int32(qp,15000000,&qi);
        enqueue_only(qd,sizeof(qd)); enqueue_only(qc,sizeof(qc)); enqueue_only(qr,sizeof(qr)); enqueue_only(qp,sizeof(qp));
        vesc_protocol_process_pending();
        if(!nearf32(set_duty[0],0.05f,0.0001f)||!nearf32(set_current[0],3.0f,0.001f)||
           !nearf32(set_rpm[0],50.0f,0.001f)||!nearf32(set_pos[0],15.0f,0.001f))return fail("rx fifo burst");
    }

    uint8_t gv[]={COMM_GET_VALUES}; if(!transact(gv,sizeof(gv),r,&rn) || check_values_reply(r,rn,false)) return 1;
    /* Upstream VESC GET_VALUES is request/reply only. The host owns the polling
     * cadence; firmware must not inject unsolicited packets that can be
     * mistaken for replies from another virtual motor. */

    /* VESC Tool can use the SETUP flavor for its dashboard; it is likewise
     * exactly one request -> one reply. */
    {
        uint8_t gvs[]={COMM_GET_VALUES_SETUP};
        if(!transact(gvs,sizeof(gvs),r,&rn)||rn<2u||r[0]!=COMM_GET_VALUES_SETUP)return fail("setup values immediate");
    }

    uint8_t gvr[]={COMM_FORWARD_CAN,2u,COMM_GET_VALUES}; if(!transact(gvr,sizeof(gvr),r,&rn) || check_values_reply(r,rn,true)) return 1;
    /* Upstream VESC: COMM_SET_DETECT selects a display mode and the 10-ms
     * periodic thread sends unsolicited COMM_ROTOR_POSITION = deg*100000. */
    {
        uint8_t sd[]={COMM_SET_DETECT,(uint8_t)DISP_POS_MODE_OBSERVER};
        if(!transact(sd,sizeof(sd),r,&rn)||rn!=0u)return fail("set detect local");
        diag_motors[0].m_phase=16384u; /* 90 electrical deg */
        tick_ms+=11u; tx_capture_len=0u; vesc_protocol_periodic(tick_ms);
        if(!decode_tx(r,&rn)||rn!=5u||r[0]!=COMM_ROTOR_POSITION)return fail("rotor stream local packet");
        int32_t pi=1; if(buffer_get_int32(r,&pi)!=9000000)return fail("rotor stream local value");
        uint8_t sdr[]={COMM_FORWARD_CAN,2u,COMM_SET_DETECT,(uint8_t)DISP_POS_MODE_OBSERVER};
        if(!transact(sdr,sizeof(sdr),r,&rn)||rn!=0u)return fail("set detect right");
        diag_motors[1].m_phase=32768u; /* 180 electrical deg */
        tick_ms+=11u; tx_capture_len=0u; vesc_protocol_periodic(tick_ms);
        if(!decode_tx(r,&rn)||rn!=5u||r[0]!=COMM_ROTOR_POSITION)return fail("rotor stream right packet");
        pi=1; if(buffer_get_int32(r,&pi)!=18000000)return fail("rotor stream right value");
        uint8_t off[]={COMM_SET_DETECT,(uint8_t)DISP_POS_MODE_NONE};
        if(!transact(off,sizeof(off),r,&rn))return fail("set detect off");
    }
    uint8_t duty[5]={COMM_SET_DUTY,0,0,0,0}; k=1; buffer_append_int32(duty,12500,&k);
    if(!transact(duty,sizeof(duty),r,&rn)||rn!=0u||!nearf32(set_duty[0],0.125f,0.0001f)) return fail("local duty");
    uint8_t dutyr[7]={COMM_FORWARD_CAN,2u,COMM_SET_DUTY,0,0,0,0}; k=3; buffer_append_int32(dutyr,12500,&k);
    if(!transact(dutyr,sizeof(dutyr),r,&rn)||rn!=0u||!nearf32(set_duty[1],-0.125f,0.0001f)) return fail("right duty sign/forward");
    uint8_t cur[7]={COMM_FORWARD_CAN,2u,COMM_SET_CURRENT,0,0,0,0}; k=3; buffer_append_int32(cur,2500,&k);
    if(!transact(cur,sizeof(cur),r,&rn)||rn!=0u)return fail("right current reply");
    if(fabsf(set_current[1]+2.5f)>0.001f||touch_count[1]==0u)return fail("right current sign/ownership");
    uint8_t rpm[5]={COMM_SET_RPM,0,0,0,0}; k=1;buffer_append_int32(rpm,300,&k);if(!transact(rpm,sizeof(rpm),r,&rn))return fail("local rpm frame");
    if(fabsf(set_rpm[0]-300.0f)>0.001f||touch_count[0]==0u)return fail("local rpm");
    uint8_t posl[5]={COMM_SET_POS,0,0,0,0}; k=1;buffer_append_int32(posl,45000000,&k);
    if(!transact(posl,sizeof(posl),r,&rn)||fabsf(set_pos[0]-45.0f)>0.001f)return fail("local position");
    uint8_t posr[7]={COMM_FORWARD_CAN,2u,COMM_SET_POS,0,0,0,0}; k=3;buffer_append_int32(posr,30000000,&k);
    if(!transact(posr,sizeof(posr),r,&rn)||fabsf(set_pos[1]+30.0f)>0.001f)return fail("right position sign/forward");
    uint8_t gmr[]={COMM_FORWARD_CAN,2u,COMM_GET_MCCONF};
    if(!transact(gmr,sizeof(gmr),r,&rn)||rn<10u||r[0]!=COMM_GET_MCCONF) return fail("get right mcconf");
    {int32_t mi=1; if(buffer_get_uint32(r,&mi)!=MCCONF_SIGNATURE) return fail("right mcconf signature");}
    uint8_t gm[]={COMM_GET_MCCONF};
    if(!transact(gm,sizeof(gm),r,&rn)||rn<10u||r[0]!=COMM_GET_MCCONF) return fail("get mcconf");
    {int32_t mi=1; if(buffer_get_uint32(r,&mi)!=MCCONF_SIGNATURE) return fail("mcconf signature");}
    uint8_t gmd[]={COMM_GET_MCCONF_DEFAULT};
    if(!transact(gmd,sizeof(gmd),r,&rn)||rn<10u||r[0]!=COMM_GET_MCCONF_DEFAULT) return fail("get mcconf default");
    {int32_t mi=1; if(buffer_get_uint32(r,&mi)!=MCCONF_SIGNATURE) return fail("mcconf default signature");}
    {
        uint8_t sm[700]; mc_configuration c=confs[0];
        c.l_current_max=9.0f; c.l_current_min=-9.0f; c.foc_sensor_mode=FOC_SENSOR_MODE_HALL; c.si_motor_poles=20u; c.si_gear_ratio=5.5f;
        const uint8_t ht[8]={255u,80u,14u,47u,147u,114u,180u,255u};
        for(int q=0;q<8;q++) c.foc_hall_table[q]=(int8_t)ht[q];
        sm[0]=COMM_SET_MCCONF; const int32_t sn=confgenerator_serialize_mcconf(sm+1,&c);
        const unsigned before=store_count[0];
        if(sn<=0 || !transact(sm,(uint16_t)(sn+1),r,&rn)||rn!=1u||r[0]!=COMM_SET_MCCONF) return fail("set mcconf ack");
        if(store_count[0]!=before+1u || !nearf32(confs[0].l_current_max,9.0f,0.01f)) return fail("set mcconf apply/store");
        if(confs[0].si_motor_poles!=20u || !nearf32(confs[0].si_gear_ratio,5.5f,0.01f)) return fail("set mcconf runtime poles/gear");
        for(int q=0;q<8;q++) if((uint8_t)confs[0].foc_hall_table[q]!=ht[q]) return fail("set mcconf hall table");

        /* Standard VESC reset-default workflow is GET_MCCONF_DEFAULT followed
         * by SET_MCCONF. Verify the default reply is independent of active
         * config and can be written/persisted normally. */
        if(!transact(gmd,sizeof(gmd),r,&rn)||r[0]!=COMM_GET_MCCONF_DEFAULT)return fail("get defaults after custom write");
        mc_configuration defc; memset(&defc,0,sizeof(defc));
        if(!confgenerator_deserialize_mcconf(r+1,&defc) || !nearf32(defc.l_current_max,15.0f,0.01f))return fail("default mcconf content");
        sm[0]=COMM_SET_MCCONF; const int32_t dn=confgenerator_serialize_mcconf(sm+1,&defc);
        const unsigned before_def=store_count[0];
        if(dn<=0 || !transact(sm,(uint16_t)(dn+1),r,&rn)||rn!=1u||r[0]!=COMM_SET_MCCONF)return fail("write defaults ack");
        if(store_count[0]!=before_def+1u || !nearf32(confs[0].l_current_max,15.0f,0.01f))return fail("write defaults apply/store");
    }
    {
        uint8_t sm[702]; mc_configuration c=confs[1];
        c.l_current_max=8.0f; c.l_current_min=-8.0f; c.foc_sensor_mode=FOC_SENSOR_MODE_HALL; c.si_motor_poles=14u; c.si_gear_ratio=2.0f;
        const uint8_t ht[8]={255u,82u,16u,49u,149u,116u,182u,255u};
        for(int q=0;q<8;q++) c.foc_hall_table[q]=(int8_t)ht[q];
        sm[0]=COMM_FORWARD_CAN; sm[1]=2u; sm[2]=COMM_SET_MCCONF;
        const int32_t sn=confgenerator_serialize_mcconf(sm+3,&c); const unsigned before=store_count[1];
        if(sn<=0 || !transact(sm,(uint16_t)(sn+3),r,&rn)||rn!=1u||r[0]!=COMM_SET_MCCONF) return fail("set right mcconf ack");
        if(store_count[1]!=before+1u || !nearf32(confs[1].l_current_max,8.0f,0.01f)) return fail("set right mcconf apply/store");
        if(confs[1].si_motor_poles!=14u || !nearf32(confs[1].si_gear_ratio,2.0f,0.01f)) return fail("set right mcconf runtime poles/gear");
        for(int q=0;q<8;q++) if((uint8_t)confs[1].foc_hall_table[q]!=ht[q]) return fail("set right mcconf hall table");
    }
    {
        /* Full signed-int32 long-range position is project-specific and rides
         * inside standard COMM_CUSTOM_APP_DATA. Standard COMM_SET_POS remains
         * VESC single-turn degrees. */
        const uint8_t magic0=0x48u, magic1=0x42u, ver=1u;
        uint8_t cp[16]={COMM_CUSTOM_APP_DATA,magic0,magic1,ver,3u};
        k=5; buffer_append_int32(cp,-1000000,&k); buffer_append_int32(cp,2000000,&k);
        if(!transact(cp,(uint16_t)k,r,&rn)||rn!=22u||r[0]!=COMM_CUSTOM_APP_DATA||r[5]!=0u) return fail("custom set limits");
        int32_t ci=6; (void)buffer_get_int32(r,&ci); (void)buffer_get_int32(r,&ci);
        if(buffer_get_int32(r,&ci)!=-1000000 || buffer_get_int32(r,&ci)!=2000000) return fail("custom limits values");

        uint8_t ct[12]={COMM_CUSTOM_APP_DATA,magic0,magic1,ver,4u};
        k=5; buffer_append_int32(ct,-345678,&k);
        if(!transact(ct,(uint16_t)k,r,&rn)||r[5]!=0u||pos_target_user[0]!=-345678) return fail("custom left target");
        if(touch_count[0]==0u) return fail("custom target ownership");

        uint8_t ctr[14]={COMM_FORWARD_CAN,2u,COMM_CUSTOM_APP_DATA,magic0,magic1,ver,4u};
        k=7; buffer_append_int32(ctr,456789,&k);
        if(!transact(ctr,(uint16_t)k,r,&rn)||r[5]!=0u||pos_target_user[1]!=456789) return fail("custom right target");

        uint8_t dg[]={COMM_CUSTOM_APP_DATA,magic0,magic1,ver,1u};
        if(!transact(dg,sizeof(dg),r,&rn)||rn<78u||r[0]!=COMM_CUSTOM_APP_DATA||r[4]!=1u||r[5]!=0u) return fail("custom diag");

        if(r[6]!=1u || r[10]!=5u) return fail("custom diag id/hall");
        int32_t di=14;
        if(buffer_get_int32(r,&di)!=3000) return fail("custom diag iq target 3A");

        /* Project tuning extension remains inside COMM_CUSTOM_APP_DATA. */
        {
            uint8_t gt[]={COMM_CUSTOM_APP_DATA,magic0,magic1,ver,6u};
            if(!transact(gt,sizeof(gt),r,&rn)||rn!=30u||r[4]!=6u||r[5]!=0u) return fail("custom get tuning");
            uint8_t st[40]={COMM_CUSTOM_APP_DATA,magic0,magic1,ver,7u}; int32_t si=5;
            const uint16_t tv[10]={1300,1400,900,1000,950,2000,3,60,1,2};
            for(int z=0;z<10;z++)buffer_append_uint16(st,tv[z],&si);
            buffer_append_uint16(st,3277u,&si); st[si++]=1u;
            unsigned before_store=store_count[0];
            if(!transact(st,(uint16_t)si,r,&rn)||rn!=30u||r[4]!=7u||r[5]!=0u) return fail("custom set tuning");
            if(diag_motors[0].m_kpq_q11!=1300u||diag_motors[0].m_kid_q16!=1000u||diag_motors[0].m_kps_q11!=950u||diag_motors[0].m_kpp_q11!=60u) return fail("custom tuning apply");
            if(diag_motors[0].m_telem_current_filter_q16!=3277u||store_count[0]!=(before_store+1u)) return fail("custom tuning filter/store");
            uint8_t idt[20]={COMM_CUSTOM_APP_DATA,magic0,magic1,ver,8u}; si=5; buffer_append_int32(idt,300,&si); buffer_append_int32(idt,0,&si);
            if(!transact(idt,(uint16_t)si,r,&rn)||rn!=6u||diag_motors[0].m_openloop_id_target_q4!=240||diag_motors[0].m_control_mode!=CONTROL_MODE_OPENLOOP_PHASE) return fail("custom id test");
            si=5; buffer_append_int32(idt,0,&si); buffer_append_int32(idt,0,&si);
            if(!transact(idt,(uint16_t)si,r,&rn)||diag_motors[0].m_control_mode!=CONTROL_MODE_NONE) return fail("custom id release");
        }
    }
    uint8_t dh[5]={COMM_DETECT_HALL_FOC,0,0,0,0}; k=1; buffer_append_int32(dh,1000,&k);
    if(!transact(dh,sizeof(dh),r,&rn)||rn!=10u||r[0]!=COMM_DETECT_HALL_FOC||r[9]!=0u||store_count[0]==0u)return fail("local hall detect");
    uint8_t dhr[7]={COMM_FORWARD_CAN,2u,COMM_DETECT_HALL_FOC,0,0,0,0}; k=3; buffer_append_int32(dhr,1000,&k);
    if(!transact(dhr,sizeof(dhr),r,&rn)||rn!=10u||r[0]!=COMM_DETECT_HALL_FOC||r[9]!=0u||store_count[1]==0u)return fail("right hall detect");
    printf("VESC_PROTOCOL_HOST_PASS fw=6.00 can=2 rightIqInternal=%.2f localRpm=%.0f posL=%.0f posRinternal=%.0f hall=ok values=ok mcconf=rw\n",set_current[1],set_rpm[0],set_pos[0],set_pos[1]);
    return 0;
}
