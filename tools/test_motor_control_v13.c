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
extern volatile int32_t positionCommandL;
extern volatile int32_t positionCommandR;

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

    /* Low-speed VESC regression: 50 ERPM @15 pole pairs is 3.333... mechanical
     * RPM. Keep that fractional target in Q16 rather than truncating control to
     * 3 RPM (=45 ERPM). Hall telemetry also derives ERPM directly from period. */
    mcpwm_foc_init();
    mcpwm_foc_set_pid_speed(50.0f,false);
    mcpwm_foc_vesc_override_touch(false);
    const int32_t q16_50_erpm=(int32_t)((50.0f/15.0f)*65536.0f+0.5f);
    if(abs(m_motor_1.m_speed_target_rpm_q16-q16_50_erpm)>2)return fail("50 ERPM fractional Q16 target");
    m_motor_1.m_hall_initialized=1u; m_motor_1.m_hall_direction=1;
    m_motor_1.m_hall_period=3200u; m_motor_1.m_hall_ticks=0u;
    if(fabsf(mcpwm_foc_get_erpm_motor(false)-50.0f)>0.05f)return fail("50 ERPM direct Hall telemetry");

    /* Stock VESC SET_CURRENT scaling: 3.00 A must become the exact Iq target. */
    mcpwm_foc_init();
    mcpwm_foc_set_current(3.0f,false);
    if(m_motor_1.m_iq_target_q4!=2400)return fail("VESC 3A Iq target scaling");

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

    /* V13 runtime tunables start from the proven V12 fixed-point values. */
    mcpwm_foc_init();
    if(m_motor_1.m_kpq_q11!=MCCONF_FOC_CURRENT_KP_Q11 || m_motor_1.m_kiq_q16!=MCCONF_FOC_CURRENT_KI_Q16)
        return fail("V13 Q current PI defaults");
    if(m_motor_1.m_kpd_q11!=MCCONF_FOC_ID_KP_Q11 || m_motor_1.m_kid_q16!=MCCONF_FOC_ID_KI_Q16)
        return fail("V13 D current PI defaults");
    if(m_motor_1.m_kps_q11!=MCCONF_SPEED_KP_Q11 || m_motor_1.m_kis_q16!=MCCONF_SPEED_KI_Q16 || m_motor_1.m_kds_q11!=0u)
        return fail("V13 speed PID defaults");
    if(m_motor_1.m_kpp_q11!=MCCONF_POSITION_KP_Q11 || m_motor_1.m_kip_q16!=MCCONF_POSITION_KI_Q16 || m_motor_1.m_kdp_q11!=MCCONF_POSITION_KD_Q11)
        return fail("V13 position PID defaults");

    /* A stock VESC mc_configuration has one common current Kp/Ki. Writing it
     * through VESC Tool intentionally applies that pair to both D and Q, while
     * the custom KPQ/KIQ/KPD/KID terminal parameters may separate them later. */
    mc_configuration tune=m_motor_1.m_conf;
    tune.foc_current_kp=1.0f; tune.foc_current_ki=100.0f;
    tune.s_pid_kp=2.0f; tune.s_pid_ki=0.5f; tune.s_pid_kd=0.25f;
    tune.p_pid_kp=3.0f; tune.p_pid_ki=0.1f; tune.p_pid_kd=0.05f;
    mcpwm_foc_set_configuration(&tune,false);
    if(m_motor_1.m_kpq_q11!=1536u || m_motor_1.m_kpd_q11!=1536u)return fail("VESC current Kp maps D/Q");
    if(abs((int)m_motor_1.m_kiq_q16-461)>1 || m_motor_1.m_kiq_q16!=m_motor_1.m_kid_q16)return fail("VESC current Ki maps D/Q");
    if(m_motor_1.m_kps_q11!=2000u || m_motor_1.m_kis_q16!=500u || m_motor_1.m_kds_q11!=250u)return fail("VESC speed PID mapping");
    if(m_motor_1.m_kpp_q11!=3000u || m_motor_1.m_kip_q16!=100u || m_motor_1.m_kdp_q11!=50u)return fail("VESC position PID mapping");

    /* Legacy mode 5 is multi-turn Hall position: 15 pole pairs * 6 sectors =
     * 90 counts/revolution (4 mechanical degrees/count). User-facing positive
     * targets are mirrored internally for motor 2. */
    mcpwm_foc_init(); enable=1u; motorRunReq=1u; set_halls(3u,3u);
    ctrlModReq=5u; positionCommandL=10; positionCommandR=10; pwml=0; pwmr=0;
    for(int i=0;i<120;i++)mcpwm_foc_adc_int_handler();
    if(m_motor_1.m_control_mode!=CONTROL_MODE_POS || m_motor_2.m_control_mode!=CONTROL_MODE_POS)return fail("mode5 position control entry");
    if(m_motor_1.m_position_target_counts!=10 || m_motor_2.m_position_target_counts!=-10)return fail("mode5 right user sign normalization");
    if(m_motor_1.m_iq_target_q4<=0 || m_motor_2.m_iq_target_q4>=0)return fail("position PID must command signed Iq");

    m_motor_1.m_position_counts=12345; m_motor_2.m_position_counts=-12345;
    mcpwm_foc_reset_position(false); mcpwm_foc_reset_position(true);
    if(m_motor_1.m_position_counts!=0 || m_motor_2.m_position_counts!=0)return fail("reset position count");
    if(m_motor_1.m_position_integrator!=0 || m_motor_2.m_position_integrator!=0)return fail("reset position PID");

    m_motor_1.m_position_min_counts=-5; m_motor_1.m_position_max_counts=5;
    mcpwm_foc_set_position_counts(100,false);
    if(m_motor_1.m_position_target_counts!=5)return fail("position PMAX clamp");
    mcpwm_foc_set_position_counts(-100,false);
    if(m_motor_1.m_position_target_counts!=-5)return fail("position PMIN clamp");

    /* Standard VESC COMM_SET_POS is single-turn rotor electrical position,
     * not this project's long-range Hall-count coordinate. With Hall-only
     * position sensing the command is quantized to the nearest 60 electrical
     * degrees. From electrical phase 0 deg, a 40 deg request therefore maps
     * to +1 Hall transition. Standard mc_values.position is the normalized
     * rotor electrical phase itself. */
    mcpwm_foc_init();
    m_motor_1.m_phase=0u;
    m_motor_1.m_position_counts=0;
    mcpwm_foc_set_pid_pos(40.0f,false);
    if(m_motor_1.m_position_target_counts!=1)return fail("VESC position electrical degrees to nearest Hall count");
    mc_values pvals;
    m_motor_1.m_phase=(uint16_t)lroundf(40.0f*(65536.0f/360.0f));
    mcpwm_foc_get_values(&pvals,false);
    if(fabsf(pvals.position-40.0f)>0.02f)return fail("VESC rotor position telemetry electrical degrees");

    /* VESC current semantics: current_motor is SIGN(Ibus)*sqrt(Id^2+Iq^2),
     * current_in is battery/DC-bus current, and Id/Iq remain separate. */
    m_motor_1.m_id_q4=600;   /* 0.75 A */
    m_motor_1.m_iq_q4=800;   /* 1.00 A -> |Idq|=1.25 A */
    m_motor_1.m_current_in_counts=-50; /* EFeru raw polarity => +1.00 A Ibus */
    mcpwm_foc_get_values(&pvals,false);
    if(fabsf(pvals.current_motor-1.25f)>0.01f)return fail("VESC Imotor signed DQ magnitude positive");
    if(fabsf(pvals.current_in-1.00f)>0.01f)return fail("VESC Ibattery positive scaling");
    if(fabsf(pvals.id-0.75f)>0.01f || fabsf(pvals.iq-1.00f)>0.01f)return fail("VESC Id/Iq scaling");
    m_motor_1.m_current_in_counts=50;
    mcpwm_foc_get_values(&pvals,false);
    if(fabsf(pvals.current_motor+1.25f)>0.01f || fabsf(pvals.current_in+1.00f)>0.01f)return fail("VESC regenerative current signs");

    /* Standard VESC Ah/Wh telemetry must be live counters, not constant zero.
     * Integrate exactly 100 ms at +1 A and then 100 ms at -1 A. The default
     * test bus is 40.0 V, so expected draw/charge are 0.1 As and 4.0 Ws. */
    mcpwm_foc_init();
    m_motor_1.m_current_in_counts=-50;
    mcpwm_foc_energy_update(1000u);
    mcpwm_foc_energy_update(1100u);
    mcpwm_foc_get_values(&pvals,false);
    if(fabsf(pvals.amp_hours-(0.1f/3600.0f))>1e-7f)return fail("VESC Ah drawn integration");
    if(fabsf(pvals.watt_hours-(pvals.amp_hours*pvals.v_in))>2e-6f)return fail("VESC Wh drawn integration");
    if(pvals.amp_hours_charged!=0.0f || pvals.watt_hours_charged!=0.0f)return fail("VESC charged counters must start zero");
    m_motor_1.m_current_in_counts=50;
    mcpwm_foc_energy_update(1200u);
    mcpwm_foc_get_values(&pvals,false);
    if(fabsf(pvals.amp_hours_charged-(0.1f/3600.0f))>1e-7f)return fail("VESC Ah charged integration");
    if(fabsf(pvals.watt_hours_charged-(pvals.amp_hours_charged*pvals.v_in))>2e-6f)return fail("VESC Wh charged integration");

    /* Standard VESC drivetrain setup is runtime configuration: motor poles
     * change electrical<->mechanical conversion; gear ratio changes output
     * shaft speed only. COMM_SET_RPM and mc_values.rpm stay ERPM. */
    {
        mc_configuration dyn=m_motor_1.m_conf;
        dyn.si_motor_poles=20u; dyn.si_gear_ratio=5.0f;
        mcpwm_foc_set_configuration(&dyn,false);
        mcpwm_foc_set_pid_speed(300.0f,false);
        if(mcpwm_foc_get_pole_pairs(false)!=10u)return fail("runtime pole-pair 20 poles -> 10pp");
        if(labs((long)m_motor_1.m_speed_target_rpm_q16-(long)(30*65536))>2)return fail("300 ERPM -> 30 motor RPM @10pp");
        m_motor_1.m_hall_initialized=0u; m_motor_1.m_rpm=100;
        if(fabsf(mcpwm_foc_get_motor_mechanical_rpm(false)-100.0f)>0.01f)return fail("runtime motor mechanical RPM");
        if(fabsf(mcpwm_foc_get_output_rpm(false)-20.0f)>0.01f)return fail("gearbox output RPM");
        dyn.si_gear_ratio=1.0f; mcpwm_foc_set_configuration(&dyn,false);
        if(fabsf(mcpwm_foc_get_output_rpm(false)-100.0f)>0.01f)return fail("direct drive output RPM");
    }

    printf("MOTOR_CONTROL_V13_PASS speed_ramp=%uRPM/s trq50=0.50A posCPR=90 VESCcmd40=%ld VESCpos=%.1f runtime_poles_gear=1\n",
           m_motor_1.m_speed_ramp_rpm_s,(long)m_motor_1.m_position_target_counts,pvals.position);
    return 0;
}
