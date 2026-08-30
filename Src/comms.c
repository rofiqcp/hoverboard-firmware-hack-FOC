/**
  * This file is part of the hoverboard-firmware-hack project.
  *
  * Copyright (C) 2020-2021 Emanuel FERU <aerdronix@gmail.com>
  *
  * This program is free software: you can redistribute it and/or modify
  * it under the terms of the GNU General Public License as published by
  * the Free Software Foundation, either version 3 of the License, or
  * (at your option) any later version.
  *
  * This program is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  * GNU General Public License for more details.
  *
  * You should have received a copy of the GNU General Public License
  * along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// Includes
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include "stm32f1xx_hal.h"
#include "config.h"
#include "defines.h"
#include "eeprom.h"
#include "motor/mcpwm_foc.h"
#include "util.h"
#include "comms.h"
#include "motor/mc_interface.h"


#define RAW_MIN -1000
#define RAW_MAX 1000


#define MAX_PARAM_WATCH 15
static uint8_t paramSilent = 0u;
static volatile uint8_t m_special_command_pending = 0u;
static char m_special_command[80];




extern InputStruct input1[];            // input structure
extern InputStruct input2[];            // input structure

extern uint16_t VirtAddVarTab[NB_OF_VAR];
extern volatile adc_buf_t m_adc_buffer;
extern int16_t speedAvg;                      // average measured speed
extern int16_t speedAvgAbs;                   // average measured speed in absolute
extern int16_t batVoltageCalib;
extern int16_t board_temp_deg_c;
extern int16_t left_dc_curr;
extern int16_t right_dc_curr;
extern int16_t dc_curr;
extern int16_t cmdL;
extern int16_t cmdR;
extern volatile uint32_t m_foc_isr_cycles;
extern volatile uint32_t m_foc_isr_cycles_max;



static int8_t calibrateCurrentOffsets(void) {
  if (input1[0].cmd != 0 || input2[0].cmd != 0 || cmdL != 0 || cmdR != 0 ||
      abs(m_motor_1.m_output.rpm) > 5 || abs(m_motor_2.m_output.rpm) > 5) {
    printf("! CALIBRATE requires STOP, ramp-down complete and |rpm|<=5\r\n");
    return 0;
  }
  currentCalibrationStart();
  printf("# CALIBRATE started samples:%u\r\n", (unsigned)ADC_CALIBRATION_SAMPLES);
  return 1;
}

enum commandTypes {READ,WRITE};
// Function0 - Function with 0 parameter
// Function1 - Function with 1 parameter (e.g. GET PARAM)
// Function2 - Function with 2 parameter (e.g. SET PARAM XXXX)
const command_entry commands[] = {
  // Type   ,Name      ,Function0         ,Function1       ,Function2      ,Help     
    {READ   ,"GET"     ,printAllParamDef  ,printParamDef   ,NULL           ,"Get Parameter/Variable"},
    {READ   ,"HELP"    ,printAllParamHelp ,printParamHelp  ,NULL           ,"Command/Parameter/Variable Help"},
    {READ   ,"WATCH"   ,NULL              ,watchParamVal   ,NULL           ,"Toggle Parameter/Variable Watch"},
    {WRITE  ,"SET"     ,NULL              ,NULL            ,setParamValExt ,"Set Parameter"},
    {WRITE  ,"INIT"    ,NULL              ,initParamVal    ,NULL           ,"Init Parameter from EEPROM or CONFIG.H"},
    {WRITE  ,"SAVE"    ,saveAllParamVal   ,NULL            ,NULL           ,"Save Parameters to EEPROM"},
    {WRITE  ,"CALIBRATE",calibrateCurrentOffsets,NULL       ,NULL           ,"Recalibrate six current ADC offsets while stopped"},
};

static const char *const errors[] = {
  "Command not found",                         /* Err1  */
  "Parameter not found",                       /* Err2  */
  "This command cannot be used with a Variable",/* Err3 */
  "Value not in range",                        /* Err4  */
  "Value expected",                            /* Err5  */
  "Start of line expected",                    /* Err6  */
  "End of line expected",                      /* Err7  */
  "Parameter expected",                        /* Err8  */
  "Uncaught error",                            /* Err9  */
  "Watch list is full"                         /* Err10 */
};

enum paramTypes {PARAMETER,VARIABLE};
const parameter_entry params[] = {
  /* Runtime mode and motor limits */
  {VARIABLE,"SENSOR_L",ADD_PARAM(m_sensor_mode_left),NULL,0,0,0,0,0,0,0,0,NULL,"Runtime Left sensor: 1 openloop, 2 Hall, 3 encoder AB"},
  {VARIABLE,"SENSOR_R",ADD_PARAM(m_sensor_mode_right),NULL,0,0,0,0,0,0,0,0,NULL,"Runtime Right sensor: 1 openloop, 2 Hall"},
  {VARIABLE,"COMM_L",ADD_PARAM(m_comm_mode_left),NULL,0,0,0,0,0,0,0,0,NULL,"Runtime Left comm: 1 six-step, 2 sine, 3 SVPWM"},
  {VARIABLE,"COMM_R",ADD_PARAM(m_comm_mode_right),NULL,0,0,0,0,0,0,0,0,NULL,"Runtime Right comm: 1 six-step, 2 sine, 3 SVPWM"},
  {VARIABLE,"CONTROL_L",ADD_PARAM(m_control_mode_sel_left),NULL,0,0,0,0,0,0,0,0,NULL,"Runtime Left control: 1 PWM, 2 current, 3 speed, 4 position"},
  {VARIABLE,"CONTROL_R",ADD_PARAM(m_control_mode_sel_right),NULL,0,0,0,0,0,0,0,0,NULL,"Runtime Right control: 1 PWM, 2 current, 3 speed"},
  {VARIABLE,"LIVE",ADD_PARAM(m_live_stream_enabled),NULL,0,0,0,0,0,0,0,0,NULL,"Binary telemetry stream: 1 on, 0 off"},
  {PARAMETER,"I_MOT_MAX",ADD_PARAM(m_mcconf_1.l_current_max),&m_mcconf_2.l_current_max,1,I_MOT_MAX,1,1,40,A2BIT_CONV,0,4,NULL,"Maximum phase current [A]"},
  {PARAMETER,"N_MOT_MAX",ADD_PARAM(m_mcconf_1.l_max_rpm),&m_mcconf_2.l_max_rpm,2,N_MOT_MAX,1,10,2000,0,0,4,NULL,"Maximum motor speed [rpm]"},

  /* Inner FOC current PI: generated fixed-point coefficients, independently tunable. */
  {PARAMETER,"IQ_KP_L",ADD_PARAM(m_mcconf_1.foc_current_kp_q),NULL,3,1229,0,0,32767,0,0,0,NULL,"Left FOC q-axis proportional coefficient raw fixed-point"},
  {PARAMETER,"IQ_KP_R",ADD_PARAM(m_mcconf_2.foc_current_kp_q),NULL,4,1229,0,0,32767,0,0,0,NULL,"Right FOC q-axis proportional coefficient raw fixed-point"},
  {PARAMETER,"IQ_KI_L",ADD_PARAM(m_mcconf_1.foc_current_ki_q),NULL,5,1229,0,0,32767,0,0,0,NULL,"Left FOC q-axis integral coefficient raw fixed-point"},
  {PARAMETER,"IQ_KI_R",ADD_PARAM(m_mcconf_2.foc_current_ki_q),NULL,6,1229,0,0,32767,0,0,0,NULL,"Right FOC q-axis integral coefficient raw fixed-point"},
  {PARAMETER,"ID_KP_L",ADD_PARAM(m_mcconf_1.foc_current_kp_d),NULL,7,819,0,0,32767,0,0,0,NULL,"Left FOC d-axis proportional coefficient raw fixed-point"},
  {PARAMETER,"ID_KP_R",ADD_PARAM(m_mcconf_2.foc_current_kp_d),NULL,8,819,0,0,32767,0,0,0,NULL,"Right FOC d-axis proportional coefficient raw fixed-point"},
  {PARAMETER,"ID_KI_L",ADD_PARAM(m_mcconf_1.foc_current_ki_d),NULL,9,737,0,0,32767,0,0,0,NULL,"Left FOC d-axis integral coefficient raw fixed-point"},
  {PARAMETER,"ID_KI_R",ADD_PARAM(m_mcconf_2.foc_current_ki_d),NULL,10,737,0,0,32767,0,0,0,NULL,"Right FOC d-axis integral coefficient raw fixed-point"},
  {PARAMETER,"CUR_FILT_L",ADD_PARAM(m_mcconf_1.foc_current_filter_const),NULL,11,7864,0,1,32767,0,0,0,NULL,"Left dq current low-pass coefficient"},
  {PARAMETER,"CUR_FILT_R",ADD_PARAM(m_mcconf_2.foc_current_filter_const),NULL,12,7864,0,1,32767,0,0,0,NULL,"Right dq current low-pass coefficient"},
  {PARAMETER,"KB_LIM_L",ADD_PARAM(m_mcconf_1.foc_current_anti_windup),NULL,13,768,0,0,32767,0,0,0,NULL,"Left FOC anti-windup back-calculation coefficient"},
  {PARAMETER,"KB_LIM_R",ADD_PARAM(m_mcconf_2.foc_current_anti_windup),NULL,14,768,0,0,32767,0,0,0,NULL,"Right FOC anti-windup back-calculation coefficient"},
  {PARAMETER,"IQ_ILIM_L",ADD_PARAM(m_mcconf_1.foc_current_i_limit),NULL,15,737,0,0,32767,0,0,0,NULL,"Left q-axis integrator limiting coefficient"},
  {PARAMETER,"IQ_ILIM_R",ADD_PARAM(m_mcconf_2.foc_current_i_limit),NULL,16,737,0,0,32767,0,0,0,NULL,"Right q-axis integrator limiting coefficient"},

  /* Outer speed PID: x1000 engineering coefficients. */
  {PARAMETER,"SPD_KP_L",ADD_PARAM(s_pid_kp_left_x1000),NULL,17,MCCONF_S_PID_KP_X1000,0,0,10000,0,0,0,mc_interface_reset_control,"Left speed Kp x1000 [command/rpm]"},
  {PARAMETER,"SPD_KI_L",ADD_PARAM(s_pid_ki_left_x1000),NULL,18,MCCONF_S_PID_KI_X1000,0,0,10000,0,0,0,mc_interface_reset_control,"Left speed Ki x1000"},
  {PARAMETER,"SPD_KD_L",ADD_PARAM(s_pid_kd_left_x1000),NULL,19,MCCONF_S_PID_KD_X1000,0,0,10000,0,0,0,mc_interface_reset_control,"Left speed Kd x1000"},
  {PARAMETER,"SPD_KP_R",ADD_PARAM(s_pid_kp_right_x1000),NULL,20,MCCONF_S_PID_KP_X1000,0,0,10000,0,0,0,mc_interface_reset_control,"Right speed Kp x1000"},
  {PARAMETER,"SPD_KI_R",ADD_PARAM(s_pid_ki_right_x1000),NULL,21,MCCONF_S_PID_KI_X1000,0,0,10000,0,0,0,mc_interface_reset_control,"Right speed Ki x1000"},
  {PARAMETER,"SPD_KD_R",ADD_PARAM(s_pid_kd_right_x1000),NULL,22,MCCONF_S_PID_KD_X1000,0,0,10000,0,0,0,mc_interface_reset_control,"Right speed Kd x1000"},
  {PARAMETER,"SPD_I_LIM",ADD_PARAM(s_pid_i_limit),NULL,23,MCCONF_S_PID_I_LIMIT,0,0,1000,0,0,0,mc_interface_reset_control,"Speed PID integral output limit"},
  {PARAMETER,"SPD_OUT_LIM",ADD_PARAM(s_pid_output_limit),NULL,24,MCCONF_S_PID_OUTPUT_LIMIT,0,1,1000,0,0,0,mc_interface_reset_control,"Speed PID torque-command limit"},

  /* Position -> speed -> torque cascade, Left encoder profile. */
  {PARAMETER,"POS_KP",ADD_PARAM(p_pid_kp_x1000),NULL,25,20,0,0,10000,0,0,0,mc_interface_reset_control,"Position Kp x1000 [rpm/count]"},
  {PARAMETER,"POS_KI",ADD_PARAM(p_pid_ki_x1000),NULL,26,0,0,0,10000,0,0,0,mc_interface_reset_control,"Position Ki x1000"},
  {PARAMETER,"POS_KD",ADD_PARAM(p_pid_kd_x1000),NULL,27,0,0,0,10000,0,0,0,mc_interface_reset_control,"Position Kd x1000"},
  {PARAMETER,"POS_I_LIM",ADD_PARAM(p_pid_i_limit_rpm),NULL,28,150,0,0,1000,0,0,0,mc_interface_reset_control,"Position integral contribution limit [rpm]"},
  {PARAMETER,"POS_SPD_LIM",ADD_PARAM(p_pid_speed_limit_rpm),NULL,29,250,0,1,1500,0,0,0,mc_interface_reset_control,"Position outer-loop speed limit [rpm]"},
  {PARAMETER,"POS_DEADBAND",ADD_PARAM(p_pid_deadband_counts),NULL,30,4,0,0,1000,0,0,0,NULL,"Position deadband [encoder counts]"},
  {PARAMETER,"POS_MIN",ADD_PARAM(p_pid_min_counts),NULL,31,-12000,0,-30000,30000,0,0,0,NULL,"Minimum allowed Left position [counts]"},
  {PARAMETER,"POS_MAX",ADD_PARAM(p_pid_max_counts),NULL,32,12000,0,-30000,30000,0,0,0,NULL,"Maximum allowed Left position [counts]"},
  {PARAMETER,"POS_TARGET",ADD_PARAM(p_pid_set_counts),NULL,33,0,0,-30000,30000,0,0,0,NULL,"Left position target [counts]"},

  {PARAMETER,"CMD_RATE",ADD_PARAM(m_command_rate),NULL,34,RATE,0,1,30000,0,0,0,NULL,"Host command rate limiter fixed-point step"},
  {PARAMETER,"CMD_FILTER",ADD_PARAM(m_command_filter),NULL,35,FILTER,0,1,32767,0,0,0,NULL,"Host command low-pass coefficient"},

  /* Encoder AB / alignment parameters. */
  {PARAMETER,"ENC_CPR",ADD_PARAM(m_encoder_counts),NULL,36,4096,0,64,30000,0,0,0,NULL,"Left quadrature counts/revolution (AB x4)"},
  {PARAMETER,"ENC_POLES",ADD_PARAM(m_encoder_pole_pairs),NULL,37,SVPWM_POLE_PAIRS,0,1,40,0,0,0,NULL,"Left motor pole pairs for encoder electrical angle"},
  {PARAMETER,"ENC_DIR",ADD_PARAM(m_encoder_direction),NULL,38,1,0,-1,1,0,0,0,NULL,"Encoder direction: +1 or -1"},
  {PARAMETER,"ENC_E_TRIM",ADD_PARAM(m_encoder_elec_trim_deg_x10),NULL,39,0,0,-1800,1800,0,0,0,NULL,"Electrical alignment trim [deg x10]"},
  {PARAMETER,"ENC_SYNC_CMD",ADD_PARAM(m_encoder_sync_command),NULL,40,80,0,10,250,0,0,0,NULL,"Boot open-loop sync command"},
  {PARAMETER,"ENC_SWEEP_MS",ADD_PARAM(m_encoder_sync_sweep_ms),NULL,41,1200,0,100,5000,0,0,0,NULL,"Boot encoder sweep duration [ms]"},
  {PARAMETER,"ENC_SETTLE_MS",ADD_PARAM(m_encoder_sync_settle_ms),NULL,42,350,0,50,3000,0,0,0,NULL,"Electrical-zero hold settling time [ms]"},
  {PARAMETER,"ENC_RET_RPM",ADD_PARAM(m_encoder_return_rpm),NULL,43,80,0,10,500,0,0,0,NULL,"Return-to-start maximum rpm after alignment"},
  {PARAMETER,"ENC_RET_TOL",ADD_PARAM(m_encoder_return_tolerance_counts),NULL,44,8,0,1,1000,0,0,0,NULL,"Return-to-start tolerance [counts]"},

  {PARAMETER,"SPD_D_FILT",ADD_PARAM(s_pid_d_filter_x1000),NULL,45,850,0,0,1000,0,0,0,mc_interface_reset_control,"Speed/position derivative low-pass coefficient x1000"},
  {PARAMETER,"FI_WEAK_ENA",ADD_PARAM(m_mcconf_1.foc_fw_enable),&m_mcconf_2.foc_fw_enable,46,FIELD_WEAK_ENA,0,0,1,0,0,0,NULL,"Enable field weakening"},
  {PARAMETER,"FI_WEAK_HI",ADD_PARAM(m_mcconf_1.foc_fw_rpm_start),&m_mcconf_2.foc_fw_rpm_start,47,FIELD_WEAK_HI,1,0,2000,0,0,4,Input_Lim_Init,"Field weakening high rpm"},
  {PARAMETER,"FI_WEAK_LO",ADD_PARAM(m_mcconf_1.foc_fw_rpm_end),&m_mcconf_2.foc_fw_rpm_end,48,FIELD_WEAK_LO,1,0,2000,0,0,4,Input_Lim_Init,"Field weakening low rpm"},
  {PARAMETER,"FI_WEAK_MAX",ADD_PARAM(m_mcconf_1.foc_fw_current_max),&m_mcconf_2.foc_fw_current_max,49,4000,0,0,30000,0,0,0,NULL,"Maximum field-weakening d current raw generated units"},
  {PARAMETER,"PHA_ADV_MAX",ADD_PARAM(m_mcconf_1.foc_fw_phase_advance_max),&m_mcconf_2.foc_fw_phase_advance_max,50,25,1,0,60,0,0,4,NULL,"Maximum phase advance [degree]"},
  {PARAMETER,"GEN_SPD_KP_L",ADD_PARAM(m_mcconf_1.s_pid_kp),NULL,51,4833,0,0,32767,0,0,0,NULL,"Generated controller speed Kp raw fixed-point"},
  {PARAMETER,"GEN_SPD_KP_R",ADD_PARAM(m_mcconf_2.s_pid_kp),NULL,52,4833,0,0,32767,0,0,0,NULL,"Generated controller speed Kp raw fixed-point"},
  {PARAMETER,"GEN_SPD_KI_L",ADD_PARAM(m_mcconf_1.s_pid_ki),NULL,53,251,0,0,32767,0,0,0,NULL,"Generated controller speed Ki raw fixed-point"},
  {PARAMETER,"GEN_SPD_KI_R",ADD_PARAM(m_mcconf_2.s_pid_ki),NULL,54,251,0,0,32767,0,0,0,NULL,"Generated controller speed Ki raw fixed-point"},
  {PARAMETER,"GEN_SPD_ILIM_L",ADD_PARAM(m_mcconf_1.s_pid_i_limit),NULL,55,246,0,0,32767,0,0,0,NULL,"Generated controller speed integrator limiting coefficient"},
  {PARAMETER,"GEN_SPD_ILIM_R",ADD_PARAM(m_mcconf_2.s_pid_i_limit),NULL,56,246,0,0,32767,0,0,0,NULL,"Generated controller speed integrator limiting coefficient"},
  {PARAMETER,"COMM_LO_L",ADD_PARAM(m_mcconf_1.foc_comm_rpm_low),NULL,57,15,1,0,1000,0,0,4,NULL,"Left commutation activation speed [rpm]"},
  {PARAMETER,"COMM_LO_R",ADD_PARAM(m_mcconf_2.foc_comm_rpm_low),NULL,58,15,1,0,1000,0,0,4,NULL,"Right commutation activation speed [rpm]"},
  {PARAMETER,"COMM_HI_L",ADD_PARAM(m_mcconf_1.foc_comm_rpm_high),NULL,59,30,1,0,1500,0,0,4,NULL,"Left commutation deactivation speed [rpm]"},
  {PARAMETER,"COMM_HI_R",ADD_PARAM(m_mcconf_2.foc_comm_rpm_high),NULL,60,30,1,0,1500,0,0,4,NULL,"Right commutation deactivation speed [rpm]"},

  /* Inputs / runtime feedback */
  {VARIABLE,"CMDL_RAW",ADD_PARAM(input1[0].raw),NULL,0,0,0,RAW_MIN,RAW_MAX,0,0,0,NULL,"Left raw command"},
  {VARIABLE,"CMDL_IN",ADD_PARAM(input1[0].cmd),NULL,0,0,0,0,0,0,0,0,NULL,"Left requested command"},
  {VARIABLE,"CMDR_RAW",ADD_PARAM(input2[0].raw),NULL,0,0,0,RAW_MIN,RAW_MAX,0,0,0,NULL,"Right raw command"},
  {VARIABLE,"CMDR_IN",ADD_PARAM(input2[0].cmd),NULL,0,0,0,0,0,0,0,0,NULL,"Right requested command"},
  {VARIABLE,"CMDL",ADD_PARAM(cmdL),NULL,0,0,0,0,0,0,0,0,NULL,"Left applied command"},
  {VARIABLE,"CMDR",ADD_PARAM(cmdR),NULL,0,0,0,0,0,0,0,0,NULL,"Right applied command"},
  {VARIABLE,"SPDL",ADD_PARAM(m_motor_1.m_output.rpm),NULL,0,0,0,0,0,0,0,0,NULL,"Left measured rpm"},
  {VARIABLE,"SPDR",ADD_PARAM(m_motor_2.m_output.rpm),NULL,0,0,0,0,0,0,0,0,NULL,"Right internal measured rpm"},
  {VARIABLE,"ENC_POS",ADD_PARAM(m_encoder_position),NULL,0,0,0,0,0,0,0,0,NULL,"Left encoder position counts"},
  {VARIABLE,"ENC_RPM",ADD_PARAM(m_encoder_rpm),NULL,0,0,0,0,0,0,0,0,NULL,"Left encoder speed rpm"},
  {VARIABLE,"POS_SPD_REF",ADD_PARAM(m_position_speed_set),NULL,0,0,0,0,0,0,0,0,NULL,"Position outer-loop speed target rpm"},
  {VARIABLE,"ENC_E_ANGLE",ADD_PARAM(m_encoder_elec_angle_deg_x10),NULL,0,0,0,0,0,0,0,0,NULL,"Encoder electrical angle deg x10"},
  {VARIABLE,"ENC_SYNC",ADD_PARAM(m_encoder_sync_state),NULL,0,0,0,0,0,0,0,0,NULL,"Encoder sync state"},
  {VARIABLE,"HALL_L",ADD_PARAM(m_sensor_hall_left),NULL,0,0,0,0,0,0,0,0,NULL,"Left Hall code; synthetic from encoder in enc_hall"},
  {VARIABLE,"HALL_R",ADD_PARAM(m_sensor_hall_right),NULL,0,0,0,0,0,0,0,0,NULL,"Right physical Hall code"},
  {VARIABLE,"SENS_RPM_L",ADD_PARAM(m_sensor_rpm_left),NULL,0,0,0,0,0,0,0,0,NULL,"Left mode-independent sensor rpm"},
  {VARIABLE,"SENS_RPM_R",ADD_PARAM(m_sensor_rpm_right),NULL,0,0,0,0,0,0,0,0,NULL,"Right internal mode-independent sensor rpm"},
  {VARIABLE,"SPD_SET_L",ADD_PARAM(m_speed_set_left_rpm),NULL,0,0,0,0,0,0,0,0,NULL,"Left speed-loop target [mechanical rpm]"},
  {VARIABLE,"SPD_SET_R",ADD_PARAM(m_speed_set_right_rpm),NULL,0,0,0,0,0,0,0,0,NULL,"Right speed-loop target [mechanical rpm]"},
  {VARIABLE,"SPD_ERR_L",ADD_PARAM(m_speed_error_left_rpm),NULL,0,0,0,0,0,0,0,0,NULL,"Left speed-loop error target-rpm [rpm]"},
  {VARIABLE,"SPD_ERR_R",ADD_PARAM(m_speed_error_right_rpm),NULL,0,0,0,0,0,0,0,0,NULL,"Right speed-loop error target-rpm [rpm]"},
  {VARIABLE,"SPD_OUT_L",ADD_PARAM(m_speed_pid_output_left),NULL,0,0,0,0,0,0,0,0,NULL,"Left speed PID torque/current command [-1000..1000]"},
  {VARIABLE,"SPD_OUT_R",ADD_PARAM(m_speed_pid_output_right),NULL,0,0,0,0,0,0,0,0,NULL,"Right speed PID host-sign torque/current command [-1000..1000]"},
  {VARIABLE,"ADC_PA2",ADD_PARAM(m_adc_buffer.adc2_spare4),NULL,0,0,0,0,0,0,0,0,NULL,"Raw ADC2 PA2 sample"},
  {VARIABLE,"ADC_PA3",ADD_PARAM(m_adc_buffer.adc2_spare5),NULL,0,0,0,0,0,0,0,0,NULL,"Raw ADC2 PA3 sample"},
  {VARIABLE,"ADC_I_VALID",ADD_PARAM(m_adc_current_valid),NULL,0,0,0,0,0,0,0,0,NULL,"1 when phase-current ADC is sampled with bridge in defined PWM state"},
  {VARIABLE,"ADC_I_VALID_L",ADD_PARAM(m_adc_current_valid_left),NULL,0,0,0,0,0,0,0,0,NULL,"Left phase-current ADC sample valid"},
  {VARIABLE,"ADC_I_VALID_R",ADD_PARAM(m_adc_current_valid_right),NULL,0,0,0,0,0,0,0,0,NULL,"Right phase-current ADC sample valid"},
  {VARIABLE,"FOC_ISR_CYC",ADD_PARAM(m_foc_isr_cycles),NULL,0,0,0,0,0,0,0,0,NULL,"Last FOC ISR cycles"},
  {VARIABLE,"FOC_ISR_MAX",ADD_PARAM(m_foc_isr_cycles_max),NULL,0,0,0,0,0,0,0,0,NULL,"Maximum FOC ISR cycles"},
};

debug_command command;
int8_t watchParamList[MAX_PARAM_WATCH] = {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1}; 

// Set Param with Value from external format
int8_t setParamValExt(uint8_t index, int32_t value) {   
  int8_t ret = 0;
  // check min and max before conversion to internal values
  if (IN_RANGE(value,params[index].min,params[index].max)){
    ret = setParamValInt(index,extToInt(index,value));
    printParamDef(index);
  }else{
    printError(4); // Error - Value out of range
  }
  return ret;
}

// Set Param with value from internal format
int8_t setParamValInt(uint8_t index, int32_t newValue) {
  int32_t oldValue = getParamValInt(index);
  if (oldValue != newValue){ 
    // if value is different, beep, cast and assign new value
    switch (params[index].datatype){
      case UINT8_T:
        if (params[index].valueL != NULL) *(volatile uint8_t*)params[index].valueL = newValue;
        if (params[index].valueR != NULL) *(volatile uint8_t*)params[index].valueR = newValue;
        break;
      case UINT16_T:
        if (params[index].valueL != NULL) *(volatile uint16_t*)params[index].valueL = newValue; 
        if (params[index].valueR != NULL) *(volatile uint16_t*)params[index].valueR = newValue;
        break;
      case UINT32_T:
        if (params[index].valueL != NULL) *(volatile uint32_t*)params[index].valueL = newValue; 
        if (params[index].valueR != NULL) *(volatile uint32_t*)params[index].valueR = newValue;
        break;
      case INT8_T:
        if (params[index].valueL != NULL) *(volatile int8_t*)params[index].valueL = newValue; 
        if (params[index].valueR != NULL) *(volatile int8_t*)params[index].valueR = newValue;
        break;
      case INT16_T:
        if (params[index].valueL != NULL) *(volatile int16_t*)params[index].valueL = newValue; 
        if (params[index].valueR != NULL) *(volatile int16_t*)params[index].valueR = newValue;
        break;
      case INT32_T:
        if (params[index].valueL != NULL) *(volatile int32_t*)params[index].valueL = newValue; 
        if (params[index].valueR != NULL) *(volatile int32_t*)params[index].valueR = newValue;
        break;
    }

    // Beep only for interactive runtime edits, never while boot-loading EEPROM.
    if (!paramSilent) beepShort(5);
  }

  // Run callback function if assigned
  if (params[index].callback_function) (*params[index].callback_function)();
  return 1;
}

// Get Parameter Internal value and translate to external 
int32_t getParamValExt(uint8_t index) {
  return intToExt(index,getParamValInt(index));
}

// Get Parameter Internal Value
int32_t getParamValInt(uint8_t index) {
  int32_t value = 0;

  int8_t countVar = 0;
  if (params[index].valueL != NULL) countVar++;
  if (params[index].valueR != NULL) countVar++;

  if (countVar > 0){
    // Read Left and Right values and calculate average 
    // If left and right have to be summed up, DIV field could be adapted to multiply by 2
    // Cast to parameter datatype
    switch (params[index].datatype){
      case UINT8_T:
        if (params[index].valueL != NULL) value += *(volatile uint8_t*)params[index].valueL;
        if (params[index].valueR != NULL) value += *(volatile uint8_t*)params[index].valueR;
        break;
      case UINT16_T:
        if (params[index].valueL != NULL) value += *(volatile uint16_t*)params[index].valueL;
        if (params[index].valueR != NULL) value += *(volatile uint16_t*)params[index].valueR;
        break;
      case UINT32_T:
        if (params[index].valueL != NULL) value += *(volatile uint32_t*)params[index].valueL;
        if (params[index].valueR != NULL) value += *(volatile uint32_t*)params[index].valueR;
        break;
      case INT8_T:
        if (params[index].valueL != NULL) value += *(volatile int8_t*)params[index].valueL;
        if (params[index].valueR != NULL) value += *(volatile int8_t*)params[index].valueR;
        break;
      case INT16_T:
        if (params[index].valueL != NULL) value += *(volatile int16_t*)params[index].valueL;
        if (params[index].valueR != NULL) value += *(volatile int16_t*)params[index].valueR;
        break;
      case INT32_T:
        if (params[index].valueL != NULL) value += *(volatile int32_t*)params[index].valueL;
        if (params[index].valueR != NULL) value += *(volatile int32_t*)params[index].valueR;
        break;
      default:
        value = 0;
    }

    // Divide by number of values provided for the parameter
    value /= countVar;
  }else{
    // No variable was provided, return init value that might contain a macro
    value = params[index].init;
  }

  return value;
}

// Add or remove parameter from watch list
int8_t watchParamVal(uint8_t index){
  int8_t i,found = 0;
  for(i=0;i < MAX_PARAM_WATCH && watchParamList[i]>-1;i++){
    if (watchParamList[i] == index) found = 1;
    if (found) watchParamList[i] = (i < MAX_PARAM_WATCH-1)?watchParamList[i+1]:-1;
  }
  if (!found){
    if (watchParamList[i] == -1){
      watchParamList[i] = index;
      return 1;
    }
    printError(10);
    return 0;
  }
  return 1;
}

// Print value for all parameters with watch flag
int8_t printParamVal(){
  int8_t i = 0; 
  for(i=0;i < MAX_PARAM_WATCH && watchParamList[i]>-1;i++){
    printf("%s:%li ", params[watchParamList[i]].name, (long)getParamValExt(watchParamList[i]));
  }
  if (i>0) printf("\r\n");
  return 1;
}

// Print help for Command
int8_t printCommandHelp(uint8_t index){
  printf("? %s:\"%s\"\r\n",commands[index].name,commands[index].help);
  return 1;
}

// Print help for parameter
int8_t printParamHelp(uint8_t index){
  printf("? %s:\"%s\" ",params[index].name,params[index].help);
  if (params[index].type == PARAMETER) printf("[min:%li max:%li]", (long)params[index].min, (long)params[index].max);
  printf("\r\n");
  return 1;
}

// Print help for all parameters
int8_t printAllParamHelp(){
  printf("? Commands\r\n");
  for (size_t i = 0; i < COMMAND_SIZE(commands); i++)
    printCommandHelp((uint8_t)i);
  printf("?\r\n");

  printf("? Parameters\r\n");
  for (size_t i = 0; i < PARAM_SIZE(params); i++){
    if (params[i].type == PARAMETER) printParamHelp((uint8_t)i);
  }
  printf("?\r\n");

  printf("? Variables\r\n");
  for (size_t i = 0; i < PARAM_SIZE(params); i++){
    if (params[i].type == VARIABLE) printParamHelp((uint8_t)i);
  }
  printf("?\r\n");

  return 1;
}

// Print definition(name,value,initial value, min, max) for parameter
int8_t printParamDef(uint8_t index){
  printf("# name:\"%s\" value:%li init:%li min:%li max:%li help:\"%s\"\r\n",
         params[index].name,
         (long)getParamValExt(index),
         (long)getParamInitExt(index),
         (long)params[index].min,
         (long)params[index].max,
         params[index].help ? params[index].help : "");
  return 1;
}

// Print definition(name,value,initial value, min, max) for all parameters
int8_t printAllParamDef(){
  for (size_t i = 0; i < PARAM_SIZE(params); i++) printParamDef((uint8_t)i);
  return 1;
}

void printError(uint8_t errornum) {
  const uint32_t errorCount = (uint32_t)(sizeof(errors) / sizeof(errors[0]));
  if ((errornum == 0u) || ((uint32_t)errornum > errorCount)) {
    printf("! Err%u:\"Unknown error\"\r\n", (unsigned)errornum);
    return;
  }
  printf("! Err%u:\"%s\"\r\n", (unsigned)errornum, errors[errornum - 1u]);
}

// Get internal Parameter value and save it to EEprom for all paraemeter with an address assigned 
int8_t saveAllParamVal() {
  if (input1[0].cmd != 0 || input2[0].cmd != 0 || cmdL != 0 || cmdR != 0 ||
      abs(m_motor_1.m_output.rpm) > 5 || abs(m_motor_2.m_output.rpm) > 5) {
    printf("! SAVE requires STOP and |rpm|<=5; values remain in RAM\r\n");
    return 0;
  }
  HAL_FLASH_Unlock();
  /* Transaction marker invalid first. If power fails during write, next boot
   * falls back to compiled defaults instead of accepting a partial parameter set. */
  EE_WriteVariable(VirtAddVarTab[0], 0u);
  for (size_t i = 0; i < PARAM_SIZE(params); ++i) {
    if (params[i].addr > 0u && params[i].addr < NB_OF_VAR)
      EE_WriteVariable(VirtAddVarTab[params[i].addr], (uint16_t)getParamValInt((uint8_t)i));
  }
  EE_WriteVariable(VirtAddVarTab[0], (uint16_t)FLASH_WRITE_KEY);
  HAL_FLASH_Lock();
  printf("# EEPROM SAVED\r\n");
  return 1;
}

void loadAllParamVal(void) {
  uint16_t key = 0u;
  paramSilent = 1u;
  HAL_FLASH_Unlock();
  const uint16_t keyStatus = EE_ReadVariable(VirtAddVarTab[0], &key);
  HAL_FLASH_Lock();
  if (keyStatus != 0u || key != FLASH_WRITE_KEY) {
    paramSilent = 0u;
    printf("# EEPROM not valid; using compiled defaults\r\n");
    return;
  }
  for (size_t i = 0; i < PARAM_SIZE(params); ++i) {
    if (params[i].addr > 0u && params[i].addr < NB_OF_VAR) {
      uint16_t v = 0u;
      HAL_FLASH_Unlock();
      const uint16_t st = EE_ReadVariable(VirtAddVarTab[params[i].addr], &v);
      HAL_FLASH_Lock();
      if (st == 0u) setParamValInt((uint8_t)i, (int16_t)v);
    }
  }
  mc_interface_reset_control();
  paramSilent = 0u;
  printf("# EEPROM loaded to RAM\r\n");
}

// Translate from Internal to External format
int32_t intToExt(uint8_t index,int32_t value){
  // Multiply for small number
  if(params[index].mul) value *= params[index].mul;
  // Divide to translate to external format
  if(params[index].div) value /= params[index].div;
  // Shift to translate to external format
  if(params[index].fix) value >>= params[index].fix;
  return value;
}

// Translate from External to Internal Format
int32_t extToInt(uint8_t index,int32_t value){
  // Multiply to translate to internal format
  if(params[index].div) value *= params[index].div;
  // Shift to translate to internal format
  if (params[index].fix) value <<= params[index].fix;
  // Divide for small number
  if(params[index].mul) value /= params[index].mul;
  return value;
}

// Get Parameter Init value(EEPROM or init/config.h) and translate to external format
int32_t getParamInitExt(uint8_t index) {
  return intToExt(index,getParamInitInt(index));
}

// Get Parameter value with EEprom data if address is avalaible, init/config.h value otherwise
int16_t getParamInitInt(uint8_t index){
  if (params[index].addr){
    // if EEPROM address is specified, init from EEPROM address
    uint16_t writeCheck, readVal;
    
    HAL_FLASH_Unlock();
    EE_ReadVariable(VirtAddVarTab[0], &writeCheck);
    EE_ReadVariable(VirtAddVarTab[params[index].addr] , &readVal);
    HAL_FLASH_Lock();
    
    // EEPROM was written, use stored value
    if (writeCheck == FLASH_WRITE_KEY){
      return readVal;
    }else{
      // Use init value from array
      if (params[index].initFormat){
        // Init Value is in External format (e.g. PHA_ADV_MAX is 25 deg)
        return extToInt(index,params[index].init);
      }else{
        return params[index].init;
      }
    }
  }else{
    if (params[index].initFormat){
      // Init Value is in External format (e.g. PHA_ADV_MAX is 25 deg)
      return extToInt(index,params[index].init);
    }else{
      return params[index].init;
    }
  }
}


// initialize Parameter value with EEprom data if address is avalaible, init/config.h value otherwise
int8_t initParamVal(uint8_t index) {
  int8_t ret = 0;
  ret = setParamValInt(index,(int32_t) getParamInitInt(index));
  printParamDef(index);
  return ret;
}

// Find command in commands array and return index
int8_t findCommand(uint8_t *userCommand, uint32_t len){
  for (size_t index = 0; index < COMMAND_SIZE(commands); index++){
    uint8_t command_len = strlen(commands[index].name);
    if (command_len < len){
      if (memcmp(userCommand,commands[index].name,command_len)==0){
        return (int8_t)index;
      }
    }
  }
  return -1; // Not found
}

// Find parameter in params array and return index
int8_t findParam(uint8_t *userCommand, uint32_t len){
  for (size_t index = 0; index < PARAM_SIZE(params); index++){
    uint8_t param_len = strlen(params[index].name);
    if (param_len < len){
      if (memcmp(userCommand,params[index].name,param_len)==0){
        return (int8_t)index;
      }
    }
  }
  return -1; // Not found
}

static uint8_t starts_with_ci(const uint8_t *s, uint32_t len, const char *prefix) {
  uint32_t i = 0u;
  while (prefix[i] != 0) {
    if (i >= len) return 0u;
    uint8_t c = s[i];
    if (c >= 'A' && c <= 'Z') c = (uint8_t)(c - 'A' + 'a');
    if (c != (uint8_t)prefix[i]) return 0u;
    ++i;
  }
  return 1u;
}

static uint8_t motor_is_stopped_for_mode_change(void) {
  return (uint8_t)(input1[0].cmd == 0 && input2[0].cmd == 0 && cmdL == 0 && cmdR == 0 &&
                   abs(m_sensor_rpm_left) <= 5 && abs(m_sensor_rpm_right) <= 5);
}

// Parse and save the command to be executed
void handle_input(uint8_t *userCommand, uint32_t len)
{
  /* MODE/LIVE may arrive from USART IRQ context. Queue them and execute only
   * from process_debug() in the main loop so sensor re-init never runs inside
   * the UART interrupt. */
  if (starts_with_ci(userCommand, len, "mode ") || starts_with_ci(userCommand, len, "live ")) {
    if (!m_special_command_pending) {
      uint32_t n = len;
      if (n >= sizeof(m_special_command)) n = sizeof(m_special_command) - 1u;
      memcpy(m_special_command, userCommand, n);
      m_special_command[n] = 0;
      m_special_command_pending = 1u;
    }
    return;
  }

  // If there is already an unprocessed command, exit
  if (command.semaphore == 1) return;

  // Check end of line
  userCommand+=len-1; // Go to last char
  if (*userCommand != '\n' && *userCommand != '\r'){
    command.error = 7; // Error - End of line expected
    return;
  }
  userCommand-=len-1; // Come back

  int8_t  cindex = -1;
  int8_t  pindex = -1;
  uint8_t size   = 0;

  // Find Command
  cindex = findCommand(userCommand,len);
  if (cindex == -1){
    // Error - Command not found
    command.error = 1;
    return;
  }

  // Skip command characters
  size = strlen(commands[cindex].name);
  {len-=size;userCommand+=size;}
  // Skip if space
  if (*userCommand == 0x20){len-=1;userCommand+=1;}

  if (*userCommand == '\n' || *userCommand == '\r'){
    if (commands[cindex].callback_function0 != NULL){
      // Command without parameter
      command.semaphore = 1;
      command.command_index = cindex;
      command.param_index   = -1;
      command.param_value   = 0;
    }else{
      command.error = 8; // Error - Parameter expected
    }
    return;
  }

  // Find parameter
  pindex = findParam(userCommand,len);
  if (pindex == -1){
    // Error - Parameter not found
    command.error = 2;
    return;
  }

  // Skip parameter characters
  size = strlen(params[pindex].name);
  {len-=size;userCommand+=size;}
  // Skip if space
  if (*userCommand == 0x20){len-=1;userCommand+=1;}
   
  if (commands[cindex].type == WRITE && params[pindex].type == VARIABLE){
    // Error - This command cannot be used with a Variable
    command.error = 3;
    return;
  }
  
  if (commands[cindex].callback_function1 != NULL){
    if (*userCommand == '\n' || *userCommand == '\r'){
      // Command with parameter
      command.semaphore = 1;
      command.command_index = cindex;
      command.param_index   = pindex;
      command.param_value   = 0;
    }else{
      command.error = 7; // Error - End of line expected
    }
    return;
  }
  
  int32_t value = 0;
  int8_t  sign  = 1;
  int8_t  count = 0;

  // Read sign
  if (*userCommand == '-'){len-=1;userCommand+=1;sign =-1;} 
  // Read value
  for (value=0; (unsigned)*userCommand-'0'<10; userCommand++){
    value = 10*value+(*userCommand-'0');
    count++;
    // Error - Value out of range
    if (value>INT16_MAX){command.error = 4;return;}
  }

  if (count == 0){
    // Error - Value required
    command.error = 5;
    return;
  }
      
  // Apply sign
  value*= sign;

  // Command with parameter and value
  if (commands[cindex].callback_function2 != NULL){
    if (*userCommand == '\n' || *userCommand == '\r'){
      command.semaphore = 1;
      command.command_index = cindex;
      command.param_index   = pindex;
      command.param_value   = value;
    }else{
      command.error = 7; // Error - End of line expected
    }
    return;
  }

  // Uncaught error
  command.error = 9;

}

void process_debug()
{
  if (m_special_command_pending) {
    char local[sizeof(m_special_command)];
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    memcpy(local, m_special_command, sizeof(local));
    m_special_command_pending = 0u;
    if (!primask) __enable_irq();

    if ((local[0] == 'm' || local[0] == 'M') && !motor_is_stopped_for_mode_change()) {
      printf("! MODE requires STOP and |rpm|<=5\r\n");
    } else {
      (void)mc_interface_debug_command(local);
    }
  }
  
  // Print parameters from watch list
  printParamVal();

  // Show Error if any
  if(command.error> 0){
    printError(command.error);
    command.error = 0;
    return;
  }

  // Nothing to do
  if (command.semaphore == 0) return;

  int8_t ret = 0;
  if (commands[command.command_index].callback_function0 != NULL && 
      command.param_index == -1){
    // This function needs no parameter
    ret = (*commands[command.command_index].callback_function0)();
    if (ret==1){printf("OK\r\n");}
    command.semaphore = 0;
    return;
  }

  if (commands[command.command_index].callback_function1 != NULL &&
      command.param_index != -1){
    // This function needs only a parameter
    ret = (*commands[command.command_index].callback_function1)(command.param_index);
    if (ret==1){printf("OK\r\n");}
    command.semaphore = 0;
    return;
  }  

  if (commands[command.command_index].callback_function2 != NULL && 
      command.param_index != -1){
    // This function needs an additional parameter
    ret = (*commands[command.command_index].callback_function2)(command.param_index,command.param_value);
    if (ret==1){printf("OK\r\n");}
    command.semaphore = 0;
  }
}

