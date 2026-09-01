/** Bare-metal command/diagnostic interface with VESC-style motor variables. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f1xx_hal.h"
#include "config.h"
#include "defines.h"
#include "eeprom.h"
#include "util.h"
#include "comms.h"
#include "motor/mcpwm_foc.h"
#include "motor/mc_interface.h"

#define RAW_MIN -1500
#define RAW_MAX 1500
#define MAX_PARAM_WATCH 15

extern InputStruct input1[];
extern InputStruct input2[];
extern uint16_t VirtAddVarTab[NB_OF_VAR];
extern int16_t speedAvg;
extern int16_t speedAvgAbs;
extern uint8_t ctrlModReqRaw;
extern int16_t batVoltageCalib;
extern int16_t board_temp_deg_c;
extern int16_t left_dc_curr;
extern int16_t right_dc_curr;
extern int16_t dc_curr;
extern int16_t cmdL;
extern int16_t cmdR;
extern volatile uint32_t foc_isr_cycles;
extern volatile uint32_t foc_isr_cycles_max;
extern volatile uint8_t motorRunReq;
extern volatile uint16_t svpwmOpenloopRpm;

static int16_t vescCurrentMaxA = I_MOT_MAX;
static void applyVescCurrentMax(void) {
  m_motor_1.m_conf.l_current_max = m_motor_2.m_conf.l_current_max = (float)vescCurrentMaxA;
  m_motor_1.m_conf.l_current_min = m_motor_2.m_conf.l_current_min = -(float)vescCurrentMaxA;
}

enum commandTypes {READ,WRITE};
const command_entry commands[] = {
    {READ,"GET",printAllParamDef,printParamDef,NULL,"Get Parameter/Variable"},
    {READ,"HELP",printAllParamHelp,printParamHelp,NULL,"Command/Parameter/Variable Help"},
    {READ,"WATCH",NULL,watchParamVal,NULL,"Toggle Parameter/Variable Watch"},
    {WRITE,"SET",NULL,NULL,setParamValExt,"Set Parameter"},
    {WRITE,"INIT",NULL,initParamVal,NULL,"Init Parameter from EEPROM or CONFIG.H"},
    {WRITE,"SAVE",saveAllParamVal,NULL,NULL,"Save Parameters to EEPROM"},
};

enum paramTypes {PARAMETER,VARIABLE};
const parameter_entry params[] = {
    {PARAMETER,"CTRL_MOD",ADD_PARAM(ctrlModReqRaw),NULL,0,CTRL_MOD_REQ,0,1,4,0,0,0,NULL,"1 duty/VLT 2 speed 3 current-Iq(cA) 4 sensorless openloop Id(A)"},
    {PARAMETER,"MOT_RUN",ADD_PARAM(motorRunReq),NULL,0,1,0,0,1,0,0,0,NULL,"1 run, 0 controlled current brake then neutral"},
    {PARAMETER,"SVPWM_RPM",ADD_PARAM(svpwmOpenloopRpm),NULL,0,SVPWM_OPENLOOP_RPM_DEFAULT,0,1,SVPWM_OPENLOOP_RPM_MAX,0,0,0,NULL,"Mode4 mechanical open-loop RPM"},
    {PARAMETER,"L_CURRENT_MAX",ADD_PARAM(vescCurrentMaxA),NULL,1,I_MOT_MAX,1,1,I_MOT_MAX,0,0,0,applyVescCurrentMax,"VESC l_current_max ampere; hardware fixed-point ceiling 15 A"},

    {VARIABLE,"CMDL_RAW",ADD_PARAM(input1[0].raw),NULL,0,0,0,RAW_MIN,RAW_MAX,0,0,0,NULL,"Left raw command"},
    {VARIABLE,"CMDL_IN",ADD_PARAM(input1[0].cmd),NULL,0,0,0,0,0,0,0,0,NULL,"Left requested command"},
    {VARIABLE,"CMDR_RAW",ADD_PARAM(input2[0].raw),NULL,0,0,0,RAW_MIN,RAW_MAX,0,0,0,NULL,"Right raw command"},
    {VARIABLE,"CMDR_IN",ADD_PARAM(input2[0].cmd),NULL,0,0,0,0,0,0,0,0,NULL,"Right requested command"},
    {VARIABLE,"CMDL",ADD_PARAM(cmdL),NULL,0,0,0,0,0,0,0,0,NULL,"Left applied command"},
    {VARIABLE,"CMDR",ADD_PARAM(cmdR),NULL,0,0,0,0,0,0,0,0,NULL,"Right applied command"},
    {VARIABLE,"SPDL",ADD_PARAM(m_motor_1.m_rpm),NULL,0,0,0,0,0,0,0,0,NULL,"Motor1 mechanical RPM (legacy/debug)"},
    {VARIABLE,"SPDR",ADD_PARAM(m_motor_2.m_rpm),NULL,0,0,0,0,0,0,0,0,NULL,"Motor2 internal mechanical RPM (mirrored sign)"},
    {VARIABLE,"IQ_REF_L",ADD_PARAM(m_motor_1.m_iq_set_q4),NULL,0,0,0,0,0,A2BIT_CONV,100,4,NULL,"Active slewed Iq reference centiampere"},
    {VARIABLE,"IQ_TGT_L",ADD_PARAM(m_motor_1.m_iq_target_q4),NULL,0,0,0,0,0,A2BIT_CONV,100,4,NULL,"Requested Iq target centiampere"},
    {VARIABLE,"IQ_REF_R",ADD_PARAM(m_motor_2.m_iq_set_q4),NULL,0,0,0,0,0,A2BIT_CONV,100,4,NULL,"Right active Iq reference internal sign"},
    {VARIABLE,"IQ_TGT_R",ADD_PARAM(m_motor_2.m_iq_target_q4),NULL,0,0,0,0,0,A2BIT_CONV,100,4,NULL,"Right requested Iq target internal sign"},
    {VARIABLE,"IQ_L",ADD_PARAM(m_motor_1.m_iq_q4),NULL,0,0,0,0,0,A2BIT_CONV,100,4,NULL,"VESC iq motor1 centiampere"},
    {VARIABLE,"IQ_R",ADD_PARAM(m_motor_2.m_iq_q4),NULL,0,0,0,0,0,A2BIT_CONV,100,4,NULL,"VESC iq motor2 centiampere internal sign"},
    {VARIABLE,"ID_L",ADD_PARAM(m_motor_1.m_id_q4),NULL,0,0,0,0,0,A2BIT_CONV,100,4,NULL,"VESC id motor1 centiampere"},
    {VARIABLE,"ID_R",ADD_PARAM(m_motor_2.m_id_q4),NULL,0,0,0,0,0,A2BIT_CONV,100,4,NULL,"VESC id motor2 centiampere"},
    {VARIABLE,"VD_L",ADD_PARAM(m_motor_1.m_vd),NULL,0,0,0,0,0,0,0,0,NULL,"FOC vd fixed voltage unit"},
    {VARIABLE,"VQ_L",ADD_PARAM(m_motor_1.m_vq),NULL,0,0,0,0,0,0,0,0,NULL,"FOC vq fixed voltage unit"},
    {VARIABLE,"VD_R",ADD_PARAM(m_motor_2.m_vd),NULL,0,0,0,0,0,0,0,0,NULL,"FOC vd fixed voltage unit"},
    {VARIABLE,"VQ_R",ADD_PARAM(m_motor_2.m_vq),NULL,0,0,0,0,0,0,0,0,NULL,"FOC vq fixed voltage unit"},
    {VARIABLE,"DUTY_L",ADD_PARAM(m_motor_1.m_duty_now_permille),NULL,0,0,0,0,0,0,0,0,NULL,"VESC duty_now x1000"},
    {VARIABLE,"DUTY_R",ADD_PARAM(m_motor_2.m_duty_now_permille),NULL,0,0,0,0,0,0,0,0,NULL,"VESC duty_now x1000"},
    {VARIABLE,"DC_CURR",ADD_PARAM(dc_curr),NULL,0,0,0,0,0,0,0,0,NULL,"Total DC current centiampere"},
    {VARIABLE,"RDC_CURR",ADD_PARAM(right_dc_curr),NULL,0,0,0,0,0,0,0,0,NULL,"Right DC current centiampere"},
    {VARIABLE,"LDC_CURR",ADD_PARAM(left_dc_curr),NULL,0,0,0,0,0,0,0,0,NULL,"Left DC current centiampere"},
    {VARIABLE,"BATV",ADD_PARAM(batVoltageCalib),NULL,0,0,0,0,0,0,0,0,NULL,"Input voltage x100"},
    {VARIABLE,"TEMP",ADD_PARAM(board_temp_deg_c),NULL,0,0,0,0,0,0,0,0,NULL,"Board temperature x10"},
    {VARIABLE,"FOC_ISR_CYC",ADD_PARAM(foc_isr_cycles),NULL,0,0,0,0,0,0,0,0,NULL,"Last ADC/FOC ISR cycles"},
    {VARIABLE,"FOC_ISR_MAX",ADD_PARAM(foc_isr_cycles_max),NULL,0,0,0,0,0,0,0,0,NULL,"Max ADC/FOC ISR cycles"},
};

const char *errors[10] = {
  "Command not found", // Err1
  "Parameter not found", // Err2
  "This command cannot be used with a Variable", // Err3
  "Value not in range", // Err4
  "Value expected", // Err5
  "Start of line expected", // Err6
  "End of line expected", // Err7
  "Parameter expected", // Err8
  "Uncaught error", // Err9
  "Watch list is full" // Err10
};

debug_command command;
int8_t watchParamList[MAX_PARAM_WATCH] = {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1}; 

// Set Param with Value from external format
int8_t setParamValExt(uint8_t index, int32_t value) {   
  int8_t ret = 0;
  /* Control mode may change only after the requested commands are zero AND
   * the rate-limited commands have fully ramped to zero. */
  if (strcmp(params[index].name, "CTRL_MOD") == 0 &&
      (input1[0].cmd != 0 || input2[0].cmd != 0 || cmdL != 0 || cmdR != 0)) {
    printf("! CTRL_MOD requires STOP and completed ramp-down (cmdL=0 cmdR=0)\r\n");
    return 0;
  }
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

    // Beep if value was modified
    beepShort(5);
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
  for(int i=0;i<COMMAND_SIZE(commands);i++)
    printCommandHelp(i);
  printf("?\r\n");

  printf("? Parameters\r\n");
  for(int i=0;i<PARAM_SIZE(params);i++){
    if (params[i].type == PARAMETER) printParamHelp(i);
  }
  printf("?\r\n");

  printf("? Variables\r\n");
  for(int i=0;i<PARAM_SIZE(params);i++){
    if (params[i].type == VARIABLE) printParamHelp(i);
  }
  printf("?\r\n");

  return 1;
}

// Print definition(name,value,initial value, min, max) for parameter
int8_t printParamDef(uint8_t index){
  printf("# name:\"%s\" value:%li init:%li min:%li max:%li\r\n",
         params[index].name,     // Parameter Name
         (long)getParamValExt(index),  // Parameter Value translated to external format
         (long)getParamInitExt(index), // Parameter Init Value translated to external format
         (long)params[index].min,      // Parameter Min Value with External format 
         (long)params[index].max);     // Parameter Max Value with External format
  return 1;
}

// Print definition(name,value,initial value, min, max) for all parameters
int8_t printAllParamDef(){
  for(int i=0;i<PARAM_SIZE(params);i++) printParamDef(i);
  return 1;
}

void printError(uint8_t errornum ){
  printf("! Err%i:\"%s\"\r\n",errornum,errors[errornum-1]);
}

// Get internal Parameter value and save it to EEprom for all paraemeter with an address assigned 
int8_t saveAllParamVal() {
  HAL_FLASH_Unlock();
  EE_WriteVariable(VirtAddVarTab[0] , (uint16_t)FLASH_WRITE_KEY);
  for(int i=0;i<PARAM_SIZE(params);i++){ 
    // Only Parameters with eeprom address can be saved
    if (params[i].addr){
      EE_WriteVariable(VirtAddVarTab[params[i].addr] , (uint16_t)getParamValInt(i));    
    }
  }
  HAL_FLASH_Lock();
  return 1;
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
  for(int index=0;index<COMMAND_SIZE(commands);index++){
    uint8_t command_len = strlen(commands[index].name);
    if (command_len < len){
      if (memcmp(userCommand,commands[index].name,command_len)==0){
        return index;
      }
    }
  }
  return -1; // Not found
}

// Find parameter in params array and return index
int8_t findParam(uint8_t *userCommand, uint32_t len){
  for(int index=0;index<PARAM_SIZE(params);index++){
    uint8_t param_len = strlen(params[index].name);
    if (param_len < len){
      if (memcmp(userCommand,params[index].name,param_len)==0){
        return index;
      }
    }
  }
  return -1; // Not found
}

// Parse and save the command to be executed
void handle_input(uint8_t *userCommand, uint32_t len)
{

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
    if (value > INT16_MAX) { command.error = 4; return; }
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

