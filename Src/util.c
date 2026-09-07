#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"
#include "eeprom.h"
#include "util.h"
#include "motor/mcpwm_foc.h"
#include "motor/mc_interface.h"
#include "vesc/vesc_protocol.h"
#include "vesc/app_vesc.h"
#include "comms.h"

extern UART_HandleTypeDef huart3;
extern uint8_t buzzerCount;
extern uint8_t buzzerFreq;
extern uint8_t buzzerPattern;
extern uint8_t enable;

InputStruct input1[INPUTS_NR] = {{0, 0, 0, PRI_INPUT1}};
InputStruct input2[INPUTS_NR] = {{0, 0, 0, PRI_INPUT2}};

int16_t speedAvg = 0;
int16_t speedAvgAbs = 0;
uint8_t timeoutFlgSerial = 1;
uint8_t ctrlModReqRaw = CTRL_MOD_REQ;
uint8_t ctrlModReq = OPEN_MODE;

uint16_t VirtAddVarTab[NB_OF_VAR] = {1000, 1001, 1002, 1003, 1004, 1005, 1006, 1007, 1008, 1009, 1010, 1011, 1012, 1013, 1014, 1015, 1016, 1017, 1018, 1019, 1020, 1021, 1022, 1023, 1024, 1025, 1026, 1027, 1028, 1029, 1030, 1031, 1032, 1033, 1034, 1035, 1036, 1037, 1038, 1039, 1040, 1041, 1042, 1043, 1044, 1045, 1046, 1047, 1048, 1049, 1050, 1051, 1052, 1053, 1054, 1055, 1056, 1057, 1058, 1059, 1060, 1061, 1062, 1063, 1064, 1065, 1066, 1067, 1068, 1069, 1070, 1071, 1072, 1073, 1074, 1075, 1076, 1077, 1078, 1079, 1080, 1081, 1082, 1083, 1084, 1085, 1086, 1087, 1088, 1089, 1090, 1091, 1092, 1093, 1094, 1095, 1096, 1097, 1098, 1099, 1100, 1101, 1102, 1103, 1104, 1105, 1106, 1107, 1108, 1109, 1110, 1111, 1112, 1113, 1114, 1115, 1116, 1117, 1118, 1119, 1120, 1121, 1122, 1123, 1124, 1125, 1126, 1127, 1128, 1129, 1130, 1131, 1132, 1133, 1134, 1135, 1136, 1137, 1138, 1139, 1140, 1141, 1142, 1143, 1144, 1145, 1146, 1147, 1148, 1149, 1150, 1151, 1152, 1153, 1154, 1155, 1156, 1157, 1158, 1159, 1160, 1161, 1162, 1163, 1164, 1165, 1166, 1167, 1168, 1169, 1170, 1171, 1172, 1173, 1174, 1175, 1176, 1177, 1178, 1179, 1180, 1181, 1182, 1183, 1184, 1185, 1186, 1187, 1188, 1189, 1190, 1191, 1192, 1193, 1194, 1195, 1196, 1197, 1198, 1199, 1200, 1201, 1202, 1203, 1204, 1205, 1206, 1207, 1208, 1209, 1210, 1211, 1212, 1213, 1214, 1215, 1216, 1217, 1218, 1219, 1220, 1221, 1222, 1223, 1224, 1225, 1226, 1227, 1228, 1229, 1230, 1231, 1232, 1233, 1234, 1235, 1236, 1237, 1238, 1239, 1240, 1241, 1242, 1243, 1244, 1245, 1246, 1247, 1248, 1249, 1250, 1251, 1252, 1253, 1254, 1255, 1256, 1257, 1258, 1259};

static int16_t inputMax = 1000;
static int16_t inputMin = -1000;
static uint8_t rxBuffer[SERIAL_BUFFER_SIZE];
static volatile uint32_t usart3RxErrorCount = 0u;
static volatile uint32_t usart3RxRestartCount = 0u;
static volatile uint32_t usart3ForcedRecoveryCount = 0u;
static uint32_t usart3LastByteMs = 0u;
static uint32_t usart3LastValidFrameMs = 0u;
static uint32_t usart3LastValidCount = 0u;
static uint32_t usart3LastForcedRecoveryMs = 0u;
static uint8_t usart3RecoveryStreak = 0u;
static uint8_t usart3EverValid = 0u;
#define USART3_VALID_PROGRESS_TIMEOUT_MS 1500u
#define USART3_RAW_RECENT_MS 250u
#define USART3_RECOVERY_COOLDOWN_MS 1500u
#define USART3_RECOVERY_BEFORE_RESET 4u
static uint32_t usart3RxOldPos = 0u;
static uint16_t serialTimeoutCount = SERIAL_TIMEOUT;
static SerialCommand serialCommand = {SERIAL_START_FRAME, 0, 0, 0};

static uint8_t frameBuffer[sizeof(SerialCommand)];
static uint8_t frameIndex = 0;
static uint8_t debugLine[SERIAL_DEBUG_LINE_SIZE];
static uint8_t debugIndex = 0;

#ifdef __GNUC__
int _write(int file, char *data, int len) {
  (void)file;
  (void)data;
  if (len <= 0) return 0;
  /* USART3 PB10/PB11 is an exclusive native VESC 6.00 transport at validated 115200 baud.
   * Raw printf/debug bytes are never legal on this wire. */
  return len;
}
#endif

void BLDC_Init(void) {
  mc_interface_init(true);
}

void Input_Lim_Init(void) {
  inputMax = 1500;
  inputMin = -1500;
}

void UART_EnableRxErrorRecovery(UART_HandleTypeDef *huart) {
  if (!huart || huart->Instance != USART3) return;
  SET_BIT(huart->Instance->CR3, USART_CR3_EIE);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (!huart || huart->Instance != USART3) return;
  ++usart3RxErrorCount;
  usart3RxOldPos = 0u;
  huart->ErrorCode = HAL_UART_ERROR_NONE;
  /* HAL has already ended/aborted the DMA RX transfer before this callback.
   * Restart the same circular buffer immediately so one FE/NE/ORE can lose at
   * most the current VESC frame, never the complete session. */
  if (HAL_UART_Receive_DMA(huart, rxBuffer, sizeof(rxBuffer)) == HAL_OK) {
    ++usart3RxRestartCount;
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
  }
}

uint32_t usart3_rx_error_count(void) { return usart3RxErrorCount; }
uint32_t usart3_rx_restart_count(void) { return usart3RxRestartCount; }
uint32_t usart3_forced_recovery_count(void) { return usart3ForcedRecoveryCount; }

void Input_Init(void) {
  UART3_Init();
  vesc_protocol_init();
  HAL_UART_Receive_DMA(&huart3, rxBuffer, sizeof(rxBuffer));
  UART_EnableRxErrorRecovery(&huart3);
  usart3LastByteMs = HAL_GetTick();
  usart3LastValidFrameMs = usart3LastByteMs;
  usart3LastValidCount = 0u;
  usart3LastForcedRecoveryMs = 0u;
  usart3RecoveryStreak = 0u;
  usart3EverValid = 0u;

  HAL_FLASH_Unlock();
  (void)EE_Init();
  HAL_FLASH_Lock();
  /* Load only fields that are actually implemented by this fixed-point port.
   * Defaults remain active when EEPROM is blank or incompatible. */
  (void)mc_interface_load_configuration_motor(false);
  (void)mc_interface_load_configuration_motor(true);
  /* Blank/incompatible EEPROM leaves the safe compiled LEFT config active,
   * but no set_configuration() call has necessarily initialized the shared
   * PB6/PB7 ABI peripheral. Always materialize the selected LEFT sensor mode
   * into hardware at boot; encoder sync itself remains false until alignment. */
  mcpwm_foc_refresh_encoder_configuration(false, true);
  /* Steering span is project calibration, intentionally independent from
   * standard VESC MC configuration signature. */
  (void)mc_interface_load_steering_calibration();
  /* App Config EEPROM must be loaded only after EE_Init(). vesc_protocol_init()
   * runs earlier so USART3 can come up immediately, but it only installs safe
   * defaults at that point. */
  (void)app_vesc_load_configuration(false);
  (void)app_vesc_load_configuration(true);
}

void poweronMelody(void) {
  buzzerCount = 0;
  for (int i = 8; i >= 0; --i) {
    buzzerFreq = (uint8_t)i;
    HAL_Delay(100);
  }
  buzzerFreq = 0;
}

void beepCount(uint8_t cnt, uint8_t freq, uint8_t pattern) {
  buzzerCount = cnt;
  buzzerFreq = freq;
  buzzerPattern = pattern;
}

void beepShort(uint8_t freq) {
  buzzerCount = 0;
  buzzerFreq = freq;
  HAL_Delay(100);
  buzzerFreq = 0;
}

void calcAvgSpeed(void) {
  const int16_t rpmL = mcpwm_foc_get_motor_const(false)->m_rpm;
  const int16_t rpmR = mcpwm_foc_get_motor_const(true)->m_rpm;
  speedAvg = (rpmL - rpmR) / 2;
  speedAvgAbs = abs(speedAvg);
}

static void serialAcceptCommand(const uint8_t *frame) {
  SerialCommand candidate;
  memcpy(&candidate, frame, sizeof(candidate));
  const uint16_t checksum = (uint16_t)(candidate.start ^ (uint16_t)candidate.cmdL ^ (uint16_t)candidate.cmdR);
  if (candidate.start == SERIAL_START_FRAME && candidate.checksum == checksum) {
    serialCommand = candidate;
    serialTimeoutCount = 0;
    timeoutFlgSerial = 0;
  }
}

static void debugAcceptByte(uint8_t byte) {
  if (byte == '\r' || byte == '\n') {
    if (debugIndex == 0) return;
    debugLine[debugIndex++] = '\n';
    handle_input(debugLine, debugIndex);
    debugIndex = 0;
    return;
  }
  if (byte < 0x20 || byte > 0x7e) {
    debugIndex = 0;
    return;
  }
  if (debugIndex < SERIAL_DEBUG_LINE_SIZE - 1) {
    debugLine[debugIndex++] = byte;
  } else {
    debugIndex = 0;
  }
}

static void __attribute__((unused)) serialAcceptLegacyByte(uint8_t byte) {
  const uint8_t startLo = (uint8_t)(SERIAL_START_FRAME & 0xffu);
  const uint8_t startHi = (uint8_t)(SERIAL_START_FRAME >> 8);

  if (frameIndex == 0) {
    if (byte == startLo) {
      frameBuffer[frameIndex++] = byte;
    } else {
      debugAcceptByte(byte);
    }
    return;
  }

  if (frameIndex == 1) {
    if (byte == startHi) {
      frameBuffer[frameIndex++] = byte;
    } else {
      debugAcceptByte(frameBuffer[0]);
      frameIndex = 0;
      serialAcceptLegacyByte(byte);
    }
    return;
  }

  frameBuffer[frameIndex++] = byte;
  if (frameIndex == sizeof(SerialCommand)) {
    serialAcceptCommand(frameBuffer);
    frameIndex = 0;
  }
}

static void serialAcceptByte(uint8_t byte) {
  /* USART3 is VESC-exclusive. The VESC parser safely ignores non 2/3/4 start
   * bytes while idle, so never fall back into the obsolete legacy parser. */
  (void)vesc_protocol_rx_byte(byte);
}

void usart3_rx_check(void) {
  /* Safety net in addition to HAL_UART_ErrorCallback: if a DMA error left RX
   * disabled for any reason, restart it from main context. */
  if ((huart3.Instance->CR3 & USART_CR3_DMAR) == 0u ||
      huart3.hdmarx == NULL || (huart3.hdmarx->Instance->CCR & DMA_CCR_EN) == 0u) {
    usart3RxOldPos = 0u;
    huart3.ErrorCode = HAL_UART_ERROR_NONE;
    huart3.RxState = HAL_UART_STATE_READY;
    if (HAL_UART_Receive_DMA(&huart3, rxBuffer, sizeof(rxBuffer)) == HAL_OK) {
      ++usart3RxRestartCount;
      __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
    }
    return;
  }
  const uint32_t pos = sizeof(rxBuffer) - __HAL_DMA_GET_COUNTER(huart3.hdmarx);
  if (pos == usart3RxOldPos) return;
  usart3LastByteMs = HAL_GetTick();

  if (pos > usart3RxOldPos) {
    for (uint32_t i = usart3RxOldPos; i < pos; ++i) serialAcceptByte(rxBuffer[i]);
  } else {
    for (uint32_t i = usart3RxOldPos; i < sizeof(rxBuffer); ++i) serialAcceptByte(rxBuffer[i]);
    for (uint32_t i = 0; i < pos; ++i) serialAcceptByte(rxBuffer[i]);
  }
  usart3RxOldPos = (pos == sizeof(rxBuffer)) ? 0u : pos;
}

void usart3_recovery_tick(uint32_t now_ms) {
  const uint32_t valid = vesc_protocol_rx_ok_count();
  if (valid != usart3LastValidCount) {
    usart3LastValidCount = valid;
    usart3LastValidFrameMs = now_ms;
    usart3RecoveryStreak = 0u;
    usart3EverValid = 1u;
    return;
  }
  if ((uint32_t)(now_ms - usart3LastByteMs) > USART3_RAW_RECENT_MS) return;
  if ((uint32_t)(now_ms - usart3LastValidFrameMs) < USART3_VALID_PROGRESS_TIMEOUT_MS) return;
  if ((uint32_t)(now_ms - usart3LastForcedRecoveryMs) < USART3_RECOVERY_COOLDOWN_MS) return;

  /* Bytes are physically arriving but no CRC-valid VESC frame has progressed.
   * Fail closed first, then reset only the communication transport. */
  mcpwm_foc_release_motor(false);
  mcpwm_foc_release_motor(true);
  vesc_protocol_transport_reset();
  (void)HAL_UART_DMAStop(&huart3);
  (void)HAL_UART_DeInit(&huart3);
  UART3_Init();
  usart3RxOldPos = 0u;
  huart3.ErrorCode = HAL_UART_ERROR_NONE;
  huart3.RxState = HAL_UART_STATE_READY;
  if (HAL_UART_Receive_DMA(&huart3, rxBuffer, sizeof(rxBuffer)) == HAL_OK) {
    ++usart3RxRestartCount;
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
  }
  UART_EnableRxErrorRecovery(&huart3);
  ++usart3ForcedRecoveryCount;
  if (usart3RecoveryStreak < 0xffu) ++usart3RecoveryStreak;
  usart3LastForcedRecoveryMs = now_ms;
  usart3LastValidFrameMs = now_ms; // fresh grace interval after UART restart

  if (usart3EverValid && usart3RecoveryStreak >= USART3_RECOVERY_BEFORE_RESET) {
    /* A once-healthy live link remained corrupt across several peripheral
     * recoveries. Motors are already released; one MCU reset is the final tier.
     * After reboot, reset-loop protection requires a valid frame before this
     * tier can arm again. */
    HAL_Delay(10u);
    NVIC_SystemReset();
  }
}

void readCommand(void) {
  input1[0].raw = CLAMP(serialCommand.cmdL, inputMin, inputMax);
  input2[0].raw = CLAMP(serialCommand.cmdR, inputMin, inputMax);
  input1[0].cmd = input1[0].raw;
  input2[0].cmd = input2[0].raw;

  if (serialTimeoutCount < SERIAL_TIMEOUT) {
    ++serialTimeoutCount;
  } else {
    timeoutFlgSerial = 1;
  }

  if (timeoutFlgSerial) {
    input1[0].cmd = 0;
    input2[0].cmd = 0;
    ctrlModReq = OPEN_MODE;
  } else {
    ctrlModReq = ctrlModReqRaw;
  }
}

void poweroff(void) {
  enable = 0;
  LEFT_TIM->BDTR &= ~TIM_BDTR_MOE;
  RIGHT_TIM->BDTR &= ~TIM_BDTR_MOE;
#if POWER_OFF_ENABLE
  printf("-- Motors disabled / power latch off --\r\n");
  buzzerCount = 0;
  buzzerPattern = 0;
  for (uint8_t i = 0; i < 8; ++i) {
    buzzerFreq = i;
    HAL_Delay(100);
  }
  buzzerFreq = 0;
  HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, GPIO_PIN_RESET);
  while (1) { }
#else
  /* Development mode: never drop PA5 OFF latch, so USART3/VESC Tool stays alive. */
  HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, GPIO_PIN_SET);
#endif
}

void poweroffPressCheck(void) {
#if POWER_BUTTON_BYPASS || !POWER_OFF_ENABLE
  return;
#else
  if (!HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) return;
  uint16_t pressedMs = 0;
  enable = 0;
  while (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) {
    HAL_Delay(10);
    if (pressedMs < 60000) pressedMs += 10;
  }
  if (pressedMs >= 80) poweroff();
#endif
}

void filtLowPass32(int32_t u, uint16_t coef, int32_t *y) {
  int64_t tmp = ((int64_t)((u << 4) - (*y >> 12)) * coef) >> 4;
  tmp = CLAMP(tmp, -2147483648LL, 2147483647LL);
  *y = (int32_t)tmp + *y;
}

void rateLimiter16(int16_t u, int16_t rate, int16_t *y) {
  int16_t delta = (int16_t)((u << 4) - *y);
  if (delta > rate) delta = rate;
  if (delta < -rate) delta = (int16_t)-rate;
  *y = (int16_t)(*y + delta);
}

