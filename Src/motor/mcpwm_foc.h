#ifndef MCPWM_FOC_H_
#define MCPWM_FOC_H_

#include <stdbool.h>
#include <stdint.h>
#include "config.h"
#include "motor/foc_math.h"
#include "motor/mcconf_default.h"

/*
 * Compatibility shell around the hoverboard control pipeline, written with the
 * same state/configuration conventions used by VESC mcpwm_foc. The EEPROM
 * facing tuning fields intentionally remain integer/fixed-point so existing
 * addresses and serial SET/SAVE behavior do not change on STM32F103.
 */

typedef enum {
  MC_STATE_OFF = 0,
  MC_STATE_RUNNING
} mc_state;

typedef enum {
  CONTROL_MODE_DUTY = 0,
  CONTROL_MODE_SPEED,
  CONTROL_MODE_CURRENT,
  CONTROL_MODE_CURRENT_BRAKE,
  CONTROL_MODE_POS,
  CONTROL_MODE_NONE
} mc_control_mode;

typedef enum {
  MOTOR_TYPE_COMMUTATION = COM_CTRL,
  MOTOR_TYPE_SINE = SIN_CTRL,
  MOTOR_TYPE_FOC = FOC_CTRL
} mc_motor_type;

typedef struct {
  /* Limits. Raw units are kept compatible with the existing EEPROM layer. */
  int16_t l_current_max;                 /* current ADC counts * 16 (Q4) */
  int16_t l_max_rpm;                     /* mechanical rpm * 16 */

  /* FOC current controller. Coefficients keep the proven fixed-point scaling. */
  uint16_t foc_current_kp_q;
  uint16_t foc_current_ki_q;
  uint16_t foc_current_kp_d;
  uint16_t foc_current_ki_d;
  uint16_t foc_current_filter_const;
  uint16_t foc_current_anti_windup;
  uint16_t foc_current_i_limit;

  /* Legacy speed coefficients remain persisted for EEPROM compatibility. */
  uint16_t s_pid_kp;
  uint16_t s_pid_ki;
  uint16_t s_pid_i_limit;

  /* Hall / FOC transition and field-weakening parameters. */
  int16_t foc_comm_rpm_low;              /* rpm * 16 */
  int16_t foc_comm_rpm_high;             /* rpm * 16 */
  uint8_t foc_fw_enable;
  int16_t foc_fw_current_max;            /* current ADC counts * 16 (Q4) */
  int16_t foc_fw_phase_advance_max;      /* degree * 16 */
  int16_t foc_fw_rpm_start;              /* rpm * 16 */
  int16_t foc_fw_rpm_end;                /* rpm * 16 */

  uint8_t si_motor_pole_pairs;
  mc_motor_type motor_type;
  uint8_t foc_current_sample_map;         /* 0: AB measured, 1: BC measured */
  uint8_t foc_encoder_enable;
  uint8_t sensor_mode;                  /* 1 open-loop, 2 Hall, 3 encoder AB (Left only) */
  uint8_t comm_mode;                    /* 1 six-step, 2 sine PWM, 3 SVPWM/FOC */
  uint8_t m_diag_enable;
} mc_configuration;

typedef struct {
  bool enable;
  mc_control_mode control_mode;
  int16_t control_setpoint;
  uint8_t hall_a;
  uint8_t hall_b;
  uint8_t hall_c;
  int16_t current_adc_1;
  int16_t current_adc_2;
  int16_t current_input;
  int16_t phase_encoder_deg_x16;
  uint16_t phase_openloop_q16;
  int16_t rpm_sensor;
} motor_input_t;

typedef struct {
  int16_t duty_a;
  int16_t duty_b;
  int16_t duty_c;
  uint8_t fault_code;
  int16_t rpm;
  int16_t phase_electrical_deg;
  int16_t iq;
  int16_t id;
} motor_output_t;

typedef struct {
  int32_t integrator;
  bool saturated;
} foc_pi_state_t;

typedef struct {
  int32_t iq_filter_state;
  int32_t id_filter_state;
  int16_t iq;
  int16_t id;
  int16_t iq_target;
  int16_t id_target;
  int16_t vq;
  int16_t vd;
} foc_state_t;

typedef struct {
  bool initialized;
  int8_t sector;
  int8_t direction;
  uint16_t ticks;
  uint16_t period;
} hall_state_t;

typedef struct {
  mc_configuration *m_conf;
  mc_state m_state;
  mc_control_mode m_control_mode;
  motor_input_t m_input;
  motor_output_t m_output;
  foc_state_t m_motor_state;
  foc_pi_state_t m_iq_pi;
  foc_pi_state_t m_id_pi;
  foc_pi_state_t m_speed_pi;
  hall_state_t m_hall;
  int16_t m_speed_est_fast;
  uint16_t m_phase_now;
  uint8_t m_last_motor_type;
} motor_all_state_t;

extern motor_all_state_t m_motor_1;
extern motor_all_state_t m_motor_2;
extern mc_configuration m_mcconf_1;
extern mc_configuration m_mcconf_2;

extern const int8_t m_hall_to_sector[8];
extern const int8_t m_commutation_map[18];
extern const int16_t m_sine_phase_a_q14[181];
extern const int16_t m_sine_phase_b_q14[181];
extern const int16_t m_sine_phase_c_q14[181];
extern const int16_t m_sin_q15[256];

void mcpwm_foc_init_defaults(void);
void mcpwm_foc_init(mc_configuration *conf_m1, mc_configuration *conf_m2);
void mcpwm_foc_reset(motor_all_state_t *motor);
void mcpwm_foc_control(motor_all_state_t *motor);
/* Hardware/ISR telemetry. DMA1_Channel1_IRQHandler is implemented in mcpwm_foc.c
 * and keeps the validated ADC/DMA ordering from the proven firmware. */
extern volatile uint32_t m_foc_isr_cycles;
extern volatile uint32_t m_foc_isr_cycles_max;
extern volatile int16_t m_foc_iq_left_q4;
extern volatile int16_t m_foc_iq_right_q4;
extern volatile int16_t m_foc_id_left_q4;
extern volatile int16_t m_foc_id_right_q4;
extern volatile uint16_t m_sensor_hall_left;
extern volatile uint16_t m_sensor_hall_right;
extern volatile int16_t m_sensor_rpm_left;
extern volatile int16_t m_sensor_rpm_right;
extern volatile uint8_t m_adc_current_valid;
extern volatile uint8_t m_adc_current_valid_left;
extern volatile uint8_t m_adc_current_valid_right;

void currentCalibrationStart(void);
uint8_t currentCalibrationActive(void);
uint16_t currentCalibrationProgressPermille(void);
uint8_t currentCalibrationResetPending(void);
void currentCalibrationFinalizeReset(void);
void mcpwm_foc_sensor_state_reset(uint8_t is_second_motor);


#endif /* MCPWM_FOC_H_ */
