#include <math.h>
#include <string.h>
#include "stm32f1xx_hal.h"
#include "config.h"
#include "defines.h"
#include "motor/mc_interface.h"
#include "motor/mcpwm_foc.h"
#include "vesc/app_vesc.h"

extern volatile adc_buf_t adc_buffer;

typedef struct {
    uint32_t last_ms;
    uint32_t zero_ms;
    float filt_v1;
    float filt_v2;
    float ramp;
    uint8_t initialized;
} app_adc_state_t;

static app_configuration s_conf[2];
static app_adc_state_t s_state[2];
static volatile float s_v1 = 0.0f;
static volatile float s_v2 = 0.0f;
static volatile float s_dec1 = 0.0f;
static volatile float s_dec2 = 0.0f;
static volatile bool s_range_ok = true;

static float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static float mapf(float x, float in0, float in1, float out0, float out1) {
    if (fabsf(in1 - in0) < 1e-9f) return out0;
    return out0 + (x - in0) * (out1 - out0) / (in1 - in0);
}

static void deadband(float *v, float tres) {
    tres = clampf(tres, 0.0f, 0.99f);
    if (fabsf(*v) < tres) {
        *v = 0.0f;
    } else {
        const float k = 1.0f / (1.0f - tres);
        *v = (*v > 0.0f) ? (k * *v + 1.0f - k) : -(k * -*v + 1.0f - k);
    }
}

/* Same three curve families used by upstream VESC utils_throttle_curve(). */
static float throttle_curve(float val, float curve_acc, float curve_brake, int mode) {
    val = clampf(val, -1.0f, 1.0f);
    const float a = fabsf(val);
    const float curve = val >= 0.0f ? curve_acc : curve_brake;
    float ret = a;
    if (mode == THR_EXP_EXPO) {
        ret = curve >= 0.0f ? 1.0f - powf(1.0f - a, 1.0f + curve) : powf(a, 1.0f - curve);
    } else if (mode == THR_EXP_NATURAL) {
        if (fabsf(curve) >= 1e-10f) {
            ret = curve >= 0.0f ?
                1.0f - ((expf(curve * (1.0f - a)) - 1.0f) / (expf(curve) - 1.0f)) :
                (expf(-curve * a) - 1.0f) / (expf(-curve) - 1.0f);
        }
    } else if (mode == THR_EXP_POLY) {
        ret = curve >= 0.0f ? 1.0f - ((1.0f - a) / (1.0f + curve * a)) :
                              a / (1.0f - curve * (1.0f - a));
    }
    return val < 0.0f ? -ret : ret;
}

void app_vesc_defaults(app_configuration *a, uint8_t id) {
    if (!a) return;
    memset(a, 0, sizeof(*a));
    a->controller_id = id;
    a->timeout_msec = 1000u;
    a->timeout_brake_current = 0.0f;
    a->can_baud_rate = CAN_BAUD_500K;
    a->pairing_done = true;
    a->permanent_uart_enabled = true;
    a->can_mode = CAN_MODE_VESC;
    a->app_to_use = APP_UART;
    a->app_uart_baudrate = 115200u;
    a->app_adc_conf.ctrl_type = ADC_CTRL_TYPE_NONE;
    a->app_adc_conf.hyst = 0.15f;
    a->app_adc_conf.voltage_start = 0.9f;
    a->app_adc_conf.voltage_end = 3.0f;
    a->app_adc_conf.voltage_min = 0.0f;
    a->app_adc_conf.voltage_max = 3.3f;
    a->app_adc_conf.voltage_center = 1.65f;
    a->app_adc_conf.voltage2_start = 0.9f;
    a->app_adc_conf.voltage2_end = 3.0f;
    a->app_adc_conf.use_filter = true;
    a->app_adc_conf.safe_start = SAFE_START_REGULAR;
    a->app_adc_conf.throttle_exp_mode = THR_EXP_EXPO;
    a->app_adc_conf.ramp_time_pos = 0.3f;
    a->app_adc_conf.ramp_time_neg = 0.1f;
    a->app_adc_conf.update_rate_hz = 200u;
}

void app_vesc_init(void) {
    app_vesc_defaults(&s_conf[0], 1u);
    app_vesc_defaults(&s_conf[1], 2u);
    mcpwm_foc_vesc_timeout_configure(false, s_conf[0].timeout_msec, s_conf[0].timeout_brake_current);
    mcpwm_foc_vesc_timeout_configure(true, s_conf[1].timeout_msec, s_conf[1].timeout_brake_current);
    memset(s_state, 0, sizeof(s_state));
}

const app_configuration *app_vesc_get_configuration(bool second) {
    return &s_conf[second ? 1 : 0];
}

bool app_vesc_set_configuration(bool second, const app_configuration *conf) {
    if (!conf) return false;
    app_configuration c = *conf;
    c.controller_id = second ? 2u : 1u;
    c.can_mode = CAN_MODE_VESC;
    c.permanent_uart_enabled = true; /* USART3 must always remain reachable by VESC Tool. */
    if (c.app_to_use > APP_ADC_PAS) c.app_to_use = APP_UART;
    if (c.app_adc_conf.ctrl_type > ADC_CTRL_TYPE_PID_REV_BUTTON) c.app_adc_conf.ctrl_type = ADC_CTRL_TYPE_NONE;
    if (c.app_adc_conf.update_rate_hz == 0u) c.app_adc_conf.update_rate_hz = 1u;
    /* This board has one physical VESC UART. Keep its electrical link fixed at
     * 115200 so writing App Config cannot strand VESC Tool on an unknown baud. */
    c.app_uart_baudrate = 115200u;
    s_conf[second ? 1 : 0] = c;
    mcpwm_foc_vesc_timeout_configure(second, c.timeout_msec, c.timeout_brake_current);
    memset(&s_state[second ? 1 : 0], 0, sizeof(s_state[0]));
    return true;
}

static bool adc_app_enabled(const app_configuration *a) {
    return a->app_to_use == APP_ADC || a->app_to_use == APP_ADC_UART || a->app_to_use == APP_ADC_PAS;
}

static void touch(bool second) {
    mcpwm_foc_vesc_override_touch(second);
}

static void select_motor(bool second) {
    mc_interface_select_motor_thread(second ? 2 : 1);
}

static void set_current_user(bool second, float amp) {
    select_motor(second);
    touch(second);
    mc_interface_set_current(second ? -amp : amp);
}

static void set_brake_user(bool second, float amp) {
    select_motor(second);
    if (amp < 0.0f) amp = -amp;
    touch(second);
    mc_interface_set_brake_current(amp);
}

static void set_duty_user(bool second, float duty) {
    select_motor(second);
    touch(second);
    mc_interface_set_duty(second ? -duty : duty);
}

static void set_rpm_user(bool second, float erpm) {
    select_motor(second);
    touch(second);
    mc_interface_set_pid_speed(second ? -erpm : erpm);
}

static void apply_adc(bool second, const app_configuration *a, uint32_t now_ms, float raw_v1, float raw_v2) {
    app_adc_state_t *st = &s_state[second ? 1 : 0];
    const adc_config *c = &a->app_adc_conf;
    uint32_t hz = c->update_rate_hz;
    if (hz < 1u) hz = 1u;
    if (hz > 200u) hz = 200u; /* main housekeeping cadence is 200 Hz */
    const uint32_t period = (1000u + hz - 1u) / hz;
    if (st->initialized && (uint32_t)(now_ms - st->last_ms) < period) return;
    const uint32_t dt = st->initialized ? (uint32_t)(now_ms - st->last_ms) : period;
    st->last_ms = now_ms;
    if (!st->initialized) {
        st->filt_v1 = raw_v1;
        st->filt_v2 = raw_v2;
        st->initialized = 1u;
    } else {
        st->filt_v1 += (raw_v1 - st->filt_v1) * 0.2f;
        st->filt_v2 += (raw_v2 - st->filt_v2) * 0.2f;
    }

    float v1 = c->use_filter ? st->filt_v1 : raw_v1;
    float v2 = c->use_filter ? st->filt_v2 : raw_v2;
    bool range_ok = v1 >= c->voltage_min && v1 <= c->voltage_max;
    float pwr;
    switch (c->ctrl_type) {
    case ADC_CTRL_TYPE_CURRENT_REV_CENTER:
    case ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_CENTER:
    case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_CENTER:
    case ADC_CTRL_TYPE_DUTY_REV_CENTER:
    case ADC_CTRL_TYPE_PID_REV_CENTER:
        pwr = v1 < c->voltage_center ? mapf(v1, c->voltage_start, c->voltage_center, 0.0f, 0.5f) :
                                      mapf(v1, c->voltage_center, c->voltage_end, 0.5f, 1.0f);
        break;
    default:
        pwr = mapf(v1, c->voltage_start, c->voltage_end, 0.0f, 1.0f);
        break;
    }
    pwr = clampf(pwr, 0.0f, 1.0f);
    if (c->voltage_inverted) pwr = 1.0f - pwr;

    float brake = clampf(mapf(v2, c->voltage2_start, c->voltage2_end, 0.0f, 1.0f), 0.0f, 1.0f);
    if (c->voltage2_inverted) brake = 1.0f - brake;

    s_v1 = v1; s_v2 = v2; s_dec1 = pwr; s_dec2 = brake; s_range_ok = range_ok;

    switch (c->ctrl_type) {
    case ADC_CTRL_TYPE_CURRENT_REV_CENTER:
    case ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_CENTER:
    case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_CENTER:
    case ADC_CTRL_TYPE_DUTY_REV_CENTER:
    case ADC_CTRL_TYPE_PID_REV_CENTER:
        pwr = pwr * 2.0f - 1.0f;
        break;
    case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_ADC:
    case ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_ADC:
        pwr -= brake;
        break;
    default:
        break;
    }
    deadband(&pwr, c->hyst);
    pwr = throttle_curve(pwr, c->throttle_exp, c->throttle_exp_brake, c->throttle_exp_mode);

    const float ramp_t = fabsf(pwr) > fabsf(st->ramp) ? c->ramp_time_pos : c->ramp_time_neg;
    if (ramp_t > 0.01f) {
        const float step = (float)dt / (ramp_t * 1000.0f);
        if (st->ramp < pwr) st->ramp = fminf(st->ramp + step, pwr);
        else if (st->ramp > pwr) st->ramp = fmaxf(st->ramp - step, pwr);
        pwr = st->ramp;
    } else {
        st->ramp = pwr;
    }

    if (fabsf(pwr) < 0.001f) st->zero_ms += dt; else st->zero_ms = 0u;
    if ((c->safe_start != SAFE_START_DISABLED && st->zero_ms < 500u) || !range_ok ||
        (mc_interface_get_fault_motor(second) != FAULT_CODE_NONE && c->safe_start != SAFE_START_NO_FAULT)) {
        set_brake_user(second, a->timeout_brake_current);
        return;
    }

    const volatile mc_configuration *mc = mc_interface_get_configuration_motor(second);
    switch (c->ctrl_type) {
    case ADC_CTRL_TYPE_NONE:
        set_current_user(second, 0.0f);
        break;
    case ADC_CTRL_TYPE_CURRENT:
    case ADC_CTRL_TYPE_CURRENT_REV_CENTER:
    case ADC_CTRL_TYPE_CURRENT_REV_BUTTON:
        set_current_user(second, pwr >= 0.0f ? pwr * mc->l_current_max : (-pwr) * mc->l_current_min);
        break;
    case ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_CENTER:
    case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_CENTER:
    case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_BUTTON:
    case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_ADC:
    case ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_ADC:
        if (pwr >= 0.0f) set_current_user(second, pwr * mc->l_current_max);
        else set_brake_user(second, (-pwr) * fabsf(mc->l_current_min));
        break;
    case ADC_CTRL_TYPE_DUTY:
    case ADC_CTRL_TYPE_DUTY_REV_CENTER:
    case ADC_CTRL_TYPE_DUTY_REV_BUTTON:
        set_duty_user(second, pwr * mc->l_max_duty);
        break;
    case ADC_CTRL_TYPE_PID:
    case ADC_CTRL_TYPE_PID_REV_CENTER:
    case ADC_CTRL_TYPE_PID_REV_BUTTON:
        set_rpm_user(second, pwr >= 0.0f ? pwr * mc->l_max_erpm : (-pwr) * mc->l_min_erpm);
        break;
    default:
        set_current_user(second, 0.0f);
        break;
    }
}

void app_vesc_process(uint32_t now_ms) {
    const float v1 = (float)adc_buffer.adc2_spare4 * (3.3f / 4095.0f); /* PA2 / ADC2 CH2 */
    const float v2 = (float)adc_buffer.adc2_spare5 * (3.3f / 4095.0f); /* PA3 / ADC2 CH3 */
    const bool local_adc = adc_app_enabled(&s_conf[0]);
    if (local_adc) apply_adc(false, &s_conf[0], now_ms, v1, v2);
    if (local_adc && s_conf[0].app_adc_conf.multi_esc) {
        apply_adc(true, &s_conf[0], now_ms, v1, v2);
    } else if (adc_app_enabled(&s_conf[1])) {
        apply_adc(true, &s_conf[1], now_ms, v1, v2);
    }
    mc_interface_select_motor_thread(1);
}

float app_vesc_adc_decoded(bool second_channel) { return second_channel ? s_dec2 : s_dec1; }
float app_vesc_adc_voltage(bool second_channel) { return second_channel ? s_v2 : s_v1; }
bool app_vesc_adc_range_ok(void) { return s_range_ok; }
