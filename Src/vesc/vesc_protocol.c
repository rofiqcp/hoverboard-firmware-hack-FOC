#include <string.h>
#include "stm32f1xx_hal.h"
#include "config.h"
#include "motor/mcpwm_foc.h"
#include "motor/mcconf_default.h"
#include "motor/mc_interface.h"
#include "vesc/datatypes.h"
#include "vesc/buffer.h"
#include "vesc/crc.h"
#include "vesc/mcconf_serial.h"
#include "vesc/vesc_protocol.h"
#include "vesc/app_vesc.h"

#define VESC_FW_MAJOR               6u
#define VESC_FW_MINOR               0u
#define VESC_LOCAL_ID               1u
#define VESC_SECOND_MOTOR_ID        2u
#define VESC_LINK_HOLD_MS        2000u
#define VESC_MAX_PAYLOAD          700u
#define VESC_MAX_FRAME      (VESC_MAX_PAYLOAD + 7u)
#define VESC_RX_QUEUE_DEPTH          4u

/* Project-specific extensions are transported inside standard
 * COMM_CUSTOM_APP_DATA, so stock VESC commands remain wire-compatible. */
#define HB_CUSTOM_MAGIC0              0x48u /* 'H' */
#define HB_CUSTOM_MAGIC1              0x42u /* 'B' */
#define HB_CUSTOM_VERSION                1u
#define HB_CUSTOM_GET_DIAG               1u
#define HB_CUSTOM_GET_POS_STATE          2u
#define HB_CUSTOM_SET_POS_LIMITS         3u
#define HB_CUSTOM_SET_POS_TARGET         4u
#define HB_CUSTOM_RESET_POSITION         5u

extern UART_HandleTypeDef huart3;
extern int16_t board_temp_deg_c;

static volatile uint8_t s_rx_active = 0u;
static volatile uint16_t s_rx_index = 0u;
static volatile uint16_t s_rx_expected = 0u;
static volatile uint16_t s_payload_start = 0u;
static volatile uint16_t s_payload_len = 0u;
static uint8_t s_rx_frame[VESC_MAX_FRAME];
static uint8_t s_pending_payload[VESC_RX_QUEUE_DEPTH][VESC_MAX_PAYLOAD];
static uint8_t s_process_payload[VESC_MAX_PAYLOAD];
static uint8_t s_config_payload[VESC_MAX_PAYLOAD];
static volatile uint16_t s_pending_len[VESC_RX_QUEUE_DEPTH];
static volatile uint8_t s_pending_head = 0u;
static volatile uint8_t s_pending_tail = 0u;
static volatile uint8_t s_pending_count = 0u;
static volatile uint32_t s_rx_queue_drop = 0u;
static volatile uint32_t s_link_last_ms = 0u;
static volatile uint32_t s_rx_ok = 0u;
static volatile uint32_t s_rx_crc_err = 0u;
static volatile uint8_t s_last_hall_store_ok[2] = {0u, 0u};

static void rx_reset(void) {
    s_rx_active = 0u;
    s_rx_index = 0u;
    s_rx_expected = 0u;
    s_payload_start = 0u;
    s_payload_len = 0u;
}

static void app_defaults(app_configuration *a, uint8_t id) {
    app_vesc_defaults(a, id);
}

void vesc_protocol_init(void) {
    rx_reset();
    s_pending_head = s_pending_tail = s_pending_count = 0u;
    memset((void *)s_pending_len, 0, sizeof(s_pending_len));
    s_rx_queue_drop = 0u;
    s_link_last_ms = 0u;
    s_rx_ok = 0u;
    s_rx_crc_err = 0u;
    s_last_hall_store_ok[0] = s_last_hall_store_ok[1] = 0u;
    app_vesc_init();
}

bool vesc_protocol_rx_in_progress(void) { return s_rx_active != 0u; }

static void complete_frame(void) {
    const uint16_t p = s_payload_start;
    const uint16_t n = s_payload_len;
    bool valid = false;
    if (n > 0u && n <= VESC_MAX_PAYLOAD && s_rx_expected >= (uint16_t)(p + n + 3u)) {
        const uint16_t rx_crc = (uint16_t)(((uint16_t)s_rx_frame[p + n] << 8) |
                                           (uint16_t)s_rx_frame[p + n + 1u]);
        const uint16_t calc = vesc_crc16(&s_rx_frame[p], n);
        valid = (s_rx_frame[p + n + 2u] == 3u) && (rx_crc == calc);
    }
    if (valid) {
        /* COMM_ALIVE is the VESC motor-command watchdog heartbeat and has no
         * response payload. Handle it immediately after CRC validation instead
         * of putting it behind realtime/config requests. This mirrors
         * timeout_reset() semantics and guarantees a full RX FIFO cannot make a
         * one-click VESC Tool setpoint expire while Send Alive is active. */
        const uint8_t *vp=&s_rx_frame[p];
        const bool alive_local=(n==1u && vp[0]==COMM_ALIVE);
        const bool alive_right=(n==3u && vp[0]==COMM_FORWARD_CAN &&
                                vp[1]==VESC_SECOND_MOTOR_ID && vp[2]==COMM_ALIVE);
        if (alive_local || alive_right) {
            mcpwm_foc_vesc_override_touch(alive_right);
            s_link_last_ms=HAL_GetTick();
            s_rx_ok++;
            rx_reset();
            return;
        }

        /* UART DMA/ISR can deliver several VESC Tool requests before the 5-ms
         * main loop processes them. V15 had one pending slot and silently
         * discarded every additional valid packet; that is especially harmful
         * to realtime polling. Keep a small bounded FIFO instead. */
        if (s_pending_count < VESC_RX_QUEUE_DEPTH) {
            const uint8_t slot = s_pending_head;
            memcpy(s_pending_payload[slot], &s_rx_frame[p], n);
            s_pending_len[slot] = n;
            s_pending_head = (uint8_t)((slot + 1u) % VESC_RX_QUEUE_DEPTH);
            s_pending_count++;
        } else {
            s_rx_queue_drop++;
        }
        s_link_last_ms = HAL_GetTick();
        s_rx_ok++;
    } else {
        s_rx_crc_err++;
    }
    rx_reset();
}

bool vesc_protocol_rx_byte(uint8_t byte) {
    if (!s_rx_active) {
        if (byte != 2u && byte != 3u && byte != 4u) return false;
        s_rx_active = 1u;
        s_rx_frame[0] = byte;
        s_rx_index = 1u;
        s_link_last_ms = HAL_GetTick(); /* suppress unsolicited terminal telemetry immediately */
        return true;
    }

    if (s_rx_index >= VESC_MAX_FRAME) {
        rx_reset();
        return true;
    }
    s_rx_frame[s_rx_index++] = byte;

    const uint8_t start = s_rx_frame[0];
    if (start == 2u && s_rx_index == 2u) {
        s_payload_start = 2u;
        s_payload_len = s_rx_frame[1];
    } else if (start == 3u && s_rx_index == 3u) {
        s_payload_start = 3u;
        s_payload_len = (uint16_t)(((uint16_t)s_rx_frame[1] << 8) | s_rx_frame[2]);
    } else if (start == 4u && s_rx_index == 4u) {
        /* STM32F103 implementation deliberately caps packets below 64 KiB. */
        const uint32_t n = ((uint32_t)s_rx_frame[1] << 16) |
                           ((uint32_t)s_rx_frame[2] << 8) | s_rx_frame[3];
        if (n > VESC_MAX_PAYLOAD) { rx_reset(); return true; }
        s_payload_start = 4u;
        s_payload_len = (uint16_t)n;
    }

    if (s_payload_start != 0u && s_rx_expected == 0u) {
        if (s_payload_len == 0u || s_payload_len > VESC_MAX_PAYLOAD) {
            rx_reset();
            return true;
        }
        s_rx_expected = (uint16_t)(s_payload_start + s_payload_len + 3u);
    }
    if (s_rx_expected != 0u && s_rx_index == s_rx_expected) complete_frame();
    return true;
}

bool vesc_protocol_link_active(void) {
    return (uint32_t)(HAL_GetTick() - s_link_last_ms) < VESC_LINK_HOLD_MS;
}
uint32_t vesc_protocol_rx_ok_count(void) { return s_rx_ok; }
uint32_t vesc_protocol_rx_crc_error_count(void) { return s_rx_crc_err; }

static void uart_send_payload(const uint8_t *payload, uint16_t len) {
    if (!payload || len == 0u || len > VESC_MAX_PAYLOAD) return;
    static uint8_t tx[VESC_MAX_FRAME];
    /* USART3 TX DMA owns this static frame until HAL marks the UART ready.
     * Wait BEFORE rebuilding the buffer, otherwise a second queued VESC reply
     * could overwrite bytes while DMA is still transmitting the first one. */
    while (huart3.gState != HAL_UART_STATE_READY) { }
    uint16_t i = 0u;
    if (len <= 255u) {
        tx[i++] = 2u;
        tx[i++] = (uint8_t)len;
    } else {
        tx[i++] = 3u;
        tx[i++] = (uint8_t)(len >> 8);
        tx[i++] = (uint8_t)len;
    }
    memcpy(&tx[i], payload, len);
    i = (uint16_t)(i + len);
    const uint16_t crc = vesc_crc16(payload, len);
    tx[i++] = (uint8_t)(crc >> 8);
    tx[i++] = (uint8_t)crc;
    tx[i++] = 3u;
    /* FOC runs at 16 kHz. Byte-by-byte blocking TX left inter-byte gaps whenever
     * the FOC ISR pre-empted HAL_UART_Transmit, dropping active-motor RT data
     * below VESC Tool's 20-ms cadence. DMA keeps the wire continuous. */
    if (HAL_UART_Transmit_DMA(&huart3, tx, i) != HAL_OK) {
        (void)HAL_UART_Transmit(&huart3, tx, i, 1000u);
    }
}

static void read_uuid(uint8_t out[12], bool second) {
#ifdef STM32F103xE
    const volatile uint8_t *uid = (const volatile uint8_t *)0x1FFFF7E8u;
    for (uint8_t i = 0u; i < 12u; ++i) out[i] = uid[i];
#else
    static const uint8_t host_uid[12] = {0x48,0x4f,0x56,0x45,0x52,0x46,0x31,0x30,0x33,0x46,0x4f,0x43};
    memcpy(out, host_uid, sizeof(host_uid));
#endif
    if (second) out[11]++;
}

static void reply_fw_version(bool second) {
    uint8_t b[80];
    int32_t i = 0;
    const char *hw = second ? "motor_right" : "motor_left";
    const char *fw = second ? "motor_right" : "motor_left";
    uint8_t uid[12];
    read_uuid(uid, second);
    b[i++] = COMM_FW_VERSION;
    b[i++] = VESC_FW_MAJOR;
    b[i++] = VESC_FW_MINOR;
    strcpy((char *)&b[i], hw); i += (int32_t)strlen(hw) + 1;
    memcpy(&b[i], uid, 12u); i += 12;
    b[i++] = 1u;                 /* pairing_done */
    b[i++] = 0u;                 /* FW_TEST_VERSION_NUMBER */
    b[i++] = HW_TYPE_VESC;
    b[i++] = 0u;                 /* custom config count */
    b[i++] = 0u;                 /* phase filters */
    b[i++] = 0u;                 /* QML HW */
    b[i++] = 0u;                 /* QML app */
    b[i++] = 0u;                 /* NRF flags */
    strcpy((char *)&b[i], fw); i += (int32_t)strlen(fw) + 1;
    /* VESC firmware 6.00 ends COMM_FW_VERSION after FW_NAME. Do not append
     * post-6.00 fields; older VESC Tool parsers otherwise see trailing bytes. */
    uart_send_payload(b, (uint16_t)i);
}

static void get_values_normalized(bool second, mc_values *v) {
    mc_interface_get_values_motor(v, second);
    /* Hoverboard temperature calibration is deci-degC (358 = 35.8C). */
    v->temp_mos = (float)board_temp_deg_c * 0.1f;
    v->temp_mos_1 = v->temp_mos;
    v->temp_mos_2 = v->temp_mos;
    v->temp_mos_3 = v->temp_mos;
    v->temp_motor = 0.0f;
    v->vesc_id = second ? VESC_SECOND_MOTOR_ID : VESC_LOCAL_ID;
    if (second) {
        /* Right power stage is physically mirrored. Expose the same positive-wheel
         * convention to VESC Tool as the local left motor. */
        v->rpm = -v->rpm;
        /* current_motor is VESC's signed current-vector magnitude and already
         * follows electrical power direction; unlike Iq it does not need the
         * right-motor phase-orientation sign normalization. */
        v->iq = -v->iq;
        v->duty_now = -v->duty_now;
        v->vq = -v->vq;
        v->tachometer = -v->tachometer;
        /* Stock VESC position is normalized to 0..360 deg. The right power
         * stage is mirrored, so expose 360-internal_angle rather than a
         * negative angle to VESC Tool. */
        if (v->position > 0.0f) {
            v->position = 360.0f - v->position;
            if (v->position >= 360.0f) v->position = 0.0f;
        }
    }
}

static void append_values_fields(uint8_t *b, int32_t *i, const mc_values *v, uint32_t mask) {
    if (mask & (1u << 0)) buffer_append_float16(b, v->temp_mos, 1e1f, i);
    if (mask & (1u << 1)) buffer_append_float16(b, v->temp_motor, 1e1f, i);
    if (mask & (1u << 2)) buffer_append_float32(b, v->current_motor, 1e2f, i);
    if (mask & (1u << 3)) buffer_append_float32(b, v->current_in, 1e2f, i);
    if (mask & (1u << 4)) buffer_append_float32(b, v->id, 1e2f, i);
    if (mask & (1u << 5)) buffer_append_float32(b, v->iq, 1e2f, i);
    if (mask & (1u << 6)) buffer_append_float16(b, v->duty_now, 1e3f, i);
    if (mask & (1u << 7)) buffer_append_float32(b, v->rpm, 1e0f, i);
    if (mask & (1u << 8)) buffer_append_float16(b, v->v_in, 1e1f, i);
    if (mask & (1u << 9)) buffer_append_float32(b, v->amp_hours, 1e4f, i);
    if (mask & (1u << 10)) buffer_append_float32(b, v->amp_hours_charged, 1e4f, i);
    if (mask & (1u << 11)) buffer_append_float32(b, v->watt_hours, 1e4f, i);
    if (mask & (1u << 12)) buffer_append_float32(b, v->watt_hours_charged, 1e4f, i);
    if (mask & (1u << 13)) buffer_append_int32(b, v->tachometer, i);
    if (mask & (1u << 14)) buffer_append_int32(b, v->tachometer_abs, i);
    if (mask & (1u << 15)) b[(*i)++] = (uint8_t)v->fault_code;
    if (mask & (1u << 16)) buffer_append_float32(b, v->position, 1e6f, i);
    if (mask & (1u << 17)) b[(*i)++] = (uint8_t)v->vesc_id;
    if (mask & (1u << 18)) {
        buffer_append_float16(b, v->temp_mos_1, 1e1f, i);
        buffer_append_float16(b, v->temp_mos_2, 1e1f, i);
        buffer_append_float16(b, v->temp_mos_3, 1e1f, i);
    }
    if (mask & (1u << 19)) buffer_append_float32(b, v->vd, 1e3f, i);
    if (mask & (1u << 20)) buffer_append_float32(b, v->vq, 1e3f, i);
    if (mask & (1u << 21)) b[(*i)++] = 0u; /* timeout/kill status */
}

static void send_values_packet(bool second, bool selective, uint32_t mask) {
    uint8_t b[128];
    int32_t i = 0;
    b[i++] = selective ? COMM_GET_VALUES_SELECTIVE : COMM_GET_VALUES;
    if (selective) buffer_append_uint32(b, mask, &i);
    mc_values v;
    get_values_normalized(second, &v);
    append_values_fields(b, &i, &v, mask);
    uart_send_payload(b, (uint16_t)i);
}

static void reply_values(bool second, bool selective, const uint8_t *data, uint16_t len) {
    uint32_t mask = 0xffffffffu;
    if (selective) {
        if (len < 4u) return;
        int32_t r = 0;
        mask = buffer_get_uint32(data, &r);
    }
    send_values_packet(second, selective, mask);

    /* Stock VESC semantics are strict request/reply: the host (VESC Tool)
     * chooses the polling rate. Never arm an unsolicited values stream here. */
}

static void send_values_setup_packet(bool second, bool selective, uint32_t mask) {
    uint8_t b[128];
    int32_t i = 0;
    const COMM_PACKET_ID id = selective ? COMM_GET_VALUES_SETUP_SELECTIVE : COMM_GET_VALUES_SETUP;
    b[i++] = (uint8_t)id;
    if (selective) buffer_append_uint32(b, mask, &i);

    mc_values v;
    get_values_normalized(second, &v);
    if (mask & (1u << 0)) buffer_append_float16(b, v.temp_mos, 1e1f, &i);
    if (mask & (1u << 1)) buffer_append_float16(b, v.temp_motor, 1e1f, &i);
    if (mask & (1u << 2)) buffer_append_float32(b, v.current_motor, 1e2f, &i);
    if (mask & (1u << 3)) buffer_append_float32(b, v.current_in, 1e2f, &i);
    if (mask & (1u << 4)) buffer_append_float16(b, v.duty_now, 1e3f, &i);
    if (mask & (1u << 5)) buffer_append_float32(b, v.rpm, 1e0f, &i);
    if (mask & (1u << 6)) buffer_append_float32(b, 0.0f, 1e3f, &i); /* vehicle speed: no wheel setup */
    if (mask & (1u << 7)) buffer_append_float16(b, v.v_in, 1e1f, &i);
    if (mask & (1u << 8)) buffer_append_float16(b, 0.0f, 1e3f, &i); /* battery level unavailable */
    if (mask & (1u << 9)) buffer_append_float32(b, v.amp_hours, 1e4f, &i);
    if (mask & (1u << 10)) buffer_append_float32(b, v.amp_hours_charged, 1e4f, &i);
    if (mask & (1u << 11)) buffer_append_float32(b, v.watt_hours, 1e4f, &i);
    if (mask & (1u << 12)) buffer_append_float32(b, v.watt_hours_charged, 1e4f, &i);
    if (mask & (1u << 13)) buffer_append_float32(b, 0.0f, 1e3f, &i); /* distance */
    if (mask & (1u << 14)) buffer_append_float32(b, 0.0f, 1e3f, &i); /* distance abs */
    if (mask & (1u << 15)) buffer_append_float32(b, v.position, 1e6f, &i);
    if (mask & (1u << 16)) b[i++] = (uint8_t)v.fault_code;
    if (mask & (1u << 17)) b[i++] = (uint8_t)v.vesc_id;
    if (mask & (1u << 18)) b[i++] = second ? 1u : 2u;
    if (mask & (1u << 19)) buffer_append_float32(b, 0.0f, 1e3f, &i); /* Wh battery left */
    if (mask & (1u << 20)) buffer_append_uint32(b, 0u, &i); /* persistent odometer not implemented */
    if (mask & (1u << 21)) buffer_append_uint32(b, HAL_GetTick(), &i);
    uart_send_payload(b, (uint16_t)i);
}

static void reply_values_setup(bool second, bool selective, const uint8_t *data, uint16_t len) {
    uint32_t mask = 0xffffffffu;
    if (selective) {
        if (len < 4u) return;
        int32_t r = 0;
        mask = buffer_get_uint32(data, &r);
    }
    send_values_setup_packet(second, selective, mask);
    /* GET_VALUES_SETUP is also one request -> one reply, matching upstream VESC. */
}

static float right_sign(bool second, float value) { return second ? -value : value; }

static void touch_motor(bool second) { mcpwm_foc_vesc_override_touch(second); }

static void reply_mcconf(bool second, COMM_PACKET_ID id) {
    static mc_configuration c;
    int32_t i = 0;
    s_config_payload[i++] = (uint8_t)id;
    if (id == COMM_GET_MCCONF_DEFAULT) {
        mcpwm_foc_get_default_configuration(&c, second);
    } else {
        mcpwm_foc_sync_tuning_to_conf(second);
        c = *mc_interface_get_configuration_motor(second);
    }
    const int32_t n = confgenerator_serialize_mcconf(&s_config_payload[i], &c);
    if (n > 0 && (uint32_t)(i + n) <= sizeof(s_config_payload)) uart_send_payload(s_config_payload, (uint16_t)(i + n));
}

static void set_mcconf(bool second, const uint8_t *data, uint16_t len) {
    static mc_configuration c;
    bool applied = false;
    c = *mc_interface_get_configuration_motor(second);
    const int32_t expected = confgenerator_serialize_mcconf(s_config_payload, &c);
    if (expected > 0 && len >= (uint16_t)expected && confgenerator_deserialize_mcconf(data, &c)) {
        c.motor_type = MOTOR_TYPE_FOC;
        if (c.l_current_max < 0.1f) c.l_current_max = 0.1f;
        if (c.l_current_max > (float)I_MOT_MAX) c.l_current_max = (float)I_MOT_MAX;
        if (c.l_current_min > -0.1f) c.l_current_min = -0.1f;
        if (c.l_current_min < -(float)I_MOT_MAX) c.l_current_min = -(float)I_MOT_MAX;
        /* Physical current sample/PI topology is fixed for this board. Motor
         * poles and drivetrain ratio are standard VESC setup fields and are live. */
        c.foc_sensor_mode = FOC_SENSOR_MODE_HALL;
        if (c.si_motor_poles < 2u || (c.si_motor_poles & 1u)) c.si_motor_poles = 30u;
        if (!(c.si_gear_ratio >= 0.01f && c.si_gear_ratio <= 1000.0f)) c.si_gear_ratio = 1.0f;
        mc_interface_select_motor_thread(second ? 2 : 1);
        mc_interface_set_configuration(&c);
        applied = mc_interface_store_configuration_motor(second);
    }
    /* Stock VESC 6.00 acknowledges SET_MCCONF with the command byte only. */
    (void)applied; /* ACK means the packet was handled; persistence is best-effort flash IO. */
    { uint8_t ack = COMM_SET_MCCONF; uart_send_payload(&ack, 1u); }
}

static void reply_appconf(bool second, COMM_PACKET_ID id) {
    int32_t i = 0;
    app_configuration defaults;
    const app_configuration *a;
    s_config_payload[i++] = (uint8_t)id;
    if (id == COMM_GET_APPCONF_DEFAULT) {
        app_defaults(&defaults, second ? VESC_SECOND_MOTOR_ID : VESC_LOCAL_ID);
        a = &defaults;
    } else {
        a = app_vesc_get_configuration(second);
    }
    const int32_t n = confgenerator_serialize_appconf(&s_config_payload[i], a);
    if (n > 0 && (uint32_t)(i + n) <= sizeof(s_config_payload)) uart_send_payload(s_config_payload, (uint16_t)(i + n));
}

static void set_appconf(bool second, const uint8_t *data, uint16_t len) {
    app_configuration tmp = *app_vesc_get_configuration(second);
    const int32_t expected = confgenerator_serialize_appconf(s_config_payload, &tmp);
    if (expected > 0 && len >= (uint16_t)expected && confgenerator_deserialize_appconf(data, &tmp)) {
        (void)app_vesc_set_configuration(second, &tmp);
    }
    uint8_t ack = COMM_SET_APPCONF;
    uart_send_payload(&ack, 1u);
}

static void reply_decoded_adc(void) {
    uint8_t b[20];
    int32_t i = 0;
    b[i++] = COMM_GET_DECODED_ADC;
    buffer_append_int32(b, (int32_t)(app_vesc_adc_decoded(false) * 1000000.0f), &i);
    buffer_append_int32(b, (int32_t)(app_vesc_adc_voltage(false) * 1000000.0f), &i);
    buffer_append_int32(b, (int32_t)(app_vesc_adc_decoded(true) * 1000000.0f), &i);
    buffer_append_int32(b, (int32_t)(app_vesc_adc_voltage(true) * 1000000.0f), &i);
    uart_send_payload(b, (uint16_t)i);
}

static void detect_hall_foc(bool second, const uint8_t *data, uint16_t len) {
    uint8_t reply[10];
    int32_t i = 0;
    float current = 1.0f;
    reply[0] = COMM_DETECT_HALL_FOC;
    for (uint8_t h = 0u; h < 8u; ++h) reply[1u + h] = 255u;
    reply[9] = 1u;
    if (len >= 4u) current = (float)buffer_get_int32(data, &i) / 1000.0f;
    if (mcpwm_foc_detect_hall(current, second, &reply[1])) {
        /* VESC result byte reports DETECTION success. Persistence is an
         * additional project feature and is exposed separately in diagnostics;
         * a flash-write problem must not be misreported as "bad hall". */
        s_last_hall_store_ok[second ? 1u : 0u] =
            mc_interface_store_configuration_motor(second) ? 1u : 0u;
        reply[9] = 0u;
    } else {
        s_last_hall_store_ok[second ? 1u : 0u] = 0u;
    }
    uart_send_payload(reply, sizeof(reply));
}

static int32_t q4_to_milliamps_normalized(int16_t q4, bool second) {
    int32_t ma = ((int32_t)q4 * 1000) / ((int32_t)A2BIT_CONV * 16);
    return second ? -ma : ma;
}

static void reply_custom_pos_state(bool second, uint8_t op, uint8_t status) {
    uint8_t b[32];
    int32_t i = 0;
    b[i++] = COMM_CUSTOM_APP_DATA;
    b[i++] = HB_CUSTOM_MAGIC0;
    b[i++] = HB_CUSTOM_MAGIC1;
    b[i++] = HB_CUSTOM_VERSION;
    b[i++] = op;
    b[i++] = status;
    buffer_append_int32(b, mcpwm_foc_get_position_user_counts(second), &i);
    buffer_append_int32(b, mcpwm_foc_get_position_target_user_counts(second), &i);
    buffer_append_int32(b, mcpwm_foc_get_position_min_user_counts(second), &i);
    buffer_append_int32(b, mcpwm_foc_get_position_max_user_counts(second), &i);
    uart_send_payload(b, (uint16_t)i);
}

static void process_custom_app(bool second, const uint8_t *data, uint16_t len) {
    if (!data || len < 4u || data[0] != HB_CUSTOM_MAGIC0 ||
        data[1] != HB_CUSTOM_MAGIC1 || data[2] != HB_CUSTOM_VERSION) {
        return;
    }
    const uint8_t op = data[3];
    const uint8_t *d = data + 4;
    const uint16_t n = (uint16_t)(len - 4u);
    int32_t k = 0;

    if (op == HB_CUSTOM_GET_POS_STATE) {
        reply_custom_pos_state(second, op, 0u);
        return;
    }
    if (op == HB_CUSTOM_SET_POS_LIMITS) {
        if (n < 8u) { reply_custom_pos_state(second, op, 1u); return; }
        const int32_t minc = buffer_get_int32(d, &k);
        const int32_t maxc = buffer_get_int32(d, &k);
        if (minc > maxc) { reply_custom_pos_state(second, op, 2u); return; }
        mcpwm_foc_set_position_user_limits(minc, maxc, second);
        reply_custom_pos_state(second, op, 0u);
        return;
    }
    if (op == HB_CUSTOM_SET_POS_TARGET) {
        if (n < 4u) { reply_custom_pos_state(second, op, 1u); return; }
        const int32_t target = buffer_get_int32(d, &k);
        touch_motor(second);
        mcpwm_foc_set_position_user_counts(target, second);
        reply_custom_pos_state(second, op, 0u);
        return;
    }
    if (op == HB_CUSTOM_RESET_POSITION) {
        mcpwm_foc_reset_position(second);
        reply_custom_pos_state(second, op, 0u);
        return;
    }
    if (op == HB_CUSTOM_GET_DIAG) {
        uint8_t b[128];
        int32_t i = 0;
        const mcpwm_foc_motor_t *m = mcpwm_foc_get_motor_const(second);
        struct {
            mc_control_mode mode; mc_state state; mc_fault_code fault;
            uint8_t hall,hall_pos,hall_prev,interp,rej_reason,rej_from,rej_to,hall_init; int8_t hall_dir;
            int16_t iq_target,iq_set,iq,id,duty,rpm;
            int32_t pos,pos_target,pos_min,pos_max;
            uint16_t phase,phase_hall,phase_target,hall_period,hall_ticks;
            uint32_t hall_invalid,current_trips,period_rejects,sequence_rejects;
        } ds;
        __disable_irq();
        ds.mode=m->m_control_mode; ds.state=m->m_state; ds.fault=m->m_fault;
        ds.hall=m->m_hall_state; ds.hall_pos=m->m_hall_pos; ds.hall_prev=m->m_hall_pos_prev;
        ds.hall_dir=m->m_hall_direction; ds.interp=m->m_hall_interp_active; ds.rej_reason=m->m_hall_last_reject_reason;
        ds.rej_from=m->m_hall_last_reject_from; ds.rej_to=m->m_hall_last_reject_to; ds.hall_init=m->m_hall_initialized;
        ds.iq_target=m->m_iq_target_q4; ds.iq_set=m->m_iq_set_q4; ds.iq=m->m_iq_q4; ds.id=m->m_id_q4;
        ds.duty=m->m_duty_now_permille; ds.rpm=m->m_rpm; ds.pos=m->m_position_counts;
        ds.pos_target=m->m_position_target_counts; ds.pos_min=m->m_position_min_counts; ds.pos_max=m->m_position_max_counts;
        ds.phase=m->m_phase; ds.phase_hall=m->m_phase_hall; ds.phase_target=m->m_phase_hall_target;
        ds.hall_period=m->m_hall_period; ds.hall_ticks=m->m_hall_ticks;
        ds.hall_invalid=m->m_hall_invalid_transition_count; ds.current_trips=m->m_current_trip_count;
        ds.period_rejects=m->m_hall_period_reject_count; ds.sequence_rejects=m->m_hall_sequence_reject_count;
        __enable_irq();
        const uint16_t pp=mcpwm_foc_get_pole_pairs(second);
        float erpm_f=(float)ds.rpm*(float)pp;
        if(ds.hall_init && ds.hall_dir!=0 && ds.hall_period>0u && ds.hall_period<MCCONF_HALL_TIMEOUT_TICKS && ds.hall_ticks<=MCCONF_HALL_TIMEOUT_TICKS)
            erpm_f=((float)PWM_FREQ*10.0f/(float)ds.hall_period)*(float)ds.hall_dir;
        if (second) erpm_f = -erpm_f;
        int32_t erpm = (int32_t)(erpm_f >= 0.0f ? erpm_f + 0.5f : erpm_f - 0.5f);
        int32_t duty = (int32_t)ds.duty * 100;
        if (second) duty = -duty;
        const int32_t pos_user = second ? (ds.pos==INT32_MIN?INT32_MAX:-ds.pos) : ds.pos;
        const int32_t target_user = second ? (ds.pos_target==INT32_MIN?INT32_MAX:-ds.pos_target) : ds.pos_target;
        const int32_t min_user = second ? (ds.pos_max==INT32_MIN?INT32_MAX:-ds.pos_max) : ds.pos_min;
        const int32_t max_user = second ? (ds.pos_min==INT32_MIN?INT32_MAX:-ds.pos_min) : ds.pos_max;

        b[i++] = COMM_CUSTOM_APP_DATA;
        b[i++] = HB_CUSTOM_MAGIC0;
        b[i++] = HB_CUSTOM_MAGIC1;
        b[i++] = HB_CUSTOM_VERSION;
        b[i++] = op;
        b[i++] = 0u;
        b[i++] = second ? VESC_SECOND_MOTOR_ID : VESC_LOCAL_ID;
        b[i++] = (uint8_t)ds.mode;
        b[i++] = (uint8_t)ds.state;
        b[i++] = (uint8_t)ds.fault;
        b[i++] = ds.hall;
        b[i++] = mcpwm_foc_vesc_override_active(second) ? 1u : 0u;
        b[i++] = s_last_hall_store_ok[second ? 1u : 0u];
        b[i++] = 0u;
        buffer_append_int32(b, q4_to_milliamps_normalized(ds.iq_target, second), &i);
        buffer_append_int32(b, q4_to_milliamps_normalized(ds.iq_set, second), &i);
        buffer_append_int32(b, q4_to_milliamps_normalized(ds.iq, second), &i);
        buffer_append_int32(b, q4_to_milliamps_normalized(ds.id, false), &i);
        buffer_append_int32(b, erpm, &i);
        buffer_append_int32(b, duty, &i);
        buffer_append_int32(b, pos_user, &i);
        buffer_append_int32(b, target_user, &i);
        buffer_append_int32(b, min_user, &i);
        buffer_append_int32(b, max_user, &i);
        buffer_append_uint32(b, ds.hall_invalid, &i);
        buffer_append_uint32(b, ds.current_trips, &i);
        buffer_append_uint32(b, s_rx_ok, &i);
        buffer_append_uint32(b, s_rx_crc_err, &i);
        for (uint8_t h = 0u; h < 8u; ++h) b[i++] = (uint8_t)m->m_conf.foc_hall_table[h];
        /* Extended Hall/phase diagnostics. Values are raw fixed-point so the
         * host can prove table->electrical-angle->FOC-phase mapping while the
         * bridge is active. Backward-compatible: legacy parsers may stop at 78. */
        b[i++] = (uint8_t)m->m_conf.foc_hall_table[ds.hall & 7u];
        b[i++] = ds.hall_pos;
        b[i++] = ds.hall_prev;
        b[i++] = (uint8_t)ds.hall_dir;
        b[i++] = ds.interp;
        b[i++] = ds.rej_reason;
        b[i++] = ds.rej_from;
        b[i++] = ds.rej_to;
        buffer_append_uint16(b, ds.phase, &i);
        buffer_append_uint16(b, ds.phase_hall, &i);
        buffer_append_uint16(b, ds.phase_target, &i);
        buffer_append_uint16(b, ds.hall_period, &i);
        buffer_append_uint16(b, ds.hall_ticks, &i);
        buffer_append_uint32(b, ds.period_rejects, &i);
        buffer_append_uint32(b, ds.sequence_rejects, &i);
        /* Drivetrain/current-calibration diagnostics are project extensions
         * appended after the stable V16 payload. VESC standard telemetry remains
         * unchanged; older host parsers can stop at byte 104. */
        {
            int16_t po0=0,po1=0,dco=0;
            mcpwm_foc_get_current_offsets(&po0,&po1,&dco,second);
            buffer_append_int16(b,po0,&i); buffer_append_int16(b,po1,&i); buffer_append_int16(b,dco,&i);
            b[i++]=(uint8_t)m->m_conf.si_motor_poles;
            b[i++]=(uint8_t)pp;
            const float gear=mcpwm_foc_get_gear_ratio(second);
            buffer_append_int32(b,(int32_t)(gear*1000.0f+0.5f),&i);
            { const float mr=(pp>0u?erpm_f/(float)pp:0.0f);
              const float orpm=mr/gear;
              buffer_append_int32(b,(int32_t)(mr>=0.0f?mr*1000.0f+0.5f:mr*1000.0f-0.5f),&i);
              buffer_append_int32(b,(int32_t)(orpm>=0.0f?orpm*1000.0f+0.5f:orpm*1000.0f-0.5f),&i); }
            buffer_append_uint32(b,s_rx_queue_drop,&i);
        }
        uart_send_payload(b, (uint16_t)i);
    }
}

static void process_command(const uint8_t *p, uint16_t len, bool second) {
    if (!p || len == 0u) return;
    const COMM_PACKET_ID id = (COMM_PACKET_ID)p[0];
    const uint8_t *d = p + 1;
    const uint16_t n = (uint16_t)(len - 1u);
    int32_t k = 0;
    mc_interface_select_motor_thread(second ? 2 : 1);

    switch (id) {
    case COMM_FW_VERSION:
        reply_fw_version(second);
        break;
    case COMM_GET_VALUES:
        reply_values(second, false, d, n);
        break;
    case COMM_GET_VALUES_SELECTIVE:
        reply_values(second, true, d, n);
        break;
    case COMM_GET_VALUES_SETUP:
        reply_values_setup(second, false, d, n);
        break;
    case COMM_GET_VALUES_SETUP_SELECTIVE:
        reply_values_setup(second, true, d, n);
        break;
    case COMM_SET_DUTY:
        if (n >= 4u) {
            const float duty = (float)buffer_get_int32(d, &k) / 100000.0f;
            touch_motor(second); mc_interface_set_duty(right_sign(second, duty));
        }
        break;
    case COMM_SET_CURRENT:
        if (n >= 4u) {
            const float current = (float)buffer_get_int32(d, &k) / 1000.0f;
            touch_motor(second); mc_interface_set_current(right_sign(second, current));
        }
        break;
    case COMM_SET_CURRENT_BRAKE:
        if (n >= 4u) {
            float current = (float)buffer_get_int32(d, &k) / 1000.0f;
            if (current < 0.0f) current = -current;
            touch_motor(second); mc_interface_set_brake_current(current);
        }
        break;
    case COMM_SET_HANDBRAKE:
        if (n >= 4u) {
            /* This hardware has no separate mechanical/electromagnetic handbrake
             * actuator. Match the VESC command contract by applying the requested
             * holding current through the FOC brake-current path. */
            float current = (float)buffer_get_int32(d, &k) / 1000.0f;
            if (current < 0.0f) current = -current;
            touch_motor(second); mc_interface_set_brake_current(current);
        }
        break;
    case COMM_SET_RPM:
        if (n >= 4u) {
            const float rpm = (float)buffer_get_int32(d, &k);
            touch_motor(second); mc_interface_set_pid_speed(right_sign(second, rpm));
        }
        break;
    case COMM_SET_POS:
        if(n>=4u){const float pos=(float)buffer_get_int32(d,&k)/1000000.0f;touch_motor(second);mc_interface_set_pid_pos(right_sign(second,pos));}
        break;
    case COMM_ALIVE:
        touch_motor(second);
        break;
    case COMM_GET_MCCONF:
    case COMM_GET_MCCONF_DEFAULT:
        reply_mcconf(second, id);
        break;
    case COMM_SET_MCCONF:
        set_mcconf(second, d, n);
        break;
    case COMM_GET_APPCONF:
    case COMM_GET_APPCONF_DEFAULT:
        reply_appconf(second, id);
        break;
    case COMM_GET_DECODED_ADC:
        reply_decoded_adc();
        break;
    case COMM_SET_APPCONF:
        set_appconf(second, d, n);
        break;
    case COMM_DETECT_HALL_FOC:
        detect_hall_foc(second, d, n);
        break;
    case COMM_CUSTOM_APP_DATA:
        process_custom_app(second, d, n);
        break;
    default:
        /* Unsupported commands are intentionally ignored, as stock VESC commands.c does
         * for command IDs without a hardware implementation. */
        break;
    }
    mc_interface_select_motor_thread(1);
}

static void process_top_packet(const uint8_t *p, uint16_t len) {
    if (!p || len == 0u) return;
    const COMM_PACKET_ID id = (COMM_PACKET_ID)p[0];
    if (id == COMM_FORWARD_CAN) {
        if (len >= 3u && p[1] == VESC_SECOND_MOTOR_ID) {
            /* Exact dual-motor VESC semantic: forwarding to the virtual second CAN ID
             * selects motor thread 2 and executes the nested packet locally. */
            process_command(p + 2, (uint16_t)(len - 2u), true);
        }
        return;
    }
    if (id == COMM_PING_CAN) {
        uint8_t b[2] = {COMM_PING_CAN, VESC_SECOND_MOTOR_ID};
        uart_send_payload(b, sizeof(b));
        return;
    }
    process_command(p, len, false);
}

void vesc_protocol_process_pending(void) {
    for (;;) {
        uint16_t n = 0u;
        uint8_t slot = 0u;
        __disable_irq();
        if (s_pending_count == 0u) {
            __enable_irq();
            break;
        }
        slot = s_pending_tail;
        n = s_pending_len[slot];
        if (n > VESC_MAX_PAYLOAD) n = VESC_MAX_PAYLOAD;
        memcpy(s_process_payload, s_pending_payload[slot], n);
        s_pending_len[slot] = 0u;
        s_pending_tail = (uint8_t)((slot + 1u) % VESC_RX_QUEUE_DEPTH);
        s_pending_count--;
        __enable_irq();
        s_link_last_ms = HAL_GetTick();
        process_top_packet(s_process_payload, n);
    }
}
