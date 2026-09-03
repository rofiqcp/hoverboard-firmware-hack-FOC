#include <string.h>
#include "stm32f1xx_hal.h"
#include "config.h"
#include "defines.h"
#include "motor/mcpwm_foc.h"
#include "motor/foc_math.h"
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
#define VESC_RX_QUEUE_DEPTH          8u
#define VESC_TX_QUEUE_DEPTH          8u

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
#define HB_CUSTOM_GET_TUNING             6u
#define HB_CUSTOM_SET_TUNING             7u
#define HB_CUSTOM_SET_ID_TEST            8u

extern UART_HandleTypeDef huart3;
extern int16_t board_temp_deg_c;
extern volatile adc_buf_t adc_buffer;

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
static volatile uint32_t s_rx_queue_highwater = 0u;
static uint32_t s_process_last_ms = 0u;
static uint32_t s_process_gap_max_ms = 0u;
static volatile uint32_t s_link_last_ms = 0u;
static volatile uint32_t s_rx_ok = 0u;
static volatile uint32_t s_rx_crc_err = 0u;
/* Framed reply FIFO. The in-flight slot remains owned by DMA until UART gState
 * returns READY, so no response buffer can be overwritten mid-transmission. */
static uint8_t s_tx_frame[VESC_TX_QUEUE_DEPTH][VESC_MAX_FRAME];
static uint16_t s_tx_len[VESC_TX_QUEUE_DEPTH];
static uint8_t s_tx_head = 0u;
static uint8_t s_tx_tail = 0u;
static uint8_t s_tx_count = 0u;
static uint8_t s_tx_active = 0u;
static uint32_t s_tx_queue_drop = 0u;
static uint32_t s_tx_start_fail = 0u;
static volatile uint8_t s_last_hall_store_ok[2] = {0u, 0u};

/* Stock VESC handles COMM_DETECT_HALL_FOC as a blocking command in a dedicated
 * worker thread. This F103 target is bare-metal, so reproduce the same external
 * behavior with a cooperative state machine: UART request/reply remains alive
 * during the ~12 s detection, while only the selected motor is locked. Detect
 * itself never applies or stores the table; VESC Tool's Apply + SET_MCCONF does. */
typedef enum {
    HALL_DETECT_IDLE = 0,
    HALL_DETECT_ALIGN,
    HALL_DETECT_SWEEP
} hall_detect_stage_t;

typedef struct {
    uint8_t active;
    uint8_t second;
    hall_detect_stage_t stage;
    uint8_t pass;
    uint8_t waiting_sample;
    uint16_t align_step;
    int16_t degree;
    uint32_t next_ms;
    float current_a;
    int64_t sum_s[8];
    int64_t sum_c[8];
    uint16_t samples[8];
} hall_detect_job_t;

static hall_detect_job_t s_hall_detect;

typedef struct {
    uint8_t active;
    uint8_t motor_index;
    float min_current_in;
    float max_current_in;
    float openloop_rpm;
    float sl_erpm;
    uint8_t hall[2][8];
} detect_all_job_t;

static detect_all_job_t s_detect_all;
static void hall_detect_periodic(uint32_t now_ms);
static void reply_mcconf(bool second, COMM_PACKET_ID id);
/* VESC Tool rotor-position display stream. Upstream commands.c stores one
 * display_position_mode per VESC instance; this dual virtual target stores the
 * last selected endpoint so COMM_FORWARD_CAN ID2 behaves like a second VESC. */
static volatile disp_pos_mode s_display_pos_mode = DISP_POS_MODE_NONE;
static volatile uint8_t s_display_second = 0u;
static uint32_t s_display_prev_ms = 0u;

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
    s_rx_queue_highwater = 0u;
    s_process_last_ms = 0u;
    s_process_gap_max_ms = 0u;
    s_link_last_ms = 0u;
    s_rx_ok = 0u;
    s_rx_crc_err = 0u;
    s_tx_head = s_tx_tail = s_tx_count = s_tx_active = 0u;
    memset(s_tx_len, 0, sizeof(s_tx_len));
    s_tx_queue_drop = 0u;
    s_tx_start_fail = 0u;
    s_last_hall_store_ok[0] = s_last_hall_store_ok[1] = 0u;
    memset(&s_hall_detect, 0, sizeof(s_hall_detect));
    memset(&s_detect_all, 0, sizeof(s_detect_all));
    s_display_pos_mode = DISP_POS_MODE_NONE;
    s_display_second = 0u;
    s_display_prev_ms = 0u;
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
            if ((uint32_t)s_pending_count > s_rx_queue_highwater) s_rx_queue_highwater = s_pending_count;
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

static void vesc_tx_service(void) {
    /* Retire the slot only after DMA + UART shift register are fully done. */
    if (s_tx_active) {
        if (huart3.gState != HAL_UART_STATE_READY) return;
        s_tx_active = 0u;
        if (s_tx_count != 0u) {
            s_tx_len[s_tx_tail] = 0u;
            s_tx_tail = (uint8_t)((s_tx_tail + 1u) % VESC_TX_QUEUE_DEPTH);
            s_tx_count--;
        }
    }
    if (s_tx_count == 0u || huart3.gState != HAL_UART_STATE_READY) return;
    const uint8_t slot = s_tx_tail;
    const uint16_t n = s_tx_len[slot];
    if (n == 0u || n > VESC_MAX_FRAME) {
        s_tx_tail = (uint8_t)((s_tx_tail + 1u) % VESC_TX_QUEUE_DEPTH);
        s_tx_count--;
        return;
    }
    if (HAL_UART_Transmit_DMA(&huart3, s_tx_frame[slot], n) == HAL_OK) {
        s_tx_active = 1u;
    } else {
        /* Never block the control/main loop. Retry the same queued frame on the
         * next service pass; a transient DMA busy state cannot starve RX. */
        s_tx_start_fail++;
    }
}

static void uart_send_payload(const uint8_t *payload, uint16_t len) {
    if (!payload || len == 0u || len > VESC_MAX_PAYLOAD) return;
    vesc_tx_service();
    if (s_tx_count >= VESC_TX_QUEUE_DEPTH) {
        s_tx_queue_drop++;
        return;
    }
    const uint8_t slot = s_tx_head;
    uint8_t *tx = s_tx_frame[slot];
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
    s_tx_len[slot] = i;
    s_tx_head = (uint8_t)((slot + 1u) % VESC_TX_QUEUE_DEPTH);
    s_tx_count++;
    vesc_tx_service();
}

static float wrap_angle_diff_deg(float a, float b) {
    float d = a - b;
    while (d > 180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

static bool display_rotor_pos(bool second, disp_pos_mode mode, float *out) {
    if (!out) return false;
    const mcpwm_foc_motor_t *m = mcpwm_foc_get_motor_const(second);
    const float phase = mcpwm_foc_get_phase_motor(second);
    switch (mode) {
    case DISP_POS_MODE_OBSERVER:
        /* Hall-only FOC has no sensorless observer. m_phase is the live
         * interpolated electrical phase actually used by Park/SVPWM, which is
         * the correct observer-equivalent display quantity on this hardware. */
        *out = phase;
        return true;
    case DISP_POS_MODE_ENCODER:
        /* No separate ABI/SPI encoder peripheral exists on this hoverboard
         * target. Keep VESC Tool's display alive with the measured rotor phase
         * rather than inventing an independent encoder signal. */
        *out = phase;
        return true;
    case DISP_POS_MODE_PID_POS:
        *out = phase;
        return true;
    case DISP_POS_MODE_PID_POS_ERROR: {
        if(m->m_pos_pid_phase_mode){
            const float target=(float)m->m_pos_pid_set_phase*(360.0f/65536.0f);
            const float err=wrap_angle_diff_deg(target,phase);
            *out=second?-err:err;
        }else{
            const int32_t dc=m->m_position_target_counts-m->m_position_counts;
            const float pp=(float)mcpwm_foc_get_pole_pairs(second);
            const float err=(pp>0.0f)?((float)dc*60.0f/pp):0.0f;
            *out=second?-err:err;
        }
        return true;
    }
    case DISP_POS_MODE_ENCODER_OBSERVER_ERROR:
        /* Encoder and observer are the same physical Hall-derived phase on this
         * board, therefore there is no independent encoder-observer error. */
        *out = 0.0f;
        return true;
    case DISP_POS_MODE_HALL_OBSERVER_ERROR: {
        const float hall = (float)m->m_phase_hall * (360.0f / 65536.0f);
        *out = wrap_angle_diff_deg(phase, hall);
        return true;
    }
    default:
        return false;
    }
}

void vesc_protocol_periodic(uint32_t now_ms) {
    hall_detect_periodic(now_ms);
    const disp_pos_mode mode = s_display_pos_mode;
    if (mode == DISP_POS_MODE_NONE) return;
    if ((uint32_t)(now_ms - s_display_prev_ms) < 10u) return;
    s_display_prev_ms = now_ms;
    if (!vesc_protocol_link_active()) return;
    /* Do not delay a solicited VESC Tool reply. Upstream uses a separate packet
     * transport thread; on this small bare-metal target we skip one 10-ms rotor
     * sample whenever RX/TX is busy instead of blocking realtime traffic. */
    if (s_rx_active || s_pending_count != 0u || huart3.gState != HAL_UART_STATE_READY) return;
    const bool second = s_display_second != 0u;
    float pos = 0.0f;
    if (!display_rotor_pos(second, mode, &pos)) return;
    /* Keep COMM_ROTOR_POSITION in the same user-facing wheel convention as
     * COMM_GET_VALUES. The right power stage is mirrored, so absolute display
     * angles are exposed as 360-internal_angle. Error modes are signed
     * differences and must not receive this absolute-angle transform. */
    if (second && (mode == DISP_POS_MODE_OBSERVER || mode == DISP_POS_MODE_ENCODER ||
                   mode == DISP_POS_MODE_PID_POS)) {
        if (pos > 0.0f) {
            pos = 360.0f - pos;
            if (pos >= 360.0f) pos = 0.0f;
        }
    }
    uint8_t b[5];
    int32_t i = 0;
    b[i++] = COMM_ROTOR_POSITION;
    buffer_append_int32(b, (int32_t)(pos * 100000.0f), &i);
    uart_send_payload(b, (uint16_t)i);
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
        {
            float commanded_abs = c.l_current_max;
            if (-c.l_current_min > commanded_abs) commanded_abs = -c.l_current_min;
            if (!(c.l_abs_current_max >= commanded_abs) ||
                c.l_abs_current_max > MCCONF_L_ABS_CURRENT_MAX) {
                c.l_abs_current_max = MCCONF_L_ABS_CURRENT_MAX;
            }
        }
        if (!(c.l_max_duty > 0.0f) || c.l_max_duty > MCCONF_L_MAX_DUTY) c.l_max_duty=MCCONF_L_MAX_DUTY;
        if (!(c.l_in_current_max >= 0.1f) || c.l_in_current_max > (float)I_DC_MAX) c.l_in_current_max=MCCONF_L_IN_CURRENT_MAX;
        if (!(c.l_in_current_min <= -0.1f) || c.l_in_current_min < -(float)I_DC_MAX) c.l_in_current_min=MCCONF_L_IN_CURRENT_MIN;
        if (!(c.m_duty_ramp_step >= 0.0001f && c.m_duty_ramp_step <= 0.20f)) c.m_duty_ramp_step=MCCONF_DUTY_RAMP_STEP_DEFAULT;
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
        /* VESC Tool SET_APPCONF is persistent. Apply first, then store only the
         * implemented application subset into EEPROM-emulation slots. */
        if (app_vesc_set_configuration(second, &tmp)) (void)app_vesc_store_configuration(second);
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

static uint8_t hall_detect_angle200(int64_t sum_s, int64_t sum_c, uint16_t n) {
    if (n <= 30u) return 255u;
    int64_t best_dot = INT64_MIN;
    uint8_t best = 255u;
    for (uint16_t a = 0u; a < 200u; ++a) {
        int16_t sn, cs;
        const uint16_t ph = (uint16_t)(((uint32_t)a * 65536u) / 200u);
        foc_sin_cos_q15(ph, &sn, &cs);
        const int64_t dot = sum_s * sn + sum_c * cs;
        if (dot > best_dot) { best_dot = dot; best = (uint8_t)a; }
    }
    return best;
}

static bool hall_detect_motor_locked(bool second) {
    return s_hall_detect.active && (s_hall_detect.second != 0u) == second;
}

static void hall_detect_start_current(bool second, float current) {
    const mcpwm_foc_motor_t *m = mcpwm_foc_get_motor_const(second);
    float max_i = m->m_conf.l_current_max;
    if (max_i <= 0.0f || max_i > (float)I_MOT_MAX) max_i = (float)I_MOT_MAX;
    if(current<0.0f)current=-current;
    if(current<0.10f)current=0.10f;
    if(current>max_i)current=max_i;

    memset(&s_hall_detect, 0, sizeof(s_hall_detect));
    s_hall_detect.active = 1u;
    s_hall_detect.second = second ? 1u : 0u;
    s_hall_detect.stage = HALL_DETECT_ALIGN;
    s_hall_detect.current_a = current;
    s_hall_detect.next_ms = HAL_GetTick();
    mc_interface_select_motor_thread(second ? 2 : 1);
    mcpwm_foc_set_openloop_phase(0.0f, 0.0f, second);
    mcpwm_foc_vesc_override_touch(second);
    mc_interface_select_motor_thread(1);
}

static void detect_all_reply(int16_t result) {
    uint8_t b[3];
    int32_t i=0;
    b[i++]=COMM_DETECT_APPLY_ALL_FOC;
    buffer_append_int16(b,result,&i);
    uart_send_payload(b,(uint16_t)i);
}

static bool detect_all_apply_motor(bool second, const uint8_t hall[8]) {
    mc_configuration c=*mc_interface_get_configuration_motor(second);
    memcpy(c.foc_hall_table,hall,8u);
    c.motor_type=MOTOR_TYPE_FOC;
    c.foc_sensor_mode=FOC_SENSOR_MODE_HALL;
    if(s_detect_all.min_current_in < -0.001f &&
       s_detect_all.min_current_in >= -(float)I_DC_MAX) c.l_in_current_min=s_detect_all.min_current_in;
    if(s_detect_all.max_current_in > 0.001f &&
       s_detect_all.max_current_in <= (float)I_DC_MAX) c.l_in_current_max=s_detect_all.max_current_in;
    if(s_detect_all.openloop_rpm > 0.001f &&
       s_detect_all.openloop_rpm <= (float)MCCONF_OPENLOOP_RPM_MAX) c.foc_openloop_rpm=s_detect_all.openloop_rpm;
    if(s_detect_all.sl_erpm > 0.001f) c.foc_sl_erpm=s_detect_all.sl_erpm;

    mc_interface_select_motor_thread(second?2:1);
    mc_interface_set_configuration(&c);
    const bool ok=mc_interface_store_configuration_motor(second);
    mc_interface_select_motor_thread(1);
    s_last_hall_store_ok[second?1u:0u]=ok?1u:0u;
    return ok;
}

static void detect_all_finish(int16_t result) {
    mc_interface_select_motor_thread(1);
    mc_interface_release_motor();
    mc_interface_select_motor_thread(2);
    mc_interface_release_motor();
    mc_interface_select_motor_thread(1);
    memset(&s_hall_detect,0,sizeof(s_hall_detect));
    s_detect_all.active=0u;
    /* Upstream sends the freshly detected MC configuration before the final
     * Detect-All result. VESC Tool relies on this to render the success page. */
    if(result>=0) reply_mcconf(false,COMM_GET_MCCONF);
    detect_all_reply(result);
}

static void hall_detect_reply_and_stop(bool success) {
    uint8_t table[8];
    uint8_t fails=0u;
    for(uint8_t h=0u;h<8u;++h){
        const uint8_t a=hall_detect_angle200(s_hall_detect.sum_s[h],s_hall_detect.sum_c[h],
                                             s_hall_detect.samples[h]);
        table[h]=a;
        if(a==255u)fails++;
    }
    if(fails!=2u)success=false;
    const bool second=s_hall_detect.second!=0u;
    mc_interface_select_motor_thread(second?2:1);
    mc_interface_release_motor();
    mcpwm_foc_vesc_override_clear(second);
    mc_interface_select_motor_thread(1);

    if(s_detect_all.active){
        if(!success){
            detect_all_finish(-10);
            return;
        }
        memcpy(s_detect_all.hall[second?1u:0u],table,8u);
        if(!detect_all_apply_motor(second,table)){
            detect_all_finish(-1);
            return;
        }
        memset(&s_hall_detect,0,sizeof(s_hall_detect));
        if(!second){
            s_detect_all.motor_index=1u;
            /* 1 A is the validated Hall-detect operating point on this board.
             * It is clamped to each motor's configured current limit. */
            hall_detect_start_current(true,1.0f);
            return;
        }
        /* Non-negative result is success to VESC Tool; 2 denotes Hall sensing. */
        detect_all_finish(2);
        return;
    }

    uint8_t reply[10];
    reply[0]=COMM_DETECT_HALL_FOC;
    memcpy(&reply[1],table,8u);
    reply[9]=success?0u:1u;
    s_last_hall_store_ok[second?1u:0u]=0u; /* standalone detect is not a store */
    memset(&s_hall_detect,0,sizeof(s_hall_detect));
    uart_send_payload(reply,sizeof(reply));
}

static void hall_detect_begin(bool second, const uint8_t *data, uint16_t len) {
    if (s_hall_detect.active || s_detect_all.active) return;
    if (len < 4u) {
        uint8_t reply[10] = {COMM_DETECT_HALL_FOC,255,255,255,255,255,255,255,255,1};
        uart_send_payload(reply, sizeof(reply));
        return;
    }
    int32_t ind = 0;
    float current = (float)buffer_get_int32(data, &ind) / 1000.0f;
    hall_detect_start_current(second,current);
}

static void detect_all_begin(const uint8_t *data,uint16_t len) {
    if(s_hall_detect.active || s_detect_all.active) return;
    if(len<21u){ detect_all_reply(-1); return; }
    int32_t k=0;
    const uint8_t detect_can=data[k++];
    (void)detect_can; /* virtual ID2 is the on-board second motor, always detect it */
    const float max_power_loss=buffer_get_float32(data,1e3f,&k);
    (void)max_power_loss; /* board-specific current PI is pre-characterized; no sensorless R/L fit */
    memset(&s_detect_all,0,sizeof(s_detect_all));
    s_detect_all.active=1u;
    s_detect_all.min_current_in=buffer_get_float32(data,1e3f,&k);
    s_detect_all.max_current_in=buffer_get_float32(data,1e3f,&k);
    s_detect_all.openloop_rpm=buffer_get_float32(data,1e3f,&k);
    s_detect_all.sl_erpm=buffer_get_float32(data,1e3f,&k);

    /* Same safety semantics as upstream blocking Detect All: application output
     * remains gated and command timeout is refreshed for the motor under test. */
    app_vesc_disable_output(180000);
    mc_interface_select_motor_thread(1); mc_interface_release_motor();
    mc_interface_select_motor_thread(2); mc_interface_release_motor();
    mc_interface_select_motor_thread(1);
    hall_detect_start_current(false,1.0f);
}

static void hall_detect_periodic(uint32_t now_ms) {
    if (!s_hall_detect.active) return;
    if ((int32_t)(now_ms - s_hall_detect.next_ms) < 0) return;
    const bool second = s_hall_detect.second != 0u;
    mcpwm_foc_motor_t *m = mcpwm_foc_get_motor(second);
    mcpwm_foc_vesc_override_touch(second);
    if (m->m_fault != FAULT_CODE_NONE) { hall_detect_reply_and_stop(false); return; }

    if (s_hall_detect.stage == HALL_DETECT_ALIGN) {
        if (s_hall_detect.align_step < 1000u) {
            s_hall_detect.align_step++;
            const float i = s_hall_detect.current_a * (float)s_hall_detect.align_step / 1000.0f;
            mcpwm_foc_set_openloop_phase(i, 0.0f, second);
            mcpwm_foc_vesc_override_touch(second);
            s_hall_detect.next_ms = now_ms + 1u;
            return;
        }
        s_hall_detect.stage = HALL_DETECT_SWEEP;
        s_hall_detect.pass = 0u;
        s_hall_detect.degree = 0;
        s_hall_detect.waiting_sample = 0u;
    }

    if (s_hall_detect.stage != HALL_DETECT_SWEEP) return;
    if (!s_hall_detect.waiting_sample) {
        mcpwm_foc_set_openloop_phase(s_hall_detect.current_a, (float)s_hall_detect.degree, second);
        mcpwm_foc_vesc_override_touch(second);
        s_hall_detect.waiting_sample = 1u;
        s_hall_detect.next_ms = now_ms + 5u;
        return;
    }

    const uint8_t h = m->m_hall_state & 7u;
    const int32_t dnorm = (s_hall_detect.degree >= 360) ? 0 : s_hall_detect.degree;
    int16_t sn, cs;
    const uint16_t ph = (uint16_t)(((uint32_t)dnorm * 65536u) / 360u);
    foc_sin_cos_q15(ph, &sn, &cs);
    s_hall_detect.sum_s[h] += sn;
    s_hall_detect.sum_c[h] += cs;
    if (s_hall_detect.samples[h] < 0xffffu) s_hall_detect.samples[h]++;
    s_hall_detect.waiting_sample = 0u;

    if (s_hall_detect.pass < 3u) {
        if (s_hall_detect.degree >= 359) {
            s_hall_detect.pass++;
            s_hall_detect.degree = (s_hall_detect.pass < 3u) ? 0 : 360;
        } else {
            s_hall_detect.degree++;
        }
    } else {
        if (s_hall_detect.degree <= 0) {
            s_hall_detect.pass++;
            if (s_hall_detect.pass >= 6u) {
                hall_detect_reply_and_stop(true);
                return;
            }
            s_hall_detect.degree = 360;
        } else {
            s_hall_detect.degree--;
        }
    }
    s_hall_detect.next_ms = now_ms;
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
    if (op == HB_CUSTOM_GET_TUNING || op == HB_CUSTOM_SET_TUNING) {
        mcpwm_foc_motor_t *m=mcpwm_foc_get_motor(second);
        if (op == HB_CUSTOM_SET_TUNING) {
            if (n < 20u) { uint8_t e[6]={COMM_CUSTOM_APP_DATA,HB_CUSTOM_MAGIC0,HB_CUSTOM_MAGIC1,HB_CUSTOM_VERSION,op,1u}; uart_send_payload(e,6u); return; }
            m->m_kpq_q11=buffer_get_uint16(d,&k); m->m_kiq_q16=buffer_get_uint16(d,&k);
            m->m_kpd_q11=buffer_get_uint16(d,&k); m->m_kid_q16=buffer_get_uint16(d,&k);
            m->m_kps_q11=buffer_get_uint16(d,&k); m->m_kis_q16=buffer_get_uint16(d,&k); m->m_kds_q11=buffer_get_uint16(d,&k);
            m->m_kpp_q11=buffer_get_uint16(d,&k); m->m_kip_q16=buffer_get_uint16(d,&k); m->m_kdp_q11=buffer_get_uint16(d,&k);
            mcpwm_foc_sync_tuning_to_conf(second);
            bool store=false;
            if (n >= 22u) {
                uint16_t fa=buffer_get_uint16(d,&k);
                if(fa<1u)fa=1u;
                m->m_telem_current_filter_q16=fa;
                m->m_conf.foc_current_filter_const=(float)fa/65535.0f;
                if(n>=23u)store=d[22]!=0u;
            } else if (n >= 21u) store=d[20]!=0u;
            if (store) (void)mc_interface_store_configuration_motor(second);
        }
        uint8_t b[40]; int32_t j=0;
        b[j++]=COMM_CUSTOM_APP_DATA; b[j++]=HB_CUSTOM_MAGIC0; b[j++]=HB_CUSTOM_MAGIC1; b[j++]=HB_CUSTOM_VERSION; b[j++]=op; b[j++]=0u;
        buffer_append_uint16(b,m->m_kpq_q11,&j); buffer_append_uint16(b,m->m_kiq_q16,&j);
        buffer_append_uint16(b,m->m_kpd_q11,&j); buffer_append_uint16(b,m->m_kid_q16,&j);
        buffer_append_uint16(b,m->m_kps_q11,&j); buffer_append_uint16(b,m->m_kis_q16,&j); buffer_append_uint16(b,m->m_kds_q11,&j);
        buffer_append_uint16(b,m->m_kpp_q11,&j); buffer_append_uint16(b,m->m_kip_q16,&j); buffer_append_uint16(b,m->m_kdp_q11,&j);
        buffer_append_uint16(b,m->m_telem_current_filter_q16,&j); buffer_append_int16(b,m->m_current_limit_q4,&j);
        uart_send_payload(b,(uint16_t)j); return;
    }
    if (op == HB_CUSTOM_SET_ID_TEST) {
        if (n < 8u) { uint8_t e[6]={COMM_CUSTOM_APP_DATA,HB_CUSTOM_MAGIC0,HB_CUSTOM_MAGIC1,HB_CUSTOM_VERSION,op,1u}; uart_send_payload(e,6u); return; }
        const int32_t ma=buffer_get_int32(d,&k); const int32_t mdeg=buffer_get_int32(d,&k);
        touch_motor(second);
        if (ma == 0) mc_interface_release_motor();
        else mcpwm_foc_set_openloop_phase((float)(ma<0?-ma:ma)/1000.0f,(float)mdeg/1000.0f,second);
        { uint8_t a[6]={COMM_CUSTOM_APP_DATA,HB_CUSTOM_MAGIC0,HB_CUSTOM_MAGIC1,HB_CUSTOM_VERSION,op,0u}; uart_send_payload(a,6u); }
        return;
    }
    if (op == HB_CUSTOM_GET_DIAG) {
        uint8_t b[208];
        int32_t i = 0;
        const mcpwm_foc_motor_t *m = mcpwm_foc_get_motor_const(second);
        struct {
            mc_control_mode mode; mc_state state; mc_fault_code fault;
            uint8_t hall,hall_pos,hall_prev,interp,rej_reason,rej_from,rej_to,hall_init; int8_t hall_dir;
            int16_t iq_target,iq_set,iq,id,duty,rpm;
            int32_t pos,pos_target,pos_min,pos_max;
            uint16_t phase,phase_hall,phase_target,hall_period,hall_ticks;
            uint32_t hall_invalid,current_trips,period_rejects,sequence_rejects;
            uint32_t phase_trips,dc_trips;
            uint8_t phase_streak,last_trip_source;
            int16_t last_trip_p0,last_trip_p1,last_trip_p2,last_trip_dc,last_trip_duty;
            int16_t driven_off0,driven_off1,driven_offdc;
            uint16_t driven_samples; uint8_t driven_valid,driven_cal;
            int16_t off0,off1,offdc; uint16_t off_samples,off_settle; uint8_t off_valid;
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
        ds.phase_trips=m->m_phase_trip_count; ds.dc_trips=m->m_dc_trip_count; ds.phase_streak=m->m_phase_overcurrent_streak;
        ds.last_trip_source=m->m_last_trip_source; ds.last_trip_p0=m->m_last_trip_phase0_counts; ds.last_trip_p1=m->m_last_trip_phase1_counts;
        ds.last_trip_p2=m->m_last_trip_phase2_counts; ds.last_trip_dc=m->m_last_trip_dc_counts; ds.last_trip_duty=m->m_last_trip_duty_permille;
        ds.driven_off0=m->m_driven_offset0; ds.driven_off1=m->m_driven_offset1; ds.driven_offdc=m->m_driven_offsetdc;
        ds.driven_samples=m->m_driven_offset_samples; ds.driven_valid=m->m_driven_offset_valid; ds.driven_cal=m->m_driven_offset_calibrating;
        ds.off0=m->m_off_offset0; ds.off1=m->m_off_offset1; ds.offdc=m->m_off_offsetdc;
        ds.off_samples=m->m_off_offset_samples; ds.off_settle=m->m_off_settle_ticks; ds.off_valid=m->m_off_offset_valid;
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
            buffer_append_uint32(b,mcpwm_foc_get_isr_cycles(),&i);
            buffer_append_uint32(b,mcpwm_foc_get_isr_cycles_max(),&i);
            /* High-duty ABS-overcurrent diagnostics. Keep legacy fields first so
             * older tools remain compatible; these bytes identify whether the
             * last FAULT_CODE_ABS_OVER_CURRENT came from phase or DC sensing. */
            buffer_append_uint32(b,ds.phase_trips,&i); buffer_append_uint32(b,ds.dc_trips,&i);
            b[i++]=ds.phase_streak; b[i++]=ds.last_trip_source;
            buffer_append_int16(b,ds.last_trip_p0,&i); buffer_append_int16(b,ds.last_trip_p1,&i);
            buffer_append_int16(b,ds.last_trip_p2,&i); buffer_append_int16(b,ds.last_trip_dc,&i);
            buffer_append_int16(b,ds.last_trip_duty,&i);
            buffer_append_int16(b,ds.driven_off0,&i); buffer_append_int16(b,ds.driven_off1,&i); buffer_append_int16(b,ds.driven_offdc,&i);
            buffer_append_uint16(b,ds.driven_samples,&i); b[i++]=ds.driven_valid; b[i++]=ds.driven_cal;
            /* Raw ADC snapshot for calibration/telemetry validation. This is
             * diagnostic-only and never participates in VESC standard packets. */
            buffer_append_uint16(b,adc_buffer.rlA,&i); buffer_append_uint16(b,adc_buffer.rlB,&i);
            buffer_append_uint16(b,adc_buffer.dcl,&i); buffer_append_uint16(b,adc_buffer.rrB,&i);
            buffer_append_uint16(b,adc_buffer.rrC,&i); buffer_append_uint16(b,adc_buffer.dcr,&i);
            buffer_append_int16(b,ds.off0,&i); buffer_append_int16(b,ds.off1,&i); buffer_append_int16(b,ds.offdc,&i);
            buffer_append_uint16(b,ds.off_samples,&i); buffer_append_uint16(b,ds.off_settle,&i); b[i++]=ds.off_valid;
            buffer_append_uint32(b,s_tx_queue_drop,&i);
            buffer_append_uint32(b,s_tx_start_fail,&i);
            buffer_append_uint32(b,s_rx_queue_highwater,&i);
            buffer_append_uint32(b,s_process_gap_max_ms,&i);
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
    case COMM_SET_DETECT:
        if (n >= 1u) {
            const uint8_t raw_mode = d[0];
            if (raw_mode <= (uint8_t)DISP_POS_MODE_HALL_OBSERVER_ERROR) {
                s_display_second = second ? 1u : 0u;
                s_display_pos_mode = (disp_pos_mode)raw_mode;
                s_display_prev_ms = HAL_GetTick();
            } else {
                s_display_pos_mode = DISP_POS_MODE_NONE;
            }
        }
        break;
    case COMM_SET_DUTY:
        if (hall_detect_motor_locked(second)) break;
        if (n >= 4u) {
            const float duty = (float)buffer_get_int32(d, &k) / 100000.0f;
            touch_motor(second); mc_interface_set_duty(right_sign(second, duty));
        }
        break;
    case COMM_SET_CURRENT:
        if (hall_detect_motor_locked(second)) break;
        if (n >= 4u) {
            const float current = (float)buffer_get_int32(d, &k) / 1000.0f;
            touch_motor(second); mc_interface_set_current(right_sign(second, current));
        }
        break;
    case COMM_SET_CURRENT_BRAKE:
        if (hall_detect_motor_locked(second)) break;
        if (n >= 4u) {
            float current = (float)buffer_get_int32(d, &k) / 1000.0f;
            if (current < 0.0f) current = -current;
            touch_motor(second); mc_interface_set_brake_current(current);
        }
        break;
    case COMM_SET_HANDBRAKE:
        if (hall_detect_motor_locked(second)) break;
        if (n >= 4u) {
            float current = (float)buffer_get_int32(d, &k) / 1000.0f;
            if (current < 0.0f) current = -current;
            touch_motor(second); mc_interface_set_handbrake(current);
        }
        break;
    case COMM_SET_RPM:
        if (hall_detect_motor_locked(second)) break;
        if (n >= 4u) {
            const float rpm = (float)buffer_get_int32(d, &k);
            touch_motor(second); mc_interface_set_pid_speed(right_sign(second, rpm));
        }
        break;
    case COMM_SET_POS:
        if (hall_detect_motor_locked(second)) break;
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
        if (hall_detect_motor_locked(second)) break;
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
    case COMM_APP_DISABLE_OUTPUT:
        /* VESC Tool sends [forward_can][time_ms] before Detect All. This dual
         * board has no physical CAN bus, so one shared app-output gate covers
         * the local and virtual motor endpoints. */
        if(n>=5u){
            const uint8_t fwd_can=d[k++];
            const int32_t time_ms=buffer_get_int32(d,&k);
            (void)fwd_can;
            app_vesc_disable_output(time_ms);
        }
        break;
    case COMM_DETECT_HALL_FOC:
        hall_detect_begin(second, d, n);
        break;
    case COMM_DETECT_APPLY_ALL_FOC:
        if(!second) detect_all_begin(d,n);
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
    const uint32_t process_now_ms = HAL_GetTick();
    if (s_process_last_ms != 0u) {
        const uint32_t gap = process_now_ms - s_process_last_ms;
        if (gap > s_process_gap_max_ms) s_process_gap_max_ms = gap;
    }
    s_process_last_ms = process_now_ms;
    vesc_tx_service();
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
        vesc_tx_service();
    }
    vesc_tx_service();
}
