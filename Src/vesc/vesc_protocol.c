#include <string.h>
#include "stm32f1xx_hal.h"
#include <stdio.h>
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
#ifdef STM32F103xE
extern volatile uint32_t main_prof_vesc_max_cycles;
extern volatile uint32_t main_prof_house_max_cycles;
extern volatile uint32_t main_prof_tail_max_cycles;
extern volatile uint32_t foc_isr_pre_max_cycles;
extern volatile uint32_t foc_isr_control_max_cycles;
extern volatile uint32_t foc_isr_post_max_cycles;
extern volatile uint32_t foc_step_left_max_cycles;
extern volatile uint32_t foc_step_right_max_cycles;
extern volatile uint32_t foc_slot_max_cycles[3];
extern volatile uint32_t foc_prof_sensor_max_cycles;
extern volatile uint32_t foc_prof_current_max_cycles;
extern volatile uint32_t foc_prof_regulator_max_cycles;
extern volatile uint32_t foc_prof_svpwm_max_cycles;
#else
volatile uint32_t main_prof_vesc_max_cycles = 0u;
volatile uint32_t main_prof_house_max_cycles = 0u;
volatile uint32_t main_prof_tail_max_cycles = 0u;
#endif

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
/* Offset odometer per endpoint. VESC menghitung odometer sebagai nilai dasar
 * ditambah trip distance. Hardware ini tidak memiliki backup-domain RTC, jadi
 * offset bersifat runtime; jarak trip tetap berasal dari edge Hall nyata. */
static int64_t s_odometer_offset_m[2] = {0, 0};
static volatile uint32_t s_rx_ok = 0u;
static volatile uint32_t s_rx_crc_err = 0u;
/* Realtime setpoint mailbox. SET_* packets have no reply and repeated packets
 * supersede older setpoints. Coalescing them here prevents stale RPM/current/
 * position commands from filling the generic request FIFO while VESC Tool is
 * simultaneously polling GET_VALUES. The main loop applies one latest command
 * per motor; configuration/detection/read requests remain strictly FIFO. */
typedef struct { uint8_t pending; uint8_t len; uint8_t payload[5]; } rt_cmd_mailbox_t;
static volatile rt_cmd_mailbox_t s_rt_cmd[2];
static volatile uint32_t s_rt_cmd_coalesced = 0u;
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
#ifdef STM32F103xE
/* Wall-cycle profiler COMM_GET_VALUES. DWT elapsed sengaja termasuk preemption
 * FOC karena yang harus dipenuhi VESC Tool adalah latency end-to-end <20 ms. */
static uint32_t s_prof_values_snapshot_max_cycles = 0u;
static uint32_t s_prof_values_position_max_cycles = 0u;
static uint32_t s_prof_values_serialize_max_cycles = 0u;
static uint32_t s_prof_values_tx_max_cycles = 0u;
#endif
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
    uint32_t align_start_ms;
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
    memset((void *)s_rt_cmd, 0, sizeof(s_rt_cmd));
    s_rt_cmd_coalesced = 0u;
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

static bool rt_command_extract(const uint8_t *vp, uint16_t n, uint8_t *motor, const uint8_t **cmdp) {
    if (!vp || !motor || !cmdp) return false;
    const uint8_t *c = vp; uint16_t cn = n; uint8_t mi = 0u;
    if (n >= 3u && vp[0] == COMM_FORWARD_CAN && vp[1] == VESC_SECOND_MOTOR_ID) {
        c = vp + 2u; cn = (uint16_t)(n - 2u); mi = 1u;
    }
    if (cn != 5u) return false;
    switch ((COMM_PACKET_ID)c[0]) {
    case COMM_SET_DUTY: case COMM_SET_CURRENT: case COMM_SET_CURRENT_BRAKE:
    case COMM_SET_HANDBRAKE: case COMM_SET_RPM: case COMM_SET_POS:
    /* COMM_SET_CURRENT_REL is the same kind of no-reply realtime setpoint as
     * COMM_SET_CURRENT (1 cmd byte + int32 = 5 bytes either way) and was
     * missing from this list, so it fell through to the plain FIFO instead
     * of getting "latest setpoint wins" coalescing like its sibling. */
    case COMM_SET_CURRENT_REL:
        *motor = mi; *cmdp = c; return true;
    default: return false;
    }
}

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
        {
            uint8_t mi=0u; const uint8_t *cp=0;
            if (rt_command_extract(vp,n,&mi,&cp)) {
                rt_cmd_mailbox_t *mb=(rt_cmd_mailbox_t *)&s_rt_cmd[mi];
                if (mb->pending) s_rt_cmd_coalesced++;
                mb->len=5u;
                memcpy((void *)mb->payload,cp,5u);
                mb->pending=1u;
                /* Reset VESC timeout at wire-receive time. Even if the main loop
                 * is busy for a few milliseconds, a valid fresh setpoint must
                 * count as alive exactly like upstream timeout_reset(). */
                mcpwm_foc_vesc_override_touch(mi!=0u);
                s_link_last_ms=HAL_GetTick(); s_rx_ok++; rx_reset(); return;
            }
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
        /* VESC main.c mengirim encoder_read_deg(): mechanical sensor angle,
         * bukan corrected electrical phase. */
        *out = (!second && m->m_encoder_configured) ?
            mcpwm_foc_get_encoder_position_motor(false) : phase;
        return true;
    case DISP_POS_MODE_PID_POS:
        /* Upstream main.c memakai mc_interface_get_pid_pos_now(): sudah
         * dikembalikan ke koordinat user (encoder inversion, direction, offset). */
        *out = mc_interface_get_pid_pos_now_motor(second);
        return true;
    case DISP_POS_MODE_PID_POS_ERROR: {
        if(m->m_pos_pid_phase_mode){
            const float target=mc_interface_get_pid_pos_set_motor(second);
            const float now=mc_interface_get_pid_pos_now_motor(second);
            float err=wrap_angle_diff_deg(target,now);
            if(second)err=-err; /* normalisasi virtual motor kanan */
            *out=err;
        }else{
            const int32_t dc=m->m_position_target_counts-m->m_position_counts;
            float err;
            if(!second && m->m_encoder_configured && m->m_encoder_counts>=4u){
                err=(float)dc*360.0f/(float)m->m_encoder_counts;
                if(m->m_conf.foc_encoder_inverted)err=-err;
            }else{
                const float pp=(float)mcpwm_foc_get_pole_pairs(second);
                err=(pp>0.0f)?((float)dc*60.0f/pp):0.0f;
                if(second)err=-err;
            }
            *out=err;
        }
        return true;
    }
    case DISP_POS_MODE_ENCODER_OBSERVER_ERROR:
        if(!second && m->m_encoder_configured)
            /* Target belum memiliki observer sensorless; active phase adalah
             * referensi FOC terdekat. Gunakan sign upstream: observer-encoder. */
            *out = wrap_angle_diff_deg(phase,mcpwm_foc_get_phase_encoder_motor(false));
        else *out = 0.0f;
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
    /* COMM_SET_DETECT explicitly enables VESC Tool rotor-position streaming.
     * Upstream keeps display_position_mode active until another SET_DETECT
     * changes/disables it; do not couple this stream to the generic UART
     * link-hold timeout. Solicited traffic is still protected below. */
    /* Do not delay a solicited VESC Tool reply. Upstream uses a separate packet
     * transport thread; on this small bare-metal target we skip one 10-ms rotor
     * sample whenever RX/TX is busy instead of blocking realtime traffic. */
    if (s_rx_active || s_pending_count != 0u || huart3.gState != HAL_UART_STATE_READY) return;
    const bool second = s_display_second != 0u;
    float pos = 0.0f;
    if (!display_rotor_pos(second, mode, &pos)) return;
    /* COMM_FORWARD_CAN on dual-motor VESC only selects motor thread 2.
     * It never changes packet coordinates; direction is handled by the normal
     * Motor Configuration DIR_MULT path in mc_interface. */
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
    const mc_configuration *conf=(const mc_configuration *)mc_interface_get_configuration_motor(second);
    /* COMM_GET_VALUES upstream melewati mc_interface: direction inversion
     * berlaku pada RPM/duty/Iq/Vq/tachometer, sedangkan motor/input current dan
     * d-axis tetap tidak dibalik. Position adalah user PID position. */
    if(conf && conf->m_invert_direction){
        v->rpm=-v->rpm; v->iq=-v->iq; v->duty_now=-v->duty_now; v->vq=-v->vq;
        v->tachometer=-v->tachometer;
    }
    v->position=mc_interface_get_pid_pos_now_motor(second);
    /* Hoverboard temperature calibration is deci-degC (358 = 35.8C). */
    v->temp_mos = (float)board_temp_deg_c * 0.1f;
    v->temp_mos_1 = v->temp_mos;
    v->temp_mos_2 = v->temp_mos;
    v->temp_mos_3 = v->temp_mos;
    v->temp_motor = 0.0f;
    v->vesc_id = second ? VESC_SECOND_MOTOR_ID : VESC_LOCAL_ID;
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

#ifdef STM32F103xE
    {
        /* STM32F103 has no FPU. Use the integer-scaled snapshot for both full
         * and selective GET_VALUES so telemetry latency stays bounded while
         * the 16-kHz FOC ISR is active. Selective polling previously fell back
         * to the soft-float path and could exceed the host timeout under load. */
        mcpwm_foc_values_scaled_t v;
        uint32_t pv0=DWT->CYCCNT;
        mcpwm_foc_get_values_scaled(&v, second);
        uint32_t pv1=DWT->CYCCNT;
        uint32_t dt=(uint32_t)(pv1-pv0);
        if(dt>s_prof_values_snapshot_max_cycles)s_prof_values_snapshot_max_cycles=dt;
        const mc_configuration *conf=(const mc_configuration *)mc_interface_get_configuration_motor(second);
        const int32_t dir=(conf && conf->m_invert_direction)?-1:1;
        if(mask&(1u<<0)) buffer_append_int16(b, board_temp_deg_c, &i);
        if(mask&(1u<<1)) buffer_append_int16(b, 0, &i);
        if(mask&(1u<<2)) buffer_append_int32(b, v.current_motor_x100, &i);
        if(mask&(1u<<3)) buffer_append_int32(b, v.current_in_x100, &i);
        if(mask&(1u<<4)) buffer_append_int32(b, v.id_x100, &i);
        if(mask&(1u<<5)) buffer_append_int32(b, dir*v.iq_x100, &i);
        if(mask&(1u<<6)) buffer_append_int16(b, (int16_t)(dir*(int32_t)v.duty_x1000), &i);
        if(mask&(1u<<7)) buffer_append_int32(b, dir*v.erpm, &i);
        if(mask&(1u<<8)) buffer_append_int16(b, v.vin_x10, &i);
        if(mask&(1u<<9)) buffer_append_int32(b, v.ah_x10000, &i);
        if(mask&(1u<<10)) buffer_append_int32(b, v.ah_charged_x10000, &i);
        if(mask&(1u<<11)) buffer_append_int32(b, v.wh_x10000, &i);
        if(mask&(1u<<12)) buffer_append_int32(b, v.wh_charged_x10000, &i);
        if(mask&(1u<<13)) buffer_append_int32(b, dir*v.tachometer, &i);
        if(mask&(1u<<14)) buffer_append_int32(b, v.tachometer_abs, &i);
        if(mask&(1u<<15)) b[i++]=v.fault;
        if(mask&(1u<<16)) {
            uint32_t pv2=DWT->CYCCNT;
            buffer_append_float32(b, mc_interface_get_pid_pos_now_motor(second), 1e6f, &i);
            uint32_t pv3=DWT->CYCCNT;
            dt=(uint32_t)(pv3-pv2);
            if(dt>s_prof_values_position_max_cycles)s_prof_values_position_max_cycles=dt;
        }
        if(mask&(1u<<17)) b[i++]=second?VESC_SECOND_MOTOR_ID:VESC_LOCAL_ID;
        if(mask&(1u<<18)) {
            buffer_append_int16(b, board_temp_deg_c, &i);
            buffer_append_int16(b, board_temp_deg_c, &i);
            buffer_append_int16(b, board_temp_deg_c, &i);
        }
        if(mask&(1u<<19)) buffer_append_int32(b, v.vd_x1000, &i);
        if(mask&(1u<<20)) buffer_append_int32(b, dir*v.vq_x1000, &i);
        if(mask&(1u<<21)) b[i++]=0u;
        uint32_t pv4=DWT->CYCCNT;
        dt=(uint32_t)(pv4-pv1);
        if(dt>s_prof_values_serialize_max_cycles)s_prof_values_serialize_max_cycles=dt;
        uart_send_payload(b,(uint16_t)i);
        uint32_t pv5=DWT->CYCCNT;
        dt=(uint32_t)(pv5-pv4);
        if(dt>s_prof_values_tx_max_cycles)s_prof_values_tx_max_cycles=dt;
        return;
    }
#endif

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

/** Batasi nilai float tanpa menarik dependensi utilitas VESC yang tidak dipakai. */
static float vesc_clampf(float value, float min_value, float max_value) {
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

/**
 * Hitung level baterai seperti VESC 6.00 dari tipe baterai, jumlah sel, dan Vin.
 * Estimasi Wh hanya diaktifkan bila kapasitas Ah memang diisi pengguna; firmware
 * tidak mengarang kapasitas baterai yang tidak diketahui.
 */
static float setup_battery_level(const mc_configuration *conf, float vin, float *wh_left) {
    if (wh_left) *wh_left = 0.0f;
    if (!conf || conf->si_battery_cells < 1) return 0.0f;

    const float cells = (float)conf->si_battery_cells;
    const float cell_v = vin / cells;
    float level = 0.0f;
    float ah_left = 0.0f;
    float ah_total = conf->si_battery_ah;
    float avg_voltage_left = 0.0f;

    if (conf->si_battery_type == BATTERY_TYPE_LIION_3_0__4_2) {
        float x = vesc_clampf((cell_v - 3.2f) / (4.2f - 3.2f), 0.0f, 1.0f);
        const float x2 = x * x;
        const float x3 = x2 * x;
        const float x4 = x3 * x;
        const float x5 = x4 * x;
        /* Polynomial resmi utils_batt_liion_norm_v_to_capacity VESC 6.00. */
        level = -2.979767f * x5 + 5.487810f * x4 - 3.501286f * x3 +
                1.675683f * x2 + 0.317147f * x;
        level = vesc_clampf(level, 0.0f, 1.0f);
        ah_total *= 0.85f;
        ah_left = level * ah_total;
        avg_voltage_left = (3.2f * cells + vin) * 0.5f;
    } else if (conf->si_battery_type == BATTERY_TYPE_LIIRON_2_6__3_6) {
        level = vesc_clampf((cell_v - 2.6f) / (3.6f - 2.6f), 0.0f, 1.0f);
        ah_left = level * ah_total;
        avg_voltage_left = (2.8f * cells + vin) * 0.5f;
    } else {
        level = vesc_clampf((cell_v - 2.1f) / (2.36f - 2.1f), 0.0f, 1.0f);
        ah_left = level * ah_total;
        avg_voltage_left = (2.1f * cells + vin) * 0.5f;
    }

    if (wh_left && ah_total > 0.0f) *wh_left = ah_left * avg_voltage_left;
    return level;
}

/**
 * Turunkan speed dan distance dari ERPM/tachometer dengan rumus VESC 6.00.
 * Tachometer firmware ini bertambah sekali per edge Hall, tepat enam edge per
 * revolusi elektrik, sehingga skala wheel*pi/(3*poles*gear) tetap identik.
 */
static void setup_motion_values(bool second, const mc_values *values,
                                float *speed_mps, float *distance_m,
                                float *distance_abs_m) {
    const mc_configuration *conf =
        (const mc_configuration *)mc_interface_get_configuration_motor(second);
    float wheel = conf->si_wheel_diameter;
    float gear = conf->si_gear_ratio;
    int poles = conf->si_motor_poles;
    if (!(wheel > 0.001f && wheel < 5.0f)) wheel = MCCONF_SI_WHEEL_DIAMETER;
    if (!(gear > 0.0f)) gear = 1.0f;
    if (poles < 2 || (poles & 1)) poles = 2;
    const float pi = 3.14159265358979323846f;
    const float pole_pairs = (float)poles * 0.5f;
    if (speed_mps) *speed_mps = (values->rpm / pole_pairs / 60.0f) * wheel * pi / gear;
    const float tacho_scale = (wheel * pi) / (3.0f * (float)poles * gear);
    if (distance_m) *distance_m = (float)values->tachometer * tacho_scale;
    if (distance_abs_m) *distance_abs_m = (float)values->tachometer_abs * tacho_scale;
}

/** Hitung odometer VESC dari offset SET_ODOMETER dan distance_abs Hall. */
static uint32_t setup_odometer_m(bool second, float distance_abs_m) {
    int64_t trip = (int64_t)(distance_abs_m >= 0.0f ? distance_abs_m : 0.0f);
    int64_t value = s_odometer_offset_m[second ? 1u : 0u] + trip;
    if (value < 0) value = 0;
    if (value > (int64_t)UINT32_MAX) value = (int64_t)UINT32_MAX;
    return (uint32_t)value;
}

static void send_values_setup_packet(bool second, bool selective, uint32_t mask) {
    uint8_t b[128];
    int32_t i = 0;
    const COMM_PACKET_ID id = selective ? COMM_GET_VALUES_SETUP_SELECTIVE : COMM_GET_VALUES_SETUP;
    b[i++] = (uint8_t)id;
    if (selective) buffer_append_uint32(b, mask, &i);

    mc_values v;
    get_values_normalized(second, &v);
    mc_values totals = v;
    if (!second) {
        /* COMM_GET_VALUES_SETUP upstream menjumlahkan controller lokal dan CAN
         * yang aktif. Motor-2 board ini adalah endpoint virtual CAN ID2, jadi
         * agregasikan arus dan counter energi yang sama seperti VESC dual. */
        mc_values right_values;
        get_values_normalized(true, &right_values);
        totals.current_motor += right_values.current_motor;
        totals.current_in += right_values.current_in;
        totals.amp_hours += right_values.amp_hours;
        totals.amp_hours_charged += right_values.amp_hours_charged;
        totals.watt_hours += right_values.watt_hours;
        totals.watt_hours_charged += right_values.watt_hours_charged;
    }
    float speed_mps = 0.0f, distance_m = 0.0f, distance_abs_m = 0.0f, wh_left = 0.0f;
    setup_motion_values(second, &v, &speed_mps, &distance_m, &distance_abs_m);
    const mc_configuration *conf =
        (const mc_configuration *)mc_interface_get_configuration_motor(second);
    const float battery_level = setup_battery_level(conf, v.v_in, &wh_left);
    if (mask & (1u << 0)) buffer_append_float16(b, v.temp_mos, 1e1f, &i);
    if (mask & (1u << 1)) buffer_append_float16(b, v.temp_motor, 1e1f, &i);
    if (mask & (1u << 2)) buffer_append_float32(b, totals.current_motor, 1e2f, &i);
    if (mask & (1u << 3)) buffer_append_float32(b, totals.current_in, 1e2f, &i);
    if (mask & (1u << 4)) buffer_append_float16(b, v.duty_now, 1e3f, &i);
    if (mask & (1u << 5)) buffer_append_float32(b, v.rpm, 1e0f, &i);
    if (mask & (1u << 6)) buffer_append_float32(b, speed_mps, 1e3f, &i);
    if (mask & (1u << 7)) buffer_append_float16(b, v.v_in, 1e1f, &i);
    if (mask & (1u << 8)) buffer_append_float16(b, battery_level, 1e3f, &i);
    if (mask & (1u << 9)) buffer_append_float32(b, totals.amp_hours, 1e4f, &i);
    if (mask & (1u << 10)) buffer_append_float32(b, totals.amp_hours_charged, 1e4f, &i);
    if (mask & (1u << 11)) buffer_append_float32(b, totals.watt_hours, 1e4f, &i);
    if (mask & (1u << 12)) buffer_append_float32(b, totals.watt_hours_charged, 1e4f, &i);
    if (mask & (1u << 13)) buffer_append_float32(b, distance_m, 1e3f, &i);
    if (mask & (1u << 14)) buffer_append_float32(b, distance_abs_m, 1e3f, &i);
    if (mask & (1u << 15)) buffer_append_float32(b, v.position, 1e6f, &i);
    if (mask & (1u << 16)) b[i++] = (uint8_t)v.fault_code;
    if (mask & (1u << 17)) b[i++] = (uint8_t)v.vesc_id;
    if (mask & (1u << 18)) b[i++] = second ? 1u : 2u;
    if (mask & (1u << 19)) buffer_append_float32(b, wh_left, 1e3f, &i);
    if (mask & (1u << 20)) buffer_append_uint32(b, setup_odometer_m(second, distance_abs_m), &i);
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

/* Forward declaration: dipakai command config/current sebelum implementasi detector. */
static bool hall_detect_motor_locked(bool second);

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
    bool decoded = false;
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
        /* LEFT supports the standard VESC ABI sensor-port mode on PB6/PB7.
         * RIGHT has no ABI timer route and is intentionally Hall-only. */
        if(second){
            c.m_sensor_port_mode=SENSOR_PORT_MODE_HALL;
            c.foc_sensor_mode=FOC_SENSOR_MODE_HALL;
        }else if(c.m_sensor_port_mode==SENSOR_PORT_MODE_ABI){
            if(c.foc_sensor_mode!=FOC_SENSOR_MODE_ENCODER && c.foc_sensor_mode!=FOC_SENSOR_MODE_ENCODER_AB)
                c.foc_sensor_mode=FOC_SENSOR_MODE_ENCODER;
            if(c.m_encoder_counts<4 || c.m_encoder_counts>65536)c.m_encoder_counts=(int32_t)MCCONF_ENCODER_COUNTS_DEFAULT;
            if(!(c.foc_encoder_ratio>=0.01f && c.foc_encoder_ratio<=1000.0f))c.foc_encoder_ratio=15.0f;
            while(c.foc_encoder_offset>=360.0f)c.foc_encoder_offset-=360.0f;
            while(c.foc_encoder_offset<0.0f)c.foc_encoder_offset+=360.0f;
        }else{
            c.m_sensor_port_mode=SENSOR_PORT_MODE_HALL; c.foc_sensor_mode=FOC_SENSOR_MODE_HALL;
        }
        if (c.si_motor_poles < 2u || (c.si_motor_poles & 1u)) c.si_motor_poles = 30u;
        if (!(c.si_gear_ratio >= 0.01f && c.si_gear_ratio <= 1000.0f)) c.si_gear_ratio = 1.0f;
        mc_interface_select_motor_thread(second ? 2 : 1);
        mc_interface_set_configuration(&c);
        /* VESC Tool SET_MCCONF is a configuration write, not a motor-detect
         * command. ABI without index is intentionally left UNSYNCED here; the
         * FOC bridge gate already prevents closed-loop drive until an explicit
         * encoder alignment/detect procedure establishes electrical zero. This
         * keeps SET_MCCONF deterministic and prevents unexpected rotor motion. */
        (void)mc_interface_store_configuration_motor(second);
        decoded = true;
    }
    /* Upstream VESC acknowledges only a successfully deserialized MC Config.
     * Persistence is best-effort flash IO, but malformed/wrong-signature packets
     * must not receive a misleading Write OK ACK. */
    if (decoded) { uint8_t ack = COMM_SET_MCCONF; uart_send_payload(&ack, 1u); }
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

/**
 * Terapkan App Configuration VESC 6.00.
 *
 * `store_to_eeprom=false` dipakai COMM_SET_APPCONF_NO_STORE sehingga tombol
 * pengaturan sementara di VESC Tool benar-benar tidak mengubah flash.
 */
static void set_appconf(bool second, const uint8_t *data, uint16_t len,
                        bool store_to_eeprom, COMM_PACKET_ID ack_id) {
    app_configuration tmp = *app_vesc_get_configuration(second);
    const int32_t expected = confgenerator_serialize_appconf(s_config_payload, &tmp);
    if (expected > 0 && len >= (uint16_t)expected && confgenerator_deserialize_appconf(data, &tmp)) {
        if (app_vesc_set_configuration(second, &tmp) && store_to_eeprom) {
            (void)app_vesc_store_configuration(second);
        }
        uint8_t ack = (uint8_t)ack_id;
        uart_send_payload(&ack, 1u);
    }
}

/** COMM_SET_CURRENT_REL: persis Commands -> mc_interface_set_current_rel VESC. */
static void set_current_relative(bool second, const uint8_t *data, uint16_t len) {
    if (len < 4u || hall_detect_motor_locked(second)) return;
    int32_t ind = 0;
    const float rel = (float)buffer_get_int32(data, &ind) / 100000.0f;
    mc_interface_select_motor_thread(second ? 2 : 1);
    touch_motor(second);
    mc_interface_set_current_rel(rel);
}

/** Kirim dua ambang battery-cut sesuai format wire VESC 6.00. */
static void reply_battery_cut(bool second) {
    const mc_configuration *c = (const mc_configuration *)mc_interface_get_configuration_motor(second);
    uint8_t b[12]; int32_t i = 0;
    b[i++] = COMM_GET_BATTERY_CUT;
    buffer_append_float32(b, c->l_battery_cut_start, 1e3f, &i);
    buffer_append_float32(b, c->l_battery_cut_end, 1e3f, &i);
    uart_send_payload(b, (uint16_t)i);
}

/**
 * Terapkan battery-cut secara live. Derating arusnya dilakukan oleh loop FOC
 * menggunakan threshold ADC yang sudah diprekomputasi saat config diterapkan.
 */
static void set_battery_cut(bool second, const uint8_t *data, uint16_t len) {
    if (len < 10u || hall_detect_motor_locked(second)) return;
    int32_t i = 0;
    const float start = buffer_get_float32(data, 1e3f, &i);
    const float end = buffer_get_float32(data, 1e3f, &i);
    const bool store = data[i++] != 0u;
    const bool forward = data[i++] != 0u;
    if (!(start > end && end >= 0.0f && start <= 80.0f)) return;

    for (uint8_t motor = 0u; motor < 2u; ++motor) {
        const bool target_second = motor != 0u;
        if (target_second != second && !(forward && !second)) continue;
        mc_configuration c = *mc_interface_get_configuration_motor(target_second);
        c.l_battery_cut_start = start;
        c.l_battery_cut_end = end;
        mc_interface_select_motor_thread(target_second ? 2 : 1);
        mc_interface_set_configuration(&c);
        if (store) (void)mc_interface_store_configuration_motor(target_second);
    }
    mc_interface_select_motor_thread(second ? 2 : 1);
    { uint8_t ack = COMM_SET_BATTERY_CUT; uart_send_payload(&ack, 1u); }
}

/** Kirim konfigurasi limit sementara yang dipakai halaman Setup VESC Tool. */
static void reply_mcconf_temp(bool second) {
    const mc_configuration *c = (const mc_configuration *)mc_interface_get_configuration_motor(second);
    uint8_t b[64]; int32_t i = 0;
    b[i++] = COMM_GET_MCCONF_TEMP;
    buffer_append_float32_auto(b, c->l_current_min_scale, &i);
    buffer_append_float32_auto(b, c->l_current_max_scale, &i);
    buffer_append_float32_auto(b, c->l_min_erpm, &i);
    buffer_append_float32_auto(b, c->l_max_erpm, &i);
    buffer_append_float32_auto(b, c->l_min_duty, &i);
    buffer_append_float32_auto(b, c->l_max_duty, &i);
    buffer_append_float32_auto(b, c->l_watt_min, &i);
    buffer_append_float32_auto(b, c->l_watt_max, &i);
    buffer_append_float32_auto(b, c->l_in_current_min, &i);
    buffer_append_float32_auto(b, c->l_in_current_max, &i);
    b[i++] = c->si_motor_poles;
    buffer_append_float32_auto(b, c->si_gear_ratio, &i);
    buffer_append_float32_auto(b, c->si_wheel_diameter, &i);
    uart_send_payload(b, (uint16_t)i);
}

/**
 * Terapkan COMM_SET_MCCONF_TEMP/SETUP sesuai layout VESC 6.00. Pada variant
 * SETUP, batas kecepatan masuk dalam m/s dan dikonversi ke ERPM menggunakan
 * pole, gear ratio, dan diameter roda dari MC Config. `forward_can` diterapkan
 * ke endpoint virtual motor-2 tanpa membuat CAN fisik palsu.
 */
static void set_mcconf_temp(bool second, COMM_PACKET_ID packet_id,
                            const uint8_t *data, uint16_t len) {
    if (len < 36u || hall_detect_motor_locked(second)) return;
    int32_t i = 0;
    const bool store = data[i++] != 0u;
    const bool forward = data[i++] != 0u;
    const bool ack = data[i++] != 0u;
    const bool divide = data[i++] != 0u;
    const float current_min_scale = buffer_get_float32_auto(data, &i);
    const float current_max_scale = buffer_get_float32_auto(data, &i);
    const float limit_min_in = buffer_get_float32_auto(data, &i);
    const float limit_max_in = buffer_get_float32_auto(data, &i);
    const float duty_min = buffer_get_float32_auto(data, &i);
    const float duty_max = buffer_get_float32_auto(data, &i);
    float watt_min = buffer_get_float32_auto(data, &i);
    float watt_max = buffer_get_float32_auto(data, &i);
    float input_min = 0.0f, input_max = 0.0f;
    const bool has_input_limits = len >= (uint16_t)(i + 8);
    if (has_input_limits) {
        input_min = buffer_get_float32_auto(data, &i);
        input_max = buffer_get_float32_auto(data, &i);
    }
    const float controllers = (divide && forward && !second) ? 2.0f : 1.0f;
    watt_min /= controllers;
    watt_max /= controllers;

    for (uint8_t motor = 0u; motor < 2u; ++motor) {
        const bool target_second = motor != 0u;
        if (target_second != second && !(forward && !second)) continue;
        mc_configuration c = *mc_interface_get_configuration_motor(target_second);
        c.l_current_min_scale = current_min_scale;
        c.l_current_max_scale = current_max_scale;
        if (packet_id == COMM_SET_MCCONF_TEMP_SETUP) {
            const float wheel = c.si_wheel_diameter > 0.001f ? c.si_wheel_diameter : MCCONF_SI_WHEEL_DIAMETER;
            const float gear = c.si_gear_ratio > 0.0f ? c.si_gear_ratio : 1.0f;
            const float fact = (((float)c.si_motor_poles * 0.5f) * 60.0f * gear) /
                               (wheel * 3.14159265358979323846f);
            c.l_min_erpm = limit_min_in * fact;
            c.l_max_erpm = limit_max_in * fact;
        } else {
            c.l_min_erpm = limit_min_in;
            c.l_max_erpm = limit_max_in;
        }
        c.l_min_duty = duty_min;
        c.l_max_duty = duty_max;
        c.l_watt_min = watt_min;
        c.l_watt_max = watt_max;
        if (has_input_limits) {
            c.l_in_current_min = input_min;
            c.l_in_current_max = input_max;
        }
        mc_interface_select_motor_thread(target_second ? 2 : 1);
        mc_interface_set_configuration(&c);
        if (store) (void)mc_interface_store_configuration_motor(target_second);
    }
    mc_interface_select_motor_thread(second ? 2 : 1);
    if (ack) { uint8_t id = (uint8_t)packet_id; uart_send_payload(&id, 1u); }
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
    s_hall_detect.align_start_ms = HAL_GetTick();
    s_hall_detect.next_ms = s_hall_detect.align_start_ms;
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
    if(fails!=2u || !mcpwm_foc_hall_table_sane(table))success=false;
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
        /* The upstream detector ramps alignment current for about one second.
         * On this bare-metal target the main loop is preempted by the 16-kHz FOC
         * ISR, so counting 1000 scheduler visits can stretch one second into
         * tens of seconds (especially on motor 2). Drive the ramp from wall time
         * instead; skipped visits simply advance to the correct current. */
        const uint32_t elapsed_ms=(uint32_t)(now_ms-s_hall_detect.align_start_ms);
        if (elapsed_ms < 1000u) {
            const uint16_t step=(uint16_t)(elapsed_ms+1u);
            s_hall_detect.align_step=step;
            const float i=s_hall_detect.current_a*(float)step/1000.0f;
            mcpwm_foc_set_openloop_phase(i,0.0f,second);
            mcpwm_foc_vesc_override_touch(second);
            s_hall_detect.next_ms=now_ms+1u;
            return;
        }
        s_hall_detect.align_step=1000u;
        mcpwm_foc_set_openloop_phase(s_hall_detect.current_a,0.0f,second);
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
                /* Custom tuning exposes Q16, while the standard VESC 6.00
                 * MC-config wire/persistence field is float16 scale 1e4.
                 * Canonicalize the mirrored MC field to the nearest 1e-4 grid
                 * so GET_TUNING -> SET_TUNING(same) is idempotent and a
                 * subsequent GET_MCCONF/reboot cannot drift by one LSB due to
                 * float truncation (e.g. 0.1018 -> 0.1017). */
                m->m_telem_current_filter_q16=fa;
                uint32_t fx10000=((uint32_t)fa*10000u+32767u)/65535u;
                if(fx10000<10u)fx10000=10u;
                if(fx10000>10000u)fx10000=10000u;
                m->m_conf.foc_current_filter_const=(float)fx10000/10000.0f;
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
        /* Payload diagnostic bertambah lintas revisi. Sisakan headroom besar dan
         * jangan lagi mengandalkan ukuran historis 208 byte yang sudah overflow. */
        uint8_t b[256];
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
        /* Logical ARM follows a live VESC binary link (VESC Tool or Python).
         * This is intentionally distinct from motor-command ownership above:
         * telemetry alone arms the control link, but never energizes the bridge. */
        b[i++] = vesc_protocol_link_active() ? 1u : 0u;
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
            buffer_append_uint32(b,main_prof_vesc_max_cycles,&i);
            buffer_append_uint32(b,main_prof_house_max_cycles,&i);
            buffer_append_uint32(b,main_prof_tail_max_cycles,&i);
#ifdef STM32F103xE
            buffer_append_uint32(b,s_prof_values_snapshot_max_cycles,&i);
            buffer_append_uint32(b,s_prof_values_position_max_cycles,&i);
            buffer_append_uint32(b,s_prof_values_serialize_max_cycles,&i);
            buffer_append_uint32(b,s_prof_values_tx_max_cycles,&i);
            /* Keep diagnostic reply below the 256-byte local payload buffer.
             * Pre/post were measured separately during profiling; retain the
             * actionable control/step maxima plus explicit overrun counters. */
            buffer_append_uint32(b,foc_prof_sensor_max_cycles,&i);
            buffer_append_uint32(b,foc_prof_current_max_cycles,&i);
            buffer_append_uint32(b,foc_prof_regulator_max_cycles,&i);
            buffer_append_uint32(b,foc_prof_svpwm_max_cycles,&i);
            buffer_append_uint32(b,m_motor_1.m_overrun_count+m_motor_2.m_overrun_count,&i);
#else
            for(uint8_t pi=0u;pi<9u;++pi)buffer_append_uint32(b,0u,&i);
#endif
        }
        if ((uint32_t)i <= sizeof(b)) uart_send_payload(b, (uint16_t)i);
    }
}

/** Kirim teks Terminal sebagai COMM_PRINT agar framing VESC Tool tetap utuh. */
static void terminal_send_text(const char *text) {
    if (!text) return;
    uint8_t b[192];
    const size_t n = strlen(text);
    const size_t copy = n > sizeof(b) - 1u ? sizeof(b) - 1u : n;
    b[0] = COMM_PRINT;
    memcpy(&b[1], text, copy);
    uart_send_payload(b, (uint16_t)(copy + 1u));
}

/**
 * Terminal minimum yang relevan untuk board dual-hoverboard.
 * Perintah tidak pernah meneruskan raw printf ke USART3; semua keluaran selalu
 * dibungkus COMM_PRINT sehingga VESC Tool tetap sinkron dengan CRC/framing.
 */
static void process_terminal_command(bool second, const uint8_t *data, uint16_t len) {
    char cmd[48];
    const uint16_t copy = len >= sizeof(cmd) ? (uint16_t)(sizeof(cmd) - 1u) : len;
    if (copy > 0u) memcpy(cmd, data, copy);
    cmd[copy] = '\0';
    /* Track the live length ourselves. The previous version re-tested the
     * loop guard against the original `copy` count instead of the string's
     * current length, so an all-CR/LF command (e.g. a bare Enter keypress in
     * the VESC Tool terminal) stripped the buffer down to "" and then read
     * cmd[strlen(cmd)-1] == cmd[(size_t)-1] -- an out-of-bounds underflow
     * read/write one byte before the stack buffer. Reproduced with ASan. */
    size_t cmd_len = copy;
    while (cmd_len > 0u && (cmd[cmd_len - 1u] == '\r' || cmd[cmd_len - 1u] == '\n')) {
        cmd[--cmd_len] = '\0';
    }

    if (strcmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0) {
        terminal_send_text("Commands: help, status, faults, stop, fw\n");
        return;
    }
    if (strcmp(cmd, "stop") == 0) {
        mc_interface_release_motor();
        terminal_send_text(second ? "motor_right released\n" : "motor_left released\n");
        return;
    }
    if (strcmp(cmd, "fw") == 0) {
        terminal_send_text(second ? "motor_right FW 6.00\n" : "motor_left FW 6.00\n");
        return;
    }
    if (strcmp(cmd, "status") == 0 || strcmp(cmd, "faults") == 0) {
        mc_values v;
        get_values_normalized(second, &v);
        const mcpwm_foc_motor_t *m = mcpwm_foc_get_motor_const(second);
        char out[180];
        const int written = snprintf(out, sizeof(out),
            "id=%u fault=%u hall=%u erpm=%ld duty=%ld/1000 Vin=%ldmV Iq=%ldmA Id=%ldmA\n",
            (unsigned)v.vesc_id, (unsigned)v.fault_code, (unsigned)m->m_hall_state,
            (long)v.rpm, (long)(v.duty_now * 1000.0f), (long)(v.v_in * 1000.0f),
            (long)(v.iq * 1000.0f), (long)(v.id * 1000.0f));
        if (written > 0) terminal_send_text(out);
        return;
    }
    terminal_send_text("Unknown command. Type help.\n");
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
            touch_motor(second); mc_interface_set_duty(duty);
        }
        break;
    case COMM_SET_CURRENT:
        if (hall_detect_motor_locked(second)) break;
        if (n >= 4u) {
            const float current = (float)buffer_get_int32(d, &k) / 1000.0f;
            touch_motor(second); mc_interface_set_current(current);
        }
        break;
    case COMM_SET_CURRENT_REL:
        set_current_relative(second, d, n);
        break;
    case COMM_SET_CURRENT_BRAKE:
        if (hall_detect_motor_locked(second)) break;
        if (n >= 4u) {
            const float current = (float)buffer_get_int32(d, &k) / 1000.0f;
            touch_motor(second); mc_interface_set_brake_current(current);
        }
        break;
    case COMM_SET_HANDBRAKE:
        if (hall_detect_motor_locked(second)) break;
        if (n >= 4u) {
            const float current = (float)buffer_get_int32(d, &k) / 1000.0f;
            touch_motor(second); mc_interface_set_handbrake(current);
        }
        break;
    case COMM_SET_RPM:
        if (hall_detect_motor_locked(second)) break;
        if (n >= 4u) {
            const float rpm = (float)buffer_get_int32(d, &k);
            touch_motor(second); mc_interface_set_pid_speed(rpm);
        }
        break;
    case COMM_SET_POS:
        if (hall_detect_motor_locked(second)) break;
        if(n>=4u){const float pos=(float)buffer_get_int32(d,&k)/1000000.0f;touch_motor(second);mc_interface_set_pid_pos(pos);}
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
    case COMM_SET_MCCONF_TEMP:
    case COMM_SET_MCCONF_TEMP_SETUP:
        set_mcconf_temp(second, id, d, n);
        break;
    case COMM_GET_MCCONF_TEMP:
        reply_mcconf_temp(second);
        break;
    case COMM_GET_BATTERY_CUT:
        reply_battery_cut(second);
        break;
    case COMM_SET_BATTERY_CUT:
        set_battery_cut(second, d, n);
        break;
    case COMM_GET_APPCONF:
    case COMM_GET_APPCONF_DEFAULT:
        reply_appconf(second, id);
        break;
    case COMM_GET_DECODED_ADC:
        reply_decoded_adc();
        break;
    case COMM_SET_ODOMETER:
        /* Tombol Set Odometer VESC Tool tidak punya ACK. Simpan offset terhadap
         * trip Hall saat ini supaya pembacaan berikutnya terus bertambah. */
        if (n >= 4u) {
            const uint32_t requested = buffer_get_uint32(d, &k);
            mc_values ov;
            float speed_unused = 0.0f, dist_unused = 0.0f, dist_abs = 0.0f;
            get_values_normalized(second, &ov);
            setup_motion_values(second, &ov, &speed_unused, &dist_unused, &dist_abs);
            const int64_t trip = (int64_t)(dist_abs >= 0.0f ? dist_abs : 0.0f);
            s_odometer_offset_m[second ? 1u : 0u] = (int64_t)requested - trip;
        }
        break;
    case COMM_SET_APPCONF:
        set_appconf(second, d, n, true, COMM_SET_APPCONF);
        break;
    case COMM_SET_APPCONF_NO_STORE:
        set_appconf(second, d, n, false, COMM_SET_APPCONF_NO_STORE);
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
    case COMM_DETECT_ENCODER: {
        float off=1001.0f, ratio=0.0f; bool inv=false;
        float current=1.0f;
        if(n>=4u) current=(float)buffer_get_int32(d,&k)/1000.0f;
        app_vesc_disable_output(60000);
        (void)mcpwm_foc_encoder_detect(current,second,&off,&ratio,&inv);
        uint8_t reply[10]; int32_t ri=0; reply[ri++]=COMM_DETECT_ENCODER;
        buffer_append_float32(reply,off,1e6f,&ri);
        buffer_append_float32(reply,ratio,1e6f,&ri);
        reply[ri++]=inv?1u:0u;
        uart_send_payload(reply,(uint16_t)ri);
        break;
    }
    case COMM_DETECT_HALL_FOC:
        hall_detect_begin(second, d, n);
        break;
    case COMM_DETECT_APPLY_ALL_FOC:
        if(!second) detect_all_begin(d,n);
        break;
    case COMM_TERMINAL_CMD:
    case COMM_TERMINAL_CMD_SYNC:
        process_terminal_command(second, d, n);
        break;
    case COMM_SHUTDOWN:
        /* Board hoverboard tidak mempunyai power-latch VESC. Pertahankan
         * command tetap aman: lepaskan bridge/motor tanpa mematikan MCU/UART. */
        mc_interface_release_motor();
        break;
    case COMM_CUSTOM_APP_DATA:
        process_custom_app(second, d, n);
        break;
    case COMM_REBOOT:
        /* Sama seperti VESC: release bridge dahulu, lalu reset MCU. Host build
         * sengaja tidak mengeksekusi register Cortex-M agar unit test aman. */
        mc_interface_release_motor();
#ifdef STM32F103xE
        NVIC_SystemReset();
#endif
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

static void process_rt_mailboxes(void) {
    for (uint8_t mi=0u; mi<2u; ++mi) {
        uint8_t p[5]; uint8_t have=0u;
        __disable_irq();
        if (s_rt_cmd[mi].pending && s_rt_cmd[mi].len==5u) {
            memcpy(p,(const void *)s_rt_cmd[mi].payload,5u);
            s_rt_cmd[mi].pending=0u; have=1u;
        }
        __enable_irq();
        if (have) process_command(p,5u,mi!=0u);
    }
}

void vesc_protocol_process_pending(void) {
    const uint32_t process_now_ms = HAL_GetTick();
    if (s_process_last_ms != 0u) {
        const uint32_t gap = process_now_ms - s_process_last_ms;
        if (gap > s_process_gap_max_ms) s_process_gap_max_ms = gap;
    }
    s_process_last_ms = process_now_ms;
    vesc_tx_service();
    process_rt_mailboxes();
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
        process_rt_mailboxes();
        vesc_tx_service();
    }
    vesc_tx_service();
}
