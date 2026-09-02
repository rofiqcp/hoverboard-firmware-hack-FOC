#!/usr/bin/env python3
from pathlib import Path
import re
R=Path(__file__).resolve().parents[1]
mc=(R/'Src/motor/mcpwm_foc.c').read_text()
mch=(R/'Src/motor/mcpwm_foc.h').read_text()
mcc=(R/'Src/motor/mcconf_default.h').read_text()
vp=(R/'Src/vesc/vesc_protocol.c').read_text()
main=(R/'Src/main.c').read_text()
dual=(R/'tools/vesc_dual.py').read_text()
host=(R/'tools/test_vesc_protocol_host.c').read_text()

# User-visible identity must be exact in VESC Tool for local and virtual motor 2.
assert 'second ? "motor_right" : "motor_left"' in vp
assert re.search(r'#define\s+VESC_SECOND_MOTOR_ID\s+2u',vp)

# Hall table values are sector centers. V16 must begin interpolation at the
# midpoint between old/new centers, not at the new center as V15 did.
assert 'hall_midpoint200' in mc and 'center_delta / 2' in mc
assert 'm->m_hall_pos = hall_midpoint200(previous_center, ad);' in mc
assert 'edge_phase = hall_angle200_to_phase(m->m_hall_pos)' in mc
assert 'm_phase_hall_target' in mch and 'phase_diff_u16' in mc
assert 'rate-limits corrected Hall phase' in mc
assert 'm_hall_reject_counted_state' in mch and 'm->m_hall_reject_counted_state != h' in mc
assert 'one chatter/outlier edge into' in mc
assert 'speed_pid_iq_target_step' in mc and 'm->m_iq_target_q4 = speed_pid_iq_target_step' in mc
speed_block=mc[mc.index('if (m->m_control_mode==CONTROL_MODE_SPEED) {'):mc.index('} else {', mc.index('if (m->m_control_mode==CONTROL_MODE_SPEED) {'))]
assert 'iq_setpoint_slew_step(m);' not in speed_block and 'm->m_iq_set_q4 = m->m_iq_target_q4;' in speed_block
assert 'speed PID\n     * produces an Iq/current request' in mc
assert 'min_erpm_q16' in mc and 'target_abs_q16 < min_erpm_q16' in mc
assert 'speed PI drives Vq directly' not in mc
mi=(R/'Src/motor/mc_interface.c').read_text()
assert 'EE_CFG_SIGNATURE_V16' in mi and 'EE_CFG_SIGNATURE_V17' in mi and 'migrate_speed_pid' in mi

# VESC-like detector: 1s current ramp, 3 forward + 3 reverse complete 1-degree sweeps.
assert 'k < 1000u' in mc and 'HAL_Delay(1u)' in mc
assert 'pass < 6u' in mc and 'const bool reverse = pass >= 3u' in mc
assert 'k < 360u' in mc and '359u - k' in mc and 'HAL_Delay(5u)' in mc
assert 'samples[h] <= 30u' in mc
assert 'mcpwm_foc_vesc_override_clear(second)' in mc
assert 'COMM_DETECT_HALL_FOC, 20.0)' in dual

# VESC 6.00 semantics: currents are zero while bridge/control is undriven.
off=mc[mc.index('if (!source_enabled || m->m_fault!=FAULT_CODE_NONE'):mc.index('} else {',mc.index('if (!source_enabled || m->m_fault!=FAULT_CODE_NONE'))]
for token in ('m->m_i_alpha_q4=0','m->m_i_beta_q4=0','m->m_id_q4=0','m->m_iq_q4=0',
              'm->m_current_in_counts=0','m->m_current_lpf_q16[0]=0'):
    assert token in off, token

# RX burst handling remains 4-deep. Values traffic follows upstream VESC:
# one request -> one reply; VESC Tool/host owns the polling cadence.
assert re.search(r'#define\s+VESC_RX_QUEUE_DEPTH\s+4u',vp)
assert 's_pending_payload[VESC_RX_QUEUE_DEPTH][VESC_MAX_PAYLOAD]' in vp
assert 's_pending_count < VESC_RX_QUEUE_DEPTH' in vp
assert 'VESC_RT_PERIOD_MS' not in vp and 's_rt_stream' not in vp
assert 'vesc_protocol_periodic' not in vp and 'vesc_protocol_periodic();' not in main
assert 'send_values_packet' in vp and 'send_values_setup_packet' in vp
assert 'strict request/reply' in vp and 'one request -> one reply' in vp
assert 'rx fifo burst' in host and 'request/reply only' in host


assert 'MCCONF_HALL_DEBOUNCE_SAMPLES' in mcc and 'm_hall_candidate_count' in mc, 'Hall GPIO debounce missing'
assert 'MCCONF_HALL_PERIOD_FILTER_WARMUP_EDGES' in mcc and 'm_hall_direction_stable_edges' in mc, 'Hall reversal/acceleration warmup missing'
assert 'm_brake_direction' in mch and 'const bool entering = m->m_control_mode != CONTROL_MODE_CURRENT_BRAKE' in mc
assert 'COMM_SET_CURRENT_BRAKE is a stop request, not a reverse-speed command' in mc

assert 'MCCONF_POSITION_CURRENT_MAX_MA' in mc or 'MCCONF_POSITION_CURRENT_MAX_MA' in mcc
assert 'MCCONF_POSITION_SETTLE_CURRENT_MA' in mc or 'MCCONF_POSITION_SETTLE_CURRENT_MA' in mcc
assert 'MCCONF_POSITION_SETTLE_MS' in mc or 'MCCONF_POSITION_SETTLE_MS' in mcc

assert 'm->m_hall_direction == dir' in mc, 'Hall period-outlier filter must not reject direction reversals'
assert 'hall_table_runtime_sane' in mc and 'Preserve the last known-good table' in mc, 'runtime Hall-table validation missing'
assert 'motor_pole_pairs(second)*4294967296ULL' in mc, 'right openloop must use per-motor pole pairs'
assert 'hall-phase' in (R/'tools/vesc_debug.py').read_text() and 'HALL_PHASE_PASS' in (R/'tools/vesc_debug.py').read_text(), 'active Hall phase-check utility missing'

print('V16_FEATURE_STATIC_PASS names=1 hall_midpoint=1 hall_rate_limit=1 hall_debounce=1 reversal_warmup=1 detect_1deg_6sweep=1 current_off_zero=1 rx_fifo4=1 vesc_request_reply=1 brake_latch=1 position_cap=1')
