#!/usr/bin/env python3
from pathlib import Path
import re
R=next(p for p in Path(__file__).resolve().parents if (p/'platformio.ini').exists())
mc=(R/'Src/motor/mcpwm_foc.c').read_text()
mch=(R/'Src/motor/mcpwm_foc.h').read_text()
mcc=(R/'Src/motor/mcconf_default.h').read_text()
vp=(R/'Src/vesc/vesc_protocol.c').read_text()
main=(R/'Src/main.c').read_text()
dual=(R/'tools/vesc_dual.py').read_text()
host=(R/'tools/tests/host/test_vesc_protocol_host.c').read_text()

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
assert 'period_for_filter' in mc and 'floor_period' in mc and 'm->m_hall_period_reject_count++' in mc, 'Hall timing outlier slew-limit path missing'
assert 'speed_pid_iq_target_step' in mc and 'm->m_iq_target_q4 = speed_pid_iq_target_step' in mc
assert 'stop_zone' in mc and 'zero-vector' in mc and 'm->m_iq_set_q4 = m->m_iq_target_q4;' in mc
assert 'm->m_iq_target_q4=0; m->m_iq_set_q4=0; m->m_iq_set_ramp_q16=0;' in mc, 'speed STOP must force VESC zero-vector reference'
assert 'VESC speed PID -> Iq' in mc
assert 'min_erpm_q16' in mc and 'target_abs_q16 < min_erpm_q16' in mc
assert 'speed PI drives Vq directly' not in mc
assert 'MCCONF_POSITION_PHASE_DEADBAND_MDEG' not in mcc and 'MCCONF_POSITION_RUN_CURRENT_MAX_MA' not in mcc and 'MCCONF_POSITION_BREAKAWAY_CURRENT_MA' not in mcc
assert 'm_position_prev_proc_phase' in mch and 'm_position_kd_proc_phase_coeff_q4' in mch
# Standard COMM_SET_POS mengikuti foc_run_pid_control_pos VESC: shortest-path
# angular PID -> Iq, lengkap dengan anti-windup dan D-on-measurement. State
# machine Hall per-sektor hanya dipakai API custom count multi-putaran.
assert 'VESC foc_run_pid_control_pos: shortest-path angular PID' in mc
assert 'p_q15=CLAMP' in mc and 'i_lim_q15=32768-' in mc
assert 'm_position_d_filter_q15' in mch and 'm_position_d_proc_filter_q15' in mch
assert 'proc_now=position_feedback_phase_u16(m,second)' in mc and 'proc_delta=(int16_t)(proc_now-m->m_position_prev_proc_phase)' in mc
assert 'out_q15=p_q15+(m->m_position_integrator>>16)+' in mc
assert 'm_position_step_braking' in mch and 'm_position_brake_direction' in mch
assert 'hall_motion_same_direction' in mc and 'position_brake_iq_q4' in mc
assert 'm_position_motion_seen' in mch
mi=(R/'Src/motor/mc_interface.c').read_text()
assert 'EE_CFG_SIGNATURE_V16' in mi and 'EE_CFG_SIGNATURE_V17' in mi and 'migrate_speed_pid' in mi

# VESC-like detector: 1s current ramp, 3 forward + 3 reverse complete 1-degree sweeps.
assert 'k < 1000u' in mc and 'HAL_Delay(1u)' in mc
assert 'pass < 6u' in mc and 'const bool reverse = pass >= 3u' in mc
assert 'k < 360u' in mc and '359u - k' in mc and 'HAL_Delay(5u)' in mc
assert 'hall_detect_angle200' in mc and 'pass_n[h]' in mc and 'hall_detect_distance200' in mc
assert 'forward_ref' in mc and 'reverse_ref' in mc and '> 4u' in mc and '> 8u' in mc
assert 'hall_detect_begin' in vp and 'hall_detect_periodic' in vp and 'standalone detect is not a store' in vp
assert 'mc_interface_store_configuration_motor(second)' in vp and 'case COMM_SET_MCCONF:' in vp and 'case COMM_DETECT_APPLY_ALL_FOC:' in vp
assert 'm->m_iq_target_q4 != 0 || m->m_iq_set_q4 != 0' in mc and 'm->m_fault != FAULT_CODE_NONE' in mc
assert (R/'tools/tests/hardware/test_hall_detect_repeat.py').exists()
assert 'mcpwm_foc_vesc_override_clear(second)' in mc
assert 'COMM_DETECT_HALL_FOC, 20.0)' in dual

# Upstream VESC reports zero public motor-current telemetry while released.
# Keep the separately calibrated high-Z/raw ADC path diagnostic-only.
assert 'leftOffTelemValid' in mc and 'rightOffTelemValid' in mc and 'off_telem_deadband_counts' in mc
assert 'm_id_telem_q4' in mc and 'm_current_in_telem_counts' in mc
off=mc[mc.index('if (!source_enabled || !feedback_ready || m->m_fault!=FAULT_CODE_NONE'):mc.index('return;',mc.index('if (!source_enabled || !feedback_ready || m->m_fault!=FAULT_CODE_NONE'))]
for token in ('m->m_vd=0','m->m_vq=0','m->m_pwm_a=0','m->m_pwm_b=0','m->m_pwm_c=0'):
    assert token in off, token
for token in ('m->m_id_q4=0','m->m_iq_q4=0','m->m_current_in_counts=0'):
    assert token in off, token

# RX burst handling remains bounded and 8-deep. GET_VALUES stays strict request/reply;
# the only standard unsolicited stream is COMM_ROTOR_POSITION after SET_DETECT,
# matching vedderb/bldc's 10-ms periodic_thread behavior.
assert re.search(r'#define\s+VESC_RX_QUEUE_DEPTH\s+8u',vp)
assert 's_pending_payload[VESC_RX_QUEUE_DEPTH][VESC_MAX_PAYLOAD]' in vp
assert 's_pending_count < VESC_RX_QUEUE_DEPTH' in vp
assert 'VESC_RT_PERIOD_MS' not in vp and 's_rt_stream' not in vp
assert 'case COMM_SET_DETECT:' in vp and 'COMM_ROTOR_POSITION' in vp
assert 'vesc_protocol_periodic(uint32_t now_ms)' in vp
assert 'vesc_protocol_periodic(HAL_GetTick())' in main
assert 'send_values_packet' in vp and 'send_values_setup_packet' in vp
assert 'strict request/reply' in vp and 'one request -> one reply' in vp
assert 'realtime mailbox latest setpoint' in host and 'request/reply only' in host
assert 'rotor stream local value' in host and 'rotor stream right value' in host


assert 'MCCONF_HALL_DEBOUNCE_SAMPLES' in mcc and 'm_hall_candidate_count' in mc, 'Hall GPIO debounce missing'
assert 'MCCONF_HALL_PERIOD_FILTER_WARMUP_EDGES' in mcc and 'm_hall_direction_stable_edges' in mc, 'Hall reversal/acceleration warmup missing'
assert 'm_brake_direction' not in mch and 'm_brake_current_q4' in mch and 'feedback_motion_direction' in mc and 'encoder_motion_fresh' in mc
assert 'm->m_hall_ticks>fresh' in mc and 'MCCONF_TRQ_STOP_RPM_DEADBAND' in mc and 'CONTROL_MODE_CURRENT_BRAKE' in mc

assert 'MCCONF_POSITION_CURRENT_MAX_MA' in mc or 'MCCONF_POSITION_CURRENT_MAX_MA' in mcc
assert 'MCCONF_POSITION_SETTLE_CURRENT_MA' not in mc and 'MCCONF_POSITION_SETTLE_CURRENT_MA' not in mcc
assert 'MCCONF_POSITION_SETTLE_MS' not in mc and 'MCCONF_POSITION_SETTLE_MS' not in mcc

assert 'm->m_hall_direction == dir' in mc, 'Hall period-outlier filter must not reject direction reversals'
assert 'hall_table_runtime_sane' in mc and 'Preserve the last known-good table' in mc, 'runtime Hall-table validation missing'
assert 'Standard VESC OPENLOOP_CURRENT' in mc and 'm->m_iq_target_q4=amp_to_q4(m,current)' in mc and '60*(int64_t)PWM_FREQ' in mc, 'standard VESC openloop must rotate signed Iq at electrical RPM without pole-pair multiplication'
assert 'hall-phase' in (R/'tools/vesc_debug.py').read_text() and 'HALL_PHASE_PASS' in (R/'tools/vesc_debug.py').read_text(), 'active Hall phase-check utility missing'


# Standard VESC handbrake command must not be silently ignored.
assert 'case COMM_SET_HANDBRAKE:' in vp and 'mc_interface_set_handbrake(current)' in vp
dual=(R/'tools/vesc_dual.py').read_text()
assert 'COMM_SET_HANDBRAKE = 10' in dual and 'def handbrake(' in dual

assert 'Jangan hapus nilai telemetry itu' in mc, 'idle live current telemetry path missing'
print('V16_FEATURE_STATIC_PASS names=1 hall_midpoint=1 hall_rate_limit=1 hall_debounce=1 reversal_warmup=1 detect_1deg_6sweep=1 current_idle_live=1 rx_fifo8=1 vesc_request_reply=1 brake_dynamic=1 std_pos=1 custom_count_cap=1 std_openloop=1')
