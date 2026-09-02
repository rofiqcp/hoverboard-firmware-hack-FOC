#!/usr/bin/env python3
from pathlib import Path
import re
R=Path(__file__).resolve().parents[1]
mc=(R/'Src/motor/mcpwm_foc.c').read_text()
mch=(R/'Src/motor/mcpwm_foc.h').read_text()
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

# RX burst handling and VESC packet realtime stream: 4-deep FIFO, 20 ms = 50 Hz.
assert re.search(r'#define\s+VESC_RX_QUEUE_DEPTH\s+4u',vp)
assert re.search(r'#define\s+VESC_RT_PERIOD_MS\s+20u',vp)
assert 's_pending_payload[VESC_RX_QUEUE_DEPTH][VESC_MAX_PAYLOAD]' in vp
assert 's_pending_count < VESC_RX_QUEUE_DEPTH' in vp
assert 'void vesc_protocol_periodic(void)' in vp and 'vesc_protocol_periodic();' in main
assert 'send_values_packet' in vp and 'send_values_setup_packet' in vp
assert 's_rt_stream.setup' in vp
assert 'rx fifo burst' in host and 'rt 50hz periodic' in host

print('V16_FEATURE_STATIC_PASS names=1 hall_midpoint=1 hall_rate_limit=1 detect_1deg_6sweep=1 current_off_zero=1 rx_fifo4=1 rt50_values_setup=1')
