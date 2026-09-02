#!/usr/bin/env python3
from pathlib import Path
import re
R=Path(__file__).resolve().parents[1]
mc=(R/'Src/motor/mcpwm_foc.c').read_text()
mch=(R/'Src/motor/mcpwm_foc.h').read_text()
mcc=(R/'Src/motor/mcconf_default.h').read_text()
vp=(R/'Src/vesc/vesc_protocol.c').read_text()
main=(R/'Src/main.c').read_text()
com=(R/'Src/comms.c').read_text()
dual=(R/'tools/vesc_dual.py').read_text()
dbg=(R/'tools/vesc_debug.py').read_text()
halltest=(R/'tools/test_hall_detect_algorithm.c').read_text()

# 50 ERPM must survive Hall timeout and retain fractional mechanical target.
assert re.search(r'#define\s+MCCONF_HALL_TIMEOUT_TICKS\s+8000u',mcc)
assert 'm_speed_target_rpm_q16' in mch and 'erpm_to_mech_rpm_q16' in mc
assert 'measured_mech_rpm_q16' in mc
assert 'const int32_t e_q4_raw=(m->m_speed_set_ramp_q16-measured_q16)>>12;' in mc
assert '((float)PWM_FREQ*10.0f)/(float)m->m_hall_period' in mc

# Hall detect fixed phase must not be overwritten by rotating mode-4 updater.
assert 'm->m_control_mode==CONTROL_MODE_OPENLOOP_PHASE' in mc
phase_branch=mc[mc.index('if (m->m_control_mode==CONTROL_MODE_OPENLOOP)'):mc.index('/* Hall estimator needs')]
assert 'openloop_update(m);' in phase_branch and 'openloop_current_ramp_update(m);' in phase_branch
assert 'otherwise it overwrites m_phase_openloop' in phase_branch
assert 'for (uint8_t pass = 0u; pass < 6u; ++pass)' in mc and ('const bool reverse = (pass & 1u) != 0u' in mc or 'const bool reverse = pass >= 3u' in mc)
assert 'mcpwm_foc_adc_int_handler();' in halltest and 'for(uint32_t t=0;t<ms;t++)' in halltest and 'isr<16u' in halltest

# VESC ownership/bridge gating is per motor, not one shared any-motor flag.
assert 'leftSourceEnable=(enable!=0u)||mcpwm_foc_vesc_override_active(false)' in mc
assert 'rightSourceEnable=(enable!=0u)||mcpwm_foc_vesc_override_active(true)' in mc
assert 'leftOpenloop = (m_motor_1.m_control_mode==CONTROL_MODE_OPENLOOP ||' in mc
assert 'CONTROL_MODE_OPENLOOP_PHASE);' in mc and 'leftDcLimit=leftOpenloop' in mc and 'rightDcLimit=rightOpenloop' in mc

# No user live toggle. Automatic legacy telemetry is 50 Hz; VESC link suppresses it.
assert '"LIVE"' not in com
assert 'telemetryNowMs - legacyTelemetryPrevMs) >= 20u' in main
assert 'if (!vescLinkActive && !timeoutFlgSerial' in main

# Exact VESC 6.00 identity: FW response stops after FW_NAME.
fwfun=vp[vp.index('static void reply_fw_version'):vp.index('static void get_values_normalized')]
assert 'VESC_FW_MAJOR' in fwfun and 'VESC_FW_MINOR' in fwfun
assert 'post-6.00 fields' in fwfun and 'buffer_append_uint32' not in fwfun

# Hall result is standard [cmd + 8 table + result], with detector success distinct from EEPROM status.
det=vp[vp.index('static void detect_hall_foc'):vp.index('static int32_t q4_to_milliamps_normalized')]
assert 'uint8_t reply[10]' in det and 'reply[9] = 0u' in det
assert 's_last_hall_store_ok' in det and 'uart_send_payload(reply, sizeof(reply))' in det

# Standard VESC position stays single-turn; project multi-turn uses CUSTOM_APP_DATA signed int32.
assert 'case COMM_SET_POS:' in vp and 'COMM_CUSTOM_APP_DATA' in vp
assert 'HB_CUSTOM_SET_POS_LIMITS' in vp and 'HB_CUSTOM_SET_POS_TARGET' in vp
assert 'standard VESC position must be 0..360 degrees' in dual
assert 'def set_position_limits(' in dual and 'def set_position_counts(' in dual

# Complete hardware diagnostic tool includes 3A, 50 ERPM, Hall, RT 50Hz and position tests.
for token in ('current --motor left --amps 3','rpm --motor left --erpm 50','rt --motor both --hz 50',
              'hall --motor left --amps 1.0','pos-limits','pos-count','CURRENT_COMMAND_PATH_PASS',
              'RESULT: PASS realtime polling'):
    assert token in dbg, token
assert 'DEFAULT_HZ = 50.0' in dbg
print('V15_FEATURE_STATIC_PASS current3A=1 rpm50_fractional=1 rt50=1 hall_isr_sweep=1 pos_int32=1 live_toggle=removed fw600_exact=1')
