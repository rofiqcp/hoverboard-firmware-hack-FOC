#!/usr/bin/env python3
from pathlib import Path
import hashlib, re
R=Path(__file__).resolve().parents[1]
com=(R/'Src/comms.c').read_text()
mc=(R/'Src/motor/mcpwm_foc.c').read_text()
vp=(R/'Src/vesc/vesc_protocol.c').read_text()
main=(R/'Src/main.c').read_text()
hov=(R/'tools/hoverserial.py').read_text()
dual=(R/'tools/vesc_dual.py').read_text()

# V13 tuning/position parameters remain, but the obsolete user-toggleable live
# switch was intentionally removed in V15. Telemetry is automatic instead.
for name in ('KPQ','KIQ','KPD','KID','KPS','KIS','KDS','KPP','KIP','KDP','PMIN','PMAX','PSETL','PSETR'):
    assert f'"{name}"' in com, f'missing parameter {name}'
assert '"LIVE"' not in com, 'obsolete LIVE parameter must be removed'
assert 'op == "live"' not in com.lower(), 'obsolete live command handler must be removed'
assert '"RESETPOS"' in com, 'position reset command missing'
assert 'int64_t value = 0;' in com, 'parameter averaging is not int64-safe'
assert '2147483648LL' in com and '2147483647LL' in com, 'debug parser does not cover full int32 range'
assert 'positionMinUser = INT32_MIN' in com and 'positionMaxUser = INT32_MAX' in com
assert 'mcpwm_foc_set_position_user_limits(positionMinUser, positionMaxUser, false)' in com
assert 'mcpwm_foc_set_position_user_limits(positionMinUser, positionMaxUser, true)' in com
assert 'user_position_to_internal' in mc and 'positionCommandR' in mc
assert 'CONTROL_MODE_POS' in mc and 'm_position_target_counts' in mc
assert 'm->m_iq_target_q4=pid_run_state' in mc, 'position PID must feed Iq target'
assert 'm->m_kpq_q11,m->m_kiq_q16' in mc and 'm->m_kpd_q11,m->m_kid_q16' in mc
assert 'm->m_kps_q11' in mc and 'm->m_kis_q16' in mc and 'm->m_kds_q11' in mc and 'speed_pid_iq_target_step' in mc
assert 'm->m_kpp_q11,m->m_kip_q16,m->m_kdp_q11' in mc
assert 'telemetryNowMs - legacyTelemetryPrevMs) >= 20u' in main, 'automatic 50 Hz legacy telemetry missing'
assert 'no user-controlled live telemetry switch' in main, 'live-removal rationale missing'
assert 'case COMM_SET_POS:' in vp and 'COMM_FORWARD_CAN' in vp and 'COMM_PING_CAN' in vp
assert 'v->tachometer = -v->tachometer;' in vp and 'v->position = 360.0f - v->position' in vp, 'right VESC position telemetry not normalized'
assert 'op == "live"' not in hov.lower() and 'toggle custom live telemetry' not in hov.lower()
assert 'reset pos' in hov and 'mode 5' in hov
assert 'COMM_SET_POS = 9' in dual and 'def set_pos(' in dual and 'RIGHT_ID = 2' in dual
# vesc/datatypes.h must remain exact reference, never custom-extended for these parameters.
h=hashlib.sha256((R/'Src/vesc/datatypes.h').read_bytes()).hexdigest()
assert h=='4ecae1f31c12c1ab415d47dd997396d0792e94249203cbeb877ada75f76d5340', h
print('V13_FEATURE_STATIC_PASS full_int32=1 live_toggle=removed host_polling=1 pos=1 separate_dq=1 can2=1 datatypes_exact=1')
