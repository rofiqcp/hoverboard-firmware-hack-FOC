#!/usr/bin/env python3
from pathlib import Path
import hashlib, re, shutil, subprocess, sys, tempfile

ROOT = Path(__file__).resolve().parents[1]
EXPECTED_DATATYPES_SHA256 = '4ecae1f31c12c1ab415d47dd997396d0792e94249203cbeb877ada75f76d5340'

def run(cmd):
    print('+', ' '.join(map(str, cmd)))
    r = subprocess.run(cmd, cwd=ROOT, text=True)
    if r.returncode:
        raise SystemExit(r.returncode)

def check_static():
    required = [
        'platformio.ini','STM32F103RCTx_FLASH.ld',
        'Src/main.c','Src/setup.c','Src/util.c','Src/comms.c','Src/config.h',
        'Src/motor/foc_math.c','Src/motor/mc_interface.c','Src/motor/mcpwm_foc.c',
        'Src/vesc/buffer.c','Src/vesc/crc.c','Src/vesc/mcconf_serial.c','Src/vesc/vesc_protocol.c',
        'Src/motor/mcpwm_foc.c','Src/motor/mcpwm_foc.h','Src/motor/mc_interface.c','Src/motor/mc_interface.h',
        'Src/motor/foc_math.c','Src/motor/foc_math.h','Src/motor/mcconf_default.h',
        'Src/vesc/datatypes.h','Src/vesc/vesc_protocol.c','Src/vesc/vesc_protocol.h',
        'Src/vesc/buffer.c','Src/vesc/crc.c','Src/vesc/mcconf_serial.c',
        'tools/vesc_dual.py','tools/hoverserial.py','tools/test_motor_control_v10.py','tools/test_motor_control_v10.c'
    ]
    missing=[x for x in required if not (ROOT/x).exists()]
    assert not missing, f'missing required files: {missing}'
    for obsolete in ('Src/BLDC_controller.c','Src/BLDC_controller.h','Src/BLDC_controller_data.c','Src/bldc.c','Src/rtwtypes.h','Src/current_scale.h'):
        assert not (ROOT/obsolete).exists(), f'obsolete generated file still present: {obsolete}'
    port_files=list((ROOT/'Src').rglob('port_*.c')) + list((ROOT/'Src').rglob('port_*.h'))
    assert not port_files, f'port wrapper files still present: {[str(x.relative_to(ROOT)) for x in port_files]}'
    source_text='\n'.join(p.read_text(errors='ignore') for p in (ROOT/'Src').rglob('*') if p.is_file())
    for token in ('BLDC_controller','rtwtypes.h','rtP_Left','rtP_Right','rtDW_Left','rtDW_Right'):
        assert token not in source_text, f'obsolete generated dependency in live source: {token}'
    ini=(ROOT/'platformio.ini').read_text()
    for token in ('src_dir = Src','default_envs = VARIANT_USART','board = genericSTM32F103RC','build_src_flags =','-Wall','-Wextra','-Werror','-I.'):
        assert token in ini, f'platformio.ini missing {token}'
    # Warning policy: project sources use -Wall/-Wextra/-Werror via build_src_flags only.
    # Framework STM32Cube must not inherit project -Werror (avoids HAL_PCD unused-parameter build failure).
    before_build_flags=ini.split('build_flags =',1)[0]
    assert '-Werror' in before_build_flags and 'build_src_flags =' in before_build_flags, 'project -Werror is not scoped with build_src_flags'
    global_build=ini.split('build_flags =',1)[1]
    assert '-Werror' not in global_build and '-Wextra' not in global_build and '-Wall' not in global_build, 'warning flags leaked into framework build_flags'
    cfg=(ROOT/'Src/config.h').read_text()
    assert 'current_scale.h' not in cfg and 'CURRENT_COUNTS_PER_A' not in cfg, 'obsolete current_scale dependency remains'
    assert re.search(r'#define\s+A2BIT_CONV\s+50\b',cfg), 'A2BIT_CONV must be defined directly as 50 in config.h'
    assert re.search(r'#define\s+SERIAL_BUFFER_SIZE\s+768\b',cfg), 'USART3 RX DMA buffer is not 768 bytes'
    mc=(ROOT/'Src/motor/mcpwm_foc.c').read_text()
    mathc=(ROOT/'Src/motor/foc_math.c').read_text()
    mcc=(ROOT/'Src/motor/mcconf_default.h').read_text()
    assert re.search(r'#define\s+MCCONF_FOC_CONTROL_DIV\s+3u',mcc), 'FOC scheduler must match generated 1-of-3 cadence'
    assert 'e_q4_raw=((int32_t)m->m_speed_set_rpm-(int32_t)m->m_rpm)<<4' in mc, 'speed PI error must be Q4 mechanical RPM'
    assert 'speed PI drives Vq directly' in mc and 'm->m_iq_set_q4=pi_run_state' not in mc, 'mode2 must use proven speed-PI-to-Vq architecture'
    assert 'MCCONF_FOC_CLOSED_LOOP_VOLTAGE_MAX' in mc and 'voltage_circle_q_limit' in mc, 'closed-loop voltage-circle anti-windup missing'
    assert 'iq_setpoint_slew_step' in mc and 'MCCONF_CURRENT_SLEW_A_PER_S' in mc, 'mode3 current setpoint slew missing'
    assert 'leftPhaseTrip=leftBridgeWasOn' in mc and 'rightPhaseTrip=rightBridgeWasOn' in mc, 'phase over-current must protect all powered modes'
    assert 'v->rpm=mcpwm_foc_get_erpm_motor(second)' in mc, 'VESC mc_values.rpm must be ERPM'
    assert 'erpm_to_mech_rpm' in mc, 'VESC COMM_SET_RPM ERPM conversion missing'
    assert 'm->m_phase_openloop : m->m_phase_hall' in mc and 'm_phase_openloop + (65536/12)' not in mc, 'mode4 has incorrect +30deg phase offset'
    assert '(base_pos+1u)%6u' in mc, 'reverse Hall pos+1 convention missing'
    assert 'MCCONF_HALL_INTERP_ON_RPM' in mc and 'MCCONF_HALL_INTERP_OFF_RPM' in mc, 'low-speed Hall interpolation hysteresis missing'
    assert 'phase_current_counts_to_q4' in mc and '27200' in mc, 'generated current input saturation missing'
    assert 'm->m_duty_set_permille*MCCONF_FOC_VOLTAGE_MAX' in mc, 'mode1 permille-to-voltage scaling missing'
    assert 'trq_ca_to_q4' in mc and 'A2BIT_CONV*16)/100' in mc, 'mode3 centiampere scaling missing'
    assert 'm_openloop_id_ramp_q16' in mc and 'MCCONF_OPENLOOP_ID_SLEW_A_S' in mc, 'mode4 Id slew protection missing'
    assert re.search(r'#define\s+SVPWM_MAX_ID_A\s+6u',cfg), 'mode4 Id safety ceiling must be 6A'
    assert re.search(r'#define\s+SVPWM_PHASE_LIMIT_A\s+8u',cfg), 'mode4 phase-current trip must be 8A'
    assert re.search(r'#define\s+SVPWM_DC_LIMIT_A\s+8u',cfg), 'mode4 DC-link trip must be 8A'
    assert re.search(r'#define\s+SVPWM_OPENLOOP_RPM_DEFAULT\s+10u',cfg), 'mode4 default open-loop speed must be 10 rpm'
    assert re.search(r'#define\s+SVPWM_ID_SLEW_A_PER_S\s+4u',cfg), 'mode4 Id slew must be 4 A/s'
    assert '(((i_sum >> 16) << 1) + (int32_t)p_term) >> 1' in mathc, 'PI equation no longer matches generated PI_clamp_fixdt'
    vp=(ROOT/'Src/vesc/vesc_protocol.c').read_text()
    assert re.search(r'#define\s+VESC_MAX_PAYLOAD\s+700u',vp), 'VESC payload buffer is not 700 bytes'
    assert 'COMM_FORWARD_CAN' in vp and 'COMM_PING_CAN' in vp, 'virtual CAN routing missing'
    assert 'VESC_SECOND_ID' in vp or 'MCCONF_VESC_RIGHT_ID' in vp or 'RIGHT' in vp, 'virtual right ID declaration not found'
    dual=(ROOT/'tools/vesc_dual.py').read_text()
    assert 'RIGHT_ID = 2' in dual and 'COMM_FORWARD_CAN = 34' in dual, 'Python right virtual CAN routing mismatch'
    mainc=(ROOT/'Src/main.c').read_text()
    assert 'feedback.dutyR_x1000 = (int16_t)(-m_motor_2.m_duty_now_permille);' in mainc, 'custom telemetry right duty sign not normalized'
    assert 'feedback.vqR_cV = focVoltageToCentiVolt((int16_t)-m_motor_2.m_vq);' in mainc, 'custom telemetry right Vq sign not normalized'
    h=hashlib.sha256((ROOT/'Src/vesc/datatypes.h').read_bytes()).hexdigest()
    assert h == EXPECTED_DATATYPES_SHA256, f'datatypes.h SHA mismatch: {h}'
    # Verify quoted project includes resolve locally, excluding STM32Cube/CMSIS framework includes.
    framework_prefixes=('stm32f1xx','core_cm','cmsis')
    for d in ('Src',):
        for p in (ROOT/d).rglob('*.[ch]'):
            txt=p.read_text(errors='ignore')
            for inc in re.findall(r'#include\s+"([^"]+)"',txt):
                if inc.startswith(framework_prefixes):
                    continue
                candidates=[p.parent/inc, ROOT/inc, ROOT/'Src'/inc, ROOT/'Src/motor'/inc, ROOT/'Src/vesc'/inc]
                assert any(c.exists() for c in candidates), f'unresolved local include {inc} from {p.relative_to(ROOT)}'
    print('STATIC_PROJECT_AUDIT_PASS')
    print('DATATYPES_SHA256',h)

def config_size_check():
    compilers=[c for c in ('gcc','clang') if shutil.which(c)]
    assert compilers, 'gcc/clang unavailable'
    with tempfile.TemporaryDirectory(prefix='vesc-config-size-') as td:
        for cc in compilers:
            out=Path(td)/f'cfg_{cc}'
            run([cc,'-std=c11','-O0','-Wall','-Wextra','-Werror','-I.','-ISrc',
                 'tools/test_config_sizes.c','Src/vesc/buffer.c','Src/vesc/mcconf_serial.c','-o',str(out)])
            run([str(out)])
    print('CONFIG_SERIALIZER_GCC_CLANG_PASS')

if __name__ == '__main__':
    check_static()
    run([sys.executable,'-m','py_compile','tools/hoverserial.py','tools/vesc_dual.py','tools/test_vesc_dual.py'])
    run([sys.executable,'tools/host_compile_check.py'])
    run([sys.executable,'tools/test_foc_math.py'])
    run([sys.executable,'tools/test_motor_control_v10.py'])
    run([sys.executable,'tools/test_vesc_protocol_host.py'])
    config_size_check()
    run([sys.executable,'tools/test_vesc_dual.py'])
    print('ALL_FINAL_HOST_CHECKS_PASS')
