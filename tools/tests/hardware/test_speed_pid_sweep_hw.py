#!/usr/bin/env python3
import sys
from pathlib import Path
TOOLS_DIR = next(p for p in Path(__file__).resolve().parents if p.name == 'tools')
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))
import argparse,csv,json,math,os,statistics,struct,subprocess,tempfile,time
from vesc_dual import VescDual,COMM_SET_RPM,COMM_ALIVE
COMM_GET_MCCONF=14; COMM_SET_MCCONF=13
GAIN_TOOL='/tmp/mc_speed_gain_tool'

def get_mc(link,right):
    req=bytes([COMM_GET_MCCONF]); p=link.transact(link.fwd(req) if right else req,COMM_GET_MCCONF,1.2); return p[1:]
def set_mc(link,right,raw):
    req=bytes([COMM_SET_MCCONF])+raw; p=link.transact(link.fwd(req) if right else req,COMM_SET_MCCONF,1.2)
    if p!=bytes([COMM_SET_MCCONF]): raise RuntimeError(f'bad SET_MCCONF ack {p.hex()}')
def patch_gain(raw,kp,ki,kd):
    with tempfile.TemporaryDirectory() as td:
        a=Path(td)/'a.bin'; b=Path(td)/'b.bin'; a.write_bytes(raw)
        cp=subprocess.run([GAIN_TOOL,str(a),str(b),str(kp),str(ki),str(kd)],capture_output=True,text=True,check=True)
        return b.read_bytes(),cp.stdout.strip().splitlines()[-1]
def cmd_rpm(link,right,erpm):
    # VESC protocol firmware normalizes mirrored motor-2 internally. Send the same user-facing ERPM to both endpoints.
    x=int(round(erpm)); p=bytes([COMM_SET_RPM])+struct.pack('>i',x)
    with link.io_lock: link.send(link.fwd(p) if right else p)
def alive(link,right):
    p=bytes([COMM_ALIVE]);
    with link.io_lock: link.send(link.fwd(p) if right else p)
def release(link,right):
    cmd_rpm(link,right,0); alive(link,right)

def metrics(rows,target,run_s):
    rr=[r for r in rows if r['phase']=='run']; ss=[r for r in rows if r['phase']=='stop']; sgn=1 if target>0 else -1
    y=[sgn*r['erpm'] for r in rr]; tt=[r['t'] for r in rr]; tgt=abs(target)
    peak=max(y) if y else 0.0; trough=min(y) if y else 0.0
    rise=None
    for t,v in zip(tt,y):
        if v>=.9*tgt: rise=t; break
    steady=[sgn*r['erpm'] for r in rr if r['t']>=max(0,run_s-.30)]
    sm=statistics.mean(steady) if steady else 0.0; sstd=statistics.pstdev(steady) if len(steady)>1 else 0.0
    overs=max(0.0,(peak-tgt)/tgt*100.0) if tgt else 0.0
    sse=(sm-tgt)/tgt*100.0 if tgt else 0.0
    stop_rev=max([max(0.0,-sgn*r['erpm']) for r in ss],default=0.0)
    max_i=max([abs(r['imotor']) for r in rows],default=0.0); max_iq=max([abs(r['iq']) for r in rows],default=0.0)
    max_d=max([abs(r['duty']) for r in rows],default=0.0)
    di=max([abs(rows[i]['iq']-rows[i-1]['iq']) for i in range(1,len(rows))],default=0.0)
    return dict(peak_erpm=peak,min_norm_erpm=trough,rise90_s=rise,steady_erpm=sm,steady_std=sstd,overshoot_pct=overs,steady_error_pct=sse,stop_reverse_erpm=stop_rev,peak_current_a=max_i,peak_iq_a=max_iq,peak_duty=max_d,max_sample_diq_a=di)

def run_case(link,right,motor,kp,ki,kd,target,run_s,stop_s,hz,max_erpm,max_current,writer,case_id):
    rows=[]; abort=''; t0=time.monotonic(); last_alive=0
    try:
        cmd_rpm(link,right,target)
        while time.monotonic()-t0<run_s:
            now=time.monotonic()
            if now-last_alive>.18: alive(link,right); last_alive=now
            v=link.values(right); r=dict(case=case_id,motor=motor,kp=kp,ki=ki,kd=kd,target=target,phase='run',t=now-t0,erpm=v.rpm,iq=v.iq,id=v.id,imotor=v.current_motor,iin=v.current_in,duty=v.duty,vq=v.vq,vd=v.vd,fault=v.fault)
            rows.append(r); writer.writerow(r)
            if v.fault: abort=f'fault={v.fault}'; break
            if abs(v.rpm)>max_erpm: abort=f'erpm={v.rpm:.0f}'; break
            if abs(v.current_motor)>max_current: abort=f'I={v.current_motor:.2f}'; break
            time.sleep(max(0,1/hz))
        release(link,right); ts=time.monotonic()
        while time.monotonic()-ts<stop_s:
            now=time.monotonic(); alive(link,right); v=link.values(right); r=dict(case=case_id,motor=motor,kp=kp,ki=ki,kd=kd,target=0,phase='stop',t=now-ts,erpm=v.rpm,iq=v.iq,id=v.id,imotor=v.current_motor,iin=v.current_in,duty=v.duty,vq=v.vq,vd=v.vd,fault=v.fault)
            rows.append(r); writer.writerow(r)
            if v.fault and not abort: abort=f'stop_fault={v.fault}'
            time.sleep(max(0,1/hz))
    finally:
        try: release(link,right)
        except Exception: pass
    d=link.diag(right); m=metrics(rows,target,run_s); m.update(case=case_id,motor=motor,kp=kp,ki=ki,kd=kd,target_erpm=target,abort=abort,fault=d.fault,hall_bad=d.hall_invalid,seq_reject=d.hall_sequence_rejects or 0,period_reject=d.hall_period_rejects or 0,qdrop=d.rx_queue_drops or 0,isr_max=d.foc_isr_cycles_max or 0)
    return m

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('port',nargs='?',default='/dev/ttyUSB0'); ap.add_argument('--target',type=float,default=300); ap.add_argument('--run-s',type=float,default=1.2); ap.add_argument('--stop-s',type=float,default=.8); ap.add_argument('--hz',type=float,default=18); ap.add_argument('--max-erpm',type=float,default=900); ap.add_argument('--max-current',type=float,default=1.2); ap.add_argument('--out',default='tools/results/speed_pid'); ap.add_argument('--quick',action='store_true'); ap.add_argument('--gains',default='',help='semicolon-separated kp,ki,kd triples'); ap.add_argument('--targets',default='',help='comma-separated positive ERPM magnitudes'); ap.add_argument('--repeats',type=int,default=1); a=ap.parse_args()
    base=[(.002,.002,0),(.004,.004,0),(.006,.008,0),(.008,.015,0),(.010,.025,0)]
    if a.gains:
        base=[]
        for item in a.gains.split(';'):
            kp,ki,kd=(float(x) for x in item.split(',')); base.append((kp,ki,kd))
    elif a.quick: base=base[:3]
    targets=[a.target] if not a.targets else [float(x) for x in a.targets.split(',') if x.strip()]
    out=Path(a.out); out.mkdir(parents=True,exist_ok=True); stamp=time.strftime('%Y%m%d_%H%M%S'); rawcsv=out/f'raw_{stamp}.csv'; summ=out/f'summary_{stamp}.csv'; meta=out/f'meta_{stamp}.json'
    link=VescDual(a.port,115200,timeout=.65); originals={}; results=[]
    fields=['case','motor','kp','ki','kd','target','phase','t','erpm','iq','id','imotor','iin','duty','vq','vd','fault']
    try:
        for right,motor in [(False,'left'),(True,'right')]: originals[motor]=get_mc(link,right)
        (out/f'mc_left_original_{stamp}.bin').write_bytes(originals['left']); (out/f'mc_right_original_{stamp}.bin').write_bytes(originals['right'])
        with rawcsv.open('w',newline='') as f:
            wr=csv.DictWriter(f,fieldnames=fields); wr.writeheader()
            for right,motor in [(False,'left'),(True,'right')]:
                orig=originals[motor]
                for kp,ki,kd in base:
                    patched,desc=patch_gain(orig,kp,ki,kd); set_mc(link,right,patched); time.sleep(.12)
                    for mag in targets:
                        for rep in range(1,max(1,a.repeats)+1):
                            for target in (mag,-mag):
                                cid=f'{motor}_kp{kp:g}_ki{ki:g}_kd{kd:g}_{target:+.0f}_r{rep}'
                                m=run_case(link,right,motor,kp,ki,kd,target,a.run_s,a.stop_s,a.hz,a.max_erpm,a.max_current,wr,cid); m['repeat']=rep; results.append(m); f.flush()
                                print(json.dumps(m,sort_keys=True)); time.sleep(.25)
                                if m['fault'] or m['abort'].startswith('fault') or m['abort'].startswith('I='): break
                set_mc(link,right,orig); time.sleep(.15); release(link,right)
    finally:
        for right,motor in [(False,'left'),(True,'right')]:
            try:
                if motor in originals: set_mc(link,right,originals[motor]); release(link,right)
            except Exception as e: print('RESTORE_WARN',motor,e)
        link.close()
    keys=sorted({k for r in results for k in r});
    with summ.open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=keys); w.writeheader(); w.writerows(results)
    meta.write_text(json.dumps(dict(port=a.port,target=a.target,targets=targets,repeats=a.repeats,run_s=a.run_s,stop_s=a.stop_s,hz=a.hz,max_erpm=a.max_erpm,max_current=a.max_current,cases=base,stamp=stamp),indent=2))
    print('RAW',rawcsv); print('SUMMARY',summ); print('META',meta)
if __name__=='__main__': main()
