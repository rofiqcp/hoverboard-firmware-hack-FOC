#!/usr/bin/env python3
import sys
from pathlib import Path
TOOLS_DIR = next(p for p in Path(__file__).resolve().parents if p.name == 'tools')
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))
import argparse,csv,json,subprocess,tempfile,time
from vesc_dual import VescDual
from test_speed_pid_sweep_hw import get_mc,set_mc,run_case,release
TOOL='/tmp/mc_speed_tool2'

def patch(raw,kp,ki,kd,ramp,min_erpm=75):
    with tempfile.TemporaryDirectory() as td:
        a=Path(td)/'a'; b=Path(td)/'b'; a.write_bytes(raw)
        cp=subprocess.run([TOOL,str(a),str(b),str(kp),str(ki),str(kd),str(ramp),str(min_erpm)],capture_output=True,text=True,check=True)
        return b.read_bytes(),cp.stdout.strip()

def main():
    ap=argparse.ArgumentParser(description='Guarded speed-ramp hardware sweep.')
    ap.add_argument('--port',default='/dev/ttyUSB0')
    ap.add_argument('--arm',action='store_true',help='required to actuate the motor')
    a=ap.parse_args()
    if not a.arm: ap.error('motor actuation requires --arm')
    stamp=time.strftime('%Y%m%d_%H%M%S'); out=TOOLS_DIR/'results'/'speed_pid'/stamp; out.mkdir(parents=True,exist_ok=True)
    rawp=out/'ramp_raw.csv'; sump=out/'ramp_summary.csv'
    configs={'left':(.009,.020,0.0),'right':(.0095,.022,0.0)}; ramps=[600,900,1200,1500]
    fields=['case','motor','kp','ki','kd','target','phase','t','erpm','iq','id','imotor','iin','duty','vq','vd','fault']
    link=VescDual(a.port,1000000,timeout=.65); orig={}; res=[]
    try:
        for right,motor in [(False,'left'),(True,'right')]: orig[motor]=get_mc(link,right)
        with rawp.open('w',newline='') as f:
            wr=csv.DictWriter(f,fieldnames=fields);wr.writeheader()
            for right,motor in [(False,'left'),(True,'right')]:
                kp,ki,kd=configs[motor]
                for ramp in ramps:
                    raw,desc=patch(orig[motor],kp,ki,kd,ramp); set_mc(link,right,raw); time.sleep(.15)
                    for rep in (1,2):
                        for target in (300,-300):
                            cid=f'{motor}_ramp{ramp}_{target:+d}_r{rep}'
                            m=run_case(link,right,motor,kp,ki,kd,target,2.2,.8,18,1000,1.2,wr,cid); m['ramp']=ramp;m['repeat']=rep;res.append(m);f.flush();print(json.dumps(m,sort_keys=True),flush=True);time.sleep(.2)
                    set_mc(link,right,orig[motor]);release(link,right);time.sleep(.2)
    finally:
        for right,motor in [(False,'left'),(True,'right')]:
            try:
                if motor in orig:set_mc(link,right,orig[motor]);release(link,right)
            except Exception as e:print('RESTORE_WARN',motor,e,flush=True)
        link.close()
    keys=sorted({k for x in res for k in x});
    with sump.open('w',newline='') as f:w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows(res)
    print('RAW',rawp);print('SUMMARY',sump)
if __name__=='__main__':main()
