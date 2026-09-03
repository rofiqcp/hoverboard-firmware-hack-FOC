#!/usr/bin/env python3
import csv,struct,subprocess,tempfile,time
from pathlib import Path
from vesc_dual import VescDual,COMM_SET_RPM,COMM_SET_CURRENT_BRAKE,COMM_SET_HANDBRAKE,COMM_ALIVE
COMM_GET_MCCONF=14; COMM_SET_MCCONF=13; GAIN_TOOL='/tmp/mc_speed_gain_tool'
def getmc(l,r):
 p=l.transact(l.fwd(bytes([COMM_GET_MCCONF])) if r else bytes([COMM_GET_MCCONF]),COMM_GET_MCCONF,1.2); return p[1:]
def setmc(l,r,raw):
 q=bytes([COMM_SET_MCCONF])+raw; p=l.transact(l.fwd(q) if r else q,COMM_SET_MCCONF,1.2); assert p==bytes([COMM_SET_MCCONF])
def patch(raw,kp=.0095,ki=.022,kd=0):
 with tempfile.TemporaryDirectory() as td:
  a=Path(td)/'a';b=Path(td)/'b';a.write_bytes(raw);subprocess.run([GAIN_TOOL,str(a),str(b),str(kp),str(ki),str(kd)],check=True,capture_output=True);return b.read_bytes()
def send(l,r,c,val,scale):
 p=bytes([c])+struct.pack('>i',round(val*scale));
 with l.io_lock:l.send(l.fwd(p) if r else p)
def alive(l,r):
 with l.io_lock:l.send(l.fwd(bytes([COMM_ALIVE])) if r else bytes([COMM_ALIVE]))
def release(l,r): send(l,r,COMM_SET_RPM,0,1);alive(l,r)
def sample(l,r,phase,t0,row):
 v=l.values(r); row.update(phase=phase,t=time.monotonic()-t0,erpm=v.rpm,iq=v.iq,imotor=v.current_motor,duty=v.duty,fault=v.fault);return v
def main():
 out=Path('tools/results_speed_pid');out.mkdir(exist_ok=True);stamp=time.strftime('%Y%m%d_%H%M%S');fpath=out/f'brake_handbrake_{stamp}.csv'
 L=VescDual('/dev/ttyUSB0',115200,timeout=.7);orig={};rows=[]
 try:
  for r,m in [(False,'left'),(True,'right')]:orig[m]=getmc(L,r);setmc(L,r,patch(orig[m]));release(L,r)
  for r,m in [(False,'left'),(True,'right')]:
   for sign in (1,-1):
    for ba in (.2,.4):
     # accelerate with safe speed PID
     t0=time.monotonic();send(L,r,COMM_SET_RPM,450*sign,1)
     while time.monotonic()-t0<1.3:
      alive(L,r);v=sample(L,r,'drive',t0,dict(test='brake',motor=m,sign=sign,amps=ba),);rows.append(dict(test='brake',motor=m,sign=sign,amps=ba,phase='drive',t=time.monotonic()-t0,erpm=v.rpm,iq=v.iq,imotor=v.current_motor,duty=v.duty,fault=v.fault));
      if v.fault or abs(v.rpm)>900 or abs(v.current_motor)>1.2:break
      time.sleep(.055)
     send(L,r,COMM_SET_CURRENT_BRAKE,ba,1000);tb=time.monotonic()
     while time.monotonic()-tb<1.2:
      alive(L,r);v=L.values(r);rows.append(dict(test='brake',motor=m,sign=sign,amps=ba,phase='brake',t=time.monotonic()-tb,erpm=v.rpm,iq=v.iq,imotor=v.current_motor,duty=v.duty,fault=v.fault));time.sleep(.055)
     release(L,r);time.sleep(.25)
     br=[x for x in rows if x['test']=='brake' and x['motor']==m and x['sign']==sign and x['amps']==ba and x['phase']=='brake'];norm=[sign*x['erpm'] for x in br]
     print('BRAKE',m,sign,ba,'start',round(norm[0],1) if norm else None,'min',round(min(norm),1) if norm else None,'final',round(norm[-1],1) if norm else None,'peakIq',round(max(abs(x['iq']) for x in br),3) if br else None,'fault',max(x['fault'] for x in br) if br else None)
   # handbrake at standstill, no preceding drive
   for ha in (.1,.2,.4):
    release(L,r);time.sleep(.35);send(L,r,COMM_SET_HANDBRAKE,ha,1000);th=time.monotonic();hs=[]
    while time.monotonic()-th<.9:
     alive(L,r);v=L.values(r);x=dict(test='handbrake',motor=m,sign=0,amps=ha,phase='hold',t=time.monotonic()-th,erpm=v.rpm,iq=v.iq,imotor=v.current_motor,duty=v.duty,fault=v.fault);rows.append(x);hs.append(x);time.sleep(.055)
    release(L,r);time.sleep(.25)
    print('HANDBRAKE',m,ha,'peakERPM',max(abs(x['erpm']) for x in hs),'finalERPM',hs[-1]['erpm'],'peakIq',max(abs(x['iq']) for x in hs),'fault',max(x['fault'] for x in hs))
 finally:
  for r,m in [(False,'left'),(True,'right')]:
   try:
    if m in orig:setmc(L,r,orig[m]);release(L,r)
   except Exception as e:print('restore warn',m,e)
  L.close()
 with fpath.open('w',newline='') as f:
  w=csv.DictWriter(f,fieldnames=['test','motor','sign','amps','phase','t','erpm','iq','imotor','duty','fault']);w.writeheader();w.writerows(rows)
 print('CSV',fpath)
if __name__=='__main__':main()
