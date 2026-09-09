#!/usr/bin/env python3
import argparse, importlib.util, pathlib, sys
R=next(p for p in pathlib.Path(__file__).resolve().parents if (p/'platformio.ini').exists())
spec=importlib.util.spec_from_file_location('up',R/'tools/pio_vesc_upload.py')
up=importlib.util.module_from_spec(spec);sys.modules['up']=up;spec.loader.exec_module(up)

class Base(up.Link):
    def _holders(self,port): return [(1234,'/home/sirobo/agv/install/stmf4/lib/stmf4/stmf4_hmi_bridge')]
    def _resume_ros_launch(self): self._suspended_launch_pids=[]

class TcpWins(Base):
    direct=False
    def _open_tcp(self,attempts=1): self.sock=object()
    def _open_f411_direct(self): self.direct=True
    def close(self): pass
a=argparse.Namespace(transport='f411',host='127.0.0.1',port=65101,serial_port='/dev/ttyACM0',baud=1000000,tcp_wait=5.0)
x=TcpWins(a); assert x.route=='tcp65101' and not x.direct

class DirectFallback(Base):
    prepared=False; direct=False
    def _open_tcp(self,attempts=1): raise RuntimeError('tcp down')
    def _prepare_direct_f411(self): self.prepared=True
    def _open_f411_direct(self): self.f411_direct=True; self.direct=True
    def close(self): pass
a.tcp_wait=0.0
y=DirectFallback(a); assert y.route=='f411_direct' and y.prepared and y.direct
print('PIO_VESC_F411_ROUTE_PASS tcp_priority=65101 wait_max=5 direct_fallback=1')
