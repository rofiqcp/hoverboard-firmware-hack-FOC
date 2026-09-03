#!/usr/bin/env python3
import importlib.util, pathlib, struct, sys
p=next(x for x in pathlib.Path(__file__).resolve().parents if x.name=='tools')/'vesc_dual.py'
spec=importlib.util.spec_from_file_location('vd',p);vd=importlib.util.module_from_spec(spec);sys.modules['vd']=vd;spec.loader.exec_module(vd)
pl=bytes([vd.COMM_SET_CURRENT])+struct.pack('>i',2500)
f=vd.frame(pl)
assert f[0]==2 and f[1]==len(pl) and f[-1]==3
assert ((f[-3]<<8)|f[-2])==vd.crc16(pl)
d=vd.PacketDecoder(); out=[]
for chunk in (f[:2],f[2:5],f[5:]): out.extend(d.feed(chunk))
assert out==[pl]
fw=vd.VescDual.fwd(bytes([vd.COMM_FW_VERSION]))
assert fw==bytes([vd.COMM_FORWARD_CAN,2,vd.COMM_FW_VERSION])
pos_payload=bytes([vd.COMM_SET_POS])+struct.pack('>i',30000000)
assert vd.VescDual.fwd(pos_payload)==bytes([vd.COMM_FORWARD_CAN,2])+pos_payload
hall_payload=bytes([vd.COMM_DETECT_HALL_FOC])+struct.pack('>i',1000)
assert vd.COMM_DETECT_HALL_FOC==28 and vd.VescDual.fwd(hall_payload)==bytes([vd.COMM_FORWARD_CAN,2])+hall_payload
mask=vd.VALUE_MASK
payload=bytearray([vd.COMM_GET_VALUES_SELECTIVE])+bytearray(struct.pack('>I',mask))
# bits 2,3,4,5
for x in (150,-20,200,0): payload += struct.pack('>i',x)
# bit6 duty, bit7 rpm, bit8 vin
payload += struct.pack('>h',123)+struct.pack('>i',321)+struct.pack('>h',481)
# bit15 fault, bit16 position, bit17 id
payload += bytes([0])+struct.pack('>i',45000000)+bytes([2])
# bit19 vd, bit20 vq
payload += struct.pack('>i',1200)+struct.pack('>i',-5000)
v=vd.parse_selective(bytes(payload))
assert abs(v.current_motor-1.5)<1e-9 and v.vesc_id==2 and v.rpm==321 and abs(v.vq+5)<1e-9 and abs(v.position-45.0)<1e-9
print(f'PY_VESC_DUAL_PACKET_PASS crc=0x{vd.crc16(pl):04x} forward_can_id={fw[1]} values_id={v.vesc_id} pos={v.position:.1f}')
