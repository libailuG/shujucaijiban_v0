"""Run the linked ARM coordinate adapter and controller in Unicorn; no hardware I/O.
Build first: firmware/build.ps1, then fromelf --text -s --output
work/model_orientation_symbols.txt firmware/ObjectsCli/zhuban_caiji.axf.
Run from project root using a Python environment with unicorn installed.
"""
import math
import re
import struct
from pathlib import Path
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS
from unicorn.arm_const import *

ROOT = Path(__file__).resolve().parents[1]
elf = (ROOT/'firmware/ObjectsCli/zhuban_caiji.axf').read_bytes()
symbols = (ROOT/'work/model_orientation_symbols.txt').read_text(encoding='utf-8-sig')
source = (ROOT/'firmware/App/control_mode1.c').read_text(encoding='utf-8-sig')
qsb = [0.]*4
for index,value in re.findall(r'port\.quat_sb\[(\d)\]\s*=\s*([-\d.]+)f',source):
    qsb[int(index)] = float(value)
assert qsb[0] > 0 and qsb[3] < 0, 'Keep the standard body-to-model conversion'
imu_source = (ROOT/'firmware/App/imu.c').read_text(encoding='utf-8-sig')
def body_mapping(name, field):
    expression = re.search(r'float '+name+r'\[3\]\s*=\s*\{([^}]+)\}',imu_source)[1]
    parts = re.findall(r'(-?)sample\.'+field+r'\[(\d)\]',expression)
    assert len(parts)==3
    return [(int(index),-1 if minus else 1) for minus,index in parts]
accel_map = body_mapping('body_accel','accel_g')
gyro_map = body_mapping('body_gyro','gyro_dps')
assert accel_map == gyro_map == [(2,-1),(0,-1),(1,1)]
for field in ('accel_m_s2','gyro_rad_s'):
    for axis,sign,raw_axis in [('x','-','z'),('y','-','x'),('z','','y')]:
        assert f'body.{field}.{axis} = {sign}data.{field}.{raw_axis};' in imu_source

def sym(name):
    found = re.search(r'^\s*\d+\s+'+re.escape(name)+r'\s+(0x[0-9a-fA-F]+)',symbols,re.M)
    assert found,name
    return int(found[1],16)

u = Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS)
u.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_M7)
for address,size in [(0x08000000,0x100000),(0x20000000,0x20000),(0x24000000,0x80000)]:
    u.mem_map(address,size)
offset = struct.unpack_from('<I',elf,28)[0]
size,count = struct.unpack_from('<HH',elf,42)
for i in range(count):
    kind,fileoff,va,pa,filesize,memsize,flags,align = struct.unpack_from('<8I',elf,offset+i*size)
    if kind == 1 and filesize:
        u.mem_write(va,elf[fileoff:fileoff+filesize])
u.reg_write(UC_ARM_REG_C1_C0_2,0xf00000)
u.reg_write(UC_ARM_REG_FPEXC,0x40000000)

def call(name,*args):
    u.reg_write(UC_ARM_REG_SP,0x2001e000)
    u.reg_write(UC_ARM_REG_LR,0x080ff001)
    for reg,value in zip((UC_ARM_REG_R0,UC_ARM_REG_R1,UC_ARM_REG_R2,UC_ARM_REG_R3),args):
        u.reg_write(reg,value)
    u.emu_start(sym(name),0x080ff000,count=3000000)
    assert u.reg_read(UC_ARM_REG_PC) == 0x080ff000,'instruction limit reached'
    return u.reg_read(UC_ARM_REG_R0)

def floats(address,n):
    return struct.unpack('<'+'f'*n,u.mem_read(address,n*4))

def close(a,b):
    assert len(a)==len(b) and all(abs(x-y)<2e-5 for x,y in zip(a,b)),(a,b)

def multiply(a,b):
    w,x,y,z=a; W,X,Y,Z=b
    return [w*W-x*X-y*Y-z*Z,w*X+x*W+y*Z-z*Y,w*Y-x*Z+y*W+z*X,w*Z+x*Y-y*X+z*W]

port,hw,model,config,state,out = [0x20010000+i*0x1000 for i in range(6)]
call('s2_port_config_default',port)
u.mem_write(port,struct.pack('<4f',*qsb))
u.mem_write(port+40,struct.pack('<I',1))
call('s2_config_default',config)
u.mem_write(config,struct.pack('<I',1)) # gyro sensor

def mapped(q,gyro):
    u.mem_write(hw,struct.pack('<12f2I',0,0,0,0,*q,*gyro,.02,1,1))
    assert call('s2_port_map_input',port,hw,model)==0
    return floats(model+16,4),floats(model+32,3)

# Measured movements in the OLD body axes, passed through the corrected body map.
for old_body,expected in [([0,-1,0],[-1,0,0]),([-1,0,0],[0,1,0]),([0,0,1],[0,0,1])]:
    raw = [old_body[1],old_body[2],old_body[0]]
    body = [raw[index]*sign for index,sign in gyro_map]
    _,gyro = mapped([1,0,0,0],body)
    close(gyro,expected)
    call('s2_controller_reset',state)
    assert call('s2_controller_step',state,config,model,out)==1 # warmup
    close(floats(out+62*4,3),[x/(4*math.pi) for x in expected])

# Coherent quaternion transformation: known model tilts reach actual obs[60:62].
angle=.2
for axis,expected in [(0,[-angle/math.pi,0]),(1,[0,angle/math.pi])]:
    qb=[math.cos(angle/2),0,0,0];qb[axis+1]=(-1 if axis==0 else 1)*math.sin(angle/2)
    qs=multiply(qb,[qsb[0],0,0,-qsb[3]])
    result,_=mapped(qs,[0,0,0])
    close(result,qb)
    call('s2_controller_reset',state)
    assert call('s2_controller_step',state,config,model,out)==1
    close(floats(out+60*4,2),expected)

# New adapter differs from the previous one only by model XY sign (Z preserved).
newq,newgyro=mapped([1,0,0,0],[.3,-.7,.9])
u.mem_write(port+12,struct.pack('<f',-qsb[3]))
oldq,oldgyro=mapped([1,0,0,0],[.3,-.7,.9])
close(newgyro,[-oldgyro[0],-oldgyro[1],oldgyro[2]])
print('PASS: source body accel/gyro mapping (-rawZ,-rawX,rawY), unchanged standard network adapter; linked ARM: forward bend X-, right lean Y+, left turn Z+, obs gyro /4pi and coherent quaternion/gravity tilt. No hardware accessed.')
