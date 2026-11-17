"""Firmware v1: explicit little endian, 160 bytes; no checksum."""
import math
import struct
from dataclasses import dataclass

WIRE = struct.Struct('<5Ii2I31fI')
START, END = 0xAAAAAAAA, 0xEEEEEEEE
GROUPS = [('joint_angle_rad',2),('joint_speed_rad_s',2),('body_quat',4),
          ('body_gyro_rad_s',3),('joint_angle_norm',2),('joint_speed_norm',2),
          ('previous_torque_norm',2),('body_tilt_norm',2),('body_gyro_norm',3),
          ('raw_action',2),('requested_torque_nm',2),('model_torque_nm',2),('driver_torque_nm',2)]
KEYS = ['dt_s'] + [f'{name}_{i}' for name,n in GROUPS for i in range(n)]
STATUS = {0:'正常',1:'暖机',2:'禁用',-1:'输入异常',-2:'配置异常',-3:'数值异常'}

@dataclass
class Frame:
    sequence: int
    timestamp_us: int
    status: int
    valid_flags: int
    compute_time_us: int
    values: dict

    def available(self, key):
        if key == 'dt_s':
            return self.status in (0,1) and self.values[key] > 0
        group, index = key.rsplit('_',1)
        if group.startswith('joint_angle_rad') or group.startswith('joint_speed_rad_s'):
            bit = int(index)
        elif group in ('body_quat','body_gyro_rad_s'):
            bit = 2
        elif group in ('raw_action','requested_torque_nm','model_torque_nm'):
            bit = 4
        elif group == 'driver_torque_nm':
            bit = 5
        else:
            bit = 3
        # monitor_publish(hw=NULL) leaves zeros even if freshness flags survive.
        return self.status in (0,1) and bool(self.valid_flags & (1 << bit)) and math.isfinite(self.values[key])

class Parser:
    def __init__(self):
        self.buffer = bytearray()
        self.discarded = self.bad = 0

    def feed(self, data):
        self.buffer.extend(data)
        frames = []
        while len(self.buffer) >= 4:
            pos = self.buffer.find(b'\xaa'*4)
            if pos < 0:
                count = max(0,len(self.buffer)-3)
                self.discarded += count
                del self.buffer[:count]
                break
            if pos:
                self.discarded += pos
                del self.buffer[:pos]
            if len(self.buffer) < 12:
                break
            _, version, size = struct.unpack_from('<III',self.buffer)
            if version != 1 or size != WIRE.size:
                self.bad += 1
                self.discarded += 1
                del self.buffer[0]
                continue
            if len(self.buffer) < WIRE.size:
                break
            fields = WIRE.unpack_from(self.buffer)
            if fields[-1] != END:
                self.bad += 1
                self.discarded += 1
                del self.buffer[0]
                continue
            frames.append(Frame(*fields[3:8],dict(zip(KEYS,fields[8:-1]))))
            del self.buffer[:WIRE.size]
        return frames

class Timing:
    def __init__(self):
        self.last = None
        self.seconds = 0.0
        self.lost = self.duplicates = self.resets = 0

    def update(self, frame):
        if self.last is not None:
            seq = (frame.sequence-self.last.sequence) & 0xffffffff
            us = (frame.timestamp_us-self.last.timestamp_us) & 0xffffffff
            if seq == 0:
                self.duplicates += 1
            elif seq < 0x80000000 and us < 0x80000000:
                self.lost += seq-1
                self.seconds += us / 1e6
            else:
                self.resets += 1
                self.seconds += 0.02
        self.last = frame
        return self.seconds

CSV_COLUMNS = ['host_time','sequence','timestamp_us','status','valid_flags','compute_time_us'] + KEYS + [k+'_valid' for k in KEYS]

def csv_row(host, f):
    return [host,f.sequence,f.timestamp_us,f.status,f.valid_flags,f.compute_time_us] + [f.values[k] for k in KEYS] + [int(f.available(k)) for k in KEYS]

def simulated_packet(n):
    t = n*.02
    roll,pitch,yaw = .18*math.sin(t),.25*math.sin(.7*t),.4*math.sin(.5*t)
    cr,sr = math.cos(roll/2),math.sin(roll/2)
    cp,sp = math.cos(pitch/2),math.sin(pitch/2)
    cy,sy = math.cos(yaw/2),math.sin(yaw/2)
    quat = [cr*cp*cy+sr*sp*sy,sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy]
    rd,pd,yd = .18*math.cos(t),.175*math.cos(.7*t),.2*math.cos(.5*t)
    gyro = [rd-yd*math.sin(pitch),pd*math.cos(roll)+yd*math.sin(roll)*math.cos(pitch),-pd*math.sin(roll)+yd*math.cos(roll)*math.cos(pitch)]
    values = [.02]
    for name, count in GROUPS:
        values.extend(quat if name == 'body_quat' else gyro if name == 'body_gyro_rad_s' else [math.sin(t+i)*(.2 if 'norm' in name else 1) for i in range(count)])
    phase = n % 500
    status = 1 if phase < 10 else (-1 if 400 <= phase < 420 else 0)
    flags = 15 if status == 1 else (7 if status < 0 else 63)
    if status < 0:
        values = [0.]*31
    return WIRE.pack(START,1,160,n & 0xffffffff,(n*20000)&0xffffffff,status,flags,450,*values,END)
