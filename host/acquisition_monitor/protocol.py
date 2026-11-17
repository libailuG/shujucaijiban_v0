"""Mode20 UART1: 80-byte little-endian frames; uint32 device microseconds; no CRC, sequence or validity."""
import math
import struct
import time
from dataclasses import dataclass
WIRE = struct.Struct('<2I11fI4fII')
START, END = 0xAAAAAAAA, 0xEEEEEEEE
KEYS = ['left_motor_angle','left_motor_speed','right_motor_angle','right_motor_speed',
        'body_gyro_x','body_gyro_y','body_accel_z','left_thigh_angle','left_thigh_speed',
        'left_shank_angle','left_shank_speed','left_pressure','right_thigh_angle',
        'right_thigh_speed','right_shank_angle','right_shank_speed','right_pressure']
LABELS = dict(zip(KEYS, [
    '左电机角度 rad','左电机角速度 rad/s','右电机角度 rad','右电机角速度 rad/s',
    '机身X角速度 rad/s','机身Y角速度 rad/s','机身Z加速度 m/s²',
    '左大腿X角度 rad','左大腿X角速度 rad/s','左小腿X角度 rad','左小腿X角速度 rad/s',
    '左压力标志 0–3','右大腿X角度 rad','右大腿X角速度 rad/s',
    '右小腿X角度 rad','右小腿X角速度 rad/s','右压力标志 0–3']))
CATEGORIES = {'电机':KEYS[:4], '机身':KEYS[4:7], '左腿 / 压力':KEYS[7:12], '右腿 / 压力':KEYS[12:]}

@dataclass
class Frame:
    receive_index: int  # Local parser count, never a device sequence.
    timestamp_us: int
    receive_monotonic: float
    values: dict
    def available(self,key):
        return math.isfinite(self.values[key])  # Numeric sanity only, NOT sensor validity.

class Parser:
    def __init__(self):
        self.buffer=bytearray()
        self.discarded=self.bad=self.count=0
    def feed(self,data):
        self.buffer.extend(data)
        result=[]
        while len(self.buffer)>=4:
            pos=self.buffer.find(b'\xaa'*4)
            if pos<0:
                n=max(0,len(self.buffer)-3)
                self.discarded+=n
                del self.buffer[:n]
                break
            if pos:
                self.discarded+=pos
                del self.buffer[:pos]
            if len(self.buffer)<WIRE.size: break
            fields=WIRE.unpack_from(self.buffer)
            if fields[-1]!=END or fields[13]>3 or fields[18]>3:
                self.bad+=1
                self.discarded+=1
                del self.buffer[0]
                continue
            self.count+=1
            result.append(Frame(self.count,fields[1],time.monotonic(),dict(zip(KEYS,fields[2:-1]))))
            del self.buffer[:WIRE.size]
        return result

class Timing:
    def __init__(self):
        self.last=None
        self.seconds=0.0
        self.resets=0
    def update(self,frame):
        if self.last is not None:
            delta=(frame.timestamp_us-self.last)&0xffffffff
            if delta<0x80000000:
                self.seconds+=delta/1e6
            else:
                # Backward/restart or ambiguous gap >35.8min: new local segment.
                self.resets+=1
                self.seconds+=.01
        self.last=frame.timestamp_us
        return self.seconds
CSV_COLUMNS=['host_unix_s','host_monotonic_s','host_receive_index','device_timestamp_us']+KEYS

def csv_row(host,frame):
    return [host,frame.receive_monotonic,frame.receive_index,frame.timestamp_us]+[frame.values[k] for k in KEYS]

def simulated_packet(n):
    t=n*.01
    vals=[.3*math.sin(t),.3*math.cos(t),.3*math.sin(t+.4),.3*math.cos(t+.4),
          .2*math.sin(t),.1*math.cos(t),9.81+.2*math.sin(2*t),
          .5*math.sin(t),.5*math.cos(t),.6*math.sin(t+.3),.6*math.cos(t+.3),
          (n//50)%4,.5*math.sin(t+.6),.5*math.cos(t+.6),
          .6*math.sin(t+.9),.6*math.cos(t+.9),(n//50+2)%4]
    return WIRE.pack(START,(n*10000)&0xffffffff,*vals,END)

