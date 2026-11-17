"""Mode 2 UART4 telemetry, LE 80 bytes. Does not send control commands."""
import math
import struct
from dataclasses import dataclass

START, END = 0x324D43AA, 0xEE324D43
WIRE = struct.Struct('<5Ii3I10fI')
MAGIC = struct.pack('<I', START)
KEYS = ('left_angle_rad', 'right_angle_rad', 'left_speed_rad_s', 'right_speed_rad_s',
        'left_calculated_torque_nm', 'right_calculated_torque_nm',
        'left_driver_torque_nm', 'right_driver_torque_nm', 'output_scale', 'average_compute_ms')
STATUS = {0:'正常', 1:'初始化', 2:'禁用', 3:'振荡抑制', -1:'反馈异常/置零', -2:'周期超时', -3:'数值异常'}
CSV_COLUMNS = ('host_time', 'device_time_s', 'session', 'sequence', 'timestamp_us',
               'status', 'flags', 'compute_us', 'period_us', *KEYS, 'control_mode', 'amplitude_nm', 'sine_period_s')

@dataclass
class Frame:
    sequence: int
    timestamp_us: int
    status: int
    flags: int
    compute_us: int
    period_us: int
    values: tuple
    time_s: float = 0.0
    session: int = 0
    version: int = 1

    @property
    def mode(self):
        return 3 if self.version == 3 else 2

    def plot_values(self):
        return tuple(v if self.flags & (1 if i < 4 else 2) else math.nan
                     for i, v in enumerate(self.values[:6]))

def csv_row(host_time, frame):
    return (host_time, frame.time_s, frame.session, frame.sequence, frame.timestamp_us,
            frame.status, frame.flags, frame.compute_us, frame.period_us, *frame.values,
            frame.mode, frame.values[8] if frame.mode == 3 else 0.0,
            frame.values[9] if frame.mode == 3 else 0.0)

class Parser:
    def __init__(self):
        self.buffer = bytearray()
        self.bad = self.discarded = self.count = self.lost = self.restarts = 0
        self.last = None
        self.elapsed_us = 0

    def feed(self, data):
        self.buffer.extend(data)
        result = []
        while len(self.buffer) >= 4:
            pos = self.buffer.find(MAGIC)
            if pos < 0:
                n = len(self.buffer)-3
                self.discarded += n
                del self.buffer[:n]
                break
            self.discarded += pos
            del self.buffer[:pos]
            if len(self.buffer) < WIRE.size:
                break
            f = WIRE.unpack_from(self.buffer)
            if (f[1] not in (1, 3) or f[2] != WIRE.size) or f[-1] != END or f[6] & ~63 or not all(math.isfinite(v) for v in f[9:-1]):
                self.bad += 1
                self.discarded += 1
                del self.buffer[0]
                continue
            frame = Frame(*f[3:9], tuple(f[9:-1]), version=f[1])
            if self.last is not None:
                seq_delta = (frame.sequence-self.last.sequence) & 0xFFFFFFFF
                dt = (frame.timestamp_us-self.last.timestamp_us) & 0xFFFFFFFF
                if seq_delta == 0 or seq_delta >= 0x80000000 or dt >= 0x80000000:
                    self.restarts += 1
                    self.elapsed_us += 10000
                else:
                    self.lost += seq_delta-1
                    self.elapsed_us += dt
            frame.time_s = self.elapsed_us*1e-6
            frame.session = self.restarts
            self.last = frame
            self.count += 1
            result.append(frame)
            del self.buffer[:WIRE.size]
        return result

def simulated_packet(n):
    t = n*.01
    q, v = .4*math.sin(2*math.pi*t), .8*math.pi*math.cos(2*math.pi*t)
    torque = 8*math.sin(2*math.pi*t)
    permitted = int(t % 10 >= 3)
    flags = 3 | (12 if permitted else 0)
    values = (q, -q, v, -v, torque, -torque,
              torque if permitted else 0, torque if permitted else 0, 1.2, 2.0)
    return WIRE.pack(START, 1, WIRE.size, n & 0xFFFFFFFF, (n*10000) & 0xFFFFFFFF,
                     0, flags, 2000, 10000, *values, END)
