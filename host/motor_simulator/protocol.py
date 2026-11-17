"""48-byte motor protocol only; no serial access in this module."""
from dataclasses import dataclass, field
import struct

FRAME = struct.Struct('<III7fII')
NAMES = ('torque', 'speed', 'ang', 'temp_board', 'temp_motor', 'KP', 'KD')
MODES = {0:'停止', 1:'位置/扭矩（固定反馈）', 11:'速度（固定反馈）', 2:'置零（仅观察）',
         1000:'FOC（仅观察）', 1001:'改ID（仅观察，不改ID）', 1002:'X磁校准（仅观察）',
         1003:'Y磁校准（仅观察）', 1004:'读第二磁编位置', 1005:'读电流参数', 1006:'写参数（仅观察）'}

def pack(motor_id, mode=0, values=None, error=0, feedback=False):
    return FRAME.pack(0xBBBBBBBB if feedback else 0xAAAAAAAA, motor_id, mode,
                      *(values or [0.0]*7), error, 0xDDDDDDDD if feedback else 0xEEEEEEEE)

class Parser:
    def __init__(self):
        self.buffer = bytearray()
        self.discarded = 0
        self.bad_tail = 0

    def feed(self, data):
        self.buffer.extend(data)
        frames = []
        while True:
            pos = self.buffer.find(b'\xaa'*4)
            if pos < 0:
                count = max(0, len(self.buffer)-3)
                self.discarded += count
                del self.buffer[:count]
                break
            self.discarded += pos
            del self.buffer[:pos]
            if len(self.buffer) < 48:
                break
            if self.buffer[44:48] != b'\xee'*4:
                self.bad_tail += 1
                self.discarded += 1
                del self.buffer[0]
                continue
            frames.append(FRAME.unpack_from(self.buffer))
            del self.buffer[:48]
        return frames

@dataclass
class Settings:
    motor_id: int = 126
    values: list = field(default_factory=lambda: [0., 0., 0., 25., 25., 0., 0.])
    error: int = 0
    magnet: float = 0.
    current: list = field(default_factory=lambda: [0., 0., 0., 0.])


def respond(command, settings):
    """One matched request -> one fixed status; no state-changing commands."""
    if command[1] != settings.motor_id:
        return None
    values = list(settings.values)
    if command[2] == 1004:
        values[0] = settings.magnet
    elif command[2] == 1005:
        values[:4] = settings.current
    return pack(settings.motor_id, command[2], values, settings.error, feedback=True)
