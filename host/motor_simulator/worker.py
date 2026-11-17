"""Serial I/O confined to a worker; bounded polling snapshots avoid GUI event floods."""
from collections import deque
from threading import Event, Lock
from copy import deepcopy
from PyQt6 import QtCore
import serial
from protocol import Parser, respond, MODES

class Worker(QtCore.QThread):
    def __init__(self, port, baud, settings, factory=serial.Serial):
        super().__init__()
        self.port, self.baud, self.settings = port, baud, settings
        self.factory = factory
        self.stop_event = Event()
        self.lock = Lock()
        self.events = deque(maxlen=200)
        self.stats = dict(rx=0, tx=0, mismatch=0, errors=0, discarded=0, bad_tail=0)
        self.status = '正在打开'
        self.latest = '尚无命令'

    def configure(self, settings):
        with self.lock:
            self.settings = deepcopy(settings)

    def snapshot(self):
        with self.lock:
            result = (dict(self.stats), self.status, self.latest, list(self.events))
            self.events.clear()
            return result

    def stop(self):
        self.stop_event.set()

    def run(self):
        parser = Parser()
        try:
            with self.factory(port=self.port, baudrate=self.baud, bytesize=8, parity='N',
                              stopbits=1, timeout=0.02, write_timeout=0.2,
                              xonxoff=False, rtscts=False, dsrdtr=False) as device:
                with self.lock:
                    self.status = '已打开：仅应答'
                while not self.stop_event.is_set():
                    data = device.read(min(max(device.in_waiting, 1), 4096))
                    for command in parser.feed(data):
                        if self.stop_event.is_set():
                            break
                        with self.lock:
                            cfg = deepcopy(self.settings)
                            self.stats['rx'] += 1
                            self.latest = repr(command[1:-1])
                        reply = respond(command, cfg)
                        if reply is None:
                            with self.lock:
                                self.stats['mismatch'] += 1
                            continue
                        # Count a transmission only after the entire frame is accepted.
                        offset = 0
                        while offset < len(reply) and not self.stop_event.is_set():
                            count = device.write(reply[offset:])
                            if not count:
                                raise serial.SerialTimeoutException('写入无进展')
                            offset += count
                        if offset == len(reply):
                            with self.lock:
                                self.stats['tx'] += 1
                                self.events.append(f'RX ID={command[1]} mode={command[2]} '
                                                   f'{MODES.get(command[2], "未知模式：固定反馈")} → TX {reply.hex(" ")}')
                    with self.lock:
                        self.stats['discarded'] = parser.discarded
                        self.stats['bad_tail'] = parser.bad_tail
        except Exception as exc:
            with self.lock:
                self.stats['errors'] += 1
                self.events.append(f'串口错误：{exc}')
        finally:
            with self.lock:
                self.status = '已关闭'
