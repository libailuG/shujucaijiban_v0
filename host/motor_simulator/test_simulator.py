import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import unittest
from unittest.mock import patch
import time
from protocol import FRAME, Parser, Settings, pack, respond
from worker import Worker

class ProtocolTests(unittest.TestCase):
    def test_layout(self):
        self.assertEqual(FRAME.size, 48)
        frame = pack(126)
        self.assertEqual(frame, bytes.fromhex('aaaaaaaa7e000000') + bytes(36) + bytes.fromhex('eeeeeeee'))
        reply = respond(FRAME.unpack(frame), Settings())
        self.assertEqual(reply[:4], b'\xbb'*4)
        self.assertEqual(reply[-4:], b'\xdd'*4)
        self.assertEqual(FRAME.unpack(reply)[1:3], (126, 0))

    def test_stream(self):
        frame = pack(126)
        parser = Parser()
        self.assertEqual(parser.feed(b'noise'+frame[:9]), [])
        self.assertEqual(len(parser.feed(frame[9:]+frame+frame[:47])), 2)
        self.assertEqual(len(parser.feed(frame[47:])), 1)
        corrupt = frame[:-4]+b'bad!'
        self.assertEqual(len(parser.feed(corrupt+frame)), 1)
        self.assertGreater(parser.bad_tail, 0)
        parser.feed(b'x'*100000)
        self.assertLessEqual(len(parser.buffer), 3)
        parser = Parser()
        result = []
        for byte in frame*100: result += parser.feed(bytes([byte]))
        self.assertEqual(len(result), 100)

    def test_modes(self):
        cfg = Settings(magnet=123.5, current=[1,2,3,4])
        self.assertIsNone(respond(FRAME.unpack(pack(127)), cfg))
        self.assertEqual(FRAME.unpack(respond(FRAME.unpack(pack(126,1004)), cfg))[3],123.5)
        self.assertEqual(FRAME.unpack(respond(FRAME.unpack(pack(126,1005)), cfg))[3:7],(1.,2.,3.,4.))
        for mode in (0,1,11,2,1000,1001,1002,1003,1006,9999):
            reply = FRAME.unpack(respond(FRAME.unpack(pack(126,mode)), cfg))
            self.assertEqual(reply[2], mode)
            self.assertEqual(cfg.motor_id, 126)
            self.assertEqual(cfg.current,[1,2,3,4])

class FakeSerial:
    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.input = bytearray(pack(126)*200+pack(127))
        self.output = bytearray()
        self.closed = False
    def __enter__(self): return self
    def __exit__(self, *args): self.closed = True
    @property
    def in_waiting(self): return len(self.input)
    def read(self, size):
        if not self.input: time.sleep(.002)
        size = min(size, 17)
        data = bytes(self.input[:size]); del self.input[:size]
        return data
    def write(self, data):
        count = min(7,len(data))
        self.output.extend(data[:count])
        return count

class IntegrationTests(unittest.TestCase):
    def test_worker(self):
        device = FakeSerial()
        worker = Worker('FAKE',2000000,Settings(),factory=lambda **kw: device)
        worker.start()
        deadline = time.monotonic()+3
        while time.monotonic()<deadline:
            if worker.snapshot()[0]['rx']==201: break
            time.sleep(.005)
        worker.stop()
        self.assertTrue(worker.wait(1000))
        stats = worker.snapshot()[0]
        self.assertEqual((stats['rx'],stats['tx'],stats['mismatch'],stats['errors']),(201,200,1,0))
        self.assertEqual(len(device.output),200*48)
        self.assertTrue(device.closed)

    def test_error(self):
        def fail(**kw): raise OSError('offline injected failure')
        worker = Worker('FAKE',2000000,Settings(),factory=fail)
        worker.start(); self.assertTrue(worker.wait(1000))
        self.assertEqual(worker.snapshot()[0]['errors'],1)

    def test_gui_offline(self):
        from PyQt6 import QtWidgets
        from main import Window
        app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        with patch('serial.Serial', side_effect=AssertionError('must not open')):
            window = Window()
            window.show(); app.processEvents()
            self.assertTrue(all(c.worker is None for c in window.channels))
            for c in window.channels: c.offline()
            for i in range(1100): window.log(str(i))
            self.assertLessEqual(window.logs.document().blockCount(),1000)
            window.grab().save(os.path.join(os.path.dirname(__file__),'offline_preview.png'))
            window.close(); app.processEvents()

if __name__ == '__main__': unittest.main(verbosity=2)
