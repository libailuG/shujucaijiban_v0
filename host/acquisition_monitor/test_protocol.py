import math
import struct
import unittest
from protocol import Parser,Timing,WIRE,KEYS,START,END,simulated_packet,csv_row,CSV_COLUMNS
class ProtocolTests(unittest.TestCase):
    def test_layout(self):
        data=simulated_packet(123)
        self.assertEqual(len(data),80)
        self.assertEqual(struct.unpack_from('<I',data,4)[0],1230000)
        frame=Parser().feed(data)[0]
        self.assertEqual(len(frame.values),17)
        self.assertIsInstance(frame.values['left_pressure'],int)
        self.assertEqual(frame.values['left_pressure'],2)
        self.assertEqual(frame.values['right_pressure'],0)
        self.assertEqual(len(csv_row(1,frame)),len(CSV_COLUMNS))
    def test_every_split_noise_and_glue(self):
        data=simulated_packet(7)
        for i in range(len(data)+1):
            p=Parser()
            out=p.feed(data[:i])+p.feed(data[i:])
            self.assertEqual(len(out),1)
            self.assertEqual(out[0].timestamp_us,70000)
        p=Parser(); out=[]
        for b in b'noise'+data+data: out.extend(p.feed(bytes([b])))
        self.assertEqual(len(out),2)
        self.assertEqual(p.discarded,5)
    def test_bad_tail_flags_and_payload_markers(self):
        bad=bytearray(simulated_packet(1)); bad[-1]=0
        flags=bytearray(simulated_packet(2)); struct.pack_into('<I',flags,52,8)
        p=Parser(); out=p.feed(bad+flags+simulated_packet(3))
        self.assertEqual(len(out),1); self.assertGreaterEqual(p.bad,2)
        data=bytearray(simulated_packet(4)); data[8:12]=b'\xaa'*4; data[12:16]=b'\xee'*4
        f=Parser().feed(data)
        self.assertEqual(len(f),1)
        data=bytearray(simulated_packet(0)); struct.pack_into('<f',data,8,float('nan'))
        f=Parser().feed(data)[0]; self.assertFalse(f.available(KEYS[0]))
    def test_wrap_and_restart(self):
        p=Parser(); t=Timing()
        def frame(us):
            b=bytearray(simulated_packet(0)); struct.pack_into('<I',b,4,us); return p.feed(b)[0]
        self.assertEqual(t.update(frame(0xfffffff0)),0)
        self.assertAlmostEqual(t.update(frame(0x2700)),.01)
        self.assertAlmostEqual(t.update(frame(100)),.02)
        self.assertEqual(t.resets,1)
    def test_matches_firmware_values(self):
        values=list(range(1,12))+[2]+list(range(12,16))+[3]
        f=Parser().feed(WIRE.pack(START,10000,*values,END))[0]
        self.assertEqual(f.values['left_thigh_angle'],8)
        self.assertEqual(f.values['right_thigh_angle'],12)
        self.assertEqual(f.values['right_pressure'],3)
if __name__=='__main__': unittest.main()
