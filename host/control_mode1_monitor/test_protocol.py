import math
import struct
import unittest
from protocol import Parser, Timing, WIRE, KEYS, simulated_packet

class ProtocolTests(unittest.TestCase):
    def test_layout(self):
        self.assertEqual(WIRE.size,160)
        self.assertEqual(len(KEYS),31)
        f = Parser().feed(simulated_packet(20))[0]
        self.assertEqual(f.sequence,20)
        self.assertEqual(f.valid_flags,63)
        self.assertAlmostEqual(f.values['dt_s'],.02)
        self.assertAlmostEqual(sum(f.values[f'body_quat_{i}']**2 for i in range(4)),1,places=6)
        self.assertAlmostEqual(f.values['driver_torque_nm_1'],math.sin(1.4))

    def test_every_split(self):
        packet = simulated_packet(20)
        for split in range(1,160):
            p = Parser()
            self.assertEqual(p.feed(packet[:split]),[])
            self.assertEqual(len(p.feed(packet[split:])),1)
        p = Parser()
        out = []
        for byte in packet:
            out.extend(p.feed(bytes([byte])))
        self.assertEqual(len(out),1)

    def test_noise_corruption_and_concatenation(self):
        valid = simulated_packet(25)
        for offset,value in ((4,2),(8,159),(156,0)):
            bad = bytearray(valid)
            struct.pack_into('<I',bad,offset,value)
            p = Parser()
            frames = p.feed(b'noise\xaa\xaa'+bad+valid+valid)
            self.assertEqual([f.sequence for f in frames],[25,25])
            self.assertGreater(p.bad,0)
            self.assertLess(len(p.buffer),160)
        p = Parser()
        self.assertEqual(p.feed(b'x'*100000),[])
        self.assertLessEqual(len(p.buffer),3)

    def test_payload_markers(self):
        b = bytearray(simulated_packet(20))
        b[40:44] = b'\xee'*4
        b[44:48] = b'\xaa'*4
        self.assertEqual(len(Parser().feed(b)),1)

    def test_rollover_loss_duplicate_reset(self):
        p = Parser()
        def f(seq,stamp):
            b = bytearray(simulated_packet(20))
            struct.pack_into('<II',b,12,seq,stamp)
            return p.feed(b)[0]
        t = Timing()
        t.update(f(0xffffffff,0xfffffff0))
        self.assertAlmostEqual(t.update(f(0,19984)),.02)
        t.update(f(3,79984))
        self.assertEqual(t.lost,2)
        t.update(f(3,79984))
        self.assertEqual(t.duplicates,1)
        t.update(f(1,20000))
        self.assertEqual(t.resets,1)

    def test_validity(self):
        p = Parser()
        warm = p.feed(simulated_packet(0))[0]
        self.assertTrue(warm.available('joint_angle_norm_0'))
        self.assertFalse(warm.available('raw_action_0'))
        bad = p.feed(simulated_packet(405))[0]
        self.assertEqual(bad.valid_flags,7)
        self.assertFalse(any(bad.available(k) for k in KEYS))
        good = p.feed(simulated_packet(20))[0]
        good.values['raw_action_0'] = float('nan')
        self.assertFalse(good.available('raw_action_0'))

if __name__ == '__main__':
    unittest.main()
