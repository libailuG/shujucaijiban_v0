import math
from pathlib import Path
import struct
import unittest
from protocol import Parser, WIRE, START, END, simulated_packet, csv_row, CSV_COLUMNS

class ProtocolTests(unittest.TestCase):
    def test_fragmentation_noise_and_resync(self):
        parser = Parser()
        broken = bytearray(simulated_packet(0)); broken[-1] ^= 1
        stream = b'noise'+broken+simulated_packet(1)+simulated_packet(2)
        frames = []
        for byte in stream:
            frames.extend(parser.feed(bytes([byte])))
        self.assertEqual([f.sequence for f in frames], [1, 2])
        self.assertEqual(parser.bad, 1)
        self.assertEqual(parser.discarded, 85)
        self.assertAlmostEqual(frames[1].time_s, .01)

    def test_validity_and_gate_are_separate(self):
        f = Parser().feed(simulated_packet(50))[0]
        self.assertTrue(f.flags & 2)
        self.assertFalse(f.flags & 4)
        self.assertNotEqual(f.values[4], 0)
        self.assertEqual(f.values[6:8], (0, 0))
        self.assertTrue(all(math.isfinite(x) for x in f.plot_values()))
        f.flags = 0
        self.assertTrue(all(math.isnan(x) for x in f.plot_values()))
        self.assertEqual(len(csv_row(None, f)), len(CSV_COLUMNS))

    def test_gaps_wrap_and_restart(self):
        def packet(seq, stamp):
            fields = list(WIRE.unpack(simulated_packet(0)))
            fields[3:5] = [seq, stamp]
            return WIRE.pack(*fields)
        parser = Parser()
        frames = parser.feed(packet(0xFFFFFFFF, 0xFFFFF000)+packet(0, 5904)+packet(3, 35904))
        self.assertAlmostEqual(frames[1].time_s, .01)
        self.assertAlmostEqual(frames[2].time_s, .04)
        self.assertEqual(parser.lost, 2)
        self.assertEqual(parser.restarts, 0)
        self.assertEqual(parser.feed(packet(0, 0))[0].session, 1)

    def test_reject_invalid_frames(self):
        frames = []
        for index, value in ((1, 2), (2, 79), (6, 64), (9, math.nan), (18, math.inf)):
            fields = list(WIRE.unpack(simulated_packet(0))); fields[index] = value
            frames.append(WIRE.pack(*fields))
        parser = Parser()
        good = parser.feed(b''.join(frames)+simulated_packet(1))
        self.assertEqual(len(good), 1)
        self.assertEqual(parser.bad, 5)

    def test_mode3_firmware_layout(self):
        raw = (Path(__file__).resolve().parents[2]/'work/control3_20261117/telemetry_fixture.bin').read_bytes()
        f = Parser().feed(raw)[0]
        self.assertEqual(f.mode, 3)
        self.assertEqual(f.flags & 15, 15)
        self.assertEqual(f.values[4:8], (4, 0, 4, 0))
        self.assertEqual(f.values[8:], (4, 1))
        self.assertEqual(csv_row(None, f)[-3:], (3, 4, 1))

    def test_actual_firmware_layout(self):
        fixture = Path(__file__).resolve().parents[2]/'work/control2_v10_20261117/telemetry_fixture.bin'
        self.assertTrue(fixture.exists(), 'Run firmware host test first')
        raw = fixture.read_bytes()
        self.assertEqual(len(raw), 80)
        self.assertEqual(struct.unpack_from('<I', raw)[0], START)
        self.assertEqual(struct.unpack_from('<I', raw, 76)[0], END)
        f = Parser().feed(raw)[0]
        self.assertEqual(f.flags & 15, 3)
        self.assertEqual(f.values[6:8], (0, 0))
        self.assertAlmostEqual(f.values[8], 1.4, places=5)
        self.assertAlmostEqual(f.values[0], -f.values[1], places=5)

if __name__ == '__main__':
    unittest.main()
