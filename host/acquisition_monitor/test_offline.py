import csv
import tempfile
import unittest
from pathlib import Path
import numpy as np
from PyQt6 import QtWidgets,QtTest
from protocol import simulated_packet,KEYS
from log_data import load_log
from main import Window,create_application

class LogTests(unittest.TestCase):
    def test_actual_log_and_round_trip(self):
        source=Path(__file__).resolve().parents[2]/'caiji_data/LOG00126.TXT'
        if not source.exists():self.skipTest('本地样本不存在')
        raw=source.read_bytes();d=load_log(source)
        self.assertEqual(len(d.time_s),1561)
        self.assertEqual(d.values.shape,(1561,17))
        self.assertAlmostEqual(d.time_s[-1],15.614034)
        self.assertEqual(int(d.timestamp_us[0]),19741880)
        self.assertEqual(d.bad,0)
        self.assertEqual(d.discarded+d.trailing,11)
        with tempfile.TemporaryDirectory() as temp:
            csv_path=Path(temp)/'log.csv';npz_path=Path(temp)/'log.npz'
            d.export_csv(csv_path);d.export_npz(npz_path)
            with open(csv_path,encoding='utf-8-sig') as f: rows=list(csv.DictReader(f))
            self.assertEqual(len(rows),1561)
            self.assertEqual(int(rows[0]['timestamp_us']),19741880)
            self.assertNotIn('host_unix_s',rows[0])
            self.assertEqual(rows[0]['left_pressure'],'1')
            with np.load(npz_path,allow_pickle=False) as z:
                np.testing.assert_array_equal(z['values'],d.values)
                np.testing.assert_array_equal(z['timestamp_us'],d.timestamp_us)
                self.assertEqual(z['channel_names'].tolist(),KEYS)
                self.assertEqual(z['left_pressure'].dtype,np.uint32)
                self.assertEqual(z['left_thigh_angle'].dtype,np.float32)
                self.assertEqual(z['units'][0],'rad')
                np.testing.assert_array_equal(z['right_shank_angle'],d.values[:,14])
        with self.assertRaises(ValueError):d.export_csv(source)
        with self.assertRaises(ValueError):d.export_npz(source)
        self.assertEqual(raw,source.read_bytes())

    def test_bad_empty_wrap_restart(self):
        import struct
        with tempfile.TemporaryDirectory() as temp:
            p=Path(temp)/'test.txt'
            p.write_bytes(b'not a log')
            with self.assertRaises(ValueError):load_log(p)
            blocks=[]
            for us in [0xfffffff0,0x2700,10]:
                b=bytearray(simulated_packet(0));struct.pack_into('<I',b,4,us);blocks.append(b)
            p.write_bytes(b'\xff'+b''.join(blocks)+b'\xff'*10)
            d=load_log(p)
            np.testing.assert_allclose(d.time_s,[0,.01,.02])
            np.testing.assert_array_equal(d.segment_id,[0,0,1])
            with self.assertRaises(InterruptedError):load_log(p,cancelled=lambda:True)

class OfflineUiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.app=QtWidgets.QApplication.instance() or create_application()
    def wait_job(self,panel):
        for _ in range(300):
            QtTest.QTest.qWait(10)
            if panel.job is None:return
        self.fail('文件任务超时')
    def test_load_plot_export_and_error_preserves_data(self):
        w=Window();w.show()
        try:
            self.assertEqual(w.tabs.tabText(1),'离线文件')
            panel=w.offline
            with tempfile.TemporaryDirectory() as temp:
                p=Path(temp)/'log.txt';p.write_bytes(b''.join(simulated_packet(i) for i in range(200)))
                self.assertTrue(panel.load_file(p));self.wait_job(panel)
                self.assertEqual(len(panel.dataset.time_s),200)
                self.assertIsNone(w.receiver)
                self.assertTrue(panel.csv_button.isEnabled())
                key='left_motor_angle';panel.checks[key].setChecked(False)
                self.assertFalse(panel.curves[key].isVisible());panel.checks[key].setChecked(True)
                self.assertEqual(sum(s.item is panel.curves[key] for s,_ in panel.legend.items),1)
                x,y=panel.curves[key].getData();self.assertEqual(len(x),200)
                panel.update_readouts(99);self.assertIn('100 / 200',panel.cursor.text())
                panel.export_file(Path(temp)/'out.csv','csv');self.wait_job(panel)
                panel.export_file(Path(temp)/'out.npz','npz');self.wait_job(panel)
                self.assertTrue((Path(temp)/'out.csv').exists())
                with np.load(Path(temp)/'out.npz',allow_pickle=False) as z:self.assertEqual(z['values'].shape,(200,17))
                old=panel.dataset;p.write_bytes(b'bad file');panel.load_file(p);self.wait_job(panel)
                self.assertIs(panel.dataset,old);self.assertIn('失败',panel.info.text())
        finally:w.close()

if __name__=='__main__':unittest.main()

