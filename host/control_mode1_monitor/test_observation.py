import csv
import tempfile
import unittest
from pathlib import Path
from PyQt6 import QtWidgets, QtTest
import observation as obs
import numpy as np
from main import Window, create_application

class ObservationTests(unittest.TestCase):
    def test_binary_file_and_exports(self):
        app=QtWidgets.QApplication.instance() or create_application()
        source=Path(__file__).resolve().parents[2]/'caiji_kongzhi1_data/LOG00128.TXT'
        with tempfile.TemporaryDirectory() as tmp:
            w=Window()
            try:
                w.show();w.open_observation_file(str(source))
                self.assertEqual(len(w.observation.history),4026)
                self.assertIn('4字节',w.observation.file_status.text())
                self.assertFalse(w.observation.follow.isChecked())
                self.assertEqual(len(w.observation.curves[0].xData),4026)
                self.assertLessEqual(w.observation.plot.viewRange()[0][0],0)
                self.assertGreaterEqual(w.observation.plot.viewRange()[0][1],80.5)
                before=w.state.text();w.update_data();self.assertEqual(w.state.text(),before)
                for kind in ['csv','npz']:w.save_observation_file(kind,str(Path(tmp)/('saved.'+kind)))
                with open(Path(tmp)/'saved.csv',encoding='utf-8-sig') as f:rows=list(csv.reader(f))
                self.assertEqual(len(rows),4027)
                self.assertEqual(rows[0],['time_s','time_basis','host_time',*obs.KEYS])
                self.assertEqual(rows[1][1:3],['nominal_50hz',''])
                with np.load(Path(tmp)/'saved.npz',allow_pickle=False) as z:
                    self.assertEqual(z['values'].shape,(4026,11))
                    self.assertEqual(z['keys'].tolist(),obs.KEYS)
                    self.assertAlmostEqual(z['time_s'][-1],80.5)
                    self.assertEqual(z['time_basis'].item(),'nominal_50hz')
                    np.testing.assert_allclose(z['values'][0],w.observation.history[0][2].values)
                # Invalid input keeps the previous successful load intact.
                invalid=Path(tmp)/'bad.txt';invalid.write_bytes(b'not a binary log')
                w.open_observation_file(str(invalid))
                self.assertIn('读取失败',w.state.text())
                self.assertEqual(len(w.observation.history),4026)
                # Import must retain all frames beyond the live 6000-frame buffer.
                longfile=Path(tmp)/'long.bin';longfile.write_bytes(obs.simulated_packet(1)*6100)
                w.open_observation_file(str(longfile));self.assertEqual(len(w.observation.history),6100)
                w.clear();self.assertIsNone(w.observation.file_source)
            finally:w.close()

    def test_stream_and_fields(self):
        values=tuple(i+.25 for i in range(11))
        packet=obs.WIRE.pack(0xAAAAAAAA,*values,0xEEEEEEEE)
        self.assertEqual(len(packet),52)
        p=obs.Parser();frames=[]
        stream=b'noise'+packet+packet
        for b in stream:frames.extend(p.feed(bytes([b])))
        self.assertEqual([f.values for f in frames],[values,values])
        self.assertEqual(p.discarded,5)
        self.assertEqual(obs.csv_row(12,frames[0]),[12,*values])

    def test_recovery(self):
        good=obs.simulated_packet(12)
        bad=good[:-4]+b'bad!'
        nonfinite=obs.WIRE.pack(0xAAAAAAAA,*([float('nan')]*11),0xEEEEEEEE)
        p=obs.Parser()
        self.assertEqual(len(p.feed(bad+nonfinite+good)),1)
        self.assertGreaterEqual(p.bad,2)

    def test_ui_record_export(self):
        app=QtWidgets.QApplication.instance() or create_application()
        with tempfile.TemporaryDirectory() as tmp:
            w=Window()
            try:
                w.show();self.assertEqual(w.tabs.currentIndex(),2)
                self.assertEqual(len(w.observation.checks),11)
                w.sim.setChecked(True);w.connect_source()
                self.assertTrue(w.receiver.observation)
                self.assertFalse(w.protocol_choice.isEnabled())
                record=str(Path(tmp)/'record.csv')
                w.receiver.commands.put(record)
                QtTest.QTest.qWait(450)
                self.assertGreater(len(w.observation.history),10)
                self.assertTrue(all(c.text()!='不可用' for c in w.observation.readouts))
                w.observation.all.click()
                self.assertTrue(all(not c.isChecked() for c in w.observation.checks))
                w.observation.all.click()
                w.toggle_pause();count=w.receiver.count
                QtTest.QTest.qWait(100);self.assertGreater(w.receiver.count,count)
                w.toggle_pause();w.update_data()
                output=str(Path(tmp)/'export.csv');w.export(output)
                preview=Path(__file__).with_name('observation_preview.png')
                w.grab().save(str(preview))
                w.disconnect_source()
                for path in (record,output):
                    with open(path,encoding='utf-8-sig') as f:rows=list(csv.reader(f))
                    self.assertEqual(rows[0],obs.CSV_COLUMNS)
                    self.assertGreater(len(rows),5)
                    self.assertTrue(all(len(row)==12 for row in rows))
                self.assertTrue(all(c.text()=='数据过期' for c in w.observation.readouts))
                w.clear();self.assertFalse(w.observation.history)
            finally:w.close()

if __name__=='__main__':unittest.main()
