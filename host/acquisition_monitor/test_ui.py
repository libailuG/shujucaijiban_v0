import csv
import tempfile
import unittest
from pathlib import Path
from PyQt6 import QtWidgets,QtTest
from main import Window,create_application
class UiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls): cls.app=QtWidgets.QApplication.instance() or create_application()
    def test_record_pause_export_disconnect(self):
        with tempfile.TemporaryDirectory() as temp:
            w=Window(); w.show()
            try:
                self.assertEqual(len(w.checks),17)
                self.assertEqual(w.groups.count(),4)
                self.assertEqual(sum(c.isChecked() for c in w.checks.values()),6)
                w.span.setValue(150); self.assertGreaterEqual(w.history.maxlen,15000)
                w.sim.setChecked(True); w.connect_source()
                path=str(Path(temp)/'record.csv'); w.receiver.commands.put(path)
                QtTest.QTest.qWait(450)
                self.assertGreater(len(w.history),15)
                w.toggle_pause(); value=w.readouts['left_motor_angle'].text()
                count=w.receiver.count; QtTest.QTest.qWait(200)
                self.assertGreater(w.receiver.count,count)
                self.assertEqual(w.readouts['left_motor_angle'].text(),value)
                w.clear(); self.assertFalse(w.history)
                QtTest.QTest.qWait(150); w.toggle_pause(); w.update_data()
                export=str(Path(temp)/'cache.csv'); w.export(export)
                self.assertTrue(w.disconnect_source())
                w.update_data(); self.assertEqual(w.readouts['left_motor_angle'].text(),'不可用')
                with open(path,encoding='utf-8-sig') as f: rows=list(csv.DictReader(f))
                self.assertGreater(len(rows),30)
                self.assertIn('device_timestamp_us',rows[0])
                self.assertIn('right_pressure',rows[0])
                self.assertEqual(len(rows[0]),21)
                self.assertGreater(Path(export).stat().st_size,100)
            finally: w.close()
    def test_y_axis_preserves_raw_values(self):
        app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        w=Window(); w.show()
        try:
            axis=w.plot.getAxis('left')
            for extent in (0.0005,0.5,500,50000):
                w.plot.setYRange(-extent,extent,padding=0)
                app.processEvents()
                axis.updateAutoSIPrefix()
                self.assertFalse(axis.autoSIPrefix)
                self.assertEqual(axis.autoSIPrefixScale,1.0)
            self.assertEqual(axis.tickStrings([-.5,0,.5],1,.1),['-0.5','0','0.5'])
            self.assertEqual(axis.tickStrings([1000],1,100),['1000'])
        finally: w.close()
    def test_legend_hide_does_not_duplicate_or_restore(self):
        from PyQt6 import QtCore
        class Click:
            def button(self): return QtCore.Qt.MouseButton.LeftButton
            def accept(self): pass
        w=Window(); w.show()
        try:
            w.sim.setChecked(True); w.connect_source(); QtTest.QTest.qWait(120)
            key='left_motor_angle'; curve=w.curves[key]
            sample=next(sample for sample,_ in w.legend.items if sample.item is curve)
            initial=len(w.legend.items)
            for _ in range(4):
                sample.mouseClickEvent(Click())
                self.assertFalse(curve.isVisible())
                QtTest.QTest.qWait(80); w.update_data()
                self.assertEqual(len(w.legend.items),initial)
                self.assertFalse(curve.isVisible())
                sample.mouseClickEvent(Click())
                QtTest.QTest.qWait(80); w.update_data()
                self.assertTrue(curve.isVisible())
                self.assertEqual(len(w.legend.items),initial)
            w.checks[key].setChecked(False)
            self.assertEqual(len(w.legend.items),initial-1)
            w.checks[key].setChecked(True)
            self.assertEqual(len(w.legend.items),initial)
            self.assertEqual(sum(s.item is curve for s,_ in w.legend.items),1)
        finally: w.close()
    def test_wrong_mode_waiting(self):
        from main import Receiver
        r=Receiver(None,921600,True)
        self.assertEqual(r.frames.maxlen,1000)
        r.frames.extend(range(1500)); self.assertEqual(len(r.take()),1000)
if __name__=='__main__': unittest.main()


