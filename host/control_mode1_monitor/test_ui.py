"""Offscreen Qt integration check, never opens a serial port."""
import csv
import tempfile
import time
import unittest
from pathlib import Path
from PyQt6 import QtWidgets, QtTest
from main import Window, Receiver

class UiTests(unittest.TestCase):
    def test_simulation_record_pause_clear_export(self):
        app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        with tempfile.TemporaryDirectory() as tmp:
            w = Window()
            w.protocol_choice.setCurrentIndex(1)
            w.show()
            self.assertEqual(w.groups.currentIndex(),2)
            self.assertEqual({k for k,c in w.checks.items() if c.isChecked()},
                             {'joint_angle_norm_0','joint_angle_norm_1',
                              'joint_speed_norm_0','joint_speed_norm_1'})
            group = w.group_checks['关节反馈']
            group.click()
            keys = ['joint_angle_rad_0','joint_angle_rad_1','joint_speed_rad_s_0','joint_speed_rad_s_1']
            self.assertTrue(all(w.checks[k].isChecked() and w.curves[k].isVisible() for k in keys))
            w.checks[keys[0]].setChecked(False)
            self.assertTrue(group.isTristate())
            group.click()
            self.assertTrue(all(w.checks[k].isChecked() for k in keys))
            group.click()
            self.assertTrue(all(not w.checks[k].isChecked() for k in keys))
            w.span.setValue(150)
            self.assertEqual(w.history.maxlen,7500)
            w.follow.setChecked(False)
            w.auto_y.setChecked(False)
            w.sim.setChecked(True)
            w.connect_source()
            record = str(Path(tmp)/'record.csv')
            w.receiver.commands.put(record)
            QtTest.QTest.qWait(700)
            self.assertGreater(len(w.history),10)
            self.assertTrue(w.imu.canvas.fresh)
            self.assertEqual(w.tabs.tabText(1),'IMU 姿态')
            w.toggle_pause()
            before = w.receiver.count
            QtTest.QTest.qWait(250)
            self.assertGreater(w.receiver.count,before)
            w.clear()
            self.assertEqual(len(w.history),0)
            QtTest.QTest.qWait(200)
            w.toggle_pause()
            w.update_data()
            output = str(Path(tmp)/'export.csv')
            w.export(output)
            self.assertTrue(w.disconnect_source())
            self.assertFalse(w.imu.canvas.fresh)
            with open(record,encoding='utf-8-sig') as saved:
                rows = list(csv.DictReader(saved))
            self.assertGreater(len(rows),10)
            self.assertIn('driver_torque_nm_0_valid',rows[0])
            self.assertTrue(Path(output).stat().st_size>100)
            w.close()

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
        app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        w=Window(); w.show()
        try:
            w.sim.setChecked(True); w.connect_source(); QtTest.QTest.qWait(120)
            key='joint_angle_norm_0'; curve=w.curves[key]
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
    def test_bounded_receive_queue(self):
        r = Receiver(None,921600,True)
        self.assertEqual(r.frames.maxlen,1000)
        r.frames.extend(range(1500))
        self.assertEqual(len(r.take()),1000)
        self.assertEqual(len(r.take()),0)

if __name__ == '__main__':
    unittest.main()


