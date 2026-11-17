import math
import unittest
import numpy as np
from PyQt6 import QtWidgets
from imu_view import rotation_matrix, ImuPanel
from protocol import Parser, simulated_packet


class ImuTests(unittest.TestCase):
    def test_hamilton_body_to_world(self):
        s = math.sqrt(.5)
        for q,axis,expected in [([s,0,0,s],[1,0,0],[0,1,0]),
                                 ([s,0,s,0],[1,0,0],[0,0,-1]),
                                 ([s,s,0,0],[0,1,0],[0,0,1])]:
            matrix = rotation_matrix(q)
            np.testing.assert_allclose(matrix @ axis,expected,atol=1e-7)
            np.testing.assert_allclose(matrix,rotation_matrix([-v for v in q]))
        for q in ([0,0,0,0],[float('nan'),0,0,0],[10,0,0,0]):
            with self.assertRaises(ValueError):
                rotation_matrix(q)

    def test_panel_freeze_and_recover(self):
        app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        panel = ImuPanel()
        panel.resize(900,500)
        panel.show()
        p = Parser()
        frame = p.feed(simulated_packet(30))[0]
        panel.set_frame(frame)
        self.assertTrue(panel.canvas.fresh)
        before = panel.canvas.matrix.copy()
        panel.set_frame(p.feed(simulated_packet(80))[0],paused=True)
        np.testing.assert_array_equal(before,panel.canvas.matrix)
        panel.set_frame(frame,stale=True)
        self.assertFalse(panel.canvas.fresh)
        self.assertIn('过期',panel.state.text())
        panel.set_frame(p.feed(simulated_packet(405))[0])
        self.assertFalse(panel.canvas.fresh)
        np.testing.assert_array_equal(before,panel.canvas.matrix)
        frame.values['body_quat_0'] = float('nan')
        panel.set_frame(frame)
        self.assertFalse(panel.canvas.fresh)
        panel.set_frame(p.feed(simulated_packet(80))[0])
        self.assertTrue(panel.canvas.fresh)
        self.assertFalse(np.allclose(before,panel.canvas.matrix))
        for name,button in panel.view_buttons.items():
            button.click()
            app.processEvents()
            self.assertEqual(panel.canvas.view,name)
            self.assertFalse(panel.grab().isNull())
        panel.reset()
        self.assertFalse(panel.canvas.has_pose)
        panel.close()

if __name__ == '__main__':
    unittest.main()
