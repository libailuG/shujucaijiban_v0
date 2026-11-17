"""Offscreen integration test: receiver, records, plots, log reload and export."""
import csv
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from pathlib import Path
import tempfile
import time
from unittest.mock import patch
from PyQt6 import QtWidgets
from main import Window, configure_app
from protocol import simulated_packet

def pump(app, duration):
    end = time.monotonic()+duration
    while time.monotonic() < end:
        app.processEvents()
        time.sleep(.01)

app = QtWidgets.QApplication([])
configure_app(app)
with tempfile.TemporaryDirectory() as directory:
    root = Path(directory)
    window = Window(simulation=True)
    window.show()
    window.toggle_connection()
    window.receiver.commands.put(str(root/'continuous.csv'))
    pump(app, .8)
    assert len(window.history) > 20
    assert len(window.curves) == 6
    assert window.curves[0].getData()[0].size > 0
    assert '禁止输出' in window.state.text()
    assert all('无效' not in label.text() for label in window.readouts)
    window.toggle_pause()
    before = len(window.history)
    pump(app, .15)
    assert len(window.history) > before
    window.toggle_pause()
    window.all_channels.click()
    assert not any(check.isChecked() for check in window.checks)
    window.all_channels.click()
    assert all(check.isChecked() for check in window.checks)
    window.stop_receiver()
    with open(root/'continuous.csv', encoding='utf-8-sig') as source:
        assert len(list(csv.reader(source))) > 20
    with patch.object(QtWidgets.QFileDialog, 'getSaveFileName', return_value=(str(root/'export.csv'), '')):
        window.export_csv()
    assert (root/'export.csv').stat().st_size > 100
    (root/'raw.bin').write_bytes(b''.join(simulated_packet(n) for n in range(200)))
    with patch.object(QtWidgets.QFileDialog, 'getOpenFileName', return_value=(str(root/'raw.bin'), '')):
        window.load_file()
    assert window.offline and len(window.history) == 200
    window.update_view()
    pump(app, .1)
    screenshot = Path(__file__).resolve().parents[2]/'work/control2_v10_20261117/host_preview.png'
    window.grab().save(str(screenshot))
    window.close()
print('PASS: simulated serial stream, six curves, gate-off computed torque, pause, selection, continuous CSV, export, offline reload')
