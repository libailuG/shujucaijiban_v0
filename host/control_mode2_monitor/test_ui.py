"""Offscreen integration test: receiver, records, plots, log reload and export."""
import csv
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from pathlib import Path
import tempfile
import time
import csv_features
from unittest.mock import patch
from PyQt6 import QtWidgets
from main import Window, configure_app
from protocol import simulated_packet, CSV_COLUMNS, Parser

def pump(app, duration):
    end = time.monotonic()+duration
    while time.monotonic() < end:
        app.processEvents()
        time.sleep(.01)

app = QtWidgets.QApplication([])
configure_app(app)
with tempfile.TemporaryDirectory() as directory:
    root = Path(directory)
    csv_features.ROOT = root
    window = Window(simulation=True)
    window.show()
    window.toggle_connection()
    window.toggle_record()
    pump(app, .8)
    assert len(window.history) > 20
    assert window.recording
    first_path = Path(window.record_path)
    assert first_path.parent == root/'data'
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
    count_before = window.receiver.recorded
    pump(app, .15)
    assert window.receiver.recorded > count_before
    window.all_channels.click()
    assert all(check.isChecked() for check in window.checks)
    window.toggle_record()
    pump(app, .1)
    assert not window.recording
    first_size = first_path.stat().st_size
    pump(app, .1)
    assert first_path.stat().st_size == first_size
    with open(first_path, encoding='utf-8-sig') as source:
        rows = list(csv.reader(source))
        assert len(rows) > 20 and len(rows[0]) == len(CSV_COLUMNS)
    window.tabs.setCurrentIndex(1)
    window.csv_review.load(first_path)
    pump(app, .2)
    assert len(window.csv_review.data) == len(rows)-1
    assert sum(check.isChecked() for check in window.csv_review.checks) == 6
    before = len(window.history)
    window.toggle_record()
    pump(app, .15)
    assert window.recording and len(window.history) > before
    second_path = Path(window.record_path)
    assert second_path != first_path and first_path.stat().st_size == first_size
    window.stop_receiver()
    pump(app, .05)
    assert not window.recording
    assert second_path.stat().st_size > 100
    with patch.object(QtWidgets.QFileDialog, 'getSaveFileName', return_value=(str(root/'export.csv'), '')):
        window.export_csv()
    assert (root/'export.csv').stat().st_size > 100
    (root/'raw.bin').write_bytes(b''.join(simulated_packet(n) for n in range(200)))
    with patch.object(QtWidgets.QFileDialog, 'getOpenFileName', return_value=(str(root/'raw.bin'), '')):
        window.load_file()
    assert window.offline and len(window.history) == 200
    window.update_view()
    pump(app, .1)
    window.tabs.setCurrentIndex(0)
    screenshot = Path(__file__).resolve().parents[2]/'work/control2_v10_20261117/host_preview.png'
    window.grab().save(str(screenshot))
    window.tabs.setCurrentIndex(1)
    window.grab().save(str(screenshot.with_name('host_csv_preview.png')))
    window.tabs.setCurrentIndex(0)
    raw3 = (Path(__file__).resolve().parents[2]/'work/control3_20261117/telemetry_fixture.bin').read_bytes()
    f = Parser().feed(raw3)[0]
    window.history.clear()
    window.history.append((0, f))
    window.update_view()
    assert '模式3' in window.state.text()
    assert '幅值 4.000 Nm' in window.timing.text() and '周期 1.000 s' in window.timing.text()
    assert '右 +0.000 Nm' in window.driver.text()
    pump(app, .1)
    window.grab().save(str(screenshot.with_name('host_mode3_preview.png')))
    window.close()
print('PASS: automatic CSV paths, all channels, stop/restart, close flush, independent review, six curves, export, offline reload')
