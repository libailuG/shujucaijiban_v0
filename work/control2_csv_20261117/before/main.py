import argparse
import csv
import queue
import sys
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np
import pyqtgraph as pg
import serial
from serial.tools import list_ports
from PyQt6 import QtCore, QtGui, QtWidgets
from protocol import Parser, STATUS, CSV_COLUMNS, csv_row, simulated_packet


class Receiver(QtCore.QThread):
    message = QtCore.pyqtSignal(str)

    def __init__(self, port, simulation=False):
        super().__init__()
        self.port, self.simulation = port, simulation
        self.parser = Parser()
        self.frames = deque(maxlen=4000)
        self.lock = threading.Lock()
        self.commands = queue.Queue()
        self.overflow = self.recorded = 0

    def take(self):
        with self.lock:
            frames = list(self.frames)
            self.frames.clear()
        return frames

    def run(self):
        device = output = writer = None
        try:
            if not self.simulation:
                device = serial.Serial()
                device.port, device.baudrate, device.timeout = self.port, 921600, .03
                device.bytesize, device.parity, device.stopbits = 8, 'N', 1
                device.rts = device.dtr = False
                device.open()
            self.message.emit('模拟接收中' if self.simulation else f'已连接 {self.port} · 921600 8N1 · 只接收')
            n, deadline = 0, time.monotonic()
            while not self.isInterruptionRequested():
                while not self.commands.empty():
                    path = self.commands.get_nowait()
                    if output:
                        output.close()
                    output = writer = None
                    if path:
                        output = open(path, 'w', newline='', encoding='utf-8-sig')
                        writer = csv.writer(output)
                        writer.writerow(CSV_COLUMNS)
                        self.recorded = 0
                    self.message.emit('正在连续记录：'+path if path else '记录已停止')
                if self.simulation:
                    data = simulated_packet(n)
                    n += 1
                    deadline += .01
                    time.sleep(max(0, deadline-time.monotonic()))
                    if time.monotonic()-deadline > .1:
                        deadline = time.monotonic()
                else:
                    data = device.read(min(max(device.in_waiting, 1), 16384))
                for frame in self.parser.feed(data):
                    item = (time.time(), frame)
                    if writer:
                        writer.writerow(csv_row(*item))
                        self.recorded += 1
                        if self.recorded % 100 == 0:
                            output.flush()
                    with self.lock:
                        if len(self.frames) == self.frames.maxlen:
                            self.overflow += 1
                        self.frames.append(item)
        except Exception as exc:
            self.message.emit(f'接收/记录错误：{exc}')
        finally:
            if device:
                device.close()
            if output:
                output.close()


class Window(QtWidgets.QMainWindow):
    def __init__(self, simulation=False):
        super().__init__()
        self.setWindowTitle('控制模式2 · 关节与助力监测（只接收）')
        self.resize(1250, 900)
        self.receiver = None
        self.history = deque(maxlen=12000)
        self.last_rx = 0
        self.offline = self.paused = self.recording = False
        root = QtWidgets.QWidget()
        self.setCentralWidget(root)
        layout = QtWidgets.QVBoxLayout(root)
        title = QtWidgets.QLabel('控制模式2 · 关节与助力监测')
        title.setStyleSheet('font-size:24px;font-weight:600;color:#eaf0ff')
        layout.addWidget(title)
        note = QtWidgets.QLabel('预留串口1 JP4 / UART4 · 921600 · 模型坐标：左右后伸为正 · 角度 rad / 角速度 rad/s / 力矩 Nm\n'
                                '计算力矩包含倍率和保护，禁止输出时仍显示；驱动命令不等于实测力矩。模式通过预留串口2遥控选择，本程序只接收。')
        note.setWordWrap(True)
        layout.addWidget(note)
        bar = QtWidgets.QHBoxLayout()
        layout.addLayout(bar)
        self.ports = QtWidgets.QComboBox()
        bar.addWidget(self.ports)
        self.refresh = QtWidgets.QPushButton('刷新串口')
        self.refresh.clicked.connect(self.refresh_ports)
        bar.addWidget(self.refresh)
        self.simulate = QtWidgets.QCheckBox('模拟数据')
        self.simulate.setChecked(simulation)
        bar.addWidget(self.simulate)
        self.connect_button = QtWidgets.QPushButton('连接')
        self.connect_button.clicked.connect(self.toggle_connection)
        bar.addWidget(self.connect_button)
        self.record_button = QtWidgets.QPushButton('开始连续记录 CSV')
        self.record_button.setEnabled(False)
        self.record_button.clicked.connect(self.toggle_record)
        bar.addWidget(self.record_button)
        export = QtWidgets.QPushButton('导出当前数据 CSV')
        export.clicked.connect(self.export_csv)
        bar.addWidget(export)
        load = QtWidgets.QPushButton('读取二进制日志')
        load.clicked.connect(self.load_file)
        bar.addWidget(load)
        bar2 = QtWidgets.QHBoxLayout()
        layout.addLayout(bar2)
        self.pause = QtWidgets.QPushButton('暂停绘图')
        self.pause.clicked.connect(self.toggle_pause)
        bar2.addWidget(self.pause)
        self.follow = QtWidgets.QCheckBox('跟随最新')
        self.follow.setChecked(True)
        bar2.addWidget(self.follow)
        bar2.addWidget(QtWidgets.QLabel('显示时长'))
        self.span = QtWidgets.QDoubleSpinBox()
        self.span.setRange(.1, 120)
        self.span.setValue(10)
        self.span.setSuffix(' s')
        bar2.addWidget(self.span)
        self.all_channels = QtWidgets.QCheckBox('全部通道')
        self.all_channels.setChecked(True)
        self.all_channels.clicked.connect(lambda on: [c.setChecked(on) for c in self.checks])
        bar2.addWidget(self.all_channels)
        bar2.addStretch()
        self.state = QtWidgets.QLabel('未接收数据')
        layout.addWidget(self.state)
        self.timing = QtWidgets.QLabel('计算耗时 — · 调用周期 — · 倍率 —')
        layout.addWidget(self.timing)
        self.driver = QtWidgets.QLabel('最终驱动命令：左 — Nm / 右 — Nm')
        layout.addWidget(self.driver)
        self.plots, self.curves, self.checks, self.readouts = [], [], [], []
        plot = pg.PlotWidget(background='#111b2b')
        plot.showGrid(x=True, y=True, alpha=.18)
        plot.setLabel('left', '幅值（单位见图例）')
        plot.setLabel('bottom', '设备累计时间', units='s')
        plot.getAxis('left').enableAutoSIPrefix(False)
        plot.addLegend(colCount=2)
        plot.setDownsampling(auto=True, mode='peak')
        plot.setClipToView(True)
        self.plots.append(plot)
        layout.addWidget(plot, 1)
        grid = QtWidgets.QGridLayout()
        layout.addLayout(grid)
        channels = [('左角度', 'rad', '#45d5ff'), ('右角度', 'rad', '#ffcb6b'),
                    ('左角速度', 'rad/s', '#c792ea'), ('右角速度', 'rad/s', '#82d9a0'),
                    ('左计算力矩', 'Nm', '#ff8b94'), ('右计算力矩', 'Nm', '#a5bfff')]
        for i, (name, unit, color) in enumerate(channels):
            check = QtWidgets.QCheckBox(f'{name} {unit}')
            check.setChecked(True)
            check.setStyleSheet(f'color:{color}')
            curve = plot.plot(name=f'{name} {unit}', pen=pg.mkPen(color, width=1.6), connect='finite')
            check.toggled.connect(curve.setVisible)
            check.toggled.connect(self.sync_channels)
            value = QtWidgets.QLabel('—')
            grid.addWidget(check, i//2, (i%2)*2)
            grid.addWidget(value, i//2, (i%2)*2+1)
            self.checks.append(check)
            self.curves.append(curve)
            self.readouts.append(value)
        self.message = QtWidgets.QLabel('等待连接；遥控 data2=2 自动上传，bit0仅控制扭矩许可')
        self.message.setWordWrap(True)
        layout.addWidget(self.message)
        self.refresh_ports()
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.update_view)
        self.timer.start(50)

    def sync_channels(self):
        count = sum(c.isChecked() for c in self.checks)
        self.all_channels.setTristate(0 < count < 6)
        self.all_channels.setCheckState(QtCore.Qt.CheckState.Checked if count == 6 else
            QtCore.Qt.CheckState.PartiallyChecked if count else QtCore.Qt.CheckState.Unchecked)

    def refresh_ports(self):
        current = self.ports.currentData()
        self.ports.clear()
        for p in list_ports.comports():
            self.ports.addItem(f'{p.device} · {p.description}', p.device)
        index = self.ports.findData(current)
        if index >= 0:
            self.ports.setCurrentIndex(index)

    def toggle_connection(self):
        if self.receiver:
            self.stop_receiver()
            return
        if not self.simulate.isChecked() and not self.ports.currentData():
            self.message.setText('请选择串口，或勾选模拟数据')
            return
        self.history = deque(maxlen=12000)
        self.offline = False
        self.last_rx = 0
        for curve in self.curves:
            curve.setData([], [])
        self.receiver = Receiver(self.ports.currentData(), self.simulate.isChecked())
        self.receiver.message.connect(self.message.setText)
        self.receiver.start()
        self.connect_button.setText('断开')
        self.ports.setEnabled(False)
        self.refresh.setEnabled(False)
        self.simulate.setEnabled(False)
        self.record_button.setEnabled(True)

    def stop_receiver(self):
        worker = self.receiver
        if worker:
            worker.requestInterruption()
            if not worker.wait(3000):
                self.message.setText('接收线程仍在结束，请稍后再次断开')
                return False
            for item in worker.take():
                self.history.append(item)
            self.receiver = None
        self.recording = False
        self.record_button.setText('开始连续记录 CSV')
        self.record_button.setEnabled(False)
        self.connect_button.setText('连接')
        self.ports.setEnabled(True)
        self.refresh.setEnabled(True)
        self.simulate.setEnabled(True)
        return True

    def toggle_record(self):
        if not self.receiver or not self.receiver.isRunning():
            return
        if self.recording:
            self.receiver.commands.put(None)
            self.recording = False
            self.record_button.setText('开始连续记录 CSV')
        else:
            path, _ = QtWidgets.QFileDialog.getSaveFileName(self, '连续记录',
                'control2_'+time.strftime('%Y%m%d_%H%M%S')+'.csv', 'CSV (*.csv)')
            if path:
                self.receiver.commands.put(path)
                self.recording = True
                self.record_button.setText('停止连续记录')

    def export_csv(self):
        if not self.history:
            self.message.setText('没有可导出的数据')
            return
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, '导出当前数据', 'control2.csv', 'CSV (*.csv)')
        if not path:
            return
        try:
            with open(path, 'w', newline='', encoding='utf-8-sig') as output:
                writer = csv.writer(output)
                writer.writerow(CSV_COLUMNS)
                writer.writerows(csv_row(*item) for item in self.history)
            self.message.setText(f'已导出 {len(self.history)} 帧：{path}')
        except Exception as exc:
            self.message.setText(f'导出失败：{exc}')

    def load_file(self):
        if self.receiver:
            self.message.setText('请先断开串口，再读取日志')
            return
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, '读取模式2原始二进制日志', '', '日志 (*.bin *.txt *.log);;全部文件 (*)')
        if not path:
            return
        try:
            parser, frames = Parser(), []
            with open(path, 'rb') as source:
                while chunk := source.read(65536):
                    frames.extend((None, frame) for frame in parser.feed(chunk))
            if not frames:
                raise ValueError('没有找到模式2的80字节有效帧；不支持其他模式或文本十六进制日志')
            self.history = frames
            self.offline = True
            self.paused = False
            self.pause.setText('暂停绘图')
            self.follow.setChecked(False)
            self.plots[0].setXRange(frames[0][1].time_s, max(.01, frames[-1][1].time_s), padding=.02)
            self.message.setText(f'{Path(path).name} · {len(frames)}帧 · 缺序号{parser.lost} · 错帧{parser.bad} · 跳过{parser.discarded}字节 · 尾部{len(parser.buffer)}字节')
            self.update_view()
        except Exception as exc:
            self.message.setText(f'读取失败：{exc}')

    def toggle_pause(self):
        self.paused = not self.paused
        self.pause.setText('恢复绘图' if self.paused else '暂停绘图')

    def update_view(self):
        if self.receiver:
            items = self.receiver.take()
            if items:
                self.last_rx = time.monotonic()
                self.history.extend(items)
            if not self.receiver.isRunning():
                self.stop_receiver()
        if not self.history:
            return
        f = self.history[-1][1]
        stale = not self.offline and time.monotonic()-self.last_rx > 1
        permission = '允许扭矩' if f.flags & 4 else '禁止输出（仍计算）'
        fault = ' · 数据过期' if stale else ''
        self.state.setText(f'状态：{STATUS.get(f.status, str(f.status))} · {permission}{fault} · '
                           f'内置保护 {bool(f.flags & 16)} / 外部抑制 {bool(f.flags & 32)}')
        self.timing.setText(f'整轮计算 {f.compute_us/1000:.3f} ms · 平均 {f.values[9]:.3f} ms · '
                            f'调用周期 {f.period_us/1000:.3f} ms · 倍率 {f.values[8]:.3f} · 序号 {f.sequence}')
        self.driver.setText(f'最终驱动命令：左 {f.values[6]:+.3f} Nm / 右 {f.values[7]:+.3f} Nm · '
                            f'有效 {bool(f.flags & 8)}（非实测/非执行ACK）')
        if self.receiver:
            p = self.receiver.parser
            self.driver.setText(self.driver.text()+f' · 缺序号 {p.lost} / 错帧 {p.bad} / 显示队列丢弃 {self.receiver.overflow} / 已记录 {self.receiver.recorded}')
        if self.paused:
            return
        for label, value in zip(self.readouts, f.plot_values()):
            label.setText('数据过期' if stale else '无效' if not np.isfinite(value) else f'{value:+.4f}')
        # Keep full history for export; only convert the visible range for plotting.
        left, right = (f.time_s-self.span.value(), f.time_s) if self.follow.isChecked() else self.plots[0].viewRange()[0]
        selected = [item[1] for item in self.history if left-.1 <= item[1].time_s <= right+.1]
        x, y = [], []
        previous = None
        for frame in selected:
            if previous and (frame.session != previous.session or ((frame.sequence-previous.sequence)&0xFFFFFFFF) != 1):
                x.append(frame.time_s)
                y.append([np.nan]*6)
            x.append(frame.time_s)
            y.append(frame.plot_values())
            previous = frame
        if selected:
            values = np.asarray(y)
            for i, curve in enumerate(self.curves):
                curve.setData(x, values[:, i])
        else:
            for curve in self.curves:
                curve.setData([], [])
        if self.follow.isChecked():
            self.plots[0].setXRange(max(0, f.time_s-self.span.value()), max(.1, f.time_s), padding=0)

    def closeEvent(self, event):
        if self.stop_receiver():
            event.accept()
        else:
            event.ignore()


def configure_app(app):
    # Offscreen Qt has no system font discovery; register the CJK font for QA.
    if not QtGui.QFontDatabase.families():
        font = Path('C:/Windows/Fonts/msyh.ttc')
        if font.exists():
            QtGui.QFontDatabase.addApplicationFont(str(font))
    app.setFont(QtGui.QFont('Microsoft YaHei', 10))
    app.setStyle('Fusion')
    app.setStyleSheet('QWidget{background:#152235;color:#dce7f7;font-size:13px;} '
                     'QPushButton,QComboBox,QDoubleSpinBox{background:#243951;padding:5px;border:1px solid #3b526e;border-radius:4px;} '
                     'QPushButton:disabled{color:#728097;}')
    pg.setConfigOptions(antialias=False)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--simulate', action='store_true')
    args = parser.parse_args()
    app = QtWidgets.QApplication(sys.argv)
    configure_app(app)
    window = Window(args.simulate)
    window.show()
    if args.simulate:
        window.toggle_connection()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
