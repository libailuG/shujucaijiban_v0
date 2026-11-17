import argparse
import csv
import queue
import sys
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np
import serial
from serial.tools import list_ports
from PyQt6 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg
from offline_view import OfflinePanel
from protocol import Parser, Timing, KEYS, LABELS, CATEGORIES, CSV_COLUMNS, csv_row, simulated_packet

def label(key): return LABELS[key]

class Receiver(QtCore.QThread):
    message = QtCore.pyqtSignal(str)

    def __init__(self, port, baud, simulation):
        super().__init__()
        self.port,self.baud,self.simulation = port,baud,simulation
        self.frames = deque(maxlen=1000)
        self.lock = threading.Lock()
        self.commands = queue.Queue()
        self.parser = Parser()
        self.overflow = self.count = 0
        self.recorded = 0
        self.record_path = None

    def take(self):
        with self.lock:
            result = list(self.frames)
            self.frames.clear()
        return result

    def run(self):
        device = output = None
        writer = None
        try:
            if not self.simulation:
                device = serial.Serial()
                device.port,device.baudrate = self.port,self.baud
                device.timeout = .05
                device.bytesize,device.parity,device.stopbits = 8,'N',1
                device.rts = device.dtr = False
                device.open()
            self.message.emit('模拟接收中' if self.simulation else f'已连接 {self.port} · 8N1 · 只接收')
            n = 0
            deadline = time.monotonic()
            while not self.isInterruptionRequested():
                while not self.commands.empty():
                    path = self.commands.get_nowait()
                    if output:
                        output.close()
                    output = writer = None
                    self.record_path = None
                    if path:
                        output = open(path,'w',newline='',encoding='utf-8-sig')
                        writer = csv.writer(output)
                        writer.writerow(CSV_COLUMNS)
                        self.record_path = path
                        self.recorded = 0
                    self.message.emit('记录已开始：'+path if path else '记录已停止')
                if self.simulation:
                    data = simulated_packet(n)
                    n += 1
                    deadline += .01
                    time.sleep(max(0,deadline-time.monotonic()))
                    if time.monotonic()-deadline > .1:
                        deadline = time.monotonic()
                else:
                    data = device.read(min(max(device.in_waiting,1),16384))
                for frame in self.parser.feed(data):
                    item = (time.time(),frame)
                    self.count += 1
                    if writer:
                        writer.writerow(csv_row(*item))
                        self.recorded += 1
                        if self.recorded % 50 == 0:
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
            self.record_path = None

class Window(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('模式20 · 数据采集（预留串口1，只接收）')
        self.resize(1200,850)
        self.receiver = None
        self.timing = Timing()
        self.history = deque(maxlen=12000)
        self.last_rx = 0
        self.paused = False
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)
        layout.setContentsMargins(22,18,22,16)
        title = QtWidgets.QLabel('实时数据采集 · 模式20')
        title.setStyleSheet('font-size: 24px; font-weight: 600;')
        layout.addWidget(title)
        bar = QtWidgets.QHBoxLayout()
        layout.addLayout(bar)
        self.ports = QtWidgets.QComboBox()
        bar.addWidget(self.ports)
        self.refresh = QtWidgets.QPushButton('刷新串口')
        self.refresh.clicked.connect(self.refresh_ports)
        bar.addWidget(self.refresh)
        self.baud = QtWidgets.QComboBox()
        self.baud.setEditable(True)
        self.baud.addItems(['921600','115200','460800'])
        bar.addWidget(self.baud)
        self.sim = QtWidgets.QCheckBox('模拟数据')
        bar.addWidget(self.sim)
        self.connect_button = QtWidgets.QPushButton('连接')
        self.connect_button.clicked.connect(self.connect_source)
        bar.addWidget(self.connect_button)
        self.disconnect_button = QtWidgets.QPushButton('断开')
        self.disconnect_button.clicked.connect(self.disconnect_source)
        bar.addWidget(self.disconnect_button)
        bar.addStretch()
        bar = QtWidgets.QHBoxLayout()
        layout.addLayout(bar)
        self.pause = QtWidgets.QPushButton('暂停显示')
        self.pause.clicked.connect(self.toggle_pause)
        bar.addWidget(self.pause)
        clear = QtWidgets.QPushButton('清空显示缓存')
        clear.clicked.connect(self.clear)
        bar.addWidget(clear)
        self.record = QtWidgets.QPushButton('开始/停止连续记录')
        self.record.clicked.connect(self.toggle_record)
        bar.addWidget(self.record)
        export = QtWidgets.QPushButton('导出缓存 CSV')
        export.clicked.connect(self.export)
        bar.addWidget(export)
        note = QtWidgets.QLabel('JP4/UART4：第3脚 PA0 TX → USB串口 RX；第1脚 GND。921600 / 8N1。模式20且 data1.bit0=1 时100Hz上传。左右为穿戴者视角。')
        note.setWordWrap(True)
        layout.addWidget(note)
        self.state = QtWidgets.QLabel('未连接')
        layout.addWidget(self.state)
        self.stats = QtWidgets.QLabel('等待数据')
        self.stats.setWordWrap(True)
        layout.addWidget(self.stats)
        self.flags = QtWidgets.QLabel('设备微秒时间戳 · 帧无序号、有效位和CRC；零值可能是失效占位，无法判定传感器健康。')
        self.flags.setWordWrap(True)
        layout.addWidget(self.flags)
        self.tabs = QtWidgets.QTabWidget()
        layout.addWidget(self.tabs,1)
        curve_page = QtWidgets.QWidget()
        self.tabs.addTab(curve_page,'实时曲线')
        self.offline = OfflinePanel()
        self.tabs.addTab(self.offline,'离线文件')
        layout = QtWidgets.QVBoxLayout(curve_page)
        view_bar = QtWidgets.QHBoxLayout()
        layout.addLayout(view_bar)
        view_bar.addWidget(QtWidgets.QLabel('历史帧数'))
        self.history_limit = QtWidgets.QDoubleSpinBox()
        self.history_limit.setDecimals(4)
        self.history_limit.setRange(.0001,10)
        self.history_limit.setSuffix(' W')
        self.history_limit.setValue(1.2)
        self.history_limit.setKeyboardTracking(False)
        self.history_limit.valueChanged.connect(self.change_capacity)
        view_bar.addWidget(self.history_limit)
        view_bar.addWidget(QtWidgets.QLabel('历史显示长度'))
        self.span = QtWidgets.QDoubleSpinBox()
        self.span.setRange(.1,1000)
        self.span.setValue(10)
        self.span.setSuffix(' 秒')
        self.span.setKeyboardTracking(False)
        self.span.valueChanged.connect(self.change_span)
        view_bar.addWidget(self.span)
        self.follow = QtWidgets.QCheckBox('跟随最新数据')
        self.follow.setChecked(True)
        view_bar.addWidget(self.follow)
        resume = QtWidgets.QPushButton('开启自动跟随')
        resume.setEnabled(False)
        self.follow.toggled.connect(lambda enabled: resume.setEnabled(not enabled))
        resume.clicked.connect(lambda: self.follow.setChecked(True))
        view_bar.addWidget(resume)
        self.auto_y = QtWidgets.QCheckBox('Y 轴范围自动适应')
        self.auto_y.setChecked(True)
        self.auto_y.toggled.connect(lambda enabled: self.plot.enableAutoRange(axis='y',enable=enabled))
        view_bar.addWidget(self.auto_y)
        view_bar.addStretch()
        pg.setConfigOptions(antialias=False)
        self.plot = pg.PlotWidget(background='#111b2b')
        self.plot.showGrid(x=True,y=True,alpha=.18)
        self.plot.setLabel('bottom','设备相对时间',units='s')
        self.plot.setLabel('left','幅值')
        y_axis = self.plot.getAxis('left')
        if hasattr(y_axis, 'setSIPrefixEnableRanges'):
            y_axis.setSIPrefixEnableRanges(())
        y_axis.enableAutoSIPrefix(False)
        self.legend = self.plot.addLegend(offset=(12,12))
        self.plot.setDownsampling(auto=True,mode='peak')
        self.plot.setClipToView(True)
        self.updating_range = False
        self.plot.getViewBox().sigXRangeChanged.connect(self.on_x_changed)
        layout.addWidget(self.plot,1)
        pen = pg.mkPen('#a9bed6',width=1,style=QtCore.Qt.PenStyle.DashLine)
        self.crosshair = [pg.InfiniteLine(angle=angle,pen=pen) for angle in (90,0)]
        for line in self.crosshair:
            self.plot.addItem(line,ignoreBounds=True)
            line.hide()
        self.mouse_position = QtWidgets.QLabel('鼠标位置：移入绘图区查看时间和幅值')
        layout.addWidget(self.mouse_position)
        self.plot.scene().sigMouseMoved.connect(self.update_crosshair)
        self.plot.viewport().installEventFilter(self)
        self.groups = QtWidgets.QTabWidget()
        layout.addWidget(self.groups)
        categories = CATEGORIES
        self.readouts,self.group_checks = {},{}
        self.checks,self.curves = {},{}
        for row,key in enumerate(KEYS):
            check = QtWidgets.QCheckBox(label(key))
            check.setChecked(key in ('left_motor_angle','right_motor_angle','left_thigh_angle','right_thigh_angle','left_shank_angle','right_shank_angle'))
            self.checks[key] = check
            color = ('#45d5ff','#ffcb6b','#c792ea','#82d9a0','#ff8b94','#a5bfff')[row%6]
            check.setStyleSheet(f'color: {color};')
            self.readouts[key] = QtWidgets.QLabel('不可用')
            self.curves[key] = self.plot.plot(pen=pg.mkPen(color,width=1.5),connect='finite')
            self.curves[key].setVisible(check.isChecked())
            if check.isChecked():
                self.legend.addItem(self.curves[key],label(key))
            check.toggled.connect(lambda _,k=key: self.selection_changed(k))
        for name,keys in categories.items():
            page = QtWidgets.QWidget()
            grid = QtWidgets.QGridLayout(page)
            group_check = QtWidgets.QCheckBox('本组全选')
            self.group_checks[name] = group_check
            group_check.clicked.connect(lambda checked,ks=keys: self.select_group(ks,checked))
            grid.addWidget(group_check,0,0)
            for i,key in enumerate(keys):
                grid.addWidget(self.checks[key],i//3+1,(i%3)*2)
                grid.addWidget(self.readouts[key],i//3+1,(i%3)*2+1)
                self.checks[key].toggled.connect(lambda _,n=name,ks=keys: self.sync_group(n,ks))
            self.sync_group(name,keys)
            self.groups.addTab(page,name)
        self.groups.setCurrentIndex(0)
        hint = QtWidgets.QLabel('滚轮缩放，拖动平移；开启自动跟随恢复。点击图例线段隐藏/显示；下方勾选加入/移除通道。CSV 保存全部通道。')
        layout.addWidget(hint)
        self.refresh_ports()
        self.set_connected(False)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.update_data)
        self.timer.start(50)

    def select_group(self, keys, checked):
        for key in keys:
            self.checks[key].setChecked(checked)

    def sync_group(self, name, keys):
        count = sum(self.checks[key].isChecked() for key in keys)
        check = self.group_checks[name]
        check.setTristate(0 < count < len(keys))
        check.setCheckState(QtCore.Qt.CheckState.Checked if count == len(keys) else QtCore.Qt.CheckState.PartiallyChecked if count else QtCore.Qt.CheckState.Unchecked)

    def selection_changed(self, key):
        curve = self.curves[key]
        visible = self.checks[key].isChecked()
        curve.setVisible(visible)
        if visible:
            if not any(sample.item is curve for sample,_ in self.legend.items):
                self.legend.addItem(curve,label(key))
            curve.setData([h[0] for h in self.history],[h[2].values[key] if h[2].available(key) else np.nan for h in self.history])
        else:
            self.legend.removeItem(curve)

    def change_capacity(self, value):
        self.history = deque(self.history,maxlen=max(1,round(value*10000)))

    def change_span(self, value):
        if value*100 > self.history.maxlen:
            self.history_limit.setValue(np.ceil(value*100)/10000)
        left = self.plot.viewRange()[0][0]
        if self.follow.isChecked() and self.history:
            left = max(self.history[-1][0],value)-value
        self.updating_range = True
        self.plot.setXRange(left,left+value,padding=0)
        self.updating_range = False

    def on_x_changed(self, *_):
        if not self.updating_range:
            left,right = self.plot.viewRange()[0]
            self.span.blockSignals(True)
            self.span.setValue(right-left)
            self.span.blockSignals(False)

    def update_crosshair(self, pos):
        view = self.plot.getViewBox()
        if view.sceneBoundingRect().contains(pos):
            p = view.mapSceneToView(pos)
            for line,value in zip(self.crosshair,(p.x(),p.y())):
                line.setPos(value)
                line.show()
            self.mouse_position.setText(f'鼠标位置：时间 X = {p.x():.4f} s    幅值 Y = {p.y():+.4f}')
        else:
            self.hide_crosshair()

    def hide_crosshair(self):
        for line in self.crosshair:
            line.hide()
        self.mouse_position.setText('鼠标位置：移入绘图区查看时间和幅值')

    def eventFilter(self, watched, event):
        if watched is self.plot.viewport() and event.type() == QtCore.QEvent.Type.Leave:
            self.hide_crosshair()
        return super().eventFilter(watched,event)

    def set_connected(self, yes):
        for widget in (self.ports,self.baud,self.sim,self.refresh,self.connect_button):
            widget.setEnabled(not yes)
        self.disconnect_button.setEnabled(yes)
        self.record.setEnabled(yes)

    def refresh_ports(self):
        old = self.ports.currentData()
        self.ports.clear()
        for p in list_ports.comports():
            self.ports.addItem(f'{p.device} · {p.description}',p.device)
        i = self.ports.findData(old)
        if i >= 0:
            self.ports.setCurrentIndex(i)

    def connect_source(self):
        if self.receiver and self.receiver.isRunning():
            return
        if not self.sim.isChecked() and not self.ports.currentData():
            self.state.setText('未检测到串口，可使用模拟数据')
            return
        try:
            baud = int(self.baud.currentText())
            if baud <= 0:
                raise ValueError()
        except ValueError:
            self.state.setText('波特率应为正整数')
            return
        self.clear()
        self.timing = Timing()
        self.last_rx = time.monotonic()
        self.receiver = Receiver(self.ports.currentData(),baud,self.sim.isChecked())
        self.receiver.message.connect(self.state.setText)
        self.receiver.finished.connect(self.receiver_finished)
        self.set_connected(True)
        self.receiver.start()

    @QtCore.pyqtSlot()
    def receiver_finished(self):
        self.set_connected(False)

    def disconnect_source(self):
        if self.receiver:
            self.receiver.requestInterruption()
            if not self.receiver.wait(2000):
                self.state.setText('正在等待接收线程结束')
                return False
        self.set_connected(False)
        self.state.setText('已断开；历史值不代表当前状态')
        return True

    def toggle_pause(self):
        self.paused = not self.paused
        self.pause.setText('恢复显示' if self.paused else '暂停显示')

    def clear(self):
        self.history.clear()
        for curve in self.curves.values():
            curve.setData([],[])
        for readout in self.readouts.values():
            readout.setText('不可用')

    def toggle_record(self):
        if not self.receiver or not self.receiver.isRunning():
            return
        if self.receiver.record_path:
            self.receiver.commands.put(None)
        else:
            path,_ = QtWidgets.QFileDialog.getSaveFileName(self,'连续记录保存为','mode20_record.csv','CSV (*.csv)')
            if path:
                self.receiver.commands.put(path)

    def export(self, path=None):
        if not path:
            path,_ = QtWidgets.QFileDialog.getSaveFileName(self,'导出历史缓存','mode20_cache.csv','CSV (*.csv)')
        if not path:
            return
        try:
            with open(path,'w',newline='',encoding='utf-8-sig') as out:
                writer = csv.writer(out)
                writer.writerow(CSV_COLUMNS)
                writer.writerows(csv_row(host,f) for _,host,f in self.history)
            self.state.setText(f'已导出 {len(self.history)} 帧：{path}')
        except OSError as exc:
            self.state.setText(f'导出失败：{exc}')

    def update_data(self):
        rx = self.receiver
        if not rx:
            return
        batch = rx.take()
        for host,f in batch:
            self.history.append((self.timing.update(f),host,f))
        if batch:
            self.last_rx = time.monotonic()
        stale = time.monotonic()-self.last_rx > 1 or not rx.isRunning()
        if not self.history:
            self.stats.setText(f'等待模式20采集数据：请在遥控端选择20并打开bit0 · 收到帧 {rx.count} · 错帧候选 {rx.parser.bad} · 丢弃字节 {rx.parser.discarded}')
            return
        f = self.history[-1][2]
        elapsed = max(.001,self.history[-1][0])
        self.stats.setText(f'设备时间戳 {f.timestamp_us} μs · 时间轴重建 {self.timing.resets} 次 · 本机接收编号 {f.receive_index} · 接收 {rx.count} 帧 · 设备时间平均 {(rx.count-1)/elapsed:.1f} 帧/秒 · UI队列溢出 {rx.overflow} · 错帧候选 {rx.parser.bad} · 丢弃字节 {rx.parser.discarded} · 已记录 {rx.recorded}\n'+('超过1秒无帧或连接断开' if stale else '正在接收（不代表各传感器有效）')+(' · 显示暂停，接收和记录继续' if self.paused else ''))
        if self.paused:
            return
        for row,key in enumerate(KEYS):
            self.readouts[key].setText((str(f.values[key]) if key.endswith('pressure') else f'{f.values[key]:+.4f}') if not stale and f.available(key) else '不可用')
            curve = self.curves[key]
            visible = self.checks[key].isChecked()
            if visible:
                curve.setData([h[0] for h in self.history],[h[2].values[key] if h[2].available(key) else np.nan for h in self.history])
        if self.follow.isChecked():
            self.updating_range = True
            end = max(self.history[-1][0],self.span.value())
            self.plot.setXRange(end-self.span.value(),end,padding=0)
            self.updating_range = False
        self.statusBar().showMessage(f'累计 {rx.count:,} 帧 | 缓存 {len(self.history):,}/{self.history.maxlen:,} 帧 | '+('显示已暂停' if self.paused else '监测中'))

    def closeEvent(self,event):
        if self.disconnect_source() and self.offline.shutdown():
            event.accept()
        else:
            event.ignore()

def create_application():
    app = QtWidgets.QApplication(sys.argv[:1])
    # Load Chinese glyphs explicitly for offscreen preview as well as Windows GUI.
    font_path = Path('C:/Windows/Fonts/msyh.ttc')
    if font_path.exists():
        QtGui.QFontDatabase.addApplicationFont(str(font_path))
    app.setFont(QtGui.QFont('Microsoft YaHei',10))
    app.setStyle('Fusion')
    app.setStyleSheet('''
        QWidget { background: #182335; color: #e5edf7; font-size: 13px; }
        QPushButton, QComboBox { background: #263750; border: 1px solid #3c516e;
            border-radius: 5px; padding: 7px 12px; }
        QPushButton:hover { background: #354d6e; }
        QPushButton:checked { background: #304a65; border: 1px solid #45d5ff; }
        QStatusBar { color: #9bb2ce; }
    ''')
    return app

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--simulate',action='store_true')
    parser.add_argument('--smoke-test',action='store_true',help='模拟运行3秒、截图和CSV，自动退出')
    args = parser.parse_args()
    app = create_application()
    window = Window()
    window.show()
    if args.simulate or args.smoke_test:
        window.sim.setChecked(True)
        window.connect_source()
    if args.smoke_test:
        def finish():
            window.grab().save(str(Path(__file__).with_name('simulation_preview.png')))
            window.export(str(Path(__file__).with_name('simulation_export.csv')))
            print(f'SMOKE: frames={len(window.history)}, status={window.stats.text()}',flush=True)
            window.close()
        QtCore.QTimer.singleShot(3000,finish)
    sys.exit(app.exec())

if __name__ == '__main__':
    main()







