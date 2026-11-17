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
from protocol import Parser, Timing, KEYS, STATUS, CSV_COLUMNS, csv_row, simulated_packet
from imu_view import ImuPanel
import observation as obs

LABELS = {'joint_angle_rad':'关节角度 rad','joint_speed_rad_s':'关节速度 rad/s',
 'body_quat':'机身四元数 wxyz','body_gyro_rad_s':'机身角速度 XYZ rad/s',
 'joint_angle_norm':'观测：关节角度/π','joint_speed_norm':'观测：关节速度/(4π)',
 'previous_torque_norm':'观测：上周期模型命令/40','body_tilt_norm':'观测：pitch、roll/π',
 'body_gyro_norm':'观测：模型角速度/(4π)','raw_action':'网络原始输出',
 'requested_torque_nm':'请求扭矩 Nm','model_torque_nm':'限幅模型扭矩 Nm','driver_torque_nm':'最终驱动命令 Nm（非实测）'}

def label(key):
    group,index = key.rsplit('_',1)
    suffix = ('wxyz'[int(index)] if group == 'body_quat' else 'XYZ'[int(index)] if 'gyro' in group else ('pitch','roll')[int(index)] if group == 'body_tilt_norm' else ('左','右')[int(index)])
    return LABELS[group]+' · '+suffix

class Receiver(QtCore.QThread):
    message = QtCore.pyqtSignal(str)

    def __init__(self, port, baud, simulation, observation=False):
        super().__init__()
        self.port,self.baud,self.simulation = port,baud,simulation
        self.frames = deque(maxlen=1000)
        self.lock = threading.Lock()
        self.commands = queue.Queue()
        self.observation = observation
        self.parser = obs.Parser() if observation else Parser()
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
                        writer.writerow(obs.CSV_COLUMNS if self.observation else CSV_COLUMNS)
                        self.record_path = path
                        self.recorded = 0
                    self.message.emit('记录已开始：'+path if path else '记录已停止')
                if self.simulation:
                    data = obs.simulated_packet(n) if self.observation else simulated_packet(n)
                    n += 1
                    deadline += .02
                    time.sleep(max(0,deadline-time.monotonic()))
                    if time.monotonic()-deadline > .1:
                        deadline = time.monotonic()
                else:
                    data = device.read(min(max(device.in_waiting,1),16384))
                for frame in self.parser.feed(data):
                    item = (time.time(),frame)
                    self.count += 1
                    if writer:
                        writer.writerow(obs.csv_row(*item) if self.observation else csv_row(*item))
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
        self.setWindowTitle('控制模式1 · 神经网络监测（只接收）')
        self.resize(1200,850)
        self.receiver = None
        self.timing = Timing()
        self.history = deque(maxlen=6000)
        self.last_rx = 0
        self.paused = False
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)
        layout.setContentsMargins(22,18,22,16)
        title = QtWidgets.QLabel('实时信号监视器 · 控制模式1')
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
        self.protocol_choice = QtWidgets.QComboBox()
        self.protocol_choice.addItems(['观测52字节（新固件）','监测160字节（旧固件）'])
        bar.addWidget(self.protocol_choice)
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
        note = QtWidgets.QLabel('JP4/UART4：第3脚 PA0 TX → USB串口 RX；第1脚 GND。921600 / 8N1。新固件需模式1且 data1.bit1=1 才上传。所有左右均为穿戴者视角。')
        note.setWordWrap(True)
        layout.addWidget(note)
        self.state = QtWidgets.QLabel('未连接')
        layout.addWidget(self.state)
        self.stats = QtWidgets.QLabel('等待数据')
        self.stats.setWordWrap(True)
        layout.addWidget(self.stats)
        self.flags = QtWidgets.QLabel('有效位：—')
        layout.addWidget(self.flags)
        self.tabs = QtWidgets.QTabWidget()
        layout.addWidget(self.tabs,1)
        curve_page = QtWidgets.QWidget()
        self.tabs.addTab(curve_page,'实时曲线')
        self.imu = ImuPanel()
        self.tabs.addTab(self.imu,'IMU 姿态')
        self.observation = obs.ObservationPanel()
        self.observation.open_file.clicked.connect(lambda:self.open_observation_file())
        self.observation.save_csv.clicked.connect(lambda:self.save_observation_file('csv'))
        self.observation.save_npz.clicked.connect(lambda:self.save_observation_file('npz'))
        self.tabs.addTab(self.observation,'网络观测 · 国际单位')
        self.tabs.setCurrentWidget(self.observation)
        self.protocol_choice.currentIndexChanged.connect(
            lambda index:self.tabs.setCurrentIndex(2 if index==0 else 0))
        layout = QtWidgets.QVBoxLayout(curve_page)
        view_bar = QtWidgets.QHBoxLayout()
        layout.addLayout(view_bar)
        view_bar.addWidget(QtWidgets.QLabel('历史帧数'))
        self.history_limit = QtWidgets.QDoubleSpinBox()
        self.history_limit.setDecimals(4)
        self.history_limit.setRange(.0001,10)
        self.history_limit.setSuffix(' W')
        self.history_limit.setValue(.6)
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
        self.plot.setLabel('bottom','设备累计时间',units='s')
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
        categories = {'关节反馈':KEYS[1:5], '机身姿态 / 角速度':KEYS[5:12],
                      '网络观测（11维）':KEYS[12:23], '网络输出 / 扭矩命令':KEYS[23:]}
        self.readouts,self.group_checks = {},{}
        self.checks,self.curves = {},{}
        for row,key in enumerate(KEYS[1:]):
            check = QtWidgets.QCheckBox(label(key))
            check.setChecked(key in ('joint_angle_norm_0','joint_angle_norm_1',
                                     'joint_speed_norm_0','joint_speed_norm_1'))
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
        self.groups.setCurrentIndex(2)
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
            self.legend.addItem(curve,label(key))
            curve.setData([h[0] for h in self.history],[h[2].values[key] if h[2].available(key) else np.nan for h in self.history])
        else:
            self.legend.removeItem(curve)

    def change_capacity(self, value):
        self.history = deque(self.history,maxlen=max(1,round(value*10000)))

    def change_span(self, value):
        if value*50 > self.history.maxlen:
            self.history_limit.setValue(np.ceil(value*50)/10000)
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
        self.observation.open_file.setEnabled(not yes)
        for widget in (self.ports,self.baud,self.sim,self.refresh,self.connect_button,self.protocol_choice):
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
        is_observation = self.protocol_choice.currentIndex()==0
        self.tabs.setCurrentIndex(2 if is_observation else 0)
        self.receiver = Receiver(self.ports.currentData(),baud,self.sim.isChecked(),is_observation)
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
        self.imu.invalidate('连接已断开')
        self.observation.render(stale=True)
        return True

    def toggle_pause(self):
        self.paused = not self.paused
        self.pause.setText('恢复显示' if self.paused else '暂停显示')

    def clear(self):
        self.history.clear()
        self.observation.reset()
        self.imu.reset()
        for curve in self.curves.values():
            curve.setData([],[])
        for readout in self.readouts.values():
            readout.setText('不可用')

    def open_observation_file(self, path=None):
        if self.receiver and self.receiver.isRunning():
            self.state.setText('请先断开串口，再读取离线文件')
            return
        if path is None:
            path,_=QtWidgets.QFileDialog.getOpenFileName(self,'读取模式1观测日志','',
                '二进制观测日志 (*.txt *.TXT *.bin *.BIN *.log *.LOG);;所有文件 (*)')
        if not path:return
        try:
            count=self.observation.load_binary(path)
            self.protocol_choice.setCurrentIndex(0)
            self.tabs.setCurrentWidget(self.observation)
            self.stats.setText(self.observation.file_status.text())
            self.flags.setText('离线文件：时间按50Hz换算，无法确认真实采样间隔或漏帧；扭矩为策略历史命令。')
            self.state.setText(f'已读取 {count} 帧：{path}')
            self.imu.invalidate('离线观测文件不包含姿态四元数')
        except (OSError,ValueError) as exc:
            self.state.setText(f'读取失败：{exc}')

    def save_observation_file(self, kind, path=None):
        if path is None:
            stem=Path(self.observation.file_source).stem if self.observation.file_source else 'mode1_observation'
            path,_=QtWidgets.QFileDialog.getSaveFileName(self,f'保存观测为 {kind.upper()}',
                f'{stem}.{kind}',f'{kind.upper()} (*.{kind})')
            if path and not Path(path).suffix:path+='.'+kind
        if not path:return
        try:
            count=self.observation.save_dataset(path)
            self.state.setText(f'已保存 {count} 帧：{path}')
        except (OSError,ValueError) as exc:
            self.state.setText(f'保存失败：{exc}')

    def toggle_record(self):
        if not self.receiver or not self.receiver.isRunning():
            return
        if self.receiver.record_path:
            self.receiver.commands.put(None)
        else:
            path,_ = QtWidgets.QFileDialog.getSaveFileName(self,'连续记录保存为','mode1_record.csv','CSV (*.csv)')
            if path:
                self.receiver.commands.put(path)

    def export(self, path=None):
        if self.observation.file_source:
            self.save_observation_file('csv',path if isinstance(path,str) else None)
            return
        if not path:
            path,_ = QtWidgets.QFileDialog.getSaveFileName(self,'导出历史缓存','mode1_cache.csv','CSV (*.csv)')
        if not path:
            return
        try:
            with open(path,'w',newline='',encoding='utf-8-sig') as out:
                writer = csv.writer(out)
                is_observation = self.receiver.observation if self.receiver else self.protocol_choice.currentIndex()==0
                history = self.observation.history if is_observation else self.history
                writer.writerow(obs.CSV_COLUMNS if is_observation else CSV_COLUMNS)
                writer.writerows((obs.csv_row(host,f) if is_observation else csv_row(host,f)) for _,host,f in history)
            self.state.setText(f'已导出 {len(history)} 帧：{path}')
        except OSError as exc:
            self.state.setText(f'导出失败：{exc}')

    def update_data(self):
        if self.observation.file_source:
            return
        rx = self.receiver
        if not rx:
            return
        batch = rx.take()
        if rx.observation:
            for host,f in batch:
                self.observation.append(host,f)
            if batch:self.last_rx=time.monotonic()
            stale=time.monotonic()-self.last_rx>1 or not rx.isRunning()
            self.stats.setText(f'52字节观测 · 接收 {rx.count} 帧 · 错帧候选 {rx.parser.bad} · 丢弃字节 {rx.parser.discarded} · UI队列溢出 {rx.overflow} · 已记录 {rx.recorded} · '+('等待数据/数据过期' if stale or not self.observation.history else '数据新鲜')+(' · 显示暂停，接收和记录继续' if self.paused else ''))
            self.flags.setText('新协议不含设备时间戳、序号和有效位；bit1开启上传，bit0独立控制扭矩许可。')
            self.imu.invalidate('52字节观测协议不包含姿态四元数')
            if not self.paused:self.observation.render(stale=stale)
            return
        for host,f in batch:
            self.history.append((self.timing.update(f),host,f))
        if batch:
            self.last_rx = time.monotonic()
        stale = time.monotonic()-self.last_rx > 1 or not rx.isRunning()
        if not self.history:
            self.stats.setText(f'等待模式1数据；普通模式无上传 · 收到帧 {rx.count} · 丢弃字节 {rx.parser.discarded}')
            self.imu.invalidate('等待有效 IMU 数据')
            return
        f = self.history[-1][2]
        self.stats.setText(f'状态：{STATUS.get(f.status,"未知")} ({f.status}) · 序号 {f.sequence} · 时间戳 {f.timestamp_us} μs · 周期 {f.values["dt_s"]*1000:.2f} ms · 计算 {f.compute_time_us} μs\n接收 {rx.count} · 序号缺失 {self.timing.lost} · 重复 {self.timing.duplicates} · 重启/乱序 {self.timing.resets} · UI队列溢出 {rx.overflow} · 错帧候选 {rx.parser.bad} · 丢弃字节 {rx.parser.discarded} · 已记录 {rx.recorded} · '+('数据过期/连接断开' if stale else '数据新鲜')+(' · 显示暂停，接收和记录继续' if self.paused else ''))
        names = ['左反馈','右反馈','IMU','观测','网络输出','最终命令']
        self.flags.setText(f'有效位 0x{f.valid_flags:02X}：'+' / '.join(name+(' ✓' if f.valid_flags & 1<<i else ' ×') for i,name in enumerate(names)))
        self.imu.set_frame(f,stale=stale,paused=self.paused)
        if self.paused:
            return
        for row,key in enumerate(KEYS[1:]):
            self.readouts[key].setText(f'{f.values[key]:+.4f}' if not stale and f.available(key) else '不可用')
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
        if self.disconnect_source():
            event.accept()
        else:
            event.ignore()

def create_application():
    app = QtWidgets.QApplication(sys.argv[:1])
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
            window.grab().save(str(Path(__file__).with_name('observation_preview.png')))
            window.export(str(Path(__file__).with_name('simulation_export.csv')))
            print(f'SMOKE: frames={len(window.observation.history)}, status={window.stats.text()}',flush=True)
            window.close()
        QtCore.QTimer.singleShot(3000,finish)
    sys.exit(app.exec())

if __name__ == '__main__':
    main()




