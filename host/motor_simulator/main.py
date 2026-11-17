import sys
from datetime import datetime
from PyQt6 import QtCore, QtGui, QtWidgets
from serial.tools import list_ports
from protocol import Settings, NAMES, MODES, FRAME, pack, respond
from worker import Worker

class Channel(QtWidgets.QGroupBox):
    def __init__(self, name, motor_id, owner):
        super().__init__(name)
        self.owner = owner
        self.worker = None
        self.layout_ = QtWidgets.QVBoxLayout(self)
        row = QtWidgets.QHBoxLayout()
        self.port = QtWidgets.QComboBox()
        self.port.setMinimumWidth(170)
        self.baud = QtWidgets.QComboBox()
        self.baud.addItems(['2000000', '1000000', '921600', '3000000', '115200'])
        self.button = QtWidgets.QPushButton('打开串口')
        self.button.clicked.connect(self.toggle)
        for widget in (self.port, self.baud, self.button): row.addWidget(widget)
        self.layout_.addLayout(row)
        form = QtWidgets.QFormLayout()
        self.id = QtWidgets.QLineEdit(str(motor_id))
        form.addRow('模拟 ID (uint32)', self.id)
        self.fields = {}
        labels = ['torque / N·m', 'speed / rad/s', 'ang / rad', '板温 / ℃', '电机温度 / ℃',
                  'KP / 电流系数', 'KD / 速度Kd', 'error_flag / uint32',
                  '1004 磁编位置 / 单位范围待定', '1005 扭矩系数', '1005 电流环Kp',
                  '1005 电流环Ki', '1005 电流环Kd']
        for key, label, value in zip((*NAMES, 'error', 'magnet', 'tk', 'ikp', 'iki', 'ikd'),
                                     labels, [0,0,0,25,25,0,0,0,0,0,0,0,0]):
            edit = QtWidgets.QLineEdit(str(value))
            self.fields[key] = edit
            form.addRow(label, edit)
        self.layout_.addLayout(form)
        apply = QtWidgets.QPushButton('应用反馈参数（打开期间也可修改）')
        apply.clicked.connect(self.apply)
        self.layout_.addWidget(apply)
        self.status = QtWidgets.QLabel('离线：未打开串口')
        self.stats = QtWidgets.QLabel('RX=0  TX=0  错误=0')
        self.latest = QtWidgets.QLabel('尚无命令')
        self.latest.setWordWrap(True)
        for widget in (self.status, self.stats, self.latest): self.layout_.addWidget(widget)
        self.mode = QtWidgets.QComboBox()
        for mode, label in MODES.items(): self.mode.addItem(f'{mode} {label}', mode)
        self.layout_.addWidget(self.mode)
        test = QtWidgets.QPushButton('离线注入一帧（仅内存，不发送串口）')
        test.clicked.connect(self.offline)
        self.layout_.addWidget(test)

    def settings(self):
        import math
        def u32(text):
            value = int(text, 0)
            if not 0 <= value <= 0xffffffff: raise ValueError('整数必须在uint32范围内')
            return value
        values = [float(self.fields[k].text()) for k in (*NAMES, 'magnet', 'tk', 'ikp', 'iki', 'ikd')]
        if not all(math.isfinite(v) for v in values): raise ValueError('浮点参数必须有限')
        cfg = Settings(u32(self.id.text()), values[:7], u32(self.fields['error'].text()), values[7], values[8:])
        # Validate float32 representability before the worker uses it.
        pack(cfg.motor_id, values=cfg.values, error=cfg.error)
        pack(cfg.motor_id, values=[cfg.magnet, *cfg.current, 0., 0.])
        return cfg

    def apply(self):
        try:
            cfg = self.settings()
            if self.worker: self.worker.configure(cfg)
            self.owner.log(f'{self.title()} 参数已应用；不代表真实设备状态')
            return cfg
        except (ValueError, OverflowError) as exc:
            QtWidgets.QMessageBox.warning(self, '参数无效', str(exc))

    def toggle(self):
        if self.worker and self.worker.isRunning():
            self.worker.stop()
            self.button.setEnabled(False)
            return
        cfg = self.apply()
        port = self.port.currentData()
        if cfg is None or not port: return
        for channel in self.owner.channels:
            if channel is not self and channel.worker and channel.worker.isRunning() and channel.worker.port.casefold() == port.casefold():
                QtWidgets.QMessageBox.warning(self, '串口已占用', '左右两路请选择不同串口')
                return
        self.worker = Worker(port, int(self.baud.currentText()), cfg)
        self.port.setEnabled(False)
        self.baud.setEnabled(False)
        self.button.setText('关闭串口')
        self.worker.start()

    def offline(self):
        cfg = self.apply()
        if cfg is None: return
        request = pack(cfg.motor_id, self.mode.currentData())
        reply = respond(FRAME.unpack(request), cfg)
        self.owner.log(f'{self.title()} 内存测试 RX={request.hex(" ")} TX={reply.hex(" ")}')

    def poll(self):
        if not self.worker: return
        stats, status, latest, events = self.worker.snapshot()
        self.status.setText(status)
        self.stats.setText('  '.join(f'{k}={v}' for k,v in stats.items()))
        self.stats.setWordWrap(True)
        self.latest.setText('最近命令 (ID, mode, 7 floats, error)：'+latest)
        for event in events: self.owner.log(self.title()+' '+event)
        if not self.worker.isRunning():
            self.port.setEnabled(True)
            self.baud.setEnabled(True)
            self.button.setText('打开串口')
            self.button.setEnabled(True)

class Window(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        QtGui.QFontDatabase.addApplicationFont('C:/Windows/Fonts/msyh.ttc')
        QtWidgets.QApplication.instance().setFont(QtGui.QFont('Microsoft YaHei UI', 9))
        self.setWindowTitle('电机串口模拟器 · 默认离线 · 48字节 <III7fII')
        root = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(root)
        note = QtWidgets.QLabel('仅响应已打开串口上的匹配ID请求；默认2Mbps / 8N1 / 无流控。\n'
            '固定反馈：无动力学；特殊模式回显mode仅供观察，不代表执行成功；改ID/校准/写参不改变状态。')
        note.setWordWrap(True)
        layout.addWidget(note)
        refresh = QtWidgets.QPushButton('刷新串口列表（不打开）')
        refresh.clicked.connect(self.refresh)
        layout.addWidget(refresh)
        row = QtWidgets.QHBoxLayout()
        self.channels = [Channel('左 ID126 · JP3 USART3', 126, self), Channel('右 ID127 · JP2 USART6', 127, self)]
        for channel in self.channels: row.addWidget(channel)
        layout.addLayout(row)
        self.logs = QtWidgets.QPlainTextEdit()
        self.logs.setReadOnly(True)
        self.logs.document().setMaximumBlockCount(1000)
        self.logs.setMaximumHeight(170)
        layout.addWidget(self.logs)
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(root)
        self.setCentralWidget(scroll)
        self.resize(1160, 960)
        self.refresh()
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(lambda: [c.poll() for c in self.channels])
        self.timer.start(100)

    def log(self, message):
        self.logs.appendPlainText(datetime.now().strftime('%H:%M:%S.%f')[:-3]+' '+message)

    def refresh(self):
        ports = list(list_ports.comports())
        for channel in self.channels:
            if channel.worker and channel.worker.isRunning(): continue
            selected = channel.port.currentData()
            channel.port.clear()
            channel.port.addItem('请选择串口（离线）', None)
            for port in ports: channel.port.addItem(f'{port.device} — {port.description}', port.device)
            index = channel.port.findData(selected)
            channel.port.setCurrentIndex(max(0, index))

    def closeEvent(self, event):
        for c in self.channels:
            if c.worker: c.worker.stop()
        # Never destroy a running QThread. Writes have a bounded timeout.
        done = all(c.worker is None or c.worker.wait(1000) for c in self.channels)
        if done: event.accept()
        else:
            self.log('等待串口线程退出，请稍后再次关闭窗口')
            event.ignore()

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    window = Window()
    window.show()
    sys.exit(app.exec())


