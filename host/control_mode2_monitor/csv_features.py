"""Continuous mode2 recording and independent CSV review."""
import csv
from datetime import datetime, timezone, timedelta
from pathlib import Path
import numpy as np
from PyQt6 import QtCore, QtWidgets as Q
import pyqtgraph as pg
from protocol import CSV_COLUMNS, csv_row, KEYS

ROOT = Path(__file__).resolve().parent
HEADER = list(CSV_COLUMNS)
PLOT_KEYS = set(KEYS[:6])
CHANNEL_LABELS = dict(zip(KEYS[:6], ('左角度 rad', '右角度 rad', '左角速度 rad/s',
    '右角速度 rad/s', '左计算力矩 Nm', '右计算力矩 Nm')))


class CsvRecorder:
    def __init__(self):
        self.stream = None
        self.path = None
        self.rows = 0

    def start(self):
        if self.stream:
            return self.path
        directory = ROOT / 'data'
        directory.mkdir(exist_ok=True)
        stamp = datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%d_%H%M%S_%f')
        path = directory / f'{stamp}.csv'
        stream = path.open('x', newline='', encoding='utf-8-sig')
        try:
            writer = csv.writer(stream)
            writer.writerow(HEADER)
            stream.flush()
        except Exception:
            stream.close()
            raise
        self.stream, self.writer, self.path, self.rows = stream, writer, path, 0
        return path

    def write(self, host_time, frame):
        if self.stream:
            self.writer.writerow(csv_row(host_time, frame))
            self.rows += 1

    def flush(self):
        if self.stream:
            self.stream.flush()

    def stop(self):
        stream, self.stream = self.stream, None
        if stream:
            stream.close()


class CsvLoader(QtCore.QThread):
    loaded = QtCore.pyqtSignal(object, object, str)
    failed = QtCore.pyqtSignal(str)

    def __init__(self, path):
        super().__init__()
        self.path = Path(path)

    def run(self):
        try:
            blocks, batch = [], []
            with self.path.open(encoding='utf-8-sig', newline='') as stream:
                reader = csv.reader(stream)
                header = next(reader, [])
                if len(header) < 2 or len(set(header)) != len(header) or not all(header):
                    raise ValueError('需要至少两列且名称不重复的 CSV 表头。')
                for row in reader:
                    if not row:
                        continue
                    try:
                        values = [float(v) if v else np.nan for v in row]
                        valid = [np.isfinite(v) or (name == 'host_time' and np.isnan(v))
                                 for name, v in zip(header, values)]
                        if len(values) != len(header) or not all(valid):
                            raise ValueError('列数不匹配或包含非有限值')
                    except ValueError as exc:
                        raise ValueError(f'第 {reader.line_num} 行无效：{exc}') from exc
                    batch.append(values)
                    if len(batch) >= 10000:
                        blocks.append(np.asarray(batch))
                        batch = []
            if batch:
                blocks.append(np.asarray(batch))
            if not blocks:
                raise ValueError('CSV 没有数据行。')
            self.loaded.emit(header, np.concatenate(blocks), str(self.path))
        except Exception as exc:
            self.failed.emit(str(exc))


class CsvReview(Q.QWidget):
    COLORS = ['#45d5ff', '#ffcb6b', '#c792ea', '#82dca0', '#ff7d92', '#72aaff', '#ffab70', '#e4d97a', '#a9bed6']

    def __init__(self):
        super().__init__()
        self.loader = None
        self.data = np.empty((0, 0))
        self.x = np.empty(0)
        self.curves, self.checks = [], []
        layout = Q.QVBoxLayout(self)
        layout.setContentsMargins(22, 18, 22, 16)
        title = Q.QLabel('CSV 历史复盘')
        title.setStyleSheet('font-size:24px; font-weight:600')
        layout.addWidget(title)
        row = Q.QHBoxLayout()
        layout.addLayout(row)
        self.button(row, '打开 CSV', self.open_file)
        self.files = Q.QComboBox()
        row.addWidget(self.files, 1)
        self.button(row, '刷新列表', self.scan)
        self.button(row, '打开所选', self.open_selected)
        self.info = Q.QLabel('选择已保存的 CSV，不影响实时接收和保存；默认同图显示六条主曲线。')
        self.info.setWordWrap(True)
        layout.addWidget(self.info)
        row = Q.QHBoxLayout()
        layout.addLayout(row)
        self.button(row, 'All · 所选曲线全览', self.fit)
        for name, axis, factor in [('X−', 0, 1.5), ('X+', 0, 1/1.5), ('Y−', 1, 1.5), ('Y+', 1, 1/1.5)]:
            self.button(row, name, lambda _, a=axis, f=factor: self.scale(a, f))
        self.button(row, '全选', lambda: self.select(True))
        self.button(row, '全隐藏', lambda: self.select(False))
        row.addStretch()
        self.plot = pg.PlotWidget(background='#111b2b')
        self.plot.showGrid(x=True, y=True, alpha=.18)
        self.legend = self.plot.addLegend(offset=(12, 12), colCount=2)
        self.plot.setDownsampling(auto=True, mode='peak')
        self.plot.setClipToView(True)
        self.plot.getAxis('left').enableAutoSIPrefix(False)
        self.plot.enableAutoRange(x=False, y=False)
        self.plot.addItem(pg.InfiniteLine(pos=0, angle=0, pen=pg.mkPen('#c1cfdf', width=2.5)), ignoreBounds=True)
        self.cross = [pg.InfiniteLine(angle=a, pen=pg.mkPen('#a9bed6', style=QtCore.Qt.PenStyle.DashLine)) for a in (90, 0)]
        for line in self.cross:
            self.plot.addItem(line, ignoreBounds=True)
            line.hide()
        self.proxy = pg.SignalProxy(self.plot.scene().sigMouseMoved, rateLimit=30, slot=self.cursor_move)
        layout.addWidget(self.plot, 1)
        self.cursor = Q.QLabel('光标：移入绘图区查看坐标和最近帧数值')
        self.cursor.setWordWrap(True)
        layout.addWidget(self.cursor)
        self.channels = Q.QGridLayout()
        layout.addLayout(self.channels)
        layout.addWidget(Q.QLabel('滚轮缩放，拖动平移；轴上滚轮可独立缩放。All 仅适配当前选中的曲线。'))
        self.scan()

    def button(self, layout, text, slot):
        b = Q.QPushButton(text)
        b.clicked.connect(slot)
        layout.addWidget(b)

    def scan(self):
        selected = self.files.currentData()
        self.files.clear()
        directory = ROOT / 'data'
        if directory.exists():
            paths = [p for p in directory.rglob('*') if p.is_file() and p.suffix.lower() == '.csv']
            for path in sorted(paths, key=lambda p: p.stat().st_mtime, reverse=True):
                self.files.addItem(str(path.relative_to(directory)), str(path))
        i = self.files.findData(selected)
        if i >= 0:
            self.files.setCurrentIndex(i)

    def open_file(self):
        path, _ = Q.QFileDialog.getOpenFileName(self, '打开 CSV', str(ROOT / 'data'), 'CSV (*.csv *.CSV)')
        if path:
            self.load(path)

    def open_selected(self):
        if self.files.currentData():
            self.load(self.files.currentData())

    def load(self, path):
        if self.loader and self.loader.isRunning():
            return
        self.loader = CsvLoader(path)
        self.loader.loaded.connect(self.show_data)
        self.loader.failed.connect(self.failed)
        self.info.setText('后台读取 CSV…')
        self.loader.start()

    def failed(self, message):
        self.info.setText(f'CSV 读取失败：{message}')
        Q.QMessageBox.warning(self, 'CSV 读取失败', message)

    def show_data(self, header, data, path):
        self.legend.clear()
        for curve in self.curves:
            self.plot.removeItem(curve)
        while self.channels.count():
            self.channels.takeAt(0).widget().deleteLater()
        self.curves, self.checks = [], []
        self.data = data
        self.header = header
        time_column = next((header.index(k) for k in ('device_time_s', 'time_s') if k in header), None)
        self.x = data[:, time_column] if time_column is not None else np.arange(1, len(data)+1, dtype=float)
        # A clear/reset during recording can reset time; preserve file order via frame numbers.
        monotonic = np.all(np.diff(self.x) > 0)
        if not monotonic:
            self.x = np.arange(1, len(data)+1, dtype=float)
        self.plot.setLabel('bottom', '采集时间 (s)' if time_column is not None and monotonic else '文件帧序号')
        for i, name in enumerate(header):
            if i == time_column:
                continue
            color_index = KEYS.index(name) if name in PLOT_KEYS else len(self.curves)
            color = self.COLORS[color_index % len(self.COLORS)]
            y = data[:, i].copy()
            if name in PLOT_KEYS and 'flags' in header:
                bit = 1 if name in KEYS[:4] else 2
                y[(data[:, header.index('flags')].astype(np.uint32) & bit) == 0] = np.nan
            label = CHANNEL_LABELS.get(name, name)
            curve = self.plot.plot(self.x, y, pen=pg.mkPen(color, width=1.5), connect='finite')
            check = Q.QCheckBox(label)
            default = name in PLOT_KEYS if PLOT_KEYS.intersection(header) else True
            check.setChecked(default)
            curve.setVisible(default)
            if default:
                self.legend.addItem(curve, label)
            check.setStyleSheet(f'color:{color}')
            check.toggled.connect(lambda on, c=curve, n=label: self.toggle_channel(c, n, on))
            self.channels.addWidget(check, len(self.checks)//5, len(self.checks)%5)
            self.curves.append(curve)
            self.checks.append(check)
        self.info.setText(f'{path} | {len(data):,} 帧' + (' | 时间不递增，按文件帧序号显示' if not monotonic else ''))
        self.fit()

    def toggle_channel(self, curve, name, checked):
        curve.setVisible(checked)
        if checked:
            self.legend.addItem(curve, name)
        else:
            self.legend.removeItem(curve)

    def select(self, checked):
        for check in self.checks:
            check.setChecked(checked)

    def fit(self):
        if not len(self.x):
            return
        self.plot.setXRange(float(self.x[0]), max(float(self.x[-1]), float(self.x[0])+1e-6), padding=.02)
        selected = [curve.yData[np.isfinite(curve.yData)] for curve, check in zip(self.curves, self.checks) if check.isChecked()]
        selected = [y for y in selected if len(y)]
        if selected:
            lo, hi = min(float(np.min(y)) for y in selected), max(float(np.max(y)) for y in selected)
            pad = max((hi-lo)*.08, abs(hi)*.01, .001)
            self.plot.setYRange(lo-pad, hi+pad, padding=0)

    def scale(self, axis, factor):
        lo, hi = self.plot.viewRange()[axis]
        mid, half = (lo+hi)/2, (hi-lo)*factor/2
        setter = self.plot.setXRange if axis == 0 else self.plot.setYRange
        setter(mid-half, mid+half, padding=0)

    def cursor_move(self, event):
        view = self.plot.getViewBox()
        if not view.sceneBoundingRect().contains(event[0]):
            for line in self.cross:
                line.hide()
            return
        p = view.mapSceneToView(event[0])
        for line, value in zip(self.cross, [p.x(), p.y()]):
            line.setPos(value)
            line.show()
        text = f'X={p.x():.5g}   Y={p.y():+.5g}'
        if len(self.x):
            i = int(np.clip(np.searchsorted(self.x, p.x()), 0, len(self.x)-1))
            if i and abs(self.x[i-1]-p.x()) < abs(self.x[i]-p.x()):
                i -= 1
            text += ' | ' + '  '.join(f'{name}={v:.6g}' for name, v in zip(self.header, self.data[i]))
        self.cursor.setText(text)
