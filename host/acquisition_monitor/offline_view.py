"""Offline viewer: file loading/exports run independently of serial reception."""
from pathlib import Path
import numpy as np
import pyqtgraph as pg
from PyQt6 import QtCore,QtWidgets
from protocol import KEYS,LABELS,CATEGORIES
from log_data import load_log

class FileJob(QtCore.QThread):
    result=QtCore.pyqtSignal(object)
    error=QtCore.pyqtSignal(str)
    progress=QtCore.pyqtSignal(int)
    def __init__(self,operation,parent=None):
        super().__init__(parent); self.operation=operation
    def run(self):
        try: self.result.emit(self.operation(self))
        except Exception as exc: self.error.emit(str(exc))

class OfflinePanel(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.dataset=None; self.job=None; self.curves={}; self.checks={}; self.readouts={}
        layout=QtWidgets.QVBoxLayout(self)
        bar=QtWidgets.QHBoxLayout(); layout.addLayout(bar)
        self.open_button=QtWidgets.QPushButton('打开日志文件…'); bar.addWidget(self.open_button)
        self.open_button.clicked.connect(self.choose_file)
        self.csv_button=QtWidgets.QPushButton('整文件导出 CSV…'); bar.addWidget(self.csv_button)
        self.csv_button.clicked.connect(lambda:self.choose_export('csv'))
        self.npz_button=QtWidgets.QPushButton('整文件导出 NPZ…'); bar.addWidget(self.npz_button)
        self.npz_button.clicked.connect(lambda:self.choose_export('npz'))
        self.full_button=QtWidgets.QPushButton('显示全部'); bar.addWidget(self.full_button)
        self.full_button.clicked.connect(self.show_all)
        bar.addStretch()
        self.progress=QtWidgets.QProgressBar(); self.progress.setMaximumWidth(150); self.progress.hide();bar.addWidget(self.progress)
        self.info=QtWidgets.QLabel('打开模式20的二进制TXT / BIN / LOG文件（80字节，带微秒时间戳）。无需连接串口。')
        self.info.setWordWrap(True);layout.addWidget(self.info)
        note=QtWidgets.QLabel('显示和导出原始上传值：角度rad、角速度rad/s；不重复修正方向或零点。图例线段可隐藏/显示，导出始终包含全部帧和17个通道。')
        note.setWordWrap(True);layout.addWidget(note)
        self.plot=pg.PlotWidget(background='#111b2b');self.plot.showGrid(x=True,y=True,alpha=.18)
        self.plot.setLabel('bottom','设备相对时间',units='s');self.plot.setLabel('left','幅值')
        axis=self.plot.getAxis('left')
        if hasattr(axis,'setSIPrefixEnableRanges'):axis.setSIPrefixEnableRanges(())
        axis.enableAutoSIPrefix(False)
        self.plot.setDownsampling(auto=True,mode='peak');self.plot.setClipToView(True)
        self.legend=self.plot.addLegend(offset=(12,12));layout.addWidget(self.plot,1)
        self.crosshair=[pg.InfiniteLine(angle=angle,pen=pg.mkPen('#a9bed6',style=QtCore.Qt.PenStyle.DashLine)) for angle in (90,0)]
        for line in self.crosshair:self.plot.addItem(line,ignoreBounds=True);line.hide()
        self.cursor=QtWidgets.QLabel('移动鼠标查看最近采样点；滚轮缩放、拖动平移。')
        layout.addWidget(self.cursor)
        self.plot.scene().sigMouseMoved.connect(self.mouse_moved)
        self.groups=QtWidgets.QTabWidget();layout.addWidget(self.groups)
        defaults={'left_motor_angle','right_motor_angle','left_thigh_angle','left_shank_angle','right_thigh_angle','right_shank_angle'}
        for name,keys in CATEGORIES.items():
            page=QtWidgets.QWidget();grid=QtWidgets.QGridLayout(page)
            select=QtWidgets.QPushButton('本组全选');clear=QtWidgets.QPushButton('本组取消')
            grid.addWidget(select,0,0);grid.addWidget(clear,0,1)
            select.clicked.connect(lambda _,ks=keys:self.select(ks,True))
            clear.clicked.connect(lambda _,ks=keys:self.select(ks,False))
            for j,key in enumerate(keys):
                check=QtWidgets.QCheckBox(LABELS[key]);check.setChecked(key in defaults)
                self.checks[key]=check;self.readouts[key]=QtWidgets.QLabel('—')
                color=('#45d5ff','#ffcb6b','#c792ea','#82d9a0','#ff8b94','#a5bfff')[KEYS.index(key)%6]
                check.setStyleSheet(f'color: {color};')
                self.curves[key]=self.plot.plot(pen=pg.mkPen(color,width=1.5),connect='finite')
                grid.addWidget(check,j//3+1,j%3*2);grid.addWidget(self.readouts[key],j//3+1,j%3*2+1)
                check.toggled.connect(lambda _,k=key:self.selection_changed(k))
                self.selection_changed(key)
            self.groups.addTab(page,name)
        self.set_busy(False)

    def select(self,keys,value):
        for key in keys:self.checks[key].setChecked(value)

    def selection_changed(self,key):
        curve=self.curves[key];selected=self.checks[key].isChecked();curve.setVisible(selected)
        if selected:
            if not any(sample.item is curve for sample,_ in self.legend.items):self.legend.addItem(curve,LABELS[key])
        else:self.legend.removeItem(curve)

    def set_busy(self,busy):
        self.open_button.setEnabled(not busy)
        for button in (self.csv_button,self.npz_button,self.full_button):button.setEnabled(not busy and self.dataset is not None)
        self.progress.setVisible(busy)

    def start_job(self,operation):
        if self.job is not None:return False
        self.set_busy(True);self.progress.setRange(0,0)
        self.job=FileJob(operation,self)
        self.job.result.connect(self.job_result);self.job.error.connect(self.job_error)
        self.job.progress.connect(self.show_progress);self.job.finished.connect(self.job_finished)
        self.job.start();return True

    @QtCore.pyqtSlot(int)
    def show_progress(self,value):self.progress.setRange(0,100);self.progress.setValue(value)

    @QtCore.pyqtSlot(object)
    def job_result(self,result):
        if isinstance(result,str):self.info.setText('已导出：'+result)
        else:self.set_dataset(result)

    @QtCore.pyqtSlot(str)
    def job_error(self,message):self.info.setText('文件操作失败：'+message+'；此前已加载的数据保留。')

    @QtCore.pyqtSlot()
    def job_finished(self):
        job=self.job;self.job=None;self.set_busy(False)
        if job:job.deleteLater()

    def choose_file(self):
        path,_=QtWidgets.QFileDialog.getOpenFileName(self,'打开模式20日志','','采集日志 (*.txt *.TXT *.bin *.BIN *.log *.LOG);;所有文件 (*)')
        if path:self.load_file(path)

    def load_file(self,path):
        self.info.setText('正在读取：'+str(path))
        return self.start_job(lambda job:load_log(path,job.progress.emit,job.isInterruptionRequested))

    def set_dataset(self,data):
        self.dataset=data
        for i,key in enumerate(KEYS):
            values=data.values[:,i].astype(float)
            values[~np.isfinite(values)]=np.nan
            self.curves[key].setData(data.time_s,values)
            self.selection_changed(key)
        duration=data.time_s[-1];rate=(len(data.time_s)-1)/duration if duration>0 else 0
        self.info.setText(f'{data.source}\n{len(data.time_s):,}帧 · {duration:.3f}秒 · 平均{rate:.2f}Hz · 时间段{int(data.segment_id[-1])+1} · 错帧候选{data.bad} · 跳过字节{data.discarded} · 尾部残留{data.trailing}字节')
        self.show_all();self.set_busy(self.job is not None)
        self.update_readouts(0)

    def show_all(self):
        if self.dataset is None:return
        for line in self.crosshair:line.hide()
        self.plot.enableAutoRange(axis='y',enable=True)
        self.plot.setXRange(0,max(.01,float(self.dataset.time_s[-1])),padding=.01)

    def mouse_moved(self,pos):
        if self.dataset is None:return
        if not self.plot.getViewBox().sceneBoundingRect().contains(pos):
            for line in self.crosshair:line.hide()
            return
        point=self.plot.getViewBox().mapSceneToView(pos)
        self.crosshair[0].setPos(point.x());self.crosshair[1].setPos(point.y())
        for line in self.crosshair:line.show()
        t=self.dataset.time_s;i=int(np.clip(np.searchsorted(t,point.x()),0,len(t)-1))
        if i and abs(t[i-1]-point.x())<abs(t[i]-point.x()):i-=1
        self.update_readouts(i)

    def update_readouts(self,i):
        d=self.dataset
        self.cursor.setText(f'采样点 {i+1} / {len(d.time_s)} · 时间 {d.time_s[i]:.6f}s · 原始时间戳 {int(d.timestamp_us[i])}μs · 时间段 {int(d.segment_id[i])+1}')
        for j,key in enumerate(KEYS):
            value=d.values[i,j]
            self.readouts[key].setText(str(int(value)) if key.endswith('pressure') else f'{value:+.5f}')

    def choose_export(self,kind):
        if self.dataset is None:return
        path,_=QtWidgets.QFileDialog.getSaveFileName(self,'整文件导出 '+kind.upper(),str(Path(self.dataset.source).with_suffix('.'+kind)),kind.upper()+' (*.'+kind+')')
        if path:self.export_file(path,kind)

    def export_file(self,path,kind):
        data=self.dataset
        if data is None:return False
        def operation(job):
            if kind=='csv':data.export_csv(path)
            elif kind=='npz':data.export_npz(path)
            else:raise ValueError('不支持的导出类型')
            return str(path)
        return self.start_job(operation)

    def shutdown(self):
        if self.job:
            self.job.requestInterruption()
            return self.job.wait(2000)
        return True

