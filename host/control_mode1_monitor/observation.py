"""52-byte SI observation protocol and flat display panel."""
import math
import struct
import csv
from pathlib import Path
import numpy as np
from collections import deque
from dataclasses import dataclass
from PyQt6 import QtWidgets
import pyqtgraph as pg

WIRE = struct.Struct('<I11fI')
KEYS = ['left_angle_rad','right_angle_rad','left_speed_rad_s','right_speed_rad_s',
        'previous_left_torque_nm','previous_right_torque_nm','forward_tilt_rad',
        'lateral_tilt_rad','waist_gyro_x_rad_s','waist_gyro_y_rad_s','waist_gyro_z_rad_s']
LABELS = ['左关节角度 rad','右关节角度 rad','左关节角速度 rad/s','右关节角速度 rad/s',
          '上一周期左主动扭矩命令 Nm','上一周期右主动扭矩命令 Nm',
          '前后倾斜角 rad','左右倾斜角 rad','腰部X轴角速度 rad/s','腰部Y轴角速度 rad/s','腰部Z轴角速度 rad/s']
CSV_COLUMNS = ['host_time'] + KEYS

@dataclass
class Observation:
    values: tuple

class Parser:
    def __init__(self):
        self.buffer=bytearray()
        self.bad=self.discarded=0

    def feed(self,data):
        self.buffer.extend(data)
        frames=[]
        while len(self.buffer)>=4:
            pos=self.buffer.find(b'\xaa'*4)
            if pos<0:
                count=len(self.buffer)-3
                self.discarded+=count
                del self.buffer[:count]
                break
            self.discarded+=pos
            del self.buffer[:pos]
            if len(self.buffer)<WIRE.size:
                break
            fields=WIRE.unpack_from(self.buffer)
            if fields[-1]!=0xEEEEEEEE or not all(math.isfinite(x) for x in fields[1:-1]):
                self.bad+=1
                self.discarded+=1
                del self.buffer[0]
                continue
            frames.append(Observation(fields[1:-1]))
            del self.buffer[:WIRE.size]
        return frames

def csv_row(host,frame):
    return [host,*frame.values]

def simulated_packet(n):
    t=n*.02
    return WIRE.pack(0xAAAAAAAA,*[math.sin(t+i)*(.5 if i<4 or i>5 else 10) for i in range(11)],0xEEEEEEEE)

class ObservationPanel(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.history=deque(maxlen=6000)
        self.origin=None
        self.file_source=None
        layout=QtWidgets.QVBoxLayout(self)
        note=QtWidgets.QLabel('52字节 · 11项观测 · 模型坐标 · 国际单位制 · data1.bit1开启上传。时间轴使用电脑接收时间；协议无设备时间戳、序号和有效位。')
        note.setWordWrap(True)
        layout.addWidget(note)
        file_bar=QtWidgets.QHBoxLayout()
        layout.addLayout(file_bar)
        self.open_file=QtWidgets.QPushButton('读取日志文件')
        self.save_csv=QtWidgets.QPushButton('保存为 CSV')
        self.save_npz=QtWidgets.QPushButton('保存为 NPZ')
        for button in (self.open_file,self.save_csv,self.save_npz):file_bar.addWidget(button)
        self.file_status=QtWidgets.QLabel('可读取52字节二进制 TXT / BIN / LOG；离线时间按50Hz换算')
        file_bar.addWidget(self.file_status,1)
        bar=QtWidgets.QHBoxLayout()
        layout.addLayout(bar)
        self.all=QtWidgets.QCheckBox('全选')
        self.all.setChecked(True)
        bar.addWidget(self.all)
        self.follow=QtWidgets.QCheckBox('跟随最新数据')
        self.follow.setChecked(True)
        bar.addWidget(self.follow)
        bar.addWidget(QtWidgets.QLabel('显示长度'))
        self.span=QtWidgets.QDoubleSpinBox()
        self.span.setRange(.1,120)
        self.span.setValue(10)
        self.span.setSuffix(' 秒')
        self.span.valueChanged.connect(self.change_span)
        bar.addWidget(self.span)
        bar.addStretch()
        self.plot=pg.PlotWidget(background='#111b2b')
        self.plot.showGrid(x=True,y=True,alpha=.18)
        self.plot.setLabel('bottom','电脑接收累计时间',units='s')
        self.plot.setLabel('left','幅值（各通道单位见名称）')
        axis=self.plot.getAxis('left')
        if hasattr(axis,'setSIPrefixEnableRanges'):axis.setSIPrefixEnableRanges(())
        axis.enableAutoSIPrefix(False)
        self.plot.addLegend(colCount=2)
        self.plot.setDownsampling(auto=True,mode='peak')
        self.plot.setClipToView(True)
        layout.addWidget(self.plot,1)
        grid=QtWidgets.QGridLayout()
        layout.addLayout(grid)
        self.checks=[];self.readouts=[];self.curves=[]
        for i,name in enumerate(LABELS):
            color=pg.mkColor(('#45d5ff','#ffcb6b','#c792ea','#82d9a0','#ff8b94',
                              '#a5bfff','#f6a6df','#b9e769','#ffad70','#7ce8d5','#e1c7ff')[i])
            check=QtWidgets.QCheckBox(name)
            check.setChecked(True)
            readout=QtWidgets.QLabel('不可用')
            curve=self.plot.plot(name=name,pen=pg.mkPen(color,width=1.5))
            check.setStyleSheet(f'color: {color.name()};')
            check.toggled.connect(lambda on,c=curve:c.setVisible(on))
            check.toggled.connect(lambda _:self.render())
            grid.addWidget(check,i//2,(i%2)*2)
            grid.addWidget(readout,i//2,(i%2)*2+1)
            self.checks.append(check);self.readouts.append(readout);self.curves.append(curve)
        self.all.clicked.connect(lambda on:[c.setChecked(on) for c in self.checks])
        for check in self.checks:check.toggled.connect(self.sync_all)

    def sync_all(self):
        count=sum(c.isChecked() for c in self.checks)
        self.all.setTristate(0<count<11)
        from PyQt6 import QtCore
        self.all.setCheckState(QtCore.Qt.CheckState.Checked if count==11 else QtCore.Qt.CheckState.PartiallyChecked if count else QtCore.Qt.CheckState.Unchecked)

    def change_span(self,value):
        if self.follow.isChecked():self.render()
        else:
            left=self.plot.viewRange()[0][0]
            self.plot.setXRange(left,left+value,padding=0)

    def append(self,host,frame):
        if self.origin is None:self.origin=host
        self.history.append((host-self.origin,host,frame))

    def render(self,stale=False):
        if not self.history:return
        for i,(readout,curve) in enumerate(zip(self.readouts,self.curves)):
            readout.setText('数据过期' if stale else f'{self.history[-1][2].values[i]:+.4f}')
            if self.checks[i].isChecked():
                curve.setData([h[0] for h in self.history],[h[2].values[i] for h in self.history])
        if self.follow.isChecked():
            end=max(self.history[-1][0],self.span.value())
            self.plot.setXRange(end-self.span.value(),end,padding=0)

    def reset(self):
        self.history=deque(maxlen=6000);self.origin=None;self.file_source=None
        self.file_status.setText('可读取52字节二进制 TXT / BIN / LOG；离线时间按50Hz换算')
        self.plot.setLabel('bottom','电脑接收累计时间',units='s')
        for curve in self.curves:curve.setData([],[])
        for readout in self.readouts:readout.setText('不可用')

    def load_binary(self,path):
        parser=Parser()
        frames=[]
        with open(path,'rb') as stream:
            for chunk in iter(lambda:stream.read(65536),b''):
                frames.extend(parser.feed(chunk))
        if not frames:raise ValueError('未找到有效52字节观测帧，请确认文件协议')
        self.reset()
        self.file_source=str(Path(path).resolve())
        self.history=deque(((i/50,None,f) for i,f in enumerate(frames)),maxlen=max(6000,len(frames)))
        self.follow.setChecked(False)
        self.plot.setLabel('bottom','名义时间（按50Hz换算，无设备时间戳）',units='s')
        self.render()
        self.span.setMaximum(max(120,len(frames)/50))
        self.span.setValue(max(.1,len(frames)/50))
        self.plot.setXRange(0,max(.02,(len(frames)-1)/50),padding=.01)
        self.file_status.setText(f'{Path(path).name} · {len(frames)}帧 · 名义{len(frames)/50:.2f}秒 · 跳过/尾部{parser.discarded+len(parser.buffer)}字节 · 错帧候选{parser.bad}')
        return len(frames)

    def save_dataset(self,path):
        if not self.history:raise ValueError('没有可保存的数据')
        rows=list(self.history)
        values=np.asarray([r[2].values for r in rows],dtype=np.float32)
        times=np.asarray([r[0] for r in rows],dtype=np.float64)
        host=np.asarray([np.nan if r[1] is None else r[1] for r in rows],dtype=np.float64)
        timebase='nominal_50hz' if self.file_source else 'host_receive_time'
        if Path(path).suffix.lower()=='.npz':
            np.savez_compressed(path,values=values,keys=np.asarray(KEYS),time_s=times,
                host_time=host,time_basis=np.asarray(timebase),
                source=np.asarray(self.file_source or 'serial_or_simulation'),
                nominal_sample_rate_hz=np.asarray(50.0))
        else:
            with open(path,'w',newline='',encoding='utf-8-sig') as stream:
                writer=csv.writer(stream)
                writer.writerow(['time_s','time_basis','host_time',*KEYS])
                for i,row in enumerate(values):
                    writer.writerow([times[i],timebase,'' if np.isnan(host[i]) else host[i],*row])
        return len(rows)
