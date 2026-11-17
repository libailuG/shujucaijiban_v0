from pathlib import Path
import numpy as np
from PyQt6 import QtWidgets,QtGui
import pyqtgraph as pg
import pyqtgraph.exporters
root=Path(__file__).resolve().parents[1]
out=root/'work/control1_log_analysis_20261116'
z=np.load(out/'decoded.npz');x=z['values'];t=z['nominal_time_s']
app=QtWidgets.QApplication([]);app.setFont(QtGui.QFont('Microsoft YaHei',10))
pg.setConfigOptions(background='w',foreground='#182335',antialias=True)
w=pg.GraphicsLayoutWidget();w.resize(1500,1450)
w.addLabel('控制模式1 · LOG00128 · 4026帧 · 名义80.52秒',size='18pt',colspan=1);w.nextRow()
for cols,scale,title,unit,names in [
    ([0,1],180/np.pi,'关节角度（模型坐标）','°',['左','右']),
    ([2,3],1,'关节角速度','rad/s',['左','右']),
    ([4,5],1,'上一周期主动扭矩命令（非实测）','Nm',['左','右']),
    ([6,7],180/np.pi,'腰部倾角','°',['前后倾','左右倾']),
    ([8,9,10],1,'腰部模型角速度','rad/s',['X','Y','Z'])]:
    p=w.addPlot(title=title);p.addLegend(colCount=3);p.showGrid(x=True,y=True,alpha=.15)
    p.setLabel('left',unit);p.setLabel('bottom','名义时间（按50Hz换算）',units='s')
    p.setXRange(0,t[-1],padding=0)
    for col,name,color in zip(cols,names,['#1674b8','#e46b27','#268b63']):p.plot(t,x[:,col]*scale,pen=pg.mkPen(color,width=1),name=name)
    p.addItem(pg.InfiniteLine(pos=16.4,angle=90,pen=pg.mkPen('#bf4444',style=__import__('PyQt6.QtCore',fromlist=['Qt']).Qt.PenStyle.DashLine)))
    w.nextRow()
w.addLabel('红虚线：约16.4秒出现11帧连续零历史扭矩，疑似策略复位后重新暖机。无时间戳，无法测量停发时长。',size='10pt')
w.show();app.processEvents()
export=pg.exporters.ImageExporter(w.scene());export.parameters()['width']=1800;export.export(str(out/'overview.png'))
print(out/'overview.png')
mask=np.max(abs(x[:,4:6]),axis=1)<1e-10
edges=np.diff(np.r_[False,mask,False].astype(int))
print('zero runs:',[(int(a),int(b),a/50,b/50) for a,b in zip(np.flatnonzero(edges==1),np.flatnonzero(edges==-1))])
print('angle pair corr',np.corrcoef(x[:,0],x[:,1])[0,1])
for j in [0,1]:
    v=x[1000:3500,j];f=np.fft.rfftfreq(len(v),.02);s=abs(np.fft.rfft((v-v.mean())*np.hanning(len(v))))**2
    index=np.flatnonzero((f>.3)&(f<3));k=index[np.argmax(s[index])]
    print('nominal dominant Hz',j,f[k],'estimated steps/min if walking',f[k]*120)
