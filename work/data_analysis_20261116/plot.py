from pathlib import Path
import sys,numpy as np
sys.path.insert(0,str(Path('host/acquisition_monitor').resolve()))
from protocol import KEYS
from PyQt6 import QtWidgets,QtGui
import pyqtgraph as pg
import pyqtgraph.exporters
app=QtWidgets.QApplication([])
QtGui.QFontDatabase.addApplicationFont('C:/Windows/Fonts/msyh.ttc')
app.setFont(QtGui.QFont('Microsoft YaHei',10))
z=np.load('work/data_analysis_20261116/data.npz');t,a,dt=z['t'],z['a'],z['dt']
w=pg.GraphicsLayoutWidget();w.resize(1500,1400);w.setBackground('w')
w.addLabel('LOG00126 · 1561 frames · 15.614 s · 99.91 Hz',color='#152a40',size='16pt');w.nextRow()
colors=['#0077aa','#d57800','#7749aa','#00956a','#e24c4c','#4c66aa']
for row,(ids,title,unit) in enumerate([([0,2,7,9,12,14],'Angles','deg'),([8,10,13,15],'Limb angular speeds','rad/s'),([6],'Body acceleration Z','m/s²'),([11,16],'Pressure contact code','code'),([], 'Frame interval','ms')]):
    p=w.addPlot(title=title);p.showGrid(x=True,y=True,alpha=.2);p.setLabel('left',unit);p.setLabel('bottom','Relative device time',units='s');p.getAxis('left').enableAutoSIPrefix(False)
    p.addLegend()
    for j,k in enumerate(ids):p.plot(t,np.rad2deg(a[:,k]) if row==0 else a[:,k],pen=pg.mkPen(colors[j],width=1.4),name=KEYS[k])
    if row==4:p.plot(t[1:],dt*1000,pen=pg.mkPen(colors[0],width=1))
    p.setXRange(0,t[-1],padding=0);w.nextRow()
w.show();app.processEvents()
export=pg.exporters.ImageExporter(w.scene());export.parameters()['width']=1800
export.export('work/data_analysis_20261116/overview.png')
w.close()
print('Saved overview.png')
