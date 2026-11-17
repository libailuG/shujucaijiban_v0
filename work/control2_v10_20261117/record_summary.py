exec(open('work/control2_v10_20261117/analyze_record.py',encoding='utf-8-sig').read().split("print('rows'")[0])
def ranges(m):
 z=np.r_[0,np.flatnonzero(np.diff(m.astype(int)))+1,len(m)]
 return [(s,e) for s,e in zip(z[:-1],z[1:]) if m[s]]
eps=ranges(gate & (abs(d['left_driver_torque_nm'])>.05));bad=ranges(d['status']==-1)
print('onset_rel',t[np.flatnonzero(gate)[0]]-t[0],'duration_on',sum(gate)*.01,'torque_episodes',len(eps),'bad_episodes',len(bad),'bad_samples',sum(d['status']==-1),'init_samples',sum(d['status']==1))
print('episode durations ms',[(e-s)*10 for s,e in eps]); print('compute_on p50/max',np.median(d['compute_us'][gate]),max(d['compute_us'][gate]));print('period allmax',max(d['period_us'])); print('pairmaxerror',max(abs(d['left_calculated_torque_nm']+d['right_calculated_torque_nm'])),'mapLerr',max(abs(d['left_driver_torque_nm'][gate]-d['left_calculated_torque_nm'][gate])),'mapRerr',max(abs(d['right_driver_torque_nm'][gate]+d['right_calculated_torque_nm'][gate])))
from PyQt6 import QtWidgets,QtGui
import pyqtgraph as pg
app=QtWidgets.QApplication([]);QtGui.QFontDatabase.addApplicationFont('C:/Windows/Fonts/msyh.ttc');app.setFont(QtGui.QFont('Microsoft YaHei',10));pg.setConfigOptions(background='w',foreground='k')
w=pg.GraphicsLayoutWidget();w.resize(1350,950)
x=t-t[0]
for row,(cols,title) in enumerate([(['left_angle_rad','right_angle_rad'],'Angle (rad)'),(['left_speed_rad_s','right_speed_rad_s'],'Speed (rad/s)'),(['left_calculated_torque_nm','left_driver_torque_nm'],'Calculated (blue) / commanded (red) torque, Nm'),(['status'],'Status / output permission (orange)')]):
 plot=w.addPlot(row=row,col=0);plot.setLabel('left',title);plot.showGrid(x=True,y=True);plot.addLegend()
 for col,color in zip(cols,['#1762bf','#db3a37']):
  v=d[col].copy()
  if row<3:v[(flags&1)==0]=np.nan
  plot.plot(x,v,pen=pg.mkPen(color,width=1.4),name=col,connect='finite')
 if row==3:plot.plot(x,gate.astype(int),pen=pg.mkPen('#d28a00',width=2),name='output permission')
 for s,e in ranges(gate):
  reg=pg.LinearRegionItem([x[s],x[e-1]],movable=False,brush=(255,170,0,20),pen=pg.mkPen(None));plot.addItem(reg);reg.setZValue(-10)
 plot.setXRange(0,x[-1],padding=.01)
 plot.setLabel('bottom','Seconds from start of CSV')
w.show();app.processEvents();w.grab().save('work/control2_v10_20261117/record_analysis.png');w.close()
