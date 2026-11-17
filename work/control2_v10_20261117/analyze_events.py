exec(open('work/control2_v10_20261117/analyze_record.py',encoding='utf-8-sig').read().split("print('rows'")[0])
def runs(mask):
 ix=np.r_[0,np.where(np.diff(mask.astype(int)))[0]+1,len(mask)]
 return [(int(s),int(e)) for s,e in zip(ix[:-1],ix[1:]) if mask[s]]
print('STATUS RUNS')
for s,e in runs(d['status']!=0):print(round(t[s],4),round(t[e-1],4),e-s,np.unique(d['status'][s:e],return_counts=True),np.unique(flags[s:e]))
print('TORQUE EPISODES ON')
for s,e in runs(gate & (np.abs(d['left_driver_torque_nm'])>.05)):
 print(round(t[s],4),round(t[e-1],4),'N',e-s,'T range',d['left_driver_torque_nm'][s:e].min(),d['left_driver_torque_nm'][s:e].max())
 for i in range(max(0,s-3),min(len(a),e+5)):
  print(round(t[i],4),int(d['status'][i]),int(flags[i]),*[round(d[k][i],3) for k in ['left_angle_rad','right_angle_rad','left_speed_rad_s','right_speed_rad_s','left_driver_torque_nm']])
print('feedback failures ON',np.sum(gate &(d['status']==-1)))
print('nonzero commanded ON frames',np.sum(gate &(abs(d['left_driver_torque_nm'])>.05)))
print('max speed valid timestamp',[(k,float(t[np.argmax(abs(d[k]))]),float(d[k][np.argmax(abs(d[k]))])) for k in ['left_speed_rad_s','right_speed_rad_s']])
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig,axs=plt.subplots(4,1,figsize=(12,9),sharex=True)
x=t-t[0]
for ax,cols,unit in zip(axs[:3],[['left_angle_rad','right_angle_rad'],['left_speed_rad_s','right_speed_rad_s'],['left_calculated_torque_nm','left_driver_torque_nm']],['Angle (rad)','Speed (rad/s)','Torque (Nm)']):
 for k in cols:
  y=d[k].copy();y[(flags&1)==0]=np.nan;ax.plot(x,y,label=k,lw=1)
 ax.set_ylabel(unit);ax.legend(loc='upper right',fontsize=8);ax.grid(alpha=.2)
axs[3].step(x,gate.astype(int),label='torque permission');axs[3].step(x,d['status'],label='status');axs[3].set_ylabel('State');axs[3].legend();axs[3].grid(alpha=.2)
for ax in axs:
 for s,e in runs(gate):ax.axvspan(x[s],x[e-1],alpha=.07,color='orange')
axs[-1].set_xlabel('Seconds from first recorded frame')
fig.tight_layout();fig.savefig('work/control2_v10_20261117/record_analysis.png',dpi=140)
