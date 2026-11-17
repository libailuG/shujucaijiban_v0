import csv,numpy as np
from pathlib import Path
p=Path('host/control_mode2_monitor/data/20261117_101118_643535.csv')
with p.open(encoding='utf-8-sig') as f:
 r=csv.DictReader(f); names=r.fieldnames; a=np.array([[float(row[k]) for k in names] for row in r])
d={k:a[:,i] for i,k in enumerate(names)};t=d['device_time_s']; flags=d['flags'].astype(int);gate=(flags&4)>0
print('rows',len(a),'time',t[0],t[-1],'statuses',np.unique(d['status'],return_counts=True),'flags',np.unique(flags,return_counts=True))
ix=np.r_[0,np.where(np.diff(gate.astype(int)))[0]+1,len(a)]
for s,e in zip(ix[:-1],ix[1:]):
 print('\nSEG',s,e,'t',round(t[s],3),round(t[e-1],3),'gate',gate[s])
 for k in ['left_angle_rad','right_angle_rad','left_speed_rad_s','right_speed_rad_s','left_calculated_torque_nm','right_calculated_torque_nm','left_driver_torque_nm','right_driver_torque_nm','compute_us','period_us']:
  v=d[k][s:e];print(k,'min/max',np.round([v.min(),v.max()],4),'rms',round(float(np.sqrt(np.mean(v*v))),4))
for j in ix[1:-1]:
 print('\nSWITCH',t[j]);
 for i in range(max(0,j-5),min(len(a),j+16)):
  print(i,round(t[i],4),int(flags[i]),*[round(d[k][i],4) for k in ['left_angle_rad','right_angle_rad','left_speed_rad_s','right_speed_rad_s','left_calculated_torque_nm','left_driver_torque_nm','right_driver_torque_nm']])
print('seq gaps',np.unique(np.diff(d['sequence']),return_counts=True),'session',np.unique(d['session']))
