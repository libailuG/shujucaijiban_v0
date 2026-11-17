from pathlib import Path
import sys,json
import numpy as np
sys.path.insert(0,str(Path('host/acquisition_monitor').resolve()))
from protocol import Parser,KEYS,csv_row,CSV_COLUMNS
import csv
p=Path('caiji_data/LOG00126.TXT'); raw=p.read_bytes(); parser=Parser(); frames=parser.feed(raw)
a=np.array([[f.values[k] for k in KEYS] for f in frames]); us=np.array([f.timestamp_us for f in frames],dtype=np.int64)
dt=((np.diff(us))%(2**32))/1e6; t=np.r_[0,np.cumsum(dt)]
r={'file':str(p),'bytes':len(raw),'frames':len(frames),'bad_candidates':parser.bad,'discarded_bytes':parser.discarded,'remaining_bytes':len(parser.buffer),'first_us':int(us[0]),'last_us':int(us[-1]),'duration_s':float(t[-1]),'rate_hz':float(1/np.mean(dt)),'dt_ms':dict(zip(['min','p50','p95','p99','max'],np.percentile(dt*1000,[0,50,95,99,100]).tolist())),'interval_over_15ms':int(sum(dt>.015)),'nonfinite':int((~np.isfinite(a)).sum()),'channels':{},'pressure':{},'angle_speed':{}}
for i,k in enumerate(KEYS):
    x=a[:,i]
    r['channels'][k]={'min':float(x.min()),'max':float(x.max()),'mean':float(x.mean()),'std':float(x.std()),'start':float(np.median(x[:100])),'end':float(np.median(x[-100:])),'zero_count':int(sum(x==0)),'max_step':float(np.max(np.abs(np.diff(x))))}
    if 'pressure' in k:
        vals,counts=np.unique(x,return_counts=True)
        changes=np.flatnonzero(np.diff(x)!=0)+1
        r['pressure'][k]={'counts':dict(zip(map(lambda z:str(int(z)),vals),map(int,counts))),'changes':int(len(changes)),'events':[[round(float(t[j]),3),int(x[j])] for j in changes]}
for base in ['left_motor','right_motor','left_thigh','left_shank','right_thigh','right_shank']:
    x=a[:,KEYS.index(base+'_angle')]; v=a[:,KEYS.index(base+'_speed')]
    # Central slope over ~50 ms smooths sensor quantization, compared with gyro/feedback speed.
    slope=(x[5:]-x[:-5])/(t[5:]-t[:-5]); speed=(v[5:]+v[:-5])/2
    r['angle_speed'][base]={'corr_50ms':float(np.corrcoef(slope,speed)[0,1]),'slope_rms':float(np.sqrt(np.mean(slope**2))),'speed_rms':float(np.sqrt(np.mean(speed**2))),'difference_rms':float(np.sqrt(np.mean((slope-speed)**2)))}
Path('work/data_analysis_20261116/stats.json').write_text(json.dumps(r,ensure_ascii=False,indent=2),encoding='utf-8')
with open('work/data_analysis_20261116/decoded.csv','w',newline='',encoding='utf-8-sig') as f:
    w=csv.writer(f); w.writerow(['relative_s','timestamp_us']+KEYS)
    w.writerows([float(ti),int(ui),*row] for ti,ui,row in zip(t,us,a))
print(json.dumps(r,ensure_ascii=False,indent=2))
np.savez('work/data_analysis_20261116/data.npz',t=t,a=a,dt=dt)
