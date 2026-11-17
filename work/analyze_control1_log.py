from pathlib import Path
import json
import struct
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
SOURCE=ROOT/'caiji_kongzhi1_data/LOG00128.TXT'
OUT=ROOT/'work/control1_log_analysis_20261116'
OUT.mkdir(exist_ok=True)
keys=['left_angle_rad','right_angle_rad','left_speed_rad_s','right_speed_rad_s',
      'previous_left_torque_nm','previous_right_torque_nm','forward_tilt_rad','lateral_tilt_rad',
      'waist_gyro_x_rad_s','waist_gyro_y_rad_s','waist_gyro_z_rad_s']
b=SOURCE.read_bytes(); packet=struct.Struct('<I11fI')
rows=[];offsets=[];i=0;bad=0;skipped=0
while i<len(b):
    pos=b.find(b'\xaa'*4,i)
    if pos<0:skipped+=len(b)-i;break
    skipped+=pos-i
    if len(b)-pos<52:skipped+=len(b)-pos;break
    f=packet.unpack_from(b,pos)
    if f[-1]!=0xEEEEEEEE or not np.isfinite(f[1:-1]).all():
        bad+=1;skipped+=1;i=pos+1;continue
    rows.append(f[1:-1]);offsets.append(pos);i=pos+52
x=np.array(rows);n=len(x);t=np.arange(n)/50
def runs(mask):
    edges=np.diff(np.r_[False,mask,False].astype(int));starts=np.flatnonzero(edges==1);ends=np.flatnonzero(edges==-1)
    return sorted([{'start_s_nominal':float(a/50),'end_s_nominal':float(z/50),'duration_s_nominal':float((z-a)/50)} for a,z in zip(starts,ends)],key=lambda r:-r['duration_s_nominal'])
stats={}
for j,k in enumerate(keys):
    v=x[:,j];stats[k]={name:float(val) for name,val in zip(['min','max','mean','std','rms','p01','p50','p99'],
        [v.min(),v.max(),v.mean(),v.std(),np.sqrt(np.mean(v*v)),*np.percentile(v,[1,50,99])])}
torque={}
for side,j in [('left',4),('right',5)]:
    v=x[:,j];d=np.diff(v)
    torque[side]={'fraction_abs_ge_19_99':float(np.mean(abs(v)>=19.99)),
        'fraction_abs_ge_6':float(np.mean(abs(v)>=6)), 'fraction_positive':float(np.mean(v>0)),
        'max_adjacent_change_nm':float(abs(d).max()),'max_change_at_s_nominal':float((np.argmax(abs(d))+1)/50),
        'saturation_runs':runs(abs(v)>=19.99)[:5],
        'power_proxy_mean_w':float(np.mean(v*x[:,j-2])),
        'power_proxy_abs_mean_w':float(np.mean(abs(v*x[:,j-2])))}
windows=[]
for a in range(0,n,500):
    v=x[a:a+500]
    windows.append({'start_s_nominal':a/50,'end_s_nominal':(a+len(v))/50,
        'angle_range_deg':np.rad2deg(np.ptp(v[:,:2],axis=0)).tolist(),
        'speed_rms':np.sqrt(np.mean(v[:,2:4]**2,axis=0)).tolist(),
        'torque_mean':v[:,4:6].mean(axis=0).tolist(),
        'torque_rms':np.sqrt(np.mean(v[:,4:6]**2,axis=0)).tolist(),
        'tilt_mean_deg':np.rad2deg(v[:,6:8].mean(axis=0)).tolist()})
correlation=np.corrcoef(x,rowvar=False)
# Consistency check assumes continuous nominal sampling, not verified device timing.
velocity_check=[]
for j in range(2):
    measured=x[1:-1,2+j];derived=(x[2:,j]-x[:-2,j])/.04
    velocity_check.append({'correlation':float(np.corrcoef(measured,derived)[0,1]),
        'derived_rms':float(np.sqrt(np.mean(derived**2))), 'reported_rms':float(np.sqrt(np.mean(measured**2)))})
result={'source':str(SOURCE),'byte_count':len(b),'frames':n,'skipped_bytes':skipped,'bad_candidates':bad,
        'first_frame_offset':offsets[0],'last_frame_offset':offsets[-1],
        'noncontiguous_frame_gaps':int(np.sum(np.diff(offsets)!=52)),
        'nominal_duration_s':n/50,'time_warning':'No timestamps or sequence numbers; 50Hz is assumed, gaps cannot be measured.',
        'stats':stats,'torque':torque,'windows':windows,'velocity_consistency_nominal':velocity_check,
        'correlations':{keys[j]:{keys[k]:float(correlation[j,k]) for k in range(11)} for j in range(11)}}
(OUT/'analysis.json').write_text(json.dumps(result,indent=2,ensure_ascii=False),encoding='utf-8')
np.savez_compressed(OUT/'decoded.npz',values=x,keys=keys,nominal_time_s=t)
print(json.dumps({k:v for k,v in result.items() if k!='correlations'},ensure_ascii=False,indent=2))

try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams['font.sans-serif']=['Microsoft YaHei']
    plt.rcParams['axes.unicode_minus']=False
    fig,axes=plt.subplots(5,1,figsize=(14,13),sharex=True,constrained_layout=True)
    for ax,cols,scale,title,unit,names in [
        (axes[0],[0,1],180/np.pi,'关节角度（模型坐标）','°',['左','右']),
        (axes[1],[2,3],1,'关节角速度','rad/s',['左','右']),
        (axes[2],[4,5],1,'上一周期主动扭矩命令（非实测）','Nm',['左','右']),
        (axes[3],[6,7],180/np.pi,'腰部倾角','°',['前后倾','左右倾']),
        (axes[4],[8,9,10],1,'腰部模型角速度','rad/s',['X','Y','Z'])]:
        for col,name in zip(cols,names):ax.plot(t,x[:,col]*scale,label=name,lw=.8)
        ax.set_title(title,loc='left',fontsize=11);ax.set_ylabel(unit);ax.grid(alpha=.2);ax.legend(loc='upper right',ncols=3)
    axes[2].axhline(20,color='red',ls='--',alpha=.4);axes[2].axhline(-20,color='red',ls='--',alpha=.4)
    axes[-1].set_xlabel('名义时间 / s（按50Hz换算，无时间戳，无法确认真实时长）')
    fig.suptitle(f'控制模式1 · LOG00128 · {n}帧 / 名义{n/50:.2f}s',fontsize=15)
    fig.savefig(OUT/'overview.png',dpi=150)
    plt.close(fig)
    print('PLOT:',OUT/'overview.png')
except ImportError:print('matplotlib unavailable')
