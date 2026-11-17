from pathlib import Path
import numpy as np,sys
sys.path.insert(0,str(Path('host/acquisition_monitor').resolve()))
from protocol import KEYS
z=np.load('work/data_analysis_20261116/data.npz'); t,a,dt=z['t'],z['a'],z['dt']
for k in ['right_motor_angle','left_shank_speed']:
    x=a[:,KEYS.index(k)]
    ids=np.flatnonzero(x==0) if k=='right_motor_angle' else np.array([np.argmax(x),np.argmin(x),np.argmax(np.abs(np.diff(x)))+1])
    print(k)
    for j in ids:
        print('t',round(float(t[j]),4),'index',j,'neighbors',[(round(float(t[i]),4),*[round(float(a[i,KEYS.index(c)]),4) for c in (['right_motor_angle','right_motor_speed'] if k=='right_motor_angle' else ['left_shank_angle','left_shank_speed','body_accel_z'])]) for i in range(max(0,j-1),min(len(t),j+2))])
# Right encoder derivative correlation with zeros and their 50ms derivative windows excluded.
x=a[:,2];v=a[:,3];valid=(x!=0)
slope=(x[5:]-x[:-5])/(t[5:]-t[:-5]); speed=(v[5:]+v[:-5])/2
ok=np.array([valid[i:i+6].all() for i in range(len(valid)-5)])
print('right motor corr excluding zero windows',np.corrcoef(slope[ok],speed[ok])[0,1])
for k in [0,2,7,9,12,14]: print(KEYS[k],np.rad2deg(a[:,k]).min(),np.rad2deg(a[:,k]).max())
try:
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.font_manager import FontProperties
    font=FontProperties(fname='C:/Windows/Fonts/msyh.ttc');plt.rcParams['axes.unicode_minus']=False
    fig,axs=plt.subplots(5,1,figsize=(13,13),sharex=True,layout='constrained')
    for k in [0,2,7,9,12,14]: axs[0].plot(t,np.rad2deg(a[:,k]),label=KEYS[k],lw=1)
    axs[0].set_ylabel('Angle (deg)');axs[0].legend(ncol=3,fontsize=8)
    for k in [8,10,13,15]:axs[1].plot(t,a[:,k],label=KEYS[k],lw=1)
    axs[1].set_ylabel('Gyro (rad/s)');axs[1].legend(ncol=4,fontsize=8)
    axs[2].plot(t,a[:,6],label='Body accel Z',lw=1);axs[2].axhline(9.81,c='grey',ls='--');axs[2].set_ylabel('m/s²');axs[2].legend()
    for k in [11,16]:axs[3].step(t,a[:,k],where='post',label=KEYS[k],lw=1)
    axs[3].set_yticks([0,1,2,3]);axs[3].set_ylabel('Contact code');axs[3].legend()
    axs[4].plot(t[1:],dt*1000,lw=.6);axs[4].set_ylabel('Frame interval (ms)');axs[4].set_xlabel('Relative device time (s)')
    for ax in axs:ax.grid(alpha=.25); ax.ticklabel_format(axis='y',style='plain',useOffset=False)
    fig.suptitle('LOG00126 · 1561帧 · 15.614秒 · 99.91Hz',fontproperties=font,fontsize=15)
    fig.savefig('work/data_analysis_20261116/overview.png',dpi=150);print('PLOT OK')
except ImportError as exc:print('PLOT unavailable',exc)
