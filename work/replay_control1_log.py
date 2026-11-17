from pathlib import Path
import re,json
import numpy as np
root=Path(__file__).resolve().parents[1]
out=root/'work/control1_log_analysis_20261116'
x=np.load(out/'decoded.npz')['values'].astype(np.float32)
source=(root/'firmware/App/control1_policy/s2_deploy.c').read_text(encoding='utf-8-sig')
def array(name):
    m=re.search(r'\b'+name+r'\s*\[[^]]*\]\s*=\s*\{(.*?)\};',source,re.S)
    return np.array([float(v.rstrip('fF')) for v in re.findall(r'[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?[fF]?',m[1])],dtype=np.float32)
scaled=x[:,:6]/np.array([np.pi,np.pi,4*np.pi,4*np.pi,40,40],dtype=np.float32)
obs=np.array([np.r_[scaled[i-9:i+1].reshape(-1),x[i,6:8]/np.pi,x[i,8:11]/(4*np.pi)] for i in range(9,len(x)-1)],dtype=np.float32)
a=(obs-array('s2_norm_mean'))/(array('s2_norm_std')+array('s2_norm_eps')[0])
for layer,ni,no in [(0,65,256),(2,256,128),(4,128,64),(6,64,2)]:
    a=a@array(f's2_actor_{layer}_weight').reshape(no,ni).T+array(f's2_actor_{layer}_bias')
    if layer!=6:a=np.maximum(a,0)
pred=np.clip(a,-1,1)*20
actual=x[10:,4:6]
err=pred-actual
result={
    'compared_frames':len(err),'method':'Reconstruct latest 10x6 history and current 5 IMU fields; replay current FP32 weights; compare predicted command with next recorded previous-command field.',
    'mae_nm':np.mean(abs(err),axis=0).tolist(),'max_abs_error_nm':np.max(abs(err),axis=0).tolist(),
    'p99_abs_error_nm':np.percentile(abs(err),99,axis=0).tolist(),
    'fraction_both_errors_below_0_01_nm':float(np.mean(np.max(abs(err),axis=1)<.01)),
    'largest_errors':[{'current_s_nominal':float((i+9)/50),'pred_nm':pred[i].tolist(),'next_recorded_previous_nm':actual[i].tolist(),'error_nm':err[i].tolist()} for i in np.argsort(np.max(abs(err),axis=1))[-8:][::-1]]}
(out/'replay.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(result,ensure_ascii=False,indent=2))
