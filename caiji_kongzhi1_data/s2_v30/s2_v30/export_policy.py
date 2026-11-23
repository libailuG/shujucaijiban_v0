"""Re-export exact v30 FP32 constants and deterministic self-test vectors."""
from pathlib import Path
import hashlib,json
import numpy as np
import torch
ROOT=Path(__file__).resolve().parent
SHA='25981d75a0ee2f00ca352532d40fdbfc8f0131d56687a61d1034dcc52039d8a0'
def literal(x):
 s=format(float(x),'.9g')
 return s+('' if '.' in s or 'e' in s else '.0')+'f'
def main():
 p=ROOT/'weights/S2_v30_fp32.pt'; assert hashlib.sha256(p.read_bytes()).hexdigest()==SHA
 d=torch.load(p,weights_only=True,map_location='cpu')
 assert d['version']=='v30_stm32f405' and d['widths']==[65,256,128,64,2] and d['output_cap_Nm']==12
 arrays={'norm_mean':d['mean'].numpy().reshape(-1),'norm_std':d['std'].numpy().reshape(-1),'norm_eps':np.array([d['epsilon']],np.float32)}
 arrays.update({'actor_'+k.replace('.','_'):v.numpy() for k,v in d['actor_state_dict'].items()})
 code=['/* v30 FP32 constants; source SHA256 '+SHA+' */']
 for k,a in arrays.items():
  assert a.dtype==np.float32 and np.isfinite(a).all()
  code.append('const float s2_%s[%d] = {'%(k,a.size))
  flat=a.ravel()
  for i in range(0,a.size,8):code.append('    '+', '.join(literal(x) for x in flat[i:i+8])+',')
  code.append('};')
 target=ROOT/'c/s2_deploy.c'; s=target.read_text(); begin='/* ---- s2_weights.c implementation ---- */'; end='/* ---- s2_policy.c implementation ---- */'
 s=s[:s.index(begin)+len(begin)]+'\n'+'\n'.join(code)+'\n\n'+s[s.index(end):]
 obs=np.random.default_rng(30).normal(0,.2,(3,65)).astype(np.float32)
 x=torch.from_numpy(obs); x=(x-d['mean'])/(d['std']+d['epsilon'])
 for i in [0,2,4,6]:
  x=torch.nn.functional.linear(x,d['actor_state_dict'][f'{i}.weight'],d['actor_state_dict'][f'{i}.bias'])
  if i!=6:x=x.relu()
 tests='static const float test_obs[3][65] = {\n'+',\n'.join('{'+','.join(literal(v) for v in row)+'}' for row in obs)+'\n};\n'
 tests+='static const float test_raw[3][2] = {\n'+',\n'.join('{'+','.join(literal(v) for v in row)+'}' for row in x.numpy())+'\n};\n'
 begin='/* ---- s2_selftest_data.h implementation ---- */';end='/* ---- s2_selftest.c implementation ---- */'
 s=s[:s.index(begin)+len(begin)]+'\n'+tests+s[s.index(end):];target.write_text(s)
 np.savez(ROOT/'weights/actor_weights.npz',**arrays)
 (ROOT/'weights/manifest.json').write_text(json.dumps(dict(source_sha256=SHA,version=d['version'],widths=d['widths'],activation=d['activation'],epsilon=d['epsilon'],cap_Nm=12,arrays={k:list(a.shape) for k,a in arrays.items()},weight_bytes=sum(a.nbytes for a in arrays.values())),indent=2)+'\n')
if __name__=='__main__':main()
