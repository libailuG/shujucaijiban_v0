"""Host parity/fault tests and actual v30 MuJoCo sensor/closed-loop checks."""
import ctypes as C
import hashlib,json,subprocess,sys,types
from pathlib import Path
import numpy as np
import torch
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from controller import Controller,map_hardware
from c_api import lib,State,Config,Input,Output,Workspace,F2,F3,F4,F65,PortConfig,HardwareInput,array,forward
lib.s2_controller_reset.argtypes=[C.POINTER(State)];lib.s2_config_default.argtypes=[C.POINTER(Config)]
lib.s2_selftest.argtypes=[C.POINTER(Workspace),C.POINTER(C.c_float)]
lib.s2_port_map_output.argtypes=[C.POINTER(PortConfig),C.POINTER(C.c_float),C.POINTER(C.c_float)]

def invoke(state,config,q,dq,quat,gyro,dt=.02,enabled=1,valid=1):
 i=Input(F2(*q),F2(*dq),F4(*quat),F3(*gyro),dt,enabled,valid);o=Output()
 status=lib.s2_controller_step(C.byref(state),C.byref(config),C.byref(i),C.byref(o))
 return status,o

def main():
 torch.set_num_threads(1);rng=np.random.default_rng(30);py=Controller();w=py.weights
 d=torch.load(ROOT/'weights/S2_v30_fp32.pt',weights_only=True,map_location='cpu')
 def reference(obs):
  x=torch.as_tensor(obs);x=(x-d['mean'])/(d['std']+d['epsilon'])
  for i in [0,2,4,6]:
   x=torch.nn.functional.linear(x,d['actor_state_dict'][f'{i}.weight'],d['actor_state_dict'][f'{i}.bias'])
   if i!=6:x=x.relu()
  return x.numpy()
 weight_bytes=0
 for k,a in w.items():
  c=np.ctypeslib.as_array((C.c_float*a.size).in_dll(lib,'s2_'+k))
  assert c.tobytes()==a.tobytes(),k
  weight_bytes+=a.nbytes
 x=rng.normal(0,.3,(1000,65)).astype(np.float32);ref=reference(x)
 c=np.stack([forward(row) for row in x]);p=py.forward(x)
 random_error=float(np.max(np.abs(c-ref)));numpy_error=float(np.max(np.abs(p-ref)))
 assert random_error<1e-4 and numpy_error<1e-4,(random_error,numpy_error)
 work=Workspace();error=C.c_float();assert lib.s2_selftest(C.byref(work),C.byref(error))==0
 state=State();cfg=Config();lib.s2_config_default(C.byref(cfg));assert cfg.gyro_mode==1 and cfg.torque_limit_nm==12
 seq=np.zeros(3);py.reset()
 for k in range(2000):
  t=k*.02;q=np.array([.4*np.sin(5*t),.4*np.sin(5*t+np.pi)],np.float32);dq=np.array([2*np.cos(5*t),2*np.cos(5*t+np.pi)],np.float32)
  quat=rng.normal(size=4).astype(np.float32);quat/=np.linalg.norm(quat);gyro=rng.normal(size=3).astype(np.float32)
  status,o=invoke(state,cfg,q,dq,quat,gyro);ps,cmd=py.step(q,dq,quat,gyro)
  assert status==ps
  seq=np.maximum(seq,[np.max(np.abs(array(o.obs)-py.obs)),np.max(np.abs(array(o.raw)-py.raw)),np.max(np.abs(array(o.command_nm)-cmd))])
  # Equal actual commands isolate observation construction from accumulated net rounding.
  assert lib.s2_controller_commit_command(C.byref(state),F2(*cmd))==0
 assert seq[0]<2e-6 and seq[1]<1e-4 and seq[2]<.002,seq
 mount_error=0.
 for k in range(100):
  qa=rng.normal(size=4).astype(np.float32);qa/=np.linalg.norm(qa);qb=rng.normal(size=4).astype(np.float32);qb/=np.linalg.norm(qb);g=rng.normal(size=3).astype(np.float32)
  port=PortConfig(F4(*qb),F2(-1,1),F2(.2,-.3),F2(1,-1),1);hw=HardwareInput(F2(.5,-.7),F2(2,-3),F4(*qa),F3(*g),.02,1,1);mapped=Input()
  assert lib.s2_port_map_input(C.byref(port),C.byref(hw),C.byref(mapped))==0
  expected=map_hardware([.5,-.7],[2,-3],qa,g,qb,[-1,1],[.2,-.3],[1,-1],True)
  for a,b in zip([mapped.hip_rad,mapped.hip_rad_s,mapped.quat_wb,mapped.gyro_body_rad_s],expected[:4]):mount_error=max(mount_error,float(np.max(np.abs(array(a)-b))))
 assert mount_error<2e-6
 # Check human-direction statements in header independently of motion samples.
 for axis,idx,sign in [(0,60,1),(1,61,1),(2,64,0)]:
  quat=np.zeros(4,np.float32);quat[0]=np.cos(.1);quat[axis+1]=np.sin(.1)
  _,o=invoke(State(),cfg,[0,0],[0,0],quat,[0,0,0])
  if sign:assert o.obs[idx]>0
 faults=[]
 for case,kw in [('nan',dict(q=[float('nan'),0])),('timing',dict(dt=.01)),('quaternion',dict(quat=[0,0,0,0])),('sensor',dict(valid=0)),('disable',dict(enabled=0))]:
  args=dict(q=[0,0],dq=[0,0],quat=[1,0,0,0],gyro=[0,0,0]);args.update(kw)
  state=State();state.previous_command[0]=5
  status,o=invoke(state,cfg,**args);assert status<0 or status==2
  assert not np.any(array(o.command_nm)) and not np.any(array(state.history)) and state.ticks==0
  faults.append(case)
 port=PortConfig();hw=HardwareInput();mapped=Input();assert lib.s2_port_map_input(C.byref(port),C.byref(hw),C.byref(mapped))==-1
 badcfg=Config(1,13,0,10);assert invoke(State(),badcfg,[0,0],[0,0],[1,0,0,0],[0,0,0])[0]==-2
 # Full original simulation provides the authoritative torch observation encoder.
 source=Path(sys.argv[1]).resolve();sys.path.insert(0,str(source))
 runtime=types.ModuleType('runtime_paths');runtime.ROOT=source;sys.modules['runtime_paths']=runtime
 from simulation import Simulation
 import mujoco
 from observations import unrotate
 sim=Simulation();state=State();py=Controller();motion_err=np.zeros(3);saved=[]
 def sensor(sim):
  f=sim.feedback;d=sim.data
  gyro=unrotate(f.tensor(d.xquat[f.waist])[None],f.tensor(d.cvel[f.waist,:3])[None])[0].numpy()
  return d.qpos[f.eq[:2]].astype(np.float32),d.qvel[f.ev[:2]].astype(np.float32),d.xquat[f.waist].astype(np.float32),gyro
 for k in range(1010):
  q,dq,quat,gyro=sensor(sim);status,o=invoke(state,cfg,q,dq,quat,gyro);ps,cmd=py.step(q,dq,quat,gyro)
  block,failure=sim.advance_control();assert not failure,failure
  obs=sim.last_s2_obs[0].numpy();raw=sim.last_raw[0].numpy()
  motion_err=np.maximum(motion_err,[np.max(np.abs(array(o.obs)-obs)),np.max(np.abs(array(o.raw)-raw)),np.max(np.abs(array(o.command_nm)-sim.last_active))])
  np.testing.assert_allclose(py.obs,obs,atol=2e-6,rtol=0)
  np.testing.assert_allclose(cmd,sim.last_active,atol=.002,rtol=0)
  lib.s2_controller_commit_command(C.byref(state),F2(*sim.last_active));py.commit_command(sim.last_active)
  saved.append(np.r_[q,dq,quat,gyro,obs,raw,sim.last_active])
 assert motion_err[0]<2e-6 and motion_err[1]<1e-4 and motion_err[2]<.002,motion_err
 np.savez_compressed(ROOT/'validation/mujoco_reference.npz',frames=np.stack(saved))
 # Independent second physical run: C constructs history/IMU and determines commands.
 sim=Simulation();state=State();max_torque=0.;last={}
 def c_obs(data):
  status,o=invoke(state,cfg,*sensor(sim));assert status in (0,1)
  last['raw']=array(o.raw);return sim.feedback.tensor(array(o.obs))[None]
 sim.feedback.s2_input=c_obs
 sim.s2=lambda obs:sim.feedback.tensor(last['raw'])[None]
 for _ in range(1010):
  block,failure=sim.advance_control();assert not failure,failure
  max_torque=max(max_torque,max(float(np.max(np.abs(r[4]))) for r in block))
 result=dict(passed=True,source_policy_sha256=hashlib.sha256((ROOT/'weights/S2_v30_fp32.pt').read_bytes()).hexdigest(),torch=torch.__version__,numpy=np.__version__,mujoco=mujoco.__version__,weight_bytes=weight_bytes,random_observations=1000,c_raw_max_error=random_error,numpy_raw_max_error=numpy_error,selftest_error=error.value,synthetic_frames=2000,synthetic_obs_raw_command_errors=seq.tolist(),mounting_samples=100,mounting_max_error=mount_error,fault_cases=faults+['uncalibrated','invalid_limit'],mujoco_reference_frames=1010,mujoco_obs_raw_command_errors=motion_err.tolist(),c_closed_loop_physics_steps=sim.physics_steps,c_closed_loop_seconds=sim.data.time,c_closed_loop_actual_torque_peak_Nm=max_torque,state_bytes=C.sizeof(State),workspace_bytes=C.sizeof(Workspace),hardware_validation=False)
 (ROOT/'validation/validation.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
if __name__=='__main__':main()
