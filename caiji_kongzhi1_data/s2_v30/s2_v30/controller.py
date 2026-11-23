"""Independent NumPy float32 S2 v30 deployment, no simulation/task imports.
Model coordinates and all 65 obs are documented in c/s2_deploy.h.
"""
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
PI=np.float32(np.pi)
def multiply(a,b):
 w,x,y,z=a;v,i,j,k=b
 return np.array([w*v-x*i-y*j-z*k,w*i+x*v+y*k-z*j,w*j-x*k+y*v+z*i,w*k+x*j-y*i+z*v],np.float32)
def unrotate(q,v):
 xyz=-q[1:];uv=np.cross(xyz,v)
 return (v+np.float32(2)*(q[0]*uv+np.cross(xyz,uv))).astype(np.float32)
class Controller:
 def __init__(self,weights=None,torque_limit_nm=12.,slew_rate_nm_s=0.,warmup_ticks=10):
  with np.load(weights or ROOT/'weights/actor_weights.npz',allow_pickle=False) as z:self.weights={k:z[k].copy() for k in z.files}
  self.torque_limit_nm=torque_limit_nm;self.slew_rate_nm_s=slew_rate_nm_s;self.warmup_ticks=warmup_ticks
  self.reset()
 def reset(self):
  self.history=np.zeros((10,6),np.float32);self.previous=np.zeros(2,np.float32);self.ticks=0
  self.obs=np.zeros(65,np.float32);self.raw=np.zeros(2,np.float32);self.command=np.zeros(2,np.float32)
 def forward(self,obs):
  w=self.weights;x=np.asarray(obs,np.float32)
  if x.shape[-1]!=65 or not np.isfinite(x).all():raise ValueError('Expected finite 65-dimensional observation')
  with np.errstate(over='ignore',invalid='ignore',divide='ignore'):
   x=(x-w['norm_mean'])/(w['norm_std']+w['norm_eps'][0])
   for i in (0,2,4,6):
    x=x@w[f'actor_{i}_weight'].T+w[f'actor_{i}_bias']
    if i!=6:x=np.maximum(x,np.float32(0))
  if not np.isfinite(x).all():raise ValueError('Nonfinite network output')
  return x.astype(np.float32)
 def commit_command(self,command):
  a=np.asarray(command,np.float32)
  if a.shape!=(2,) or not np.isfinite(a).all() or np.max(np.abs(a))>12:raise ValueError('Expected model active Nm within +/-12')
  self.previous=a.copy()
 def step(self,q,dq,quat_wb,gyro_body_rad_s,measured_dt_s=.02,enabled=True,sensors_valid=True):
  """Exactly once per 20ms; returns (status, active_model_Nm[2]).
  status 0=OK,1=warmup,2=disabled,-1=bad input,-2=bad config,-3=numeric.
  Fault/disable clears state and outputs zero. Hardware watchdog remains external.
  """
  def stop(status):self.reset();return status,self.command.copy()
  if not (np.isfinite(self.torque_limit_nm) and 0<=self.torque_limit_nm<=12 and np.isfinite(self.slew_rate_nm_s) and self.slew_rate_nm_s>=0 and isinstance(self.warmup_ticks,int) and self.warmup_ticks>=0):return stop(-2)
  if not enabled:return stop(2)
  try:
   q=np.asarray(q,np.float32);dq=np.asarray(dq,np.float32);quat=np.asarray(quat_wb,np.float32);gyro=np.asarray(gyro_body_rad_s,np.float32)
   if q.shape!=(2,) or dq.shape!=(2,) or quat.shape!=(4,) or gyro.shape!=(3,):return stop(-1)
   if not sensors_valid or not all(np.isfinite(a).all() for a in (q,dq,quat,gyro)) or not np.isfinite(measured_dt_s) or abs(measured_dt_s-.02)>.001:return stop(-1)
   n=np.dot(quat,quat)
   if not np.isfinite(n) or not .81<=n<=1.21:return stop(-1)
   quat=quat/np.sqrt(n)
   self.history[:-1]=self.history[1:]
   self.history[-1]=np.concatenate((q/PI,dq/(np.float32(4)*PI),self.previous/np.float32(40)))
   g=unrotate(quat,np.array([0,0,-1],np.float32))
   imu=np.r_[np.arctan2(-g[1],-g[2])/PI,np.arctan2(g[0],-g[2])/PI,gyro/(np.float32(4)*PI)].astype(np.float32)
   self.obs=np.r_[self.history.ravel(),imu].astype(np.float32);self.raw=self.forward(self.obs)
   if self.ticks<self.warmup_ticks:
    self.previous.fill(0);self.command.fill(0);self.ticks+=1;return 1,self.command.copy()
   command=np.clip(np.float32(12)*np.clip(self.raw,-1,1),-self.torque_limit_nm,self.torque_limit_nm)
   if self.slew_rate_nm_s>0:
    delta=np.float32(self.slew_rate_nm_s)*np.float32(.02)
    command=self.previous+np.clip(command-self.previous,-delta,delta)
   self.command=np.clip(command,-self.torque_limit_nm,self.torque_limit_nm).astype(np.float32)
   self.commit_command(self.command);self.ticks+=1;return 0,self.command.copy()
  except (ValueError,TypeError,FloatingPointError):return stop(-3)

def map_hardware(q,dq,quat_ws,gyro_s,quat_sb,encoder_sign,encoder_zero,motor_sign,calibration_confirmed=False):
 """Explicit installation transform; calibration must be confirmed externally.
 Returns model q,dq,q_wb,gyro_B and motor signs; no automatic zero calibration.
 """
 if not calibration_confirmed:raise ValueError('Hardware calibration required')
 q,dq,qa,g,qb,es,ez,ms=[np.asarray(x,np.float32) for x in (q,dq,quat_ws,gyro_s,quat_sb,encoder_sign,encoder_zero,motor_sign)]
 if [a.shape for a in (q,dq,qa,g,qb,es,ez,ms)]!=[(2,),(2,),(4,),(3,),(4,),(2,),(2,),(2,)]:raise ValueError('Input shape')
 if not all(np.isfinite(a).all() for a in (q,dq,qa,g,qb,es,ez,ms)) or not np.all(np.abs(es)==1) or not np.all(np.abs(ms)==1) or abs(np.dot(qb,qb)-1)>=.001:raise ValueError('Invalid calibration/input')
 if not .81<=np.dot(qa,qa)<=1.21:raise ValueError('Invalid AHRS quaternion')
 return es*(q-ez),es*dq,multiply(qa,qb),unrotate(qb,g),ms.copy()
