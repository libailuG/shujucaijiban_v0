"""Host-only ctypes binding for the same unmodified embedded C implementation."""
import ctypes as C
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
F2=C.c_float*2; F3=C.c_float*3; F4=C.c_float*4; F65=C.c_float*65
class Workspace(C.Structure):
    _fields_=[('normalized',F65),('a',C.c_float*256),('b',C.c_float*128)]
class Config(C.Structure):
    _fields_=[('gyro_mode',C.c_int),('torque_limit_nm',C.c_float),('slew_rate_nm_s',C.c_float),('warmup_ticks',C.c_uint32)]
class Input(C.Structure):
    _fields_=[('hip_rad',F2),('hip_rad_s',F2),('quat_wb',F4),('gyro_body_rad_s',F3),('measured_dt_s',C.c_float),('enabled',C.c_uint32),('sensors_valid',C.c_uint32)]
class Output(C.Structure):
    _fields_=[('obs',F65),('raw',F2),('requested_nm',F2),('command_nm',F2)]
class State(C.Structure):
    _fields_=[('history',C.c_float*60),('previous_quat',F4),('previous_command',F2),('ticks',C.c_uint32),('quat_initialized',C.c_uint32),('work',Workspace)]
class PortConfig(C.Structure):
    _fields_=[('quat_sb',F4),('encoder_sign',F2),('encoder_zero_rad',F2),('motor_sign',F2),('calibration_confirmed',C.c_uint32)]
class HardwareInput(C.Structure):
    _fields_=[('encoder_joint_rad',F2),('encoder_joint_rad_s',F2),('quat_ws',F4),('gyro_sensor_rad_s',F3),('measured_dt_s',C.c_float),('enabled',C.c_uint32),('sensors_valid',C.c_uint32)]
lib=C.CDLL(str(ROOT/'build/libs2.so'))
lib.s2_policy_forward.argtypes=[C.POINTER(C.c_float),C.POINTER(C.c_float),C.POINTER(Workspace)]
lib.s2_policy_forward.restype=C.c_int
lib.s2_controller_step.argtypes=[C.POINTER(State),C.POINTER(Config),C.POINTER(Input),C.POINTER(Output)]
lib.s2_controller_step.restype=C.c_int
lib.s2_controller_commit_command.argtypes=[C.POINTER(State),C.POINTER(C.c_float)]
lib.s2_controller_commit_command.restype=C.c_int
lib.s2_port_map_input.argtypes=[C.POINTER(PortConfig),C.POINTER(HardwareInput),C.POINTER(Input)]
lib.s2_port_map_input.restype=C.c_int
def array(a):return np.ctypeslib.as_array(a).copy()
def forward(obs,work=None):
    work=work or Workspace();raw=F2()
    assert lib.s2_policy_forward(F65(*obs),raw,C.byref(work))==0
    return array(raw)
def step(state,config,q,qd,quat,gyro=(0,0,0)):
    i=Input(F2(*q),F2(*qd),F4(*quat),F3(*gyro),.02,1,1);o=Output()
    status=lib.s2_controller_step(C.byref(state),C.byref(config),C.byref(i),C.byref(o))
    assert status in (0,1),status
    return o
