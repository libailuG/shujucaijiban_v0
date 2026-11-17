import ctypes as C, ctypes.wintypes as W, socket, struct, threading, subprocess, sys
K=C.WinDLL('kernel32',use_last_error=True); H=C.WinDLL('hid',use_last_error=True)
K.CreateFileW.argtypes=[W.LPCWSTR,W.DWORD,W.DWORD,W.LPVOID,W.DWORD,W.DWORD,W.HANDLE]; K.CreateFileW.restype=W.HANDLE
K.ReadFile.argtypes=[W.HANDLE,W.LPVOID,W.DWORD,C.POINTER(W.DWORD),W.LPVOID]
K.WriteFile.argtypes=K.ReadFile.argtypes
K.CloseHandle.argtypes=[W.HANDLE]
H.HidD_GetPreparsedData.argtypes=[W.HANDLE,C.POINTER(W.LPVOID)]
H.HidP_GetCaps.argtypes=[W.LPVOID,W.LPVOID]
H.HidD_FreePreparsedData.argtypes=[W.LPVOID]
path=r'\\?\hid#vid_04d8&pid_00df&mi_02#8&321b31a1&0&0000#{4d1e55b2-f16f-11cf-88cb-001111000030}'
h=K.CreateFileW(path,0xc0000000,3,None,3,0,None)
if h==W.HANDLE(-1).value: raise C.WinError(C.get_last_error())
p=W.LPVOID(); caps=C.create_string_buffer(64)
if not H.HidD_GetPreparsedData(h,C.byref(p)):raise C.WinError(C.get_last_error())
H.HidP_GetCaps(p,caps);H.HidD_FreePreparsedData(p)
_,_,inlen,outlen=struct.unpack_from('<4H',caps.raw)
print('Motor HID report sizes',inlen,outlen,flush=True)
server=socket.socket();server.bind(('127.0.0.1',0));server.listen(1);port=server.getsockname()[1]
errors=[]
def exact(s,n):
 b=b''
 while len(b)<n:
  part=s.recv(n-len(b))
  if not part:raise EOFError()
  b+=part
 return b
def relay():
 try:
  con,_=server.accept(); con.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
  with con:
   while True:
    sig,n,typ,_=struct.unpack('<IHBB',exact(con,8))
    if sig!=0x00504144 or typ!=1 or n>outlen-1:raise ValueError('Invalid TCP DAP request')
    cmd=exact(con,n);packet=C.create_string_buffer(b'\0'+cmd.ljust(outlen-1,b'\0'),outlen);written=W.DWORD()
    if not K.WriteFile(h,packet,outlen,C.byref(written),None):raise C.WinError(C.get_last_error())
    reply=C.create_string_buffer(inlen);got=W.DWORD()
    if not K.ReadFile(h,reply,inlen,C.byref(got),None):raise C.WinError(C.get_last_error())
    data=reply.raw[1:got.value]
    if cmd[:2]==bytes([0,254]) and len(data)>=3: data=data[:2]+bytes([1])+data[3:]
    if not data or data[0]!=cmd[0]:raise ValueError('DAP command mismatch')
    con.sendall(struct.pack('<IHBB',sig,len(data),2,0)+data)
 except EOFError:pass
 except Exception as e:errors.append(repr(e));print('RELAY ERROR',repr(e),flush=True)
 finally:server.close()
threading.Thread(target=relay,daemon=True).start()
exe=r'C:\Users\libai\Desktop\X1\imu_caiji\zhuban_troubleshooting_archive\Tools\xpack-openocd-0.12.0-7\bin\openocd.exe'
args=[exe,'-f','interface/cmsis-dap.cfg','-c','cmsis-dap backend tcp','-c','cmsis-dap tcp host 127.0.0.1','-c',f'cmsis-dap tcp port {port}','-c','transport select swd','-c','cmsis-dap quirk enable','-f','target/stm32g4x.cfg','-c','adapter speed 500']+sys.argv[1:]
try:result=subprocess.run(args,timeout=90);sys.exit(result.returncode or bool(errors))
finally:K.CloseHandle(h)
