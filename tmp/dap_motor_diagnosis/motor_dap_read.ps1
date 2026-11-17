param([string[]]$OpenOcdArgs)
Add-Type @"
using System; using System.Runtime.InteropServices; using Microsoft.Win32.SafeHandles;
public class DapReserve { [DllImport("kernel32.dll",CharSet=CharSet.Unicode,SetLastError=true)] public static extern SafeFileHandle CreateFile(string p,uint a,uint s,IntPtr x,uint c,uint f,IntPtr t); }
"@
$hold=[DapReserve]::CreateFile('\\?\hid#vid_04d8&pid_00df&mi_02#7&7036717&0&0000#{4d1e55b2-f16f-11cf-88cb-001111000030}',0,0,[IntPtr]::Zero,3,0,[IntPtr]::Zero)
if($hold.IsInvalid) { throw 'Could not reserve mainboard HID' }
try { & 'C:\Users\libai\Desktop\X1\imu_caiji\zhuban_troubleshooting_archive\Tools\xpack-openocd-0.12.0-7\bin\openocd.exe' @OpenOcdArgs } finally { $hold.Dispose() }
