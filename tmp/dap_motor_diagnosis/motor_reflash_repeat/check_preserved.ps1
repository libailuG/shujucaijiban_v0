param([ValidateSet('before','after')][string]$Phase)
$ErrorActionPreference='Stop'
foreach($offset in @(0,4096,8192,12288,16384,20480,24576,28672,129024)) {
 $size=if($offset -eq 129024){2048}else{4096}
 $name='tmp/dap_motor_diagnosis/motor_reflash_repeat/'+$Phase+'_'+$offset.ToString('x')+'.bin'
 $address='0x'+(0x08000000L+$offset).ToString('x')
 $ok=$false
 for($attempt=1;$attempt -le 3;$attempt++) {
  $log=$name+'.'+$attempt+'.log'
  & '.\tmp\dap_motor_diagnosis\motor_dap_read.ps1' -OpenOcdArgs @('-f','interface/cmsis-dap.cfg','-c','transport select swd','-f','target/stm32g4x.cfg','-c','adapter speed 500','-c','init','-c',"dump_image $name $address $size",'-c','shutdown') 2>&1 | Out-File $log
  $f=Get-Item $name -ErrorAction SilentlyContinue
  if($f -and $f.Length -eq $size -and !(Select-String -Path $log -Pattern 'Error:')) {$ok=$true;break}
 }
 if(!$ok){throw "Incomplete/failed read $address"}
 if($Phase -eq 'after') {
  $before='tmp/dap_motor_diagnosis/motor_reflash_repeat/before_'+$offset.ToString('x')+'.bin'
  if((Get-FileHash $before).Hash -ne (Get-FileHash $name).Hash){throw "Preserved region mismatch $address"}
  "UNCHANGED $address $size bytes"
 } else {"BACKUP $address $size bytes"}
}
