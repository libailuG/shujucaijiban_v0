param([switch]$Recovery64)
$ErrorActionPreference = 'Stop'
Push-Location $PSScriptRoot
try {
    $openocd = Join-Path $PSScriptRoot 'Tools/xpack-openocd-0.12.0-7/bin/openocd.exe'
    $image = if ($Recovery64) { 'Objects/zhuban_recovery64.hex' } else { 'Objects/zhuban_led.hex' }
    $imagePath = (Join-Path $PSScriptRoot $image).Replace('\', '/')
    if (!(Test-Path -LiteralPath $imagePath)) { throw "Firmware not found: $imagePath" }
    $command = 'adapter speed 100; gdb port disabled; tcl port disabled; telnet port disabled; init; stm32h7x.ap2 write_memory 0xE00E1004 32 {0x0070013F}; stm32h7x.cpu0 arp_examine; program {' + $imagePath + '} verify reset exit'
    & $openocd -f interface/stlink.cfg -f target/stm32h7x.cfg -c $command 2>&1 |
        Tee-Object -FilePath 'Objects/last_flash.log'
    if ($LASTEXITCODE -ne 0) { throw 'Download failed; see Objects/last_flash.log' }
    Write-Host "Firmware written, verified and restarted: $image"
} finally {
    Pop-Location
}
