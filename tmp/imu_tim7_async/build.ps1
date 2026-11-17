param(
    [string]$Toolchain = "$env:LOCALAPPDATA/Keil_v5/ARM/ARMCLANG/bin"
)
$ErrorActionPreference = 'Stop'
Push-Location $PSScriptRoot
try {
    New-Item -ItemType Directory -Force Objects | Out-Null
    $cc = Join-Path $Toolchain 'armclang.exe'
    if (!(Test-Path -LiteralPath $cc)) { throw "Keil Arm Compiler 6 not found: $cc" }
    $flags = @('--target=arm-arm-none-eabi', '-mcpu=cortex-m7', '-mthumb',
        '-mfpu=fpv5-d16', '-mfloat-abi=hard', '-std=c99', '-g', '-O1',
        '-Wall', '-Wextra', '-Werror', '-DSTM32H723xx',
        '-I', 'Drivers/CMSIS/Device', '-I', 'Drivers/CMSIS/Core')
    & $cc @flags -c App/main.c -o Objects/main.o
    if ($LASTEXITCODE -ne 0) { throw 'main.c compilation failed' }
    foreach ($source in @('imu','imu_bus','icm_imu_i2c','mahony')) {
        & $cc @flags -c "App/$source.c" -o "Objects/$source.o"
        if ($LASTEXITCODE -ne 0) { throw "$source compilation failed" }
    }
    & $cc @flags -c Startup/system_stm32h7xx.c -o Objects/system_stm32h7xx.o
    if ($LASTEXITCODE -ne 0) { throw 'System compilation failed' }
    & (Join-Path $Toolchain 'armasm.exe') --cpu Cortex-M7 --fpu FPv5_D16 --debug --diag_suppress=1950 -o Objects/startup.o Startup/startup_stm32h723xx.s
    if ($LASTEXITCODE -ne 0) { throw 'Startup assembly failed' }
    & (Join-Path $Toolchain 'armlink.exe') --cpu Cortex-M7 --ro_base 0x08000000 --rw_base 0x20000000 --first __Vectors --entry Reset_Handler --map --info sizes --list Objects/zhuban_led.map --output Objects/zhuban_led.axf Objects/startup.o Objects/system_stm32h7xx.o Objects/main.o Objects/imu.o Objects/imu_bus.o Objects/icm_imu_i2c.o Objects/mahony.o
    if ($LASTEXITCODE -ne 0) { throw 'Link failed' }
    & (Join-Path $Toolchain 'fromelf.exe') --i32combined --output Objects/zhuban_led.hex Objects/zhuban_led.axf
    if ($LASTEXITCODE -ne 0) { throw 'HEX generation failed' }
    & (Join-Path $Toolchain 'fromelf.exe') --bin --output Objects/zhuban_led.bin Objects/zhuban_led.axf
    if ($LASTEXITCODE -ne 0) { throw 'BIN generation failed' }
    Write-Host 'Build OK: Objects/zhuban_led.hex, Objects/zhuban_led.bin'
} finally {
    Pop-Location
}
