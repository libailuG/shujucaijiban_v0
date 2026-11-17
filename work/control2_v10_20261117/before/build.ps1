param(
    [string]$Toolchain = "$env:LOCALAPPDATA/Keil_v5/ARM/ARMCLANG/bin"
)
$ErrorActionPreference = 'Stop'
Push-Location $PSScriptRoot
try {
    # Keep CLI objects separate from Keil: compiler ABI options may differ.
    New-Item -ItemType Directory -Force ObjectsCli | Out-Null
    $cc = Join-Path $Toolchain 'armclang.exe'
    if (!(Test-Path -LiteralPath $cc)) { throw "Keil Arm Compiler 6 not found: $cc" }
    $flags = @('--target=arm-arm-none-eabi', '-mcpu=cortex-m7', '-mthumb',
        '-mfpu=fpv5-d16', '-mfloat-abi=hard', '-std=c99', '-g', '-O1',
        '-Wall', '-Wextra', '-Werror', '-DSTM32H723xx',
        '-I', 'Drivers/CMSIS/Device', '-I', 'Drivers/CMSIS/Core')
    & $cc @flags -c App/main.c -o ObjectsCli/main.o
    if ($LASTEXITCODE -ne 0) { throw 'main.c compilation failed' }
    foreach ($source in @('cascade_protocol','cascade_receiver','cascade_left','cascade_right','imu','imu_bus','icm_imu_i2c','mahony','battery','reserved_uart1','reserved_uart2','motor_bus','normal_mode','motor_debug_mode','imu_debug_mode','acquisition_mode','control_mode1')) {
        & $cc @flags -c "App/$source.c" -o "ObjectsCli/$source.o"
        if ($LASTEXITCODE -ne 0) { throw "$source compilation failed" }
    }
    & $cc @flags -ffp-contract=off -c App/control1_policy/s2_deploy.c -o ObjectsCli/s2_deploy.o
    if ($LASTEXITCODE -ne 0) { throw 's2_deploy compilation failed' }
    & $cc @flags -c Startup/system_stm32h7xx.c -o ObjectsCli/system_stm32h7xx.o
    if ($LASTEXITCODE -ne 0) { throw 'System compilation failed' }
    & (Join-Path $Toolchain 'armasm.exe') --cpu Cortex-M7 --fpu FPv5_D16 --debug --diag_suppress=1950 -o ObjectsCli/startup.o Startup/startup_stm32h723xx.s
    if ($LASTEXITCODE -ne 0) { throw 'Startup assembly failed' }
    & (Join-Path $Toolchain 'armlink.exe') --cpu Cortex-M7 --scatter board.sct --entry Reset_Handler --map --info sizes --list ObjectsCli/zhuban_caiji.map --output ObjectsCli/zhuban_caiji.axf ObjectsCli/startup.o ObjectsCli/system_stm32h7xx.o ObjectsCli/main.o ObjectsCli/cascade_protocol.o ObjectsCli/cascade_receiver.o ObjectsCli/cascade_left.o ObjectsCli/cascade_right.o ObjectsCli/imu.o ObjectsCli/imu_bus.o ObjectsCli/icm_imu_i2c.o ObjectsCli/mahony.o ObjectsCli/battery.o ObjectsCli/reserved_uart1.o ObjectsCli/reserved_uart2.o ObjectsCli/motor_bus.o ObjectsCli/normal_mode.o ObjectsCli/motor_debug_mode.o ObjectsCli/imu_debug_mode.o ObjectsCli/acquisition_mode.o ObjectsCli/control_mode1.o ObjectsCli/s2_deploy.o
    if ($LASTEXITCODE -ne 0) { throw 'Link failed' }
    & (Join-Path $Toolchain 'fromelf.exe') --i32combined --output ObjectsCli/zhuban_caiji.hex ObjectsCli/zhuban_caiji.axf
    if ($LASTEXITCODE -ne 0) { throw 'HEX generation failed' }
    & (Join-Path $Toolchain 'fromelf.exe') --bin --output ObjectsCli/zhuban_caiji.bin ObjectsCli/zhuban_caiji.axf
    if ($LASTEXITCODE -ne 0) { throw 'BIN generation failed' }
    Write-Host 'Build OK: ObjectsCli/zhuban_caiji.hex, ObjectsCli/zhuban_caiji.bin'
} finally {
    Pop-Location
}


