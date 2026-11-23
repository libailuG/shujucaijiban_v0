param(
    [string]$Toolchain = "$env:LOCALAPPDATA/Keil_v5/ARM/ARMCLANG/bin",
    [string]$OutputDirectory = 'ObjectsCli',
    [switch]$RedLedTest
)
$ErrorActionPreference = 'Stop'
Push-Location $PSScriptRoot

try {
    $imageName = 'zhuban_rtos_controls'
    if ($RedLedTest) {
        $OutputDirectory = Join-Path $OutputDirectory 'RedLedTest'
        $imageName = 'zhuban_rtos_red_led_test'
    }
    New-Item -ItemType Directory -Force $OutputDirectory | Out-Null
    $outputPath = Join-Path $OutputDirectory $imageName

    $cc = Join-Path $Toolchain 'armclang.exe'
    if (!(Test-Path -LiteralPath $cc)) { throw "Keil Arm Compiler 6 not found: $cc" }
    $kernel = 'ThirdParty/FreeRTOS-Kernel'
    $port = "$kernel/portable/GCC/ARM_CM7/r0p1"
    $flags = @('--target=arm-arm-none-eabi', '-mcpu=cortex-m7', '-mthumb',
        '-mfpu=fpv5-d16', '-mfloat-abi=hard', '-std=c99', '-g', '-O1',
        '-Wall', '-Wextra', '-Werror', '-DSTM32H723xx',
        '-DZHUBAN_RTOS_PHASE1', '-DZHUBAN_RTOS_CONTROLS',
        '-I', 'App', '-I', 'Drivers/CMSIS/Device', '-I', 'Drivers/CMSIS/Core',
        '-I', "$kernel/include", '-I', $port)
    if ($RedLedTest) { $flags += '-DZHUBAN_RED_LED_TEST' }
    $sources = @('main','cascade_protocol','cascade_receiver','cascade_left','cascade_right',
        'imu','imu_bus','icm_imu_i2c','mahony','battery','reserved_uart1','reserved_uart2',
        'motor_bus','normal_mode','imu_debug_mode','acquisition_mode','motor_debug_mode',
        'control_mode1','control_mode2','control_mode3','rtos_phase1') |
        ForEach-Object { "App/$_.c" }
    $sources += @('App/control1_policy/s2_deploy.c',
        'App/control2_policy/hip_assist_caiji_v0_policy.c',
        'App/control2_policy/assist_oscillation_monitor.c',
        'Startup/system_stm32h7xx.c', "$kernel/tasks.c", "$kernel/list.c", "$port/port.c")

    $objects = @()
    foreach ($source in $sources) {
        $object = Join-Path $OutputDirectory (([System.IO.Path]::GetFileNameWithoutExtension($source)) + '.o')
        $sourceFlags = @()
        if ($source -like 'App/control1_policy/*') { $sourceFlags += '-ffp-contract=off' }
        if ($source -like 'App/control2_policy/*') { $sourceFlags += @('-O3', '-ffp-contract=off') }
        & $cc @flags @sourceFlags -c $source -o $object
        if ($LASTEXITCODE -ne 0) { throw "$source compilation failed" }
        $objects += $object
    }
    $startup = Join-Path $OutputDirectory 'startup.o'
    & (Join-Path $Toolchain 'armasm.exe') --cpu Cortex-M7 --fpu FPv5_D16 --debug --diag_suppress=1950 -o $startup Startup/startup_stm32h723xx.s
    if ($LASTEXITCODE -ne 0) { throw 'Startup assembly failed' }
    & (Join-Path $Toolchain 'armlink.exe') --cpu Cortex-M7 --scatter board.sct --entry Reset_Handler --map --info sizes --list "$outputPath.map" --output "$outputPath.axf" $startup @objects
    if ($LASTEXITCODE -ne 0) { throw 'Link failed' }
    & (Join-Path $Toolchain 'fromelf.exe') --i32combined --output "$outputPath.hex" "$outputPath.axf"
    if ($LASTEXITCODE -ne 0) { throw 'HEX generation failed' }
    & (Join-Path $Toolchain 'fromelf.exe') --bin --output "$outputPath.bin" "$outputPath.axf"
    if ($LASTEXITCODE -ne 0) { throw 'BIN generation failed' }
    Write-Host "Build OK: $outputPath.hex, $outputPath.bin"
} finally {
    Pop-Location
}
