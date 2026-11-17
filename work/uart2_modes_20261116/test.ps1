$ErrorActionPreference='Stop'
Push-Location (Join-Path $PSScriptRoot '../..')
try {
    # TinyCC cannot parse the existing BOM in imu.h. Normalize test copies only.
    New-Item -ItemType Directory -Force work/uart2_modes_20261116/include | Out-Null
    Get-ChildItem firmware/App -Filter '*.h' | ForEach-Object {
        Set-Content (Join-Path work/uart2_modes_20261116/include $_.Name) (Get-Content $_.FullName -Raw) -Encoding utf8
    }
    Copy-Item firmware/App/imu_debug_mode.c work/uart2_modes_20261116/imu_debug_mode.c
    # Avoid TinyCC's unsupported x87 inline fabsf definition in bundled math.h.
    & ./work/cascade_test/tcc/tcc.exe -D__ia64__ -Iwork/uart2_modes_20261116/include work/uart2_modes_20261116/test_feedback.c work/uart2_modes_20261116/imu_debug_mode.c -o work/uart2_modes_20261116/test_feedback.exe
    if ($LASTEXITCODE -ne 0) { throw 'Compile failed' }
    & ./work/uart2_modes_20261116/test_feedback.exe
    if ($LASTEXITCODE -ne 0) { throw 'Tests failed' }
} finally { Pop-Location }
