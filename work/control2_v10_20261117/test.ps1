$ErrorActionPreference='Stop'
Push-Location (Join-Path $PSScriptRoot '../..')
try {
    $dir='work/control2_v10_20261117'
    Get-ChildItem firmware/App -Filter '*.h' | ForEach-Object {
        [IO.File]::WriteAllText((Join-Path (Resolve-Path "$dir/include") $_.Name),[IO.File]::ReadAllText($_.FullName),(New-Object Text.UTF8Encoding $false))
    }
    New-Item -ItemType Directory -Force "$dir/include/control1_policy" | Out-Null
    Copy-Item firmware/App/control1_policy/s2_deploy.h "$dir/include/control1_policy"
    Copy-Item firmware/App/control2_policy/*.h "$dir/include/control2_policy"
    Copy-Item firmware/App/control_mode2.c "$dir/control_mode2.c"
    # Use 32-bit TCC: the bundled legacy x64 compiler produces incorrect float state updates.
    & ./work/cascade_test/tcc/i386-win32-tcc.exe "-I$dir/include" "$dir/test.c" "$dir/control_mode2.c" firmware/App/control2_policy/g1_assist_v10_policy.c firmware/App/control2_policy/assist_oscillation_monitor.c -o "$dir/test32.exe"
    if ($LASTEXITCODE -ne 0) { throw 'Compile failed' }
    & "./$dir/test32.exe"
    if ($LASTEXITCODE -ne 0) { throw 'Control mode 2 tests failed' }
} finally { Pop-Location }
