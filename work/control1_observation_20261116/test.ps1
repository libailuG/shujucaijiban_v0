$ErrorActionPreference='Stop'
Push-Location (Join-Path $PSScriptRoot '../..')
try {
    $dir='work/control1_observation_20261116'
    New-Item -ItemType Directory -Force "$dir/include/control1_policy" | Out-Null
    Get-ChildItem firmware/App -Filter '*.h' | ForEach-Object {
        [IO.File]::WriteAllText((Join-Path (Resolve-Path "$dir/include") $_.Name),
            (Get-Content $_.FullName -Raw), (New-Object Text.UTF8Encoding $false))
    }
    Copy-Item firmware/App/control1_policy/s2_deploy.h "$dir/include/control1_policy/s2_deploy.h"
    Copy-Item firmware/App/control_mode1.c "$dir/control_mode1.c"
    & ./work/cascade_test/tcc/tcc.exe -D__ia64__ "-I$dir/include" "$dir/test.c" "$dir/control_mode1.c" firmware/App/control1_policy/s2_deploy.c -o "$dir/test.exe"
    if ($LASTEXITCODE -ne 0) { throw 'Compile failed' }
    & "./$dir/test.exe"
    if ($LASTEXITCODE -ne 0) { throw 'Observation tests failed' }
} finally { Pop-Location }
