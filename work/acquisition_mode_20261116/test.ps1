$ErrorActionPreference='Stop'
Push-Location (Join-Path $PSScriptRoot '../..')
try {
    $dir='work/acquisition_mode_20261116'
    New-Item -ItemType Directory -Force "$dir/include" | Out-Null
    Get-ChildItem firmware/App -Filter '*.h' | ForEach-Object {
        Set-Content (Join-Path "$dir/include" $_.Name) (Get-Content $_.FullName -Raw) -Encoding utf8
    }
    Copy-Item firmware/App/acquisition_mode.c "$dir/acquisition_mode.c"
    & ./work/cascade_test/tcc/tcc.exe -D__ia64__ "-I$dir/include" "$dir/test_acquisition.c" "$dir/acquisition_mode.c" -o "$dir/test_acquisition.exe"
    if ($LASTEXITCODE -ne 0) { throw 'Compile failed' }
    & "./$dir/test_acquisition.exe"
    if ($LASTEXITCODE -ne 0) { throw 'Acquisition tests failed' }
} finally { Pop-Location }
