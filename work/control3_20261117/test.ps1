$ErrorActionPreference='Stop'
Push-Location (Join-Path $PSScriptRoot '../..')
try {
 $dir='work/control3_20261117'
 Get-ChildItem firmware/App -Filter *.h | ForEach-Object { [IO.File]::WriteAllText((Join-Path (Resolve-Path "$dir/include") $_.Name),[IO.File]::ReadAllText($_.FullName),(New-Object Text.UTF8Encoding $false)) }
 Copy-Item firmware/App/control_mode3.c "$dir/control_mode3.c" -Force
 & ./work/cascade_test/tcc/i386-win32-tcc.exe "-I$dir/include" "$dir/test.c" "$dir/control_mode3.c" -o "$dir/test.exe"
 if ($LASTEXITCODE -ne 0) {throw 'compile failed'}
 & "./$dir/test.exe"
 if ($LASTEXITCODE -ne 0) {throw 'test failed'}
} finally {Pop-Location}
