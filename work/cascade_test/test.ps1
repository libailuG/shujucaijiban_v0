$ErrorActionPreference = 'Stop'
Push-Location (Join-Path $PSScriptRoot '../..')
try {
    & ./work/cascade_test/tcc/tcc.exe -Ifirmware/App work/cascade_test/test_receiver.c firmware/App/cascade_protocol.c firmware/App/cascade_receiver.c -o work/cascade_test/test_receiver.exe
    if ($LASTEXITCODE -ne 0) { throw 'Test compilation failed' }
    & ./work/cascade_test/test_receiver.exe
    if ($LASTEXITCODE -ne 0) { throw 'Receiver tests failed' }
} finally { Pop-Location }
