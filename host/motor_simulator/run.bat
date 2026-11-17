@echo off
setlocal
set "PYQT_PYTHON=%USERPROFILE%\miniconda3\envs\pyqt\python.exe"
if not exist "%PYQT_PYTHON%" (
 echo Missing conda pyqt environment. See README.md.
 pause
 exit /b 1
)
cd /d "%~dp0"
"%PYQT_PYTHON%" main.py
if errorlevel 1 pause
