@echo off
cd /d "%~dp0"
set "QT_PYTHON=%USERPROFILE%\miniconda3\envs\pyqt\python.exe"
if exist "%QT_PYTHON%" (
    "%QT_PYTHON%" main.py %*
) else (
    python main.py %*
)
if errorlevel 1 pause
