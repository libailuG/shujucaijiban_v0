@echo off
setlocal
cd /d "%~dp0"
set "PY=%USERPROFILE%\miniconda3\envs\pyqt\python.exe"
if not exist "%PY%" set "PY=python"
"%PY%" main.py %*
if errorlevel 1 pause
