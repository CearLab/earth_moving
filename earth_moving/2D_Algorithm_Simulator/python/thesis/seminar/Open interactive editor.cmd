@echo off
setlocal
set "EDITOR_PYTHON=%USERPROFILE%\miniforge3\envs\earth_moving\python.exe"
if not exist "%EDITOR_PYTHON%" (
 echo Python environment not found: "%EDITOR_PYTHON%"
 pause
 exit /b 1
)
for %%I in ("%EDITOR_PYTHON%") do set "EDITOR_PYTHON_DIR=%%~dpI"
set "PATH=%EDITOR_PYTHON_DIR%Library\bin;%EDITOR_PYTHON_DIR%;%PATH%"
if "%~1"=="" (
 "%EDITOR_PYTHON%" "%~dp0..\scripts\interactive_scenario.py"
) else (
 "%EDITOR_PYTHON%" "%~dp0..\scripts\interactive_scenario.py" --scenario "%~f1"
)
if errorlevel 1 pause
