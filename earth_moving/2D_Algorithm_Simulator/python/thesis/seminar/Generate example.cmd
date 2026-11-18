@echo off
setlocal
set "SCENARIO=%~dp0scenarios\corridor.json"
set "OUTPUT=%~dp0running_example"
if not "%~1"=="" set "SCENARIO=%~f1"
if not "%~2"=="" set "OUTPUT=%~f2"
set "SCENARIO_PYTHON=%USERPROFILE%\miniforge3\envs\earth_moving\python.exe"
if not "%~3"=="" set "SCENARIO_PYTHON=%~f3"
if not exist "%SCENARIO_PYTHON%" (
  echo Python environment not found. Pass its python.exe path as the third argument.
  exit /b 1
)
for %%I in ("%SCENARIO_PYTHON%") do set "SCENARIO_PYTHON_DIR=%%~dpI"
set "PATH=%SCENARIO_PYTHON_DIR%Library\bin;%SCENARIO_PYTHON_DIR%;%PATH%"
"%SCENARIO_PYTHON%" "%~dp0..\scripts\build_seminar_example_20261114.py" --scenario "%SCENARIO%" --output "%OUTPUT%"
if errorlevel 1 (
  echo Generation or verification failed. Do not use previous figures as new results.
  exit /b 1
)
echo.
echo Generated and verified: "%OUTPUT%\index.html"
exit /b 0
