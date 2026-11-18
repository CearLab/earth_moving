@echo off
cd /d "%~dp0"
python thesis\scripts\interactive_scenario.py --scenario thesis\seminar\scenarios\seminar_live_55.json
if errorlevel 1 pause
