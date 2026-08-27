@echo off
cd /d "%~dp0"
python sd_log_parser_gui.py
if errorlevel 1 pause
