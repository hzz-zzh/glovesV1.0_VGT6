@echo off
setlocal

rem Prefer the Windows Python launcher, but verify that it has a Python 3
rem runtime behind it.  A standalone py.exe can exist even when Python is not
rem installed.
py -3 -c "import sys" >nul 2>&1
if not errorlevel 1 (
    set "PYTHON_CMD=py -3"
    goto :check_dependencies
)

python -c "import sys" >nul 2>&1
if not errorlevel 1 (
    set "PYTHON_CMD=python"
    goto :check_dependencies
)

echo [ERROR] Python 3 is not installed or cannot be found.
echo Download it from https://www.python.org/downloads/windows/
echo During setup, enable "Add python.exe to PATH", then run this file again.
goto :failed

:check_dependencies
%PYTHON_CMD% -c "import tkinter" >nul 2>&1
if errorlevel 1 (
    echo [ERROR] This Python installation does not include tkinter.
    echo Reinstall Python from python.org and include the Tcl/Tk component.
    goto :failed
)

%PYTHON_CMD% -c "import serial" >nul 2>&1
if errorlevel 1 (
    echo [ERROR] The pyserial package is not installed.
    echo Run: %PYTHON_CMD% -m pip install pyserial
    goto :failed
)

%PYTHON_CMD% "%~dp0modbus485_monitor.py"
if errorlevel 1 (
    echo.
    echo [ERROR] Modbus485 monitor exited unexpectedly.
    goto :failed
)
exit /b 0

:failed
echo.
pause
exit /b 1
