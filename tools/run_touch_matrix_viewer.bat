@echo off
setlocal

set PORT=8765
set VIEWER_DIR=%~dp0touch_matrix_viewer

cd /d "%VIEWER_DIR%"
start "" "http://127.0.0.1:%PORT%/"

where python >nul 2>nul
if %ERRORLEVEL%==0 (
  python -m http.server %PORT% --bind 127.0.0.1
  exit /b %ERRORLEVEL%
)

where py >nul 2>nul
if %ERRORLEVEL%==0 (
  py -3 -m http.server %PORT% --bind 127.0.0.1
  exit /b %ERRORLEVEL%
)

echo Python was not found. Open tools\touch_matrix_viewer\index.html through a local web server.
pause
exit /b 1
