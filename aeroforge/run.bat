@echo off
setlocal enabledelayedexpansion
REM ============================================================
REM  AeroForge launcher - double-click to run.
REM  Creates a venv, installs dependencies (first run only),
REM  starts the server, and opens your browser.
REM ============================================================
cd /d "%~dp0"

REM ---- Locate Python (prefer 3.11, then 3.12, 3.13, then generic) ----
set "PYCMD="
py -3.11 --version >nul 2>&1 && set "PYCMD=py -3.11"
if not defined PYCMD py -3.12 --version >nul 2>&1 && set "PYCMD=py -3.12"
if not defined PYCMD py -3.13 --version >nul 2>&1 && set "PYCMD=py -3.13"
if not defined PYCMD python --version >nul 2>&1 && set "PYCMD=python"
if not defined PYCMD (
    echo.
    echo [AeroForge] Python was not found on this machine.
    echo Please install Python 3.11 or 3.12 from https://www.python.org/downloads/
    echo IMPORTANT: check "Add python.exe to PATH" during installation.
    echo Or, if winget is available:  winget install Python.Python.3.12
    echo.
    pause
    exit /b 1
)
echo [AeroForge] Using interpreter: %PYCMD%

REM ---- Create venv if missing ----
if not exist ".venv\Scripts\python.exe" (
    echo [AeroForge] Creating virtual environment...
    %PYCMD% -m venv .venv
    if errorlevel 1 (
        echo [AeroForge] Failed to create the virtual environment.
        pause
        exit /b 1
    )
)

set "VPY=.venv\Scripts\python.exe"

REM ---- Install dependencies once (marker file skips reinstall) ----
if not exist ".venv\.deps_installed" (
    echo [AeroForge] Installing dependencies - this can take a few minutes on first run...
    "%VPY%" -m pip install --upgrade pip
    "%VPY%" -m pip install -r requirements.txt
    if errorlevel 1 (
        echo [AeroForge] Dependency installation failed. See messages above.
        pause
        exit /b 1
    )
    echo ok > ".venv\.deps_installed"
)

REM ---- Start the server in its own window ----
echo [AeroForge] Starting server at http://127.0.0.1:8000 ...
start "AeroForge server" /min "%VPY%" -m uvicorn backend.main:app --host 127.0.0.1 --port 8000

REM ---- Wait for the health endpoint (up to ~120 s), then open the browser ----
set /a tries=0
:waitloop
curl -s -o nul http://127.0.0.1:8000/health
if not errorlevel 1 goto healthy
set /a tries+=1
if %tries% geq 120 goto failed
timeout /t 1 /nobreak >nul
goto waitloop

:failed
echo [AeroForge] Server did not become healthy in time. Check the "AeroForge server" window for errors.
pause
exit /b 1

:healthy
start "" http://127.0.0.1:8000
echo.
echo [AeroForge] Running. Close the "AeroForge server" window to stop the app.
echo.
exit /b 0
