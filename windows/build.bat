@echo off
REM UAVX Groundstation Windows Build Script
REM Auto-installs Python + dependencies, then builds standalone .exe via PyInstaller

setlocal enabledelayedexpansion

echo ============================================
echo  UAVX Groundstation - Windows Build
echo  (auto-installs everything needed)
echo ============================================
echo.

REM Change to the script's directory (windows/)
pushd "%~dp0"
set SRC_DIR=src

REM ---------------------------------------------------------------
REM Step 0: Ensure Python 3.10+ is installed
REM ---------------------------------------------------------------
:check_python
where python >nul 2>&1
if %ERRORLEVEL% neq 0 (
    echo [0/5] Python not found. Attempting download and install...
    echo.
    echo   Downloading Python 3.11 from python.org...
    powershell -Command "& {[Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12; Invoke-WebRequest -Uri 'https://www.python.org/ftp/python/3.11.9/python-3.11.9-amd64.exe' -OutFile '%TEMP%\python-installer.exe'}"
    if %ERRORLEVEL% neq 0 (
        echo ERROR: Failed to download Python. Please install Python 3.10+ manually from:
        echo   https://python.org
        echo   (check "Add Python to PATH" during installation)
        pause
        exit /b 1
    )
    echo   Installing Python 3.11 (silent, adds to PATH)...
    start /wait "" "%TEMP%\python-installer.exe" /quiet InstallAllUsers=1 PrependPath=1
    if %ERRORLEVEL% neq 0 (
        echo ERROR: Python installer failed. Try installing manually.
        pause
        exit /b 1
    )
    echo   Python installed. Refreshing PATH...
    REM Re-scan PATH so the newly installed python is found
    for /f "tokens=2*" %%a in ('reg query "HKEY_LOCAL_MACHINE\SYSTEM\CurrentControlSet\Control\Session Manager\Environment" /v Path 2^>nul') do set "PATH=%%b;%PATH%"
    where python >nul 2>&1 || (
        echo ERROR: Python not found after install. Log out and back in, then re-run.
        pause
        exit /b 1
    )
    goto check_python
)

REM Check Python version
python -c "import sys; exit(0 if sys.version_info >= (3,10) else 1)"
if %ERRORLEVEL% neq 0 (
    echo ERROR: Python 3.10 or later required
    echo   Current: 
    python --version
    echo   Please upgrade from https://python.org
    pause
    exit /b 1
)

echo   Python found: 
python --version

REM ---------------------------------------------------------------
REM Step 1: Upgrade pip and install Python dependencies
REM ---------------------------------------------------------------
echo.
echo [1/5] Installing Python packages (PyQt5, pyserial, PyInstaller, etc.)...

REM Check if pip is available
python -m pip --version >nul 2>&1
if %ERRORLEVEL% neq 0 (
    echo   pip not found, attempting to bootstrap via ensurepip...
    python -m ensurepip --upgrade >nul 2>&1
    if !ERRORLEVEL! neq 0 (
        echo.
        echo   ERROR: pip is not installed and ensurepip is not available.
        echo   Re-install Python from https://python.org and make sure to
        echo   check "Install pip" during setup (it is checked by default).
        echo.
        pause
        exit /b 1
    )
)

python -m pip install --upgrade pip
python -m pip install PyQt5 pyserial folium PyQtWebEngine pyttsx3 pyinstaller
if %ERRORLEVEL% neq 0 (
    echo WARNING: Some pip packages failed. Check your internet connection.
    pause
    exit /b 1
)

REM ---------------------------------------------------------------
REM Step 2: Install Visual C++ Redistributable (needed by PyQt5/Qt)
REM ---------------------------------------------------------------
echo.
echo [2/5] Checking Visual C++ Redistributable...
REM Check for vcruntime140.dll as a proxy for VC++ being present
if not exist "%SystemRoot%\System32\vcruntime140.dll" (
    echo   VC++ redist not found. Downloading and installing...
    powershell -Command "& {[Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12; Invoke-WebRequest -Uri 'https://aka.ms/vs/17/release/vc_redist.x64.exe' -OutFile '%TEMP%\vc_redist.x64.exe'}"
    start /wait "" "%TEMP%\vc_redist.x64.exe" /install /quiet /norestart
    if !ERRORLEVEL! equ 0 (
        echo   Visual C++ Redistributable installed.
    ) else (
        echo   NOTE: Could not auto-install VC++ redist. If the .exe fails to launch,
        echo   install manually from https://aka.ms/vc-redist-download
    )
) else (
    echo   VC++ redist already present.
)

REM ---------------------------------------------------------------
REM Step 3: Clean previous builds
REM ---------------------------------------------------------------
echo.
echo [3/5] Cleaning previous builds...
if exist "%SRC_DIR%\dist" rmdir /s /q "%SRC_DIR%\dist"
if exist "%SRC_DIR%\build" rmdir /s /q "%SRC_DIR%\build"

REM ---------------------------------------------------------------
REM Step 4: Build executable with PyInstaller
REM ---------------------------------------------------------------
echo.
echo [4/5] Building executable with PyInstaller (this may take a minute)...
pushd "%SRC_DIR%"

pyinstaller ^
    --onefile ^
    --windowed ^
    --name "UAVX_GCS" ^
    --add-data "airframes;airframes" ^
    --hidden-import PyQt5.QtWebEngineWidgets ^
    --hidden-import PyQt5.QtWebChannel ^
    --hidden-import serial ^
    --hidden-import serial.tools.list_ports ^
    --hidden-import folium ^
    --hidden-import pyttsx3 ^
    --hidden-import pyttsx3.drivers ^
    --hidden-import pyttsx3.drivers.sapi5 ^
    --collect-submodules core ^
    --collect-submodules ui ^
    --collect-submodules widgets ^
    --collect-submodules logger ^
    main.py

if %ERRORLEVEL% neq 0 (
    echo ERROR: PyInstaller build failed
    pause
    exit /b 1
)

popd

REM ---------------------------------------------------------------
REM Step 5: Copy airframes alongside executable
REM ---------------------------------------------------------------
echo.
echo [5/5] Copying airframes...
xcopy /e /i /y "%SRC_DIR%\airframes" "%SRC_DIR%\dist\airframes" >nul

REM ---------------------------------------------------------------
REM Done
REM ---------------------------------------------------------------
echo.
echo ============================================
echo  SUCCESS!
echo  Standalone executable:
echo    %SRC_DIR%\dist\UAVX_GCS.exe
echo.
echo  To create a Windows installer (optional):
echo    1. Install Inno Setup from https://jrsoftware.org/isdl.php
echo       (or the script will prompt you next time)
echo    2. Right-click installer.iss -^> Compile
echo ============================================
echo.

REM Check for Inno Setup and offer to install
where iscc >nul 2>&1
if %ERRORLEVEL% neq 0 (
    choice /C YN /M "Inno Setup not found. Install it now to create setup packages?"
    if !ERRORLEVEL! equ 1 (
        echo   Opening download page...
        start https://jrsoftware.org/isdl.php
    )
)

popd
pause
