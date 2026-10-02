@echo off
rem Double-click setup for Windows: no command line needed. Runs scripts\setup-windows.ps1
rem (Python if it is missing, optionally the Intel RealSense SDK) and keeps
rem the window open so the result can be read. docs\index.html is the walk-through.
title Perceptronics - Setup
cd /d "%~dp0"
if not exist "scripts\setup-windows.ps1" goto notunzipped
echo.
echo  PERCEPTRONICS SETUP
echo  This takes 2 to 10 minutes and needs the internet. Leave this window open.
echo.
set SKIPSDK=
choice /c YN /m "Will an Intel RealSense camera be plugged into THIS computer"
if errorlevel 2 set SKIPSDK=-SkipSdk
powershell -NoProfile -ExecutionPolicy Bypass -File "scripts\setup-windows.ps1" %SKIPSDK% %*
if not errorlevel 1 goto done
echo.
echo  ==========================================================
echo   SETUP DID NOT FINISH.
echo   Read the red text above, then see the Troubleshooting
echo   chart on the instructions page. It is safe to run this
echo   file again.
echo  ==========================================================
pause
exit /b 1
:done
echo.
echo  ==========================================================
echo   SETUP FINISHED.
echo   "NOT READY" and [FAIL] lines about the robot or cockpit
echo   are normal here: nothing is connected yet.
echo   Next: close this window and double-click Windows-Start
echo  ==========================================================
pause
exit /b 0
:notunzipped
echo.
echo  This file is still inside the ZIP, or was moved out of its folder.
echo  Close this window. Right-click the downloaded ZIP file, choose
echo  "Extract All...", click Extract, then open the NEW folder and
echo  double-click Windows-Setup there.
echo.
pause
exit /b 1
