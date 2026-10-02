@echo off
rem Double-click start for Windows: runs scripts\cockpit.ps1 and opens the browser.
rem Enter alone = practice mode (the sim cell: synthetic camera, no robot needed).
title Perceptronics - running (close this window to stop)
cd /d "%~dp0"
if not exist "scripts\cockpit.ps1" goto notunzipped
echo.
echo  PERCEPTRONICS
echo  Practice mode needs no robot and no camera: just press Enter.
echo  For a real robot, type the cell name your installer gave you, then Enter.
echo.
set CELL=sim
set /p CELL="Cell name [practice mode]: "
echo.
echo  Starting. Your web browser opens by itself in a few seconds.
echo  KEEP THIS WINDOW OPEN while you use it. Close it to stop.
echo.
powershell -NoProfile -ExecutionPolicy Bypass -File "scripts\cockpit.ps1" -Cell "%CELL%" %*
echo.
echo  ==========================================================
echo   Perceptronics has stopped. If you did not stop it, read
echo   the text above and see the Troubleshooting chart on the
echo   instructions page.
echo  ==========================================================
pause
exit /b 0
:notunzipped
echo.
echo  This file is still inside the ZIP, or was moved out of its folder.
echo  Close this window. Right-click the downloaded ZIP file, choose
echo  "Extract All...", click Extract, then open the NEW folder and
echo  double-click Windows-Start there.
echo.
pause
exit /b 1
