# Bring a Windows laptop up as the cell's brain: Python, the Intel RealSense SDK 2.0
# (realsense2.dll), then the pre-flight doctor. Nothing else is installed: the package
# is stdlib-only and runs from this folder.
#
#   powershell -ExecutionPolicy Bypass -File scripts\setup-windows.ps1
#   powershell -ExecutionPolicy Bypass -File scripts\setup-windows.ps1 -Cell ur20
#
# Idempotent: re-run it after plugging the camera in or fixing the network.
# Python comes from winget; the RealSense SDK installer asks for elevation itself.
[CmdletBinding()]
param(
    [string]$Cell = "sim",
    # librealsense release whose Windows installer we fetch (GitHub Releases asset name pattern:
    # RealSense.SDK-WIN10-<ver>.<build>.exe). Keep in step with Dockerfile.perceptronics's LIBREALSENSE_REF.
    [string]$SdkVersion = "2.58.4",
    # The winget package installed when no Python >= 3.10 is found.
    [string]$PythonPackage = "Python.Python.3.13",
    [switch]$SkipSdk,
    [switch]$Stream
)
$ErrorActionPreference = "Stop"
$repo = Split-Path -Parent $PSScriptRoot
Set-Location $repo
. "$PSScriptRoot\_python.ps1"

function Step($msg) { Write-Host "`n== $msg" -ForegroundColor Cyan }

# ---- 1. Python ---------------------------------------------------------------
Step "Python"
$python = Find-Python
if (-not $python) {
    if (Get-Command winget -ErrorAction SilentlyContinue) {
        Write-Host "no Python 3.10+ found; installing $PythonPackage with winget"
        winget install --id $PythonPackage -e --accept-source-agreements --accept-package-agreements
        Update-PathFromRegistry
        $python = Find-Python
    }
    if (-not $python) {
        Write-Host "Python 3.10 or newer is needed and could not be installed automatically." -ForegroundColor Red
        Write-Host "Install it from https://www.python.org/downloads/windows/ (tick 'Add python.exe to PATH')," -ForegroundColor Yellow
        Write-Host "then run this setup again." -ForegroundColor Yellow
        exit 1
    }
}
Invoke-Python $python @("--version")

# ---- 2. Intel RealSense SDK 2.0 (realsense2.dll) ----------------------------
$sdkDll = "C:\Program Files (x86)\Intel RealSense SDK 2.0\bin\x64\realsense2.dll"
if (-not $SkipSdk) {
    Step "RealSense SDK"
    if (Test-Path $sdkDll) {
        Write-Host "found $sdkDll"
    } else {
        # The asset carries a build number after the version; resolve it from the release.
        $rel = Invoke-RestMethod "https://api.github.com/repos/realsenseai/librealsense/releases/tags/v$SdkVersion"
        $asset = $rel.assets | Where-Object { $_.name -like "RealSense.SDK-WIN10-$SdkVersion*.exe" } | Select-Object -First 1
        if (-not $asset) { throw "no Windows SDK installer asset on librealsense release v$SdkVersion" }
        $exe = Join-Path $env:TEMP $asset.name
        Write-Host "downloading $($asset.name) ($([math]::Round($asset.size / 1MB)) MB)"
        Invoke-WebRequest -Uri $asset.browser_download_url -OutFile $exe
        Write-Host "running the installer (click through; defaults are fine; it needs the SDK, not the Viewer)"
        Start-Process -FilePath $exe -Wait
        if (-not (Test-Path $sdkDll)) {
            Write-Warning "realsense2.dll not at the default path after install; set REALSENSE_LIB to where it landed"
        }
    }
    if (Test-Path $sdkDll) {
        [System.Environment]::SetEnvironmentVariable("REALSENSE_LIB", $sdkDll, "User")
        $env:REALSENSE_LIB = $sdkDll
        Write-Host "REALSENSE_LIB=$sdkDll (user environment)"
    }
}

# ---- 3. pre-flight -------------------------------------------------------------
Step "doctor ($Cell)"
$doctor = @("-m", "perceptronics", "--cell", $Cell, "doctor")
if ($Stream) { $doctor += "--stream" }
Invoke-Python $python $doctor
Write-Host "`nnext: scripts\cockpit.ps1 -Cell $Cell    (the pilot's seat, opens the browser)" -ForegroundColor Green
exit 0
