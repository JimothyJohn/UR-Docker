# Bring a Windows laptop up as the cell's brain: uv (+ Python), the Intel RealSense
# SDK 2.0 (realsense2.dll), the repo's venv, then the pre-flight doctor.
#
#   powershell -ExecutionPolicy Bypass -File scripts\setup-windows.ps1
#   powershell -ExecutionPolicy Bypass -File scripts\setup-windows.ps1 -Cell ur20
#
# Idempotent: re-run it after plugging the camera in or fixing the network.
# No admin needed for uv/venv; the RealSense SDK installer asks for elevation itself.
[CmdletBinding()]
param(
    [string]$Cell = "sim",
    # librealsense release whose Windows installer we fetch (GitHub Releases asset name pattern:
    # RealSense.SDK-WIN10-<ver>.<build>.exe). Keep in step with Dockerfile.perception's LIBREALSENSE_REF.
    [string]$SdkVersion = "2.58.4",
    [switch]$SkipSdk,
    [switch]$Stream
)
$ErrorActionPreference = "Stop"
$repo = Split-Path -Parent $PSScriptRoot
Set-Location $repo

function Step($msg) { Write-Host "`n== $msg" -ForegroundColor Cyan }

# ---- 1. uv -----------------------------------------------------------------
Step "uv"
if (-not (Get-Command uv -ErrorAction SilentlyContinue)) {
    if (Get-Command winget -ErrorAction SilentlyContinue) {
        winget install --id astral-sh.uv -e --accept-source-agreements --accept-package-agreements
    } else {
        Write-Host "winget not available; installing uv with the official script"
        powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"
    }
    $env:Path = [System.Environment]::GetEnvironmentVariable("Path", "User") + ";" + $env:Path
}
uv --version

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

# ---- 3. venv -----------------------------------------------------------------
Step "uv sync"
uv sync

# ---- 4. pre-flight -------------------------------------------------------------
Step "doctor ($Cell)"
$args = @("run", "perception", "--cell", $Cell, "doctor")
if ($Stream) { $args += "--stream" }
& uv @args
Write-Host "`nnext: scripts\cockpit.ps1 -Cell $Cell    (the pilot's seat, opens the browser)" -ForegroundColor Green
