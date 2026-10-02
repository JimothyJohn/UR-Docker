# The RGB-D cockpit in Docker Desktop on Windows 11, one command:
#
#   powershell -ExecutionPolicy Bypass -File scripts\docker-windows.ps1              # camera only
#   powershell -ExecutionPolicy Bypass -File scripts\docker-windows.ps1 -Fake        # no camera: prove Docker first
#   powershell -ExecutionPolicy Bypass -File scripts\docker-windows.ps1 -Cell ur3 -RobotHost 192.168.3.3
#   powershell -ExecutionPolicy Bypass -File scripts\docker-windows.ps1 -Cell mycell -Lan   # pendant / URCap can reach it
#   powershell -ExecutionPolicy Bypass -File scripts\docker-windows.ps1 -Status | -Logs | -Down
#
# What it does, in order: checks Docker Desktop (Linux containers, WSL 2 engine),
# shares the RealSense with WSL through usbipd-win (installs it, binds the camera
# once - both raise a UAC prompt - and attaches it), builds and starts
# docker-compose.windows.yml, waits for the first frames and opens the browser.
# Idempotent: re-run it after a reboot or after re-plugging the camera (an attach
# does not survive either). docs\windows-docker.md is the walk-through.
#
# Nothing here needs Python or the RealSense SDK on Windows.
# Kept ASCII and Windows PowerShell 5.1 compatible on purpose.
[CmdletBinding()]
param(
    # A shipped cell (perceptronics\cells\<name>.env) or your own deploy\windows\<name>.env.
    [string]$Cell = "",
    # The controller's IP; wins over the cell file's UR_HOST.
    [string]$RobotHost = "",
    # Synthetic RGB-D scene: no camera, no usbipd.
    [switch]$Fake,
    # Robot actions validated and audited, nothing sent.
    [switch]$DryRun,
    # Publish :7621/:7622 on every interface (default: this PC only) and allow them
    # through Windows Firewall from the local subnet. The cockpit has no login.
    [switch]$Lan,
    # The camera's usbipd bus id (usbipd list) when it is not found by name.
    [string]$BusId = "",
    [switch]$NoBrowser,
    [switch]$Status,
    [switch]$Logs,
    [switch]$Down
)
$ErrorActionPreference = "Stop"
$repo = Split-Path -Parent $PSScriptRoot
Set-Location $repo
$composeFile = "docker-compose.windows.yml"
$url = "http://localhost:7621"
$firewallRule = "perceptronics cockpit (7621-7622, local subnet)"

function Step($msg) { Write-Host "`n== $msg" -ForegroundColor Cyan }
function Fail($msg, $fix) {
    Write-Host "FAILED: $msg" -ForegroundColor Red
    if ($fix) { Write-Host "   fix: $fix" -ForegroundColor Yellow }
    exit 1
}

# Windows PowerShell 5.1 turns a native command's stderr into terminating errors
# under ErrorActionPreference=Stop as soon as it is redirected; run probes relaxed.
function Quiet([scriptblock]$block) {
    $old = $ErrorActionPreference
    $ErrorActionPreference = "Continue"
    try { & $block 2>$null } finally { $ErrorActionPreference = $old }
}

function Compose { & docker compose -f $composeFile @args }

function Is-Admin {
    $id = [Security.Principal.WindowsIdentity]::GetCurrent()
    return ([Security.Principal.WindowsPrincipal]$id).IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
}

# Run one command elevated (a UAC prompt unless this shell already is) and return its exit code.
function Elevated([string]$file, [string[]]$arguments) {
    if (Is-Admin) {
        & $file @arguments | Out-Host
        return $LASTEXITCODE
    }
    Write-Host "   (asking for administrator rights: $file $($arguments -join ' '))"
    $p = Start-Process -FilePath $file -ArgumentList $arguments -Verb RunAs -Wait -PassThru
    return $p.ExitCode
}

function Refresh-Path {
    $env:Path = [Environment]::GetEnvironmentVariable("Path", "Machine") + ";" +
        [Environment]::GetEnvironmentVariable("Path", "User")
}

# ---- the camera, as usbipd sees it ------------------------------------------
function Get-Camera {
    $state = Quiet { usbipd state } | Out-String | ConvertFrom-Json
    $devices = @($state.Devices | Where-Object { $_.BusId })
    if ($BusId) { return $devices | Where-Object { $_.BusId -eq $BusId } | Select-Object -First 1 }
    # Intel's vendor id + the product name: D435, D435i, D455... whichever is plugged in.
    return $devices |
        Where-Object { $_.InstanceId -match "VID_8086" -and $_.Description -match "RealSense" } |
        Select-Object -First 1
}

function Show-Info {
    try { $info = Invoke-RestMethod -Uri "$url/api/info" -TimeoutSec 5 } catch { return $null }
    $cam = $info.camera
    Write-Host ("   camera : {0} (serial {1}, usb {2}), {3}x{4}" -f $cam.device.name, $cam.device.serial,
        $cam.device.usb_type, $cam.stream.width, $cam.stream.height)
    Write-Host ("   frames : {0} read, {1} fps" -f $info.frames_read, $info.fps)
    if ($info.last_error) { Write-Host "   error  : $($info.last_error)" -ForegroundColor Yellow }
    if ($info.robot) { Write-Host "   robot  : $($info.cell.UR_HOST) (cell $($info.cell.UR_CELL))" }
    else { Write-Host "   robot  : none (camera only)" }
    return $info
}

# ---- Docker Desktop -----------------------------------------------------------
function Require-Docker {
    Step "Docker Desktop"
    if (-not (Get-Command docker -ErrorAction SilentlyContinue)) {
        Fail "docker is not on PATH" "install Docker Desktop (winget install --id Docker.DockerDesktop -e), start it, open a new terminal"
    }
    $os = Quiet { docker info --format "{{.OSType}}" }
    if ($LASTEXITCODE -ne 0) {
        $exe = Join-Path $env:ProgramFiles "Docker\Docker\Docker Desktop.exe"
        if (Test-Path $exe) {
            Write-Host "   Docker Desktop is not running; starting it (up to 2 min)"
            Start-Process -FilePath $exe
            for ($i = 0; $i -lt 60; $i++) {
                Start-Sleep -Seconds 2
                $os = Quiet { docker info --format "{{.OSType}}" }
                if ($LASTEXITCODE -eq 0) { break }
            }
        }
        if ($LASTEXITCODE -ne 0) { Fail "the Docker engine is not answering" "start Docker Desktop and wait for 'Engine running'" }
    }
    if ("$os".Trim() -ne "linux") {
        Fail "Docker Desktop is in Windows-containers mode" "tray icon -> 'Switch to Linux containers...'"
    }
    $kernel = "$(Quiet { docker info --format '{{.KernelVersion}}' })".Trim()
    Write-Host "   engine ok, kernel $kernel"
    if ($kernel -notmatch "WSL2") {
        Fail "Docker Desktop is not on the WSL 2 engine (kernel '$kernel'), so usbipd cannot hand it the camera" `
            "Docker Desktop -> Settings -> General -> 'Use the WSL 2 based engine', Apply & restart"
    }
}

# ---- share the RealSense with WSL --------------------------------------------
# Returns $true when this run attached it (a cockpit that was already running must
# then be restarted: it may not see a camera that arrived after it started).
# Everything a command prints goes to Out-Host so only the boolean is returned.
function Share-Camera {
    Step "RealSense -> WSL (usbipd-win)"
    if (-not (Get-Command usbipd -ErrorAction SilentlyContinue)) {
        if (-not (Get-Command winget -ErrorAction SilentlyContinue)) {
            Fail "usbipd-win is not installed and winget is not available" "install the .msi from https://github.com/dorssel/usbipd-win/releases/latest, open a new terminal"
        }
        Write-Host "   installing usbipd-win"
        winget install --id dorssel.usbipd-win -e --accept-source-agreements --accept-package-agreements | Out-Host
        Refresh-Path
        if (-not (Get-Command usbipd -ErrorAction SilentlyContinue)) {
            Fail "usbipd is still not on PATH after the install" "open a new terminal (a reboot if the installer asked for one) and re-run this script"
        }
    }
    $cam = Get-Camera
    if (-not $cam) {
        Quiet { usbipd list } | Out-Host
        Fail "no RealSense in the list above" "plug it into a USB 3 port with its own cable (no hub); if it is listed under another name, pass -BusId <BUSID>"
    }
    Write-Host "   $($cam.BusId)  $($cam.Description)"
    if ($cam.ClientIPAddress) {
        Write-Host "   already attached"
        return $false
    }
    if (-not $cam.PersistedGuid) {
        Write-Host "   sharing it (once; survives reboots)"
        $code = Elevated "usbipd" @("bind", "--busid", "$($cam.BusId)")
        if ($code -ne 0) { Fail "usbipd bind exited $code" "run in an administrator terminal: usbipd bind --busid $($cam.BusId)" }
    }
    # Windows loses the camera while it is attached (close RealSense Viewer first);
    # -Down gives it back.
    Write-Host "   attaching (the camera leaves Windows until -Down)"
    usbipd attach --wsl --busid "$($cam.BusId)" | Out-Host
    if ($LASTEXITCODE -ne 0) {
        Fail "usbipd attach exited $LASTEXITCODE (its own message is above)" "close Intel RealSense Viewer and any camera app, run 'wsl --update', re-plug the camera, re-run this script"
    }
    return $true
}

# ---- firewall (only with -Lan) -----------------------------------------------
function Open-Firewall {
    Step "Windows Firewall (7621-7622 from the local subnet)"
    if (Get-NetFirewallRule -DisplayName $firewallRule -ErrorAction SilentlyContinue) {
        Write-Host "   rule already present"
        return
    }
    $cmd = "New-NetFirewallRule -DisplayName '$firewallRule' -Direction Inbound -Action Allow " +
        "-Protocol TCP -LocalPort 7621,7622 -RemoteAddress LocalSubnet | Out-Null"
    $code = Elevated "powershell.exe" @("-NoProfile", "-Command", $cmd)
    if ($code -ne 0) { Write-Warning "could not add the firewall rule (exit $code); the pendant may not reach this PC" }
}

# ---- the verbs ------------------------------------------------------------------
if ($Logs) { Compose logs -f --tail 100 cockpit; exit $LASTEXITCODE }

if ($Down) {
    Step "stopping"
    Compose down
    if (Get-Command usbipd -ErrorAction SilentlyContinue) {
        $cam = Get-Camera
        if ($cam -and $cam.ClientIPAddress) {
            usbipd detach --busid "$($cam.BusId)"
            Write-Host "   camera $($cam.BusId) is back with Windows"
        }
    }
    exit 0
}

if ($Status) {
    Step "camera (usbipd)"
    if (Get-Command usbipd -ErrorAction SilentlyContinue) {
        $cam = Get-Camera
        if ($cam) {
            $shared = [bool]$cam.PersistedGuid
            $attached = [bool]$cam.ClientIPAddress
            Write-Host "   $($cam.BusId)  $($cam.Description)  shared=$shared attached=$attached"
        } else { Write-Host "   no RealSense plugged in" }
    } else { Write-Host "   usbipd-win not installed" }
    Step "container"
    Compose ps
    Step "cockpit ($url)"
    if (-not (Show-Info)) { Write-Host "   not answering" }
    exit 0
}

Require-Docker

# What the container is told. These five are this script's; set or cleared on every
# run so a re-run without a flag really is without it.
$cockpitArgs = @()
$env:UR_CELL = $null
$env:UR_HOST = $null
$env:PERCEPTRONICS_FAKE = $null
$env:PERCEPTRONICS_PUBLISH = $null
if ($Cell) {
    $own = [IO.Path]::Combine($repo, "deploy", "windows", "$Cell.env")
    $shipped = [IO.Path]::Combine($repo, "perceptronics", "cells", "$Cell.env")
    if (Test-Path $own) { $env:UR_CELL = "/cells/$Cell.env" }
    elseif (Test-Path $shipped) { $env:UR_CELL = $Cell }
    else {
        $names = (Get-ChildItem ([IO.Path]::Combine($repo, "perceptronics", "cells")) -Filter *.env).BaseName -join ", "
        Fail "no cell '$Cell'" "one of: $names - or copy deploy\windows\cell.env.example to deploy\windows\$Cell.env"
    }
}
if ($RobotHost) { $env:UR_HOST = $RobotHost }
if (-not $Cell -and -not $RobotHost) { $cockpitArgs += "--no-robot" }
if ($DryRun) { $cockpitArgs += "--robot-dry-run" }
if ($Fake) { $env:PERCEPTRONICS_FAKE = "1" }
if ($Lan) { $env:PERCEPTRONICS_PUBLISH = "0.0.0.0" }
$env:COCKPIT_ARGS = $cockpitArgs -join " "

$attachedNow = $false
if (-not $Fake) { $attachedNow = Share-Camera }
if ($Lan) { Open-Firewall }

$wasRunning = [bool](Quiet { Compose ps -q cockpit })
Step "build + start (the first build compiles librealsense: about 10 minutes, then cached)"
Compose up -d --build
if ($LASTEXITCODE -ne 0) { Fail "docker compose up exited $LASTEXITCODE" "read the output above; '$PSCommandPath -Logs' for the cockpit's own" }
if ($attachedNow -and $wasRunning) {
    # The camera arrived after this container started: make it look again.
    Compose restart cockpit
}

Step "waiting for the picture"
$info = $null
for ($i = 0; $i -lt 45; $i++) {
    Start-Sleep -Seconds 2
    try { $info = Invoke-RestMethod -Uri "$url/api/info" -TimeoutSec 3 } catch { continue }
    if ($info.frames_read -gt 0) { break }
}
if (-not $info) {
    Compose logs --tail 40 cockpit
    Fail "the cockpit never answered on $url" "the log above says why; is another program on port 7621?"
}
[void](Show-Info)
if ($info.frames_read -le 0) {
    Compose logs --tail 40 cockpit
    Fail "the cockpit is up but the camera delivers no frames" "re-plug the camera into a USB 3 port, then re-run this script (docs\windows-docker.md, Troubleshooting)"
}

Write-Host "`ncockpit: $url" -ForegroundColor Green
if ($Lan) {
    $ips = @(Get-NetIPAddress -AddressFamily IPv4 -ErrorAction SilentlyContinue |
        Where-Object { $_.IPAddress -notmatch "^(127\.|169\.254\.)" -and $_.InterfaceAlias -notmatch "vEthernet|WSL|Loopback" })
    foreach ($ip in $ips) { Write-Host ("   from the cell network: http://{0}:7621  ({1})" -f $ip.IPAddress, $ip.InterfaceAlias) }
    Write-Host "   URCap 'Cockpit' field: the address on the robot's subnet; the Pick node uses :7622 on the same one"
}
Write-Host "stop: scripts\docker-windows.ps1 -Down    logs: -Logs    state: -Status"
if (-not $NoBrowser) { Start-Process $url }
