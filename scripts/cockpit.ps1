# The pilot's seat on Windows: the RGB-D cockpit for one cell.
#
#   scripts\cockpit.ps1                  # sim cell: synthetic camera + PolyScope X sim on localhost:8000
#   scripts\cockpit.ps1 -Cell ur20       # the UR20 cell (fill UR_HOST in perceptronics\cells\ur20.env first)
#   scripts\cockpit.ps1 -Cell ur3 -DryRun   # robot actions validated + audited, nothing sent
#   scripts\cockpit.ps1 -Cell ur20 -Doctor  # just the pre-flight, no cockpit
[CmdletBinding()]
param(
    [string]$Cell = "sim",
    [switch]$DryRun,
    [switch]$Fake,
    [switch]$Doctor,
    [switch]$Stream,
    [int]$Port = 7621
)
$ErrorActionPreference = "Stop"
Set-Location (Split-Path -Parent $PSScriptRoot)
. "$PSScriptRoot\_python.ps1"
Update-PathFromRegistry   # a Python installed a moment ago is not on this shell's PATH yet
$python = Find-Python
if (-not $python) {
    Write-Host "Setup has not been run yet (no Python 3.10+ was found)." -ForegroundColor Red
    Write-Host "Run Windows-Setup (scripts\setup-windows.ps1) first." -ForegroundColor Yellow
    exit 1
}
if ($Doctor) {
    $a = @("-m", "perceptronics", "--cell", $Cell, "doctor"); if ($Stream) { $a += "--stream" }
    Invoke-Python $python $a; exit $LASTEXITCODE
}
$a = @("-m", "perceptronics", "--cell", $Cell, "gui", "--port", "$Port")
if ($DryRun) { $a += "--robot-dry-run" }
if ($Fake) { $a += "--fake" }
Invoke-Python $python $a
exit $LASTEXITCODE
