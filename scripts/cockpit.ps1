# The pilot's seat on Windows: the RGB-D cockpit for one cell.
#
#   scripts\cockpit.ps1                  # sim cell: synthetic camera + PolyScope X sim on localhost:8000
#   scripts\cockpit.ps1 -Cell ur20       # the UR20 cell (fill UR_HOST in perception\cells\ur20.env first)
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
if ($Doctor) {
    $a = @("run", "perception", "--cell", $Cell, "doctor"); if ($Stream) { $a += "--stream" }
    & uv @a; exit $LASTEXITCODE
}
$a = @("run", "perception", "--cell", $Cell, "gui", "--port", "$Port")
if ($DryRun) { $a += "--robot-dry-run" }
if ($Fake) { $a += "--fake" }
& uv @a
