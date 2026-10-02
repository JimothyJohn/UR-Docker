# Shared by setup-windows.ps1 and cockpit.ps1 (dot-sourced): find a real Python >= 3.10.
# The package is stdlib-only and runs from this checkout (`python -m perceptronics`), so a
# Python is the whole install. Kept ASCII and Windows PowerShell 5.1 compatible.

# What winget / the python.org installer just added is in the registry, not in this shell.
function Update-PathFromRegistry {
    $env:Path = [Environment]::GetEnvironmentVariable("Path", "Machine") + ";" +
        [Environment]::GetEnvironmentVariable("Path", "User")
}

# The command (an array: executable + leading arguments) of a Python >= 3.10, or $null.
# The py launcher first; then python on PATH - but not the Microsoft Store stub, which
# exists as python.exe, prints an advert and runs nothing.
function Find-Python {
    foreach ($candidate in @(@("py", "-3"), @("python"), @("python3"))) {
        $exe = $candidate[0]
        $pre = @($candidate | Select-Object -Skip 1)
        if (-not (Get-Command $exe -ErrorAction SilentlyContinue)) { continue }
        # 5.1 turns a native command's stderr into a terminating error under Stop: probe relaxed.
        $old = $ErrorActionPreference
        $ErrorActionPreference = "Continue"
        try { $answer = & $exe @pre -c "import sys; print(sys.version_info >= (3, 10))" 2>$null }
        catch { $answer = "" }
        finally { $ErrorActionPreference = $old }
        if ("$answer".Trim() -eq "True") { return , $candidate }
    }
    return $null
}

# Run `python -m <module> <arguments>` with the Python Find-Python returned.
function Invoke-Python($python, [string[]]$arguments) {
    $exe = $python[0]
    $pre = @($python | Select-Object -Skip 1)
    & $exe @pre @arguments
}
