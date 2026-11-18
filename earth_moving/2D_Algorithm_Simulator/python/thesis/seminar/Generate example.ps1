param(
    [string]$Scenario = "$PSScriptRoot\scenarios\corridor.json",
    [string]$Output = "$PSScriptRoot\running_example",
    [string]$Python = "$env:USERPROFILE\miniforge3\envs\earth_moving\python.exe"
)
$ErrorActionPreference = 'Stop'
if (-not (Test-Path -LiteralPath $Python)) { throw 'Pass -Python with the path to a Python environment containing numpy, scipy, shapely and matplotlib.' }
$pythonDirectory = Split-Path -Parent $Python
$previousPath = $env:PATH
try {
    $env:PATH = "$pythonDirectory\Library\bin;$pythonDirectory;$previousPath"
    & $Python "$PSScriptRoot\..\scripts\build_seminar_example_20261114.py" --scenario $Scenario --output $Output
    if ($LASTEXITCODE -ne 0) { throw 'Generation or the full-recalculation check failed. Inspect the output before using these figures.' }
} finally {
    $env:PATH = $previousPath
}
