param(
    [string]$PythonExe = "C:\LeInstall\miniforge3\envs\gui10\python.exe"
)

$ErrorActionPreference = "Stop"

$packDir = $PSScriptRoot
$specPath = Join-Path $packDir "BrainCoControl.spec"
$distDir = Join-Path $packDir "dist"
$workDir = Join-Path $packDir "build"
$exeTarget = Join-Path $distDir "BrainCoControl.exe"

if (!(Test-Path -LiteralPath $PythonExe)) {
    throw "Python executable not found: $PythonExe"
}

$pythonBits = (& $PythonExe -c "import struct; print(struct.calcsize('P') * 8)").Trim()
if ($LASTEXITCODE -ne 0) {
    throw "Unable to determine Python architecture."
}
if ($pythonBits -ne "32") {
    throw "Windows x86 build requires a 32-bit Python interpreter; detected ${pythonBits}-bit."
}

& $PythonExe -m PyInstaller `
    --noconfirm `
    --clean `
    --distpath $distDir `
    --workpath $workDir `
    $specPath
if ($LASTEXITCODE -ne 0) {
    throw "PyInstaller build failed with exit code $LASTEXITCODE."
}

if (!(Test-Path -LiteralPath $exeTarget)) {
    throw "Expected executable not found: $exeTarget"
}

Write-Host "Build complete:"
Write-Host "  EXE: $exeTarget"
