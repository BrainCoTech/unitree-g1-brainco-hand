param(
    [string]$PythonExe = "C:\LeInstall\miniforge3\envs\gui10\python.exe"
)

$ErrorActionPreference = "Stop"

$uiClientDir = Split-Path -Parent $PSScriptRoot
$repoRoot = Split-Path -Parent $uiClientDir
$specPath = Join-Path $PSScriptRoot "UIClient.spec"
$distDir = Join-Path $repoRoot "dist"
$exeTarget = Join-Path $distDir "UIClient.exe"
$configSource = Join-Path $uiClientDir "config\client_config.yaml"
$configTarget = Join-Path $distDir "client_config.yaml"

& $PythonExe -m PyInstaller --noconfirm --clean --distpath $distDir $specPath
if ($LASTEXITCODE -ne 0) {
    throw "PyInstaller build failed with exit code $LASTEXITCODE."
}

if (!(Test-Path $exeTarget)) {
    throw "Expected executable not found: $exeTarget"
}

if (!(Test-Path $distDir)) {
    New-Item -ItemType Directory -Path $distDir | Out-Null
}

Copy-Item -LiteralPath $configSource -Destination $configTarget -Force

Write-Host "Build complete:"
Write-Host "  EXE:    $exeTarget"
Write-Host "  Config: $configTarget"
