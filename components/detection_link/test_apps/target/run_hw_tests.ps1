<#
.SYNOPSIS
Build the detection_link target test app, flash it to an ESP32-C3 and run its tests.

.DESCRIPTION
Runs every test on the board by default, including the [hw] tests that use the Wi-Fi radio.
They need one board and no second device: they talk to a peer that does not exist.
The board is found by its USB vendor ID unless -Port is given.

Exit code: 0 if all tests pass, 1 if any fail, 2 on build, flash or serial problems.

.EXAMPLE
.\components\detection_link\test_apps\target\run_hw_tests.ps1
.\components\detection_link\test_apps\target\run_hw_tests.ps1 -Port COM5 -Filter "[hw]"
#>
param(
    [string]$Port,
    # Unity menu input: * for all tests, a [tag], or a test name
    [string]$Filter = "*",
    # ESP-IDF environment script installed by the Espressif Installation Manager
    [string]$IdfProfile = "C:\Espressif\tools\Microsoft.v6.1.PowerShell_profile.ps1",
    [int]$TimeoutSeconds = 300,
    # Defaults to a folder per console type under the temp directory
    [string]$BuildDir,
    # Delete the build directory first
    [switch]$Clean
)

$ErrorActionPreference = "Stop"

function Fail([string]$message) {
    Write-Host $message -ForegroundColor Red
    exit 2
}
$appDir = $PSScriptRoot
$menuScript = Join-Path $appDir "run_unity_menu.py"

if (-not (Test-Path $IdfProfile)) {
    Fail "ESP-IDF environment script not found: $IdfProfile"
}
. $IdfProfile *> $null

$python = (Get-Command python).Source
$idfPy = Join-Path $env:IDF_PATH "tools\idf.py"

if (-not $Port) {
    $Port = & $python $menuScript --print-port
    if ($LASTEXITCODE -ne 0) {
        Fail "Could not find the board."
    }
}
$console = & $python $menuScript --port $Port --print-console
Write-Host "Using board on $Port ($console console)"

# Kept short and outside the checkout: Wi-Fi object paths overflow Windows' 260-character limit.
# One folder per console, since an existing sdkconfig takes precedence over changed defaults.
if (-not $BuildDir) {
    $BuildDir = Join-Path ([IO.Path]::GetTempPath()) "detection_link_hw_$console"
}
$defaults = "sdkconfig.defaults"
if ($console -eq "usb_serial_jtag") {
    $defaults += ";sdkconfig.usb_serial_jtag"
}

if ($Clean -and (Test-Path $buildDir)) {
    Remove-Item -Recurse -Force $buildDir
}

# Keep the generated sdkconfig in the build directory, so the QEMU and board builds do not mix
$idfArgs = @(
    $idfPy, "-C", $appDir, "-B", $buildDir,
    "-D", "SDKCONFIG=$buildDir\sdkconfig",
    "-D", "SDKCONFIG_DEFAULTS=$defaults"
)

Write-Host "Building in $buildDir"
& $python @idfArgs build 2>&1 | Select-String -Pattern "error:|FAILED" | ForEach-Object { Write-Host $_ }
if ($LASTEXITCODE -ne 0) {
    Fail "Build failed. Rerun with: python $idfPy -C $appDir -B $buildDir build"
}

Write-Host "Flashing"
& $python @idfArgs -p $Port flash 2>&1 | Select-String -Pattern "error|Error|Hard resetting" | ForEach-Object { Write-Host $_ }
if ($LASTEXITCODE -ne 0) {
    Fail "Flashing $Port failed."
}

Write-Host "Running tests matching '$Filter' (timeout ${TimeoutSeconds}s)"
& $python $menuScript --port $Port --filter $Filter --timeout $TimeoutSeconds
exit $LASTEXITCODE
