<#
.SYNOPSIS
Build the vl53l1x target test app for the ESP32-C3 and run it in QEMU.

.DESCRIPTION
QEMU has no VL53L1X on its I2C bus, so only the [config] tests run here.
They are compiled for the real RISC-V target, which checks the target compiler behaviour
without a board. The sensor tests need hardware.

The app is built with sdkconfig.qemu layered on sdkconfig.defaults, so it runs the tests
at boot and prints a summary line. This script waits for that line, then stops QEMU.

Exit code: 0 if all tests pass, 1 if any fail, 2 on build or QEMU problems.

.EXAMPLE
.\components\vl53l1x\test_apps\target\run_qemu_tests.ps1
#>
param(
    # ESP-IDF environment script installed by the Espressif Installation Manager
    [string]$IdfProfile = "C:\Espressif\tools\Microsoft.v6.1.PowerShell_profile.ps1",
    [int]$TimeoutSeconds = 120,
    # Delete the QEMU build directory first
    [switch]$Clean
)

$ErrorActionPreference = "Stop"

function Fail([string]$message) {
    Write-Host $message -ForegroundColor Red
    exit 2
}
$appDir = $PSScriptRoot
$buildDir = Join-Path $appDir "build_qemu"
$logFile = Join-Path $buildDir "qemu_output.log"
$errFile = Join-Path $buildDir "qemu_stderr.log"

if (-not (Test-Path $IdfProfile)) {
    Fail "ESP-IDF environment script not found: $IdfProfile"
}
. $IdfProfile *> $null

$python = (Get-Command python).Source
$idfPy = Join-Path $env:IDF_PATH "tools\idf.py"

if ($Clean -and (Test-Path $buildDir)) {
    Remove-Item -Recurse -Force $buildDir
}

$idfArgs = @(
    $idfPy, "-C", $appDir, "-B", $buildDir,
    "-D", "SDKCONFIG=$buildDir\sdkconfig",
    "-D", "SDKCONFIG_DEFAULTS=sdkconfig.defaults;sdkconfig.qemu"
)

Write-Host "Building for QEMU in $buildDir"
& $python @idfArgs build 2>&1 | Select-String -Pattern "error:|FAILED" | ForEach-Object { Write-Host $_ }
if ($LASTEXITCODE -ne 0) {
    Fail "Build failed. Rerun with: python $idfPy -C $appDir -B $buildDir build"
}

# The EIM profile does not put QEMU on PATH, and idf_tools.py may install it under
# <IDF_TOOLS_PATH>\tools, so look for it under the tools directory if needed
if (-not (Get-Command qemu-system-riscv32 -ErrorAction SilentlyContinue)) {
    $toolsRoot = if ($env:IDF_TOOLS_PATH) { $env:IDF_TOOLS_PATH } else { "C:\Espressif\tools" }
    $qemuExe = Get-ChildItem $toolsRoot -Recurse -Depth 6 -Filter "qemu-system-riscv32.exe" -ErrorAction SilentlyContinue |
        Sort-Object FullName -Descending | Select-Object -First 1
    if (-not $qemuExe) {
        Fail "qemu-system-riscv32 not found. Install it with: python $env:IDF_PATH\tools\idf_tools.py install qemu-riscv32"
    }
    $env:PATH = "$($qemuExe.DirectoryName);$env:PATH"
}

Write-Host "Running tests in QEMU (timeout ${TimeoutSeconds}s)"
Remove-Item -ErrorAction SilentlyContinue $logFile, $errFile
# Start-Process joins arguments without quoting, so quote each one
$quotedArgs = ($idfArgs + "qemu") | ForEach-Object { "`"$_`"" }
$qemu = Start-Process -FilePath $python -ArgumentList $quotedArgs -NoNewWindow -PassThru `
    -RedirectStandardOutput $logFile -RedirectStandardError $errFile

$result = $null
$deadline = (Get-Date).AddSeconds($TimeoutSeconds)
try {
    while ((Get-Date) -lt $deadline -and -not $qemu.HasExited) {
        Start-Sleep -Milliseconds 500
        if (Test-Path $logFile) {
            $done = Select-String -Path $logFile -Pattern "VL53L1X_QEMU_DONE failures=(\d+)" | Select-Object -First 1
            if ($done) {
                $result = [int]$done.Matches[0].Groups[1].Value
                break
            }
        }
    }
}
finally {
    # idf.py starts QEMU as a child process, so stop the whole tree
    if (-not $qemu.HasExited) {
        taskkill /PID $qemu.Id /T /F *> $null
    }
}

# Show the Unity output: each test result and the summary
if (Test-Path $logFile) {
    Get-Content $logFile | Select-String -Pattern ":(PASS|FAIL|IGNORE)|Tests \d+ Failures|^OK$|^FAIL$" |
        ForEach-Object { Write-Host $_.Line }
}

if ($null -eq $result) {
    Fail "No test summary within ${TimeoutSeconds}s. QEMU output: $logFile, $errFile"
}
if ($result -gt 0) {
    Write-Host "$result test(s) failed" -ForegroundColor Red
    exit 1
}
Write-Host "All QEMU tests passed" -ForegroundColor Green
exit 0
