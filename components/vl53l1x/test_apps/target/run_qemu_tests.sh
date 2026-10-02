#!/bin/bash
# Build the vl53l1x target test app for the ESP32-C3 and run it in QEMU.
# Linux counterpart of run_qemu_tests.ps1; CI runs it inside the espressif/idf Docker image.
#
# QEMU has no VL53L1X on its I2C bus, so only the [config] tests run here.
# The app is built with sdkconfig.qemu layered on sdkconfig.defaults, so it runs the tests at
# boot and prints a summary line. This script waits for that line, then stops QEMU.
#
# Run from an ESP-IDF environment (export.sh, or the espressif/idf image).
# Environment overrides:
#   BUILD_DIR   Build directory (default: build_qemu next to this script)
#   TIMEOUT     Seconds to wait for the test summary (default: 120)
#
# Exit code: 0 if all tests pass, 1 if any fail, 2 on build or QEMU problems.

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${BUILD_DIR:-$SCRIPT_DIR/build_qemu}"
TIMEOUT="${TIMEOUT:-120}"

if ! command -v idf.py > /dev/null; then
    echo "idf.py not found. Run from an ESP-IDF environment (source \$IDF_PATH/export.sh)." >&2
    exit 2
fi

cd "$SCRIPT_DIR" || exit 2
mkdir -p "$BUILD_DIR"

IDF_ARGS=(-B "$BUILD_DIR" -D "SDKCONFIG=$BUILD_DIR/sdkconfig" -D "SDKCONFIG_DEFAULTS=sdkconfig.defaults;sdkconfig.qemu")

echo "Building for QEMU in $BUILD_DIR"
if ! idf.py "${IDF_ARGS[@]}" build > "$BUILD_DIR/build.log" 2>&1; then
    grep -E "error:|FAILED" "$BUILD_DIR/build.log" | head -40
    echo "Build failed; full log: $BUILD_DIR/build.log" >&2
    exit 2
fi

LOG="$BUILD_DIR/qemu_output.log"
rm -f "$LOG"
echo "Running tests in QEMU (timeout ${TIMEOUT}s)"

# idf.py qemu runs until stopped. setsid gives it its own process group so the QEMU
# child process can be stopped along with it.
setsid idf.py "${IDF_ARGS[@]}" qemu < /dev/null > "$LOG" 2>&1 &
QEMU_PID=$!

deadline=$((SECONDS + TIMEOUT))
while [ $SECONDS -lt $deadline ] && kill -0 "$QEMU_PID" 2> /dev/null; do
    grep -q "VL53L1X_QEMU_DONE" "$LOG" 2> /dev/null && break
    sleep 0.5
done
kill -- "-$QEMU_PID" 2> /dev/null
wait "$QEMU_PID" 2> /dev/null

# Show the Unity output: each test result and the summary
grep -E ":(PASS|FAIL|IGNORE)|Tests [0-9]+ Failures|^OK|^FAIL" "$LOG"

result=$(sed -n 's/.*VL53L1X_QEMU_DONE failures=\([0-9]\+\).*/\1/p' "$LOG" | head -1)
if [ -z "$result" ]; then
    echo "No test summary within ${TIMEOUT}s. QEMU output: $LOG" >&2
    exit 2
fi
if [ "$result" -gt 0 ]; then
    echo "$result test(s) failed" >&2
    exit 1
fi
echo "All QEMU tests passed"
