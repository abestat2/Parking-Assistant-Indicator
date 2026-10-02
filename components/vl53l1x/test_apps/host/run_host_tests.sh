#!/bin/bash
# Build and run the vl53l1x host tests (config + simulated sensor) on the ESP-IDF linux target.
#
# Run inside WSL (or any Linux/macOS machine with ESP-IDF). From Windows, at the repo root:
#   wsl -d Ubuntu-24.04 -- bash components/vl53l1x/test_apps/host/run_host_tests.sh
#
# Needs: ruby, libbsd-dev, cmake, ninja-build, and the ESP-IDF Python env
# (python3 $IDF_PATH/tools/idf_tools.py install-python-env).
# Host builds only need the Python env and the system compiler, so this calls idf.py
# directly instead of export.sh, which would also require the chip toolchains.
#
# Environment overrides:
#   IDF_PATH             ESP-IDF checkout (default: the Windows install via /mnt/c)
#   IDF_PYTHON_ENV_PATH  ESP-IDF Python venv
#   BUILD_DIR            Build directory (default: /tmp/vl53l1x_host_build; kept off /mnt/c for speed)
#   CLEAN=1              Delete the build directory first
#   COVERAGE=1           Instrument the vl53l1x component and print an lcov coverage summary
#                        (needs lcov; builds in a separate directory)

set -u

COVERAGE="${COVERAGE:-0}"
if [ "$COVERAGE" = "1" ]; then
    BUILD_DIR="${BUILD_DIR:-/tmp/vl53l1x_host_build_coverage}"
else
    BUILD_DIR="${BUILD_DIR:-/tmp/vl53l1x_host_build}"
fi
export COVERAGE
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -n "${IDF_PATH:-}" ] && command -v idf.py > /dev/null; then
    # An activated ESP-IDF environment, such as the espressif/idf Docker image used in CI
    idf() { idf.py "$@"; }
else
    export IDF_PATH="${IDF_PATH:-/mnt/c/esp/v6.1/esp-idf}"
    export ESP_IDF_VERSION="${ESP_IDF_VERSION:-6.1}"
    export IDF_PYTHON_ENV_PATH="${IDF_PYTHON_ENV_PATH:-$(ls -d "$HOME"/.espressif/python_env/idf${ESP_IDF_VERSION}_py*_env 2>/dev/null | head -1)}"

    if [ ! -d "$IDF_PATH" ]; then
        echo "ESP-IDF not found at IDF_PATH=$IDF_PATH" >&2
        exit 2
    fi
    if [ -z "$IDF_PYTHON_ENV_PATH" ] || [ ! -x "$IDF_PYTHON_ENV_PATH/bin/python" ]; then
        echo "ESP-IDF Python env not found. Create it with:" >&2
        echo "  python3 $IDF_PATH/tools/idf_tools.py install-python-env" >&2
        exit 2
    fi
    idf() { "$IDF_PYTHON_ENV_PATH/bin/python" "$IDF_PATH/tools/idf.py" "$@"; }
fi

for tool in ruby cmake ninja; do
    if ! command -v "$tool" > /dev/null; then
        echo "Missing '$tool'. Install with: sudo apt-get install -y ruby libbsd-dev cmake ninja-build" >&2
        exit 2
    fi
done

cd "$SCRIPT_DIR" || exit 2

if [ "${CLEAN:-0}" = "1" ]; then
    rm -rf "$BUILD_DIR"
fi

if [ ! -f "$BUILD_DIR/CMakeCache.txt" ]; then
    echo "Configuring for the linux target in $BUILD_DIR"
    if ! idf -B "$BUILD_DIR" --preview set-target linux > "$BUILD_DIR.set-target.log" 2>&1; then
        tail -30 "$BUILD_DIR.set-target.log"
        exit 2
    fi
fi

echo "Building"
if ! idf -B "$BUILD_DIR" build > "$BUILD_DIR.build.log" 2>&1; then
    grep -E "error|FAILED" "$BUILD_DIR.build.log" | grep -v "error\.c\.o" | head -40
    echo "Build failed; full log: $BUILD_DIR.build.log" >&2
    exit 2
fi

if [ "$COVERAGE" = "1" ]; then
    # Drop counts from earlier runs so the report covers this run only
    find "$BUILD_DIR" -name "*.gcda" -delete
fi

echo "Running tests"
"$BUILD_DIR/vl53l1x_host_test.elf"
result=$?

if [ "$COVERAGE" = "1" ]; then
    # gcov writes annotated sources (*.gcov, "#####" marks unrun lines) into the object directory
    OBJ_DIR="$BUILD_DIR/esp-idf/vl53l1x/CMakeFiles/__idf_vl53l1x.dir"
    echo
    echo "Coverage of the vl53l1x component (ST's VL53L1X_api.c is built through vl53l1x_uld.c):"
    (cd "$OBJ_DIR" && for gcda in *.gcda; do gcov -b -c -o . "$gcda"; done) 2> /dev/null |
        awk '/^File / { file = $2; gsub(/\x27/, "", file); sub(/.*\/vl53l1x\//, "", file) }
             /^Lines executed:/ { lines = $0; sub(/^Lines executed:/, "", lines) }
             /^Taken at least once:/ { branches = $0; sub(/^Taken at least once:/, "", branches) }
             /^No branches/ { branches = "none" }
             /^Creating / && file != "" && file !~ /^\// {
                 printf "  %-28s lines %-14s branches %s\n", file, lines, branches; file = ""; branches = "" }'
    echo "Annotated sources: $OBJ_DIR/*.gcov"
fi

exit $result
