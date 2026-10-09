#!/bin/bash
# Run one component's host tests and save its line coverage as a gcovr tracefile.
# Used by CI inside the espressif/idf container, with the checkout copied to /work.
#
#   bash .github/scripts/host_tests_coverage.sh <component_dir> <out_dir>
#
# <component_dir> is relative to the repo root, e.g. components/mb1013 or
# external/ESP-NOW-Transport/detection_link (a submodule).
#
# Writes <out_dir>/<component>.json for coverage_report.sh to merge. Exits with the test result,
# so a failing test still fails the step after its coverage is saved.

set -u

COMPONENT_DIR="$1"
COMPONENT="$(basename "$COMPONENT_DIR")"
OUT_DIR="$2"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export BUILD_DIR="/tmp/${COMPONENT}_host_build"

bash "$REPO_DIR/$COMPONENT_DIR/test_apps/host/run_host_tests.sh"
result=$?

if [ ! -f "$BUILD_DIR/CMakeCache.txt" ]; then
    # The build never configured, so there is no coverage to save
    exit "$result"
fi

python -m pip install --quiet --disable-pip-version-check "gcovr==8.*" || exit 2

# Only the component's own sources are built with --coverage; the filter also drops its test code
# and vendored third-party code (vl53l1x/st_uld is ST's driver, of which only part is used)
gcovr --root "$REPO_DIR" \
    --filter "$REPO_DIR/$COMPONENT_DIR/" \
    --exclude "$REPO_DIR/$COMPONENT_DIR/test_apps/" \
    --exclude "$REPO_DIR/$COMPONENT_DIR/st_uld/" \
    --json "$OUT_DIR/$COMPONENT.json" \
    "$BUILD_DIR" || exit 2

exit "$result"
