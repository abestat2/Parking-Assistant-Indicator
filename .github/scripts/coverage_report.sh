#!/bin/bash
# Merge the host test tracefiles from host_tests_coverage.sh into the coverage report and badge.
# Used by CI inside the espressif/idf container, with the checkout copied to /work.
#
#   bash .github/scripts/coverage_report.sh <out_dir>
#
# Writes into <out_dir>:
#   html/index.html  Line-by-line report
#   coverage.txt     Per-file summary
#   coverage.svg     Badge with the total line coverage

set -eu

OUT_DIR="$1"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

if ! ls "$OUT_DIR"/*.json > /dev/null 2>&1; then
    echo "No tracefiles in $OUT_DIR; the host tests did not build" >&2
    exit 2
fi

python -m pip install --quiet --disable-pip-version-check "gcovr==8.*"

mkdir -p "$OUT_DIR/html"
gcovr --root "$REPO_DIR" \
    --add-tracefile "$OUT_DIR/*.json" \
    --txt "$OUT_DIR/coverage.txt" \
    --json-summary "$OUT_DIR/summary.json" \
    --html-details "$OUT_DIR/html/index.html"
cat "$OUT_DIR/coverage.txt"

python - "$OUT_DIR/summary.json" "$OUT_DIR/coverage.svg" <<'PY'
import json
import sys

percent = json.load(open(sys.argv[1]))["line_percent"]
value = f"{percent:.0f}%"
color = next(c for limit, c in [(90, "#4c1"), (75, "#97ca00"), (60, "#dfb317"), (40, "#fe7d37"), (0, "#e05d44")]
             if percent >= limit)

# Same layout as a shields.io flat badge; widths assume Verdana 11px (about 7px per character)
label = "coverage"
label_w = 61
value_w = 7 * len(value) + 10
width = label_w + value_w
svg = f"""<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="20" role="img" aria-label="{label}: {value}">
<title>{label}: {value}</title>
<linearGradient id="s" x2="0" y2="100%"><stop offset="0" stop-color="#bbb" stop-opacity=".1"/><stop offset="1" stop-opacity=".1"/></linearGradient>
<clipPath id="r"><rect width="{width}" height="20" rx="3" fill="#fff"/></clipPath>
<g clip-path="url(#r)">
<rect width="{label_w}" height="20" fill="#555"/>
<rect x="{label_w}" width="{value_w}" height="20" fill="{color}"/>
<rect width="{width}" height="20" fill="url(#s)"/>
</g>
<g fill="#fff" text-anchor="middle" font-family="Verdana,Geneva,DejaVu Sans,sans-serif" font-size="11">
<text x="{label_w / 2}" y="15" fill="#010101" fill-opacity=".3">{label}</text>
<text x="{label_w / 2}" y="14">{label}</text>
<text x="{label_w + value_w / 2}" y="15" fill="#010101" fill-opacity=".3">{value}</text>
<text x="{label_w + value_w / 2}" y="14">{value}</text>
</g>
</svg>
"""
open(sys.argv[2], "w").write(svg)
print(f"Line coverage: {percent:.1f}%")
PY
