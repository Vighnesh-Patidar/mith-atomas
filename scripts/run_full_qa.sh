#!/usr/bin/env bash
# Comprehensive QA + profiling harness for MithAtomas.
#
# What it does:
#   1. Build + test the three salient cmake configurations:
#         a) all options OFF              — minimum surface
#         b) UDP + AUTH + SERIAL ON       — full surface
#         c) UDP + AUTH ON (SERIAL OFF)   — typical SoC deployment
#      Each config is timed; the test binary is run twice — once for
#      pass/fail, once under /usr/bin/time for wall + RSS + page faults.
#   2. Run both flocking demos (2D + 3D) headless, capture timings.
#   3. Drop a per-config summary into a single Markdown report.
#
# Usage:
#   scripts/run_full_qa.sh [output-dir]
#
# Default output dir: ./qa-report/<timestamp>/

set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
TS="$(date -u +%Y%m%dT%H%M%SZ)"
OUT="${1:-$ROOT/qa-report/$TS}"
mkdir -p "$OUT"

REPORT="$OUT/REPORT.md"
JOBS="$(nproc)"

# Optional: GNU /usr/bin/time gives wall + RSS + page faults. Falls back
# to a lightweight wall-clock timer via `date` when missing — RSS / page
# faults are then reported as "n/a".
TIME=""
if [ -x /usr/bin/time ]; then
    TIME="/usr/bin/time"
fi

# Run `$@`. Echo the exit status and append three lines to the prof log:
#   WALL=<seconds>
#   RSS=<kB or n/a>
#   PF=<count or n/a>
run_profiled() {
    local prof_log="$1"; shift
    local t0=$(date +%s.%N)
    if [ -n "$TIME" ]; then
        "$TIME" -v "$@" >/dev/null 2>"$prof_log" || true
    else
        : > "$prof_log"
        "$@" >/dev/null 2>>"$prof_log" || true
    fi
    local t1=$(date +%s.%N)
    local wall
    wall=$(awk -v a="$t0" -v b="$t1" 'BEGIN{printf "%.2fs", b-a}')

    local rss="n/a" pf="n/a"
    if [ -n "$TIME" ]; then
        local raw_rss raw_pf
        raw_rss=$(grep -oE 'Maximum resident set size.*' "$prof_log" | awk '{print $NF}')
        raw_pf=$(grep -oE 'Minor.*page faults.*' "$prof_log" | awk '{print $NF}')
        [ -n "$raw_rss" ] && rss="$raw_rss"
        [ -n "$raw_pf" ] && pf="$raw_pf"
    fi
    printf 'WALL=%s\nRSS=%s\nPF=%s\n' "$wall" "$rss" "$pf" >> "$prof_log"
}

write_header() {
    cat > "$REPORT" <<EOF
# MithAtomas — full QA + profiling run

- Timestamp:    \`$TS\`
- Host:         \`$(uname -srm)\`
- CPU:          \`$(grep -m1 'model name' /proc/cpuinfo | sed 's/.*: //')\`
- Cores:        $JOBS
- Working tree: \`$ROOT\`
- Git HEAD:     \`$(git -C "$ROOT" rev-parse --short HEAD 2>/dev/null || echo 'n/a')\`

---

EOF
}

# Build + test one configuration. Args: <label> <cmake-flags...>
run_config() {
    local label="$1"; shift
    local build_dir="$OUT/build-$label"
    local cmake_log="$OUT/cmake-$label.log"
    local build_log="$OUT/build-$label.log"
    local test_log="$OUT/test-$label.log"
    local prof_log="$OUT/prof-$label.log"

    echo "===> Config: $label  ($*)"

    # Configure
    local t0=$(date +%s.%N)
    cmake -B "$build_dir" -S "$ROOT" "$@" -DCMAKE_BUILD_TYPE=Release \
        >"$cmake_log" 2>&1
    local t1=$(date +%s.%N)
    local cmake_s
    cmake_s=$(awk -v a="$t0" -v b="$t1" 'BEGIN{printf "%.2f", b-a}')

    # Build
    t0=$(date +%s.%N)
    cmake --build "$build_dir" -j "$JOBS" >"$build_log" 2>&1
    t1=$(date +%s.%N)
    local build_s
    build_s=$(awk -v a="$t0" -v b="$t1" 'BEGIN{printf "%.2f", b-a}')

    # Test — first pass: get pass/fail counts
    "$build_dir/tests/mith_unit_tests" >"$test_log" 2>&1 || true
    local cases assertions status
    cases=$(grep -oE 'test cases: *[0-9]+ *\| *[0-9]+ passed' "$test_log" | head -1 | awk '{print $3}')
    assertions=$(grep -oE 'assertions: *[0-9]+ *\| *[0-9]+ passed' "$test_log" | head -1 | awk '{print $2}')
    status=$(grep -oE 'Status: (SUCCESS|FAILURE)' "$test_log" | head -1 | awk '{print $2}')

    # Test — second pass: profile (GNU time if available, plain wall otherwise)
    run_profiled "$prof_log" "$build_dir/tests/mith_unit_tests"
    local wall_s rss_kb pf
    wall_s=$(grep -oE '^WALL=.*' "$prof_log" | cut -d= -f2)
    rss_kb=$(grep -oE '^RSS=.*'  "$prof_log" | cut -d= -f2)
    pf=$(grep -oE    '^PF=.*'    "$prof_log" | cut -d= -f2)

    cat >> "$REPORT" <<EOF
## Config: \`$label\`

CMake flags: \`$*\`

| Metric | Value |
|---|---|
| Configure | ${cmake_s}s |
| Build | ${build_s}s |
| Test status | $status |
| Test cases | $cases |
| Assertions | $assertions |
| Test wall time | $wall_s |
| Peak RSS | ${rss_kb} kB |
| Minor page faults | $pf |

Logs: \`$(basename "$cmake_log")\`, \`$(basename "$build_log")\`, \`$(basename "$test_log")\`, \`$(basename "$prof_log")\`

EOF

    # Remember the full-on build for the demo step
    if [ "$label" = "all-on" ]; then
        ALL_ON_BUILD="$build_dir"
    fi
}

run_demo() {
    local exe="$1"
    local label="$2"
    local out_jsonl="$OUT/demo-$label.jsonl"
    local prof_log="$OUT/prof-demo-$label.log"

    if [ ! -x "$exe" ]; then
        echo "===> Skipping demo $label — binary not built" >&2
        return
    fi

    local t0 t1 wall rss frames
    t0=$(date +%s.%N)
    if [ -n "$TIME" ]; then
        "$TIME" -v "$exe" >"$out_jsonl" 2>"$prof_log" || true
    else
        : > "$prof_log"
        "$exe" >"$out_jsonl" 2>>"$prof_log" || true
    fi
    t1=$(date +%s.%N)
    wall=$(awk -v a="$t0" -v b="$t1" 'BEGIN{printf "%.2fs", b-a}')
    rss="n/a"
    if [ -n "$TIME" ]; then
        local raw_rss
        raw_rss=$(grep -oE 'Maximum resident set size.*' "$prof_log" | awk '{print $NF}')
        [ -n "$raw_rss" ] && rss="$raw_rss"
    fi
    frames=$(wc -l <"$out_jsonl" | tr -d ' ')

    cat >> "$REPORT" <<EOF
### Demo: \`$label\`

| Metric | Value |
|---|---|
| Frames emitted | $frames |
| Wall time | $wall |
| Peak RSS | $rss kB |

JSON output: \`$(basename "$out_jsonl")\`

EOF
}

write_header

# Three salient build matrix points.
run_config "all-off" \
    -DMITH_ENABLE_UDP=OFF -DMITH_ENABLE_AUTH=OFF -DMITH_ENABLE_SERIAL=OFF

run_config "udp-auth" \
    -DMITH_ENABLE_UDP=ON -DMITH_ENABLE_AUTH=ON -DMITH_ENABLE_SERIAL=OFF

run_config "all-on" \
    -DMITH_ENABLE_UDP=ON -DMITH_ENABLE_AUTH=ON -DMITH_ENABLE_SERIAL=ON

# Demos — only meaningful from the all-on build.
cat >> "$REPORT" <<'EOF'
---

## Demos

EOF
run_demo "$ALL_ON_BUILD/examples/flocking_demo/flocking_demo"       "2d"
run_demo "$ALL_ON_BUILD/examples/flocking_demo_3d/flocking_demo_3d" "3d"

# Final tail — link to ARCHITECTURE comparison if present.
cat >> "$REPORT" <<EOF
---

Generated $(date -u +%Y-%m-%dT%H:%M:%SZ).
EOF

echo
echo "===> Report written: $REPORT"
echo "===> Artifacts in:    $OUT"
