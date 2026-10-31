#!/usr/bin/env bash
# run_all.sh [--no-chip] [--no-mac] — re-run every suite and benchmark into results/runs/<UTC timestamp>/.
#
# Host (Apple Silicon Mac): JavaScript, C++23 double, and the float MCU core at several slice budgets.
# Chip (ESP32-S3 over USB): needs the firmware flashed and the bridge running (node esp32/bridge/bridge.js).
# The two run in parallel: the chip measures itself, so host load does not affect its numbers.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
RUN="$HERE/runs/$(date -u +%Y%m%dT%H%M%SZ)"
mkdir -p "$RUN"
"$HERE/scripts/environment.sh" "$RUN"
pids=()
if [[ " $* " != *" --no-chip "* ]]; then node "$HERE/scripts/chip.mjs" "$RUN" > "$RUN/chip.log" 2>&1 & pids+=($!); fi
if [[ " $* " != *" --no-mac "* ]]; then "$HERE/scripts/mac.sh" "$RUN" > "$RUN/mac.log" 2>&1 & pids+=($!); fi
status=0
for p in "${pids[@]}"; do wait "$p" || status=1; done
node "$HERE/scripts/summarize.mjs" "$RUN"
ln -sfn "runs/$(basename "$RUN")" "$HERE/latest"
echo "results in $RUN (exit $status)"
exit $status
