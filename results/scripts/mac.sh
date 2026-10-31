#!/usr/bin/env bash
# mac.sh <runDir> — every host-side suite and benchmark, run one after another so timings don't compete.
#   js/          the browser code under Node: tests/bench.js, tests/perf.js, tests/run.js
#   cpp/         the C++23 double-precision port: cpp/bench, cpp/suite
#   host-float/  the ESP32 float core on the host: esp32/host bench, suite at several slice budgets
# Ranges are seeds-per-scenario x first seed; each suite run covers all 10 scenarios.
set -euo pipefail
RUN="$1"
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
JS_FIRST="${JS_FIRST:-1 301 601 901}"
CPP_FIRST="${CPP_FIRST:-1 301 601 901 1201 1501 1801 2101 2401 2701 3001 3301}"
HOST_FIRST="${HOST_FIRST:-1 301 601 901 1201 1501 1801 2101 2401 2701 3001 3301}"
HOST_SLICES="${HOST_SLICES:-unsliced 0.03 0.06 0.1}"
N="${SEEDS_PER_RANGE:-300}"
log() { echo "[$(date +%H:%M:%S)] $*"; }

log "build cpp/ and esp32/host/ (Release)"
cmake -S "$ROOT/cpp" -B "$ROOT/cpp/build" -DCMAKE_BUILD_TYPE=Release >/dev/null && cmake --build "$ROOT/cpp/build" -j >/dev/null
cmake -S "$ROOT/esp32/host" -B "$ROOT/esp32/host/build" -DCMAKE_BUILD_TYPE=Release >/dev/null && cmake --build "$ROOT/esp32/host/build" -j >/dev/null

mkdir -p "$RUN/js" "$RUN/cpp" "$RUN/host-float"
# Benchmarks first, on a quiet machine.
log "js bench + perf";        node "$ROOT/tests/bench.js" > "$RUN/js/bench.txt"; node "$ROOT/tests/perf.js" > "$RUN/js/perf.txt"
log "cpp bench";              "$ROOT/cpp/build/bench" > "$RUN/cpp/bench.txt"
log "host-float bench";       "$ROOT/esp32/host/build/bench" > "$RUN/host-float/bench.txt"

for s in $CPP_FIRST; do log "cpp suite $N@$s"; "$ROOT/cpp/build/suite" "$N" "$s" --dump "$RUN/cpp/missions_$s.txt" > "$RUN/cpp/suite_$s.txt" || true; done
for b in $HOST_SLICES; do
  mkdir -p "$RUN/host-float/$b"
  for s in $HOST_FIRST; do
    log "host-float suite $N@$s slice=$b"
    if [ "$b" = unsliced ]; then "$ROOT/esp32/host/build/suite" "$N" "$s" --dump "$RUN/host-float/$b/missions_$s.txt" > "$RUN/host-float/$b/suite_$s.txt" || true
    else SLICE_MS="$b" "$ROOT/esp32/host/build/suite" "$N" "$s" --dump "$RUN/host-float/$b/missions_$s.txt" > "$RUN/host-float/$b/suite_$s.txt" || true; fi
  done
done
for s in $JS_FIRST; do log "js suite $N@$s"; node "$ROOT/tests/run.js" "$N" "$s" --dump "$RUN/js/missions_$s.txt" > "$RUN/js/suite_$s.txt" || true; done
log "mac done"
