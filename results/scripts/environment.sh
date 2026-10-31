#!/usr/bin/env bash
# environment.sh <runDir> — record what the results were produced with.
set -euo pipefail
OUT="$1/environment.json"
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
IDF="${IDF_PATH:-$HOME/Projects/source/esp-idf}"
q() { printf '%s' "$1" | python3 -c 'import json,sys; print(json.dumps(sys.stdin.read().strip()))'; }
xtensa="$(ls -d "$HOME"/Projects/source/esp-idf-tools/tools/xtensa-esp-elf/*/xtensa-esp-elf/bin/xtensa-esp32s3-elf-gcc 2>/dev/null | tail -1 || true)"
cat > "$OUT" <<JSON
{
  "recordedAt": $(q "$(date -u +%Y-%m-%dT%H:%M:%SZ)"),
  "git": { "commit": $(q "$(git -C "$ROOT" rev-parse HEAD)"), "dirty": $( [ -z "$(git -C "$ROOT" status --porcelain -- . ':!results/runs' ':!results/latest')" ] && echo false || echo true ) },
  "host": {
    "model": $(q "$(sysctl -n hw.model)"),
    "cpu": $(q "$(sysctl -n machdep.cpu.brand_string)"),
    "cores": $(sysctl -n hw.ncpu),
    "memoryGB": $(( $(sysctl -n hw.memsize) / 1073741824 )),
    "os": $(q "$(sw_vers -productName) $(sw_vers -productVersion) ($(uname -m))")
  },
  "toolchains": {
    "node": $(q "$(node --version)"),
    "cxx": $(q "$(c++ --version | head -1)"),
    "cmake": $(q "$(cmake --version | head -1)"),
    "espIdf": $(q "$(git -C "$IDF" describe --tags 2>/dev/null || echo unknown)"),
    "xtensaGcc": $(q "$([ -n "$xtensa" ] && "$xtensa" --version | head -1 || echo unknown)")
  }
}
JSON
echo "wrote $OUT"
