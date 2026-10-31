# Results

Recorded runs of every test suite and benchmark in this project, across all four environments:

| Environment | What runs | Where it runs |
| --- | --- | --- |
| JavaScript | The browser app's code (`js/`) | Node on an Apple M4 |
| C++23, double | `cpp/` | Apple M4 |
| C++23, float | The MCU core (`esp32/sim`), unsliced and at several slice budgets | Apple M4 |
| ESP32-S3 | Firmware (`esp32/firmware`), real time and on-chip suites | The board, over USB |

Each run is a folder under `runs/<UTC timestamp>/`, and `latest` points at the newest one. Read
`SUMMARY.md` there first: it has the tables. `summary.json` holds the same numbers in machine-readable form.
Every number in both files is derived from the raw files next to them.

## Re-running

```sh
# once per boot of the board: flash the firmware (see esp32/README.md), then
node esp32/bridge/bridge.js &          # serial <-> WebSocket bridge, also serves the live demo page
results/run_all.sh                     # about an hour; host and chip run in parallel
results/run_all.sh --no-chip           # host only
node results/scripts/summarize.mjs results/runs/<ts>   # rebuild SUMMARY.md from raw files
```

These environment variables change how much runs:

| Variable | Default | Controls |
| --- | --- | --- |
| `SEEDS_PER_RANGE` | 300 | seeds per scenario in each suite range |
| `JS_FIRST` | `1 301 601 901` | first seed of each JS range (12,000 missions) |
| `CPP_FIRST` | 12 ranges, 1–3301 | first seed of each C++ range (36,000 missions) |
| `HOST_FIRST` | 12 ranges, 1–3301 | first seed of each float-core range |
| `HOST_SLICES` | `unsliced 0.03 0.06 0.1` | float-core slice budgets in ms; `0.06` on an M4 matches 6 ms on the ESP32-S3 (4 × 36,000 missions) |
| `CHIP_BENCH_WORLDS` | 20 | worlds for the on-chip `bench` and `profile` |
| `CHIP_SUITE_SEEDS` | 30 | seeds per scenario for the on-chip suite (300 missions) |
| `CHIP_SLICE_MS` | 6 | chip slice budget, as live |
| `CHIP_MISSIONS` | all 16 | how many of the fixed real-time missions to run |

## What a run folder contains

```
environment.json            git commit, Mac model, CPU, OS, Node, clang, CMake, ESP-IDF, Xtensa GCC
mac.log, chip.log           progress logs
js/                         bench.txt, perf.txt, suite_<firstSeed>.txt, missions_<firstSeed>.txt
cpp/                        bench.txt, suite_*.txt, missions_*.txt
host-float/                 bench.txt and <slice>/suite_*.txt, missions_*.txt for each slice budget
esp32/info.json             board, clock, number type, ESP-IDF and compiler, frame budgets, heap
esp32/bench.json            unsliced operation timings on the chip
esp32/profile.json          drop-validation phases and search start-up on the chip
esp32/realtime.json         16 fixed real-time missions: drops, transitions, plans, the chip's summary
esp32/frames/<mission>.csv  every frame of those missions: t, state, x, y, heading, speed, drop/plan/tick/frame ms
esp32/suite.json            on-chip suite rows and total; esp32/suite-missions.jsonl one line per mission
esp32/raw.jsonl             every message the chip sent during the run
screenshots/                the demos, captured after the run
summary.json, SUMMARY.md    derived tables
```

`missions_*.txt` lines read `scenario|seed|final state|sim time|drops placed|drops rejected`. That's the same format for JS and C++, so per-mission outcomes can be compared directly.

## How the numbers are measured

- **Suites** run all 10 scenarios: four static densities, four with obstacles dropped mid-run, and two where drops may seal B off. A mission passes if it reaches B. In the sealing scenarios it also passes if it ends in FAILED in a world an independent grid oracle confirms has no route.
- **Host benchmarks** run the same seeds and operations in each language. JavaScript is warmed up first.
- **Chip timings** come from `esp_timer` on the chip. Serialising and sending the JSON happens after a frame is measured, so it isn't counted. Real-time missions run at the chip's own 60 Hz pace, exactly as in the live demo.
- **The chip measures itself.** Load on the Mac doesn't affect its numbers, so the host and chip parts run in parallel. The host parts run one after another so their timings don't compete.

## Run history

| Run | Outcome |
| --- | --- |
| First run (raw output not kept here) | Superseded. See the two findings below. |
| Second run (raw output not kept here) | Incomplete: the chip rebooted 292 missions into its suite. Superseded; see the three findings below. |
| `runs/20261031T035331Z` | **Current.** 192,300 missions, 0 failures; 0 of 10,065 real-time chip frames over 16.7 ms (worst 14.8 ms). |

The first run found two real problems, both fixed before the second run:
- **C++ double: 1 failure in 36,000 missions (seed 2589, "4 drops @45px").** JS fails the same seed; JS had only ever been run to seed 1200.
  - A drop removed the agent's last route on the planner's grid, but validation accepted it because the pixel-level escape still found a squeeze.
  - The agent got stuck in the squeeze, backed out of it, and then had no route at all.
  - Fix: a drop that removes the last grid route now counts as sealing B, in JS, both C++ cores and the chip's `DropTask`. The escape is trusted only when the agent was already off the grid before the drop.
- **ESP32-S3: 12 of 10,712 real-time frames over 16.7 ms (worst 20.1 ms).** All 12 came early in a mission, while the fine grid was still being built in the background. The drop validation and the background build each ran a slice in the same frame, and the background build ignored the shared budget. It now takes its slice from the shared budget and stands down while a drop is pending.

The second run found three more, all fixed before the current run:
- **ESP32-S3 rebooted during its suite** ("SEAL 16 random", seed 23).
  - The cause was an out-of-memory abort in the suite's own oracle, not the steering code. On the 1 px grid it uses for the sealing check, its cell-by-cell stack can reach 2.4 MB, and after about 280 missions the PSRAM heap was too fragmented for it to grow.
  - Fix: the oracle now floods scanline spans, marks cells in place and reuses its buffers. It gives byte-identical answers on 12,000 host missions.
- **ESP32-S3: 1 of 10,712 real-time frames over 16.7 ms (18.3 ms)**, in `drops-108` at t = 5.83 s.
  - When a fine-grid A* finished, the same frame string-pulled the whole path, hundreds of collision sweeps, outside any budget.
  - Fix: smoothing is now its own resumable planner phase.
- **C++ float, 0.03 ms stress slice: 1 failure in 36,000** (seed 244, "16 random drops"). It didn't reproduce, because slicing follows the wall clock. Two host-only tools in `esp32/sim/world.hpp` now make such failures reproducible:
  - `SIM_FAKE_CLOCK` runs a seeded fake clock with optional stalls;
  - `SIM_CLOCK_RECORD` and `SIM_CLOCK_REPLAY` record a failing mission's clock readings and replay them exactly.

  A search over fake-clock interleavings then found a real bug in the chip's sliced drop validation, which JS and the double C++ don't have, since they validate synchronously:
  - The agent keeps moving while a drop is validated, so at commit "connected" carries over only if the agent's current cell was reached by the search.
  - But the search grows the agent's side from every cell visible from the agent. Near a wall those cells can span two components, one of them a pocket the drop closes, and the agent can step into the pocket while the check runs.
  - Fix: the search now starts from the nearest cell, so every cell it reaches is connected to B. Only if that finds no route does it retry from all seeds, so the decision itself doesn't change. A cell reached from B's side always carries.
  - Before the fix, 40 fake-clock runs (120,000 missions) found it once. After it, 120 fake-clock runs (360,000 missions) found no failures, and neither did 144,000 more missions recorded on the real clock.

