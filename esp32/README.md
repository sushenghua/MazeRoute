# Running the steering simulation on an ESP32-S3

This folder runs the simulation on an **ESP32-S3 N16R8** (dual-core 240 MHz Xtensa LX7, 512 KB SRAM,
8 MB octal PSRAM, 16 MB flash). A web page shows the chip's results live.

The page doesn't simulate anything. The chip does all of the following and streams the results over USB:
- generating the arena;
- planning, steering and collision checks;
- the state machine;
- validating obstacle drops.

The page only draws what the chip reports. Nothing here changes the JavaScript app or the C++ port in `cpp/`.

```
browser (esp32/web)  ⇄ WebSocket ⇄  bridge (esp32/bridge, Node)  ⇄ UART 921600 ⇄  ESP32-S3 (esp32/firmware)
```

| Folder | What it is |
| --- | --- |
| `sim/` | Copy of `cpp/src` that uses a `real` type, which is `float` by default (build with `-DSIM_REAL_DOUBLE` to get `double`). Every step is resumable so it fits a frame slice on the chip; see [Fitting the frame](#fitting-the-frame-on-the-chip). |
| `host/` | Builds `sim/` on the Mac to prove the float version still has zero failures before flashing. |
| `firmware/` | ESP-IDF 5.5 project that runs the simulation (see below). |
| `bridge/` | Node relay between the serial port and a WebSocket. It also serves `web/`. |
| `web/` | The page: it draws the chip's state, sends commands, and shows the on-chip benchmark and test suite. |

The firmware runs two tasks:
- **Simulation task, core 1:** the same fixed 60 Hz frame as the browser app. Each frame runs, in order:
  1. a drop-validation slice, if a drop is pending;
  2. a planner or background grid-build slice, all against one work budget;
  3. one controller tick.

  Each frame's cost is measured with `esp_timer` and sent to the page.
- **Command reader, core 0.**

## Why float

The ESP32-S3 floating-point unit is single precision, so `double` arithmetic runs in software. `sim/` is
`cpp/src` with every `double` replaced by `real`. Four things deliberately stay `double`:
- the RNG output, which must be exact and strictly below 1;
- the conversion of drop coordinates into a seed;
- world generation, which runs once per arena;
- timing.

Before flashing, the float version was checked on the Mac: **12,000 missions, 0 failures** (`host/`, seeds 1–1200 × 10 scenarios).

## Run it

```sh
# once: build the firmware and flash it
alias get_idf='. ~/Projects/source/esp-idf/export.sh'
IDF_PYTHON_CHECK_CONSTRAINTS=no get_idf      # the installed esptool is a dev build the constraint check rejects
cd esp32/firmware
idf.py build
idf.py -p /dev/cu.usbmodem141301 flash      # flash over the USB-OTG (USB-Serial/JTAG) port

# every time: start the bridge and open the page
cd ../bridge && npm install && node bridge.js   # finds the CH343 "UART" port by USB vendor id
open http://127.0.0.1:8766/
```

Port notes for this board:
- **Flashing:** use the USB-OTG port (`usbmodem1413…`). esptool resets the chip correctly through it.
- **Protocol, logs and panics:** these go over the UART port (CH343, `usbmodem5C4C…`) at 921600 baud.
- **Opening the UART port:** the board's auto-reset circuit resets the chip when the port opens with the default modem lines. The bridge releases DTR and RTS as soon as it opens the port, so the chip doesn't reboot.

### Commands (one per line, from the page or any serial terminal)

| Command | Effect |
| --- | --- |
| `gen <seed> <density>` | Generate an arena; replies with the polygons, A, B and the generation time. |
| `start`, `reset` | Start or reset a mission. State is streamed every frame, plus each transition and path. |
| `drop <x> <y> <allowSealing 0/1>` | Obstacle drop, validated a slice per frame. Replies with the polygon or the reason it was rejected, plus the work time and number of frames it took. Clicks that arrive while one is validating are queued. |
| `budget <plan ms> <build ms> <drop ms>` | Slices per 16.7 ms frame. Default 6 / 6 / 5. A drop also gets the planner's share when the planner is idle. |
| `bench <n>` | Operation benchmark on n worlds. Same operations as `cpp/src/bench.cpp`. |
| `suite <n> <firstSeed> [slice ms]` | The 10-scenario test suite, run on the chip. With `slice ms`, every step runs in slices of that size, exactly as live. `stop` aborts it. |
| `profile <n>` | Times each phase of drop validation and each search's start-up, unsliced. |

`bench`, `profile` and `suite` first free the live arena and its reserved search buffers, about 5 MB of PSRAM, because they build their own worlds. The page then asks you to Generate again. All results can be recorded with `results/run_all.sh`; see [`results/`](../results/README.md).
| `info` | Chip, clock, number type and heap. |

## Fitting the frame on the chip

Every step the browser runs in one go (because it takes well under a millisecond there) is resumable in `sim/`. Each one does a budget's worth of work per frame and carries on in the next.

| Step | Unsliced on the S3 | How it fits now |
| --- | --- | --- |
| Fine-grid seeding (start and B) | 16–42 ms | `SeedSearch`: same rings and result as `seedCells`, resumable per cell |
| Pixel escape (planning tier 3) | up to ~650 ms | `EscapeSearch`: goal-component flood, then pixel BFS, both resumable |
| Drop validation | 80–210 ms | `DropTask` (`sim/drop.hpp`), described below |
| `AStar::start` on the fine grid | 62–123 ms (clearing about 2 MB of PSRAM) | Cells are initialised lazily through generation-stamped `Labels`, so it costs about 0.02 ms |
| First use of a search buffer | 10–70 ms (allocation) | Buffers are grow-only, swapped instead of copied, and reserved right after Generate |
| Fine-grid build | 17–23 ms per 4-row band | One row per budget check |

How `DropTask` validates a drop:
- **The world isn't modified until the drop is accepted.** During each slice, World queries see the candidate polygon through `World::overlay`, and its cells are patched into the fine grid. Both are removed again before the slice returns, so the live simulation never sees a half-validated obstacle.
- **The full flood fill is gone.** It was 85% of the old cost. A meet-in-the-middle greedy search from the agent's seeds and B's seeds replaces it. The search stops as soon as the two sides touch, or as soon as the smaller side is exhausted.
- **The agent keeps moving during validation**, so the task re-checks where the agent is before committing:
  - It rejects the drop if the agent has reached the candidate.
  - It accepts a "connected" answer only if the agent's current cell was reached by the search.
  - Otherwise it checks again from the agent's current position. Answers that needed the pixel escape always get this second check, because they depend on the exact position.
  - After four retries it rejects. That's conservative, so it never seals B.
- **A drop borrows the planner's slice when the planner is idle**, which is the usual case while following a path. Validation then finishes in about 5 frames. The planner gets whatever that frame's shared budget has left.

The host harness runs drops through the same `DropTask` every frame. `SLICE_MS=0.06` slices the work on an Apple M4 about as finely as 6 ms does on the chip.

## Results on the chip

Benchmark from `bench 20` (density 36). Every operation here is unsliced, measuring total work:

| Operation | ESP32-S3 median | p90 | Max | M4 (C++ double) median |
| --- | --- | --- | --- | --- |
| Generate + coarse grid | 55 ms | 115 ms | 121 ms | 0.32 ms |
| Fine grid full build (480×320) | 889 ms | 961 ms | 1,012 ms | 3.7 ms |
| Drop validation, all slices together | 52 ms | 72 ms | 215 ms | 0.35 ms (synchronous `addObstacle`) |
| Coarse A* (96×64) | 7.3 ms | 16 ms | 17 ms | 0.093 ms |
| Fine A* (480×320) | 192 ms | 392 ms | 408 ms | 2.2 ms |
| Fine-grid seeding, start + B | 15 ms | 31 ms | 35 ms | – |
| Smoothing a fine path | 2.1 ms | 4.4 ms | 4.6 ms | – |

None of these appears as a frame. Generation runs once when you press Generate, and the fine-grid build runs in the background, a row at a time. Everything else is spread over frames.

Memory:
- **PSRAM:** 4.2–5.8 MB of 8 MB at the high-water mark. Most of it is search buffers reserved up front, so no frame pays to allocate them.
- **On-chip RAM:** about 260–330 KB stays free.

### Real-time missions with drops

Recorded in [`results/runs/20261031T035331Z`](../results/runs/20261031T035331Z/SUMMARY.md), every frame
saved as CSV. Sixteen missions with fixed seeds:
- **Setup:** four static missions at density 36, and twelve at densities 30 and 40 with 5 obstacles dropped 45, 70 or 95 px ahead of the moving agent.
- **Measurement:** the chip timed every streamed frame, idle frames included.

| Measure | Result |
| --- | --- |
| Missions | 16 of 16 arrived |
| Frames over 16.7 ms | **0 of 10,065** |
| Worst frame | 14.8 ms (mission means 1.8–3.0 ms) |
| Drops | 32 accepted, 22 rejected (all because the agent had reached the spot) |
| Drop validation time | median 5 frames and 54 ms of work; max 54 frames and 403 ms |
| Memory | PSRAM peak 5.75 MB of 8 MB; internal RAM never below 224 KB free |

The first version, before slicing, had a 150 ms frame in the same kind of test.

### Test suites

From the same recorded run:

| Where | Slicing | Missions | Failures |
| --- | --- | --- | --- |
| ESP32-S3: `suite 30 1 6` | 6 ms slices, as live | 300 (30 seeds × 10 scenarios) | **0**, in 473 s |
| Host (M4), float core | unsliced, 0.03, 0.06 (≈ chip) and 0.1 ms | 144,000 (seeds 1–3600 × 10 scenarios × 4 budgets) | **0** |

What changes when the agent keeps driving during validation:
- **Close drops are rejected more often.** With chip-like slicing, 37% of the "4 drops @45px" drops are rejected, against 23% in the synchronous C++. The extra ones are drops the agent reaches before validation finishes.
- **"Connected" must be re-proven where the agent is now.** The check started from where the agent was; at commit it carries over only if the agent's current cell is known to be connected to B, and is otherwise re-run from there. An earlier, looser version of this rule could accept a drop that closed a pocket the agent had just entered. That was found by replaying slice timings on the host (`SIM_FAKE_CLOCK`, `SIM_CLOCK_RECORD`/`SIM_CLOCK_REPLAY` in `sim/world.hpp`), and is described in the [run history](../results/README.md#run-history).
