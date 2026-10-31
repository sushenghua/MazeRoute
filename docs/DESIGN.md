# MazeRoute — design notes

A self-contained web app (vanilla JS + canvas, no build step) that shows one coherent answer to
four coupled problems: steering a directional agent through a world, colliding with irregular
shapes, a control state machine that cannot deadlock, and fitting all of it inside a fixed frame
budget.

## Run

From the repo root, open `index.html` through any static server (modules are plain scripts, so `file://` also works):

```
python3 -m http.server 8765
# then visit http://localhost:8765/
```

## Using the demo

1. **Generate** builds a new arena: random irregular blobs and bent bars, a start **A**, a goal
   **B**, and the directional agent (triangle) parked at A. A and B are guaranteed reachable.
2. **Start** plans a route (watch the A* flood spread over several frames) and then drives the
   agent to B. The dashed cyan line is the smoothed path; the small dot is the look-ahead target.
3. While the agent moves, **click in the arena** to drop a new obstacle in its way. The agent
   avoids it locally, or re-plans if the route is blocked. Drops that would overlap the agent or
   cover B are always rejected with a message.
4. **Allow drops that seal off B** (off by default) decides what happens to a drop that would cut
   B off from the agent. Off: it is rejected, so the level always stays winnable. On: it is placed,
   a message says B is now sealed off, and the agent ends in `FAILED` with the reason
   "no route exists at the agent's size".
5. The right-hand panel shows the live state, the transition log with reasons, and a stacked
   per-frame timing chart against the 16.7 ms line. Lower the **planner cap** slider to see
   planning stretch over more frames without the frame rate dropping.

## How each problem is solved

### Steering a directional agent — `js/steering.js`
The agent is car-like: it has a heading, a turn-rate limit and an acceleration limit, and its
speed target is scaled by how well it is aligned with where it wants to go, so it slows to turn.
It follows the path with **visible pure pursuit**: the look-ahead point slides along the polyline,
then walks back toward the agent until the straight line to it is collision-free for the agent's
body (first with a comfortable margin, then a tight one). Pursuit therefore never aims through a
corner, even when the agent has drifted off the path. Speed is capped ahead of **sharp corners**
from the turn angle and the braking distance, and the agent **pivots in place** when its heading
error is large instead of creeping forward into a wall. Three swept whiskers provide local
avoidance only when no part of the path ahead is visible.

### Collision with irregular shapes — `js/geometry.js`, `js/world.js`
Obstacles are arbitrary (including concave) polygons. Collision uses exact circle-vs-polygon tests:
point-in-polygon by ray casting plus distance to the nearest edge, which yields a push-out vector
whether the agent is overlapping an edge or fully inside. Swept queries (segment-vs-polygon
distance) drive the whiskers, path smoothing and runtime path validation. A spatial hash gives
broadphase so every query touches only nearby polygons. For planning, the world is rasterised into
navigation grids where a cell is blocked if a disc of the agent's radius plus a margin would touch
anything, so A* plans for the agent's footprint rather than a point. There are two:

* a **coarse grid** (10 px, wide margin) that is fast and yields smooth, comfortable routes;
* a **fine grid** (2 px, 1.1 px margin) used as a fallback, so a gap the agent physically fits
  through is never reported as "no path". Its contract: any gap that leaves at least **2.5 px
  of clearance on each side** of the agent is guaranteed to be found. Tighter gaps may be found
  or may be treated as closed, because a grid can only promise what its cell size can resolve.

Grid updates are incremental: adding an obstacle re-tests only the cells its inflated bounding box
covers. Searches seed from **every free cell the agent can see** near it (and near the goal), not
just the nearest one; the nearest cell can sit behind a thin wall or be an isolated one-cell island
in a pocket corner, and either would make the planner wrongly conclude there is no route. The
line-of-sight sweep for seeds is as strict as the grid whenever the agent has room, so a seed is
never "seen" through a neck the grid itself would not traverse.

Planning is **three-tier**: the coarse grid first, then the fine grid, then a **pixel-level
escape**. The agent's body can be pushed into spots with less clearance than any grid cell
represents, and from there no grid cell may be reachable in a straight line. The escape floods the
agent's real free space at 1 px resolution in a window around it, demanding the same margin as the
fine grid except within 4 px of the start, and stops at the first pixel that lies in a grid cell
connected to B. The route is that escape path plus a fine-grid A* path from there.

Obstacle-drop validation uses exactly the same checks, including the escape, so the planner can
always do whatever validation assumed it could.

### A state machine that never gets stuck — `js/steering.js`
States: `IDLE → PLANNING → FOLLOWING ⇄ (RECOVERING → REPLANNING) → ARRIVED | FAILED`.
Four mechanisms guarantee liveness:

* **Every state has a deadline.** If a state overstays, its `onTimeout` forces a transition.
  Following's deadline is derived from the path length.
* **Progress watchdog.** Following tracks the remaining path distance; if it has not shrunk for
  1.2 s the machine moves to Recovering (reverse and turn), then Replanning from where it is.
* **Bounded escalation.** Each recovery bumps a stuck streak that only real progress resets. At
  three the controller switches to planning on the fine grid; at seven the machine declares
  `FAILED` with a reason. Every loop in the graph therefore terminates.
* **No watchdog resets by replanning.** A replan triggered while following keeps the progress
  clock running, so a follow → replan → follow cycle cannot spin forever without moving.
* **Recovery that does not grind.** The recovery manoeuvre backs up only if the space behind is
  clear and otherwise pivots in place toward the path.
* **Honest terminal states.** `ARRIVED` and `FAILED` are explicit, and every transition is logged
  with its cause, so "stuck" can never be silent.

Runtime changes are handled too: every obstacle drop while following triggers a replan, not only
drops that block the path outright. A drop beside the path can narrow its corridor below what the
planner considers passable, and following the old path could then lead the agent through a neck
into a region the planner cannot route out of. Path validation also runs periodically while
following.

### Fixed frame budget — `js/scheduler.js`, `js/pathfinder.js`, `js/main.js`
Each frame runs three phases and measures each one: **sim**, **plan**, **render**.

* Simulation uses a fixed timestep with an accumulator and a cap on sub-steps per frame; excess
  time is dropped and reported, so a slow frame cannot cause a spiral of death.
* A* is **resumable**. `step(budgetMs)` expands nodes until its slice is spent and returns; the
  search continues next frame. The slice it receives is the slider cap, reduced by whatever the
  smoothed render estimate says is still needed to land the frame under 16.7 ms.
* The fine grid takes tens of milliseconds to rasterise, so it is **built in time slices** in the
  background after Generate, using the same per-frame slack as the planner. If the planner needs it
  before it is ready, the planner finishes the build in its own slices first.
* Grid overlays are drawn as one image per frame (one pixel per cell, scaled up), not one
  rectangle per cell, because the fine grid has about 150,000 cells.
* The pixel escape is resumable too, in both of its phases, and the planner steps it in its slice.
  Drop validation in a click handler needs an immediate answer, so it runs the escape to completion;
  that only happens for drops that nearly seal B off.
* Path smoothing is linear: collinear grid points are dropped first, then each anchor extends its
  straight segment while it stays clear. An earlier quadratic version took 649 ms on one long
  fine-grid path.
* Cheap data layout: typed arrays for the grids and a binary heap for the open set.

The budget chart makes the trade-off visible: a tight planner cap spreads planning over more
frames while the frame time stays flat.

## Files

| File | Purpose |
| --- | --- |
| `index.html`, `css/style.css` | Page and styling |
| `js/geometry.js` | Vectors, polygons, circle/segment-vs-polygon queries, shape generators |
| `js/world.js` | Arena generation, spatial hash, navigation grid, reachability |
| `js/pathfinder.js` | Time-sliced A*, binary heap, line-of-sight smoothing |
| `js/steering.js` | Agent motion model and the controller state machine |
| `js/scheduler.js` | Frame budget accounting and fixed-timestep clock |
| `js/main.js` | UI, render loop, budget allocation per frame |
| `tests/run.js` | Headless stress suite: 8 scenarios × N seeds, exit code 1 on any failure |
| `tests/perf.js` | Timing of generation, grid builds, obstacle validation and A* |
| `tests/harness.js` | Loads the browser scripts into Node and provides a ground-truth solvability oracle |
| `tests/bench.js` | Per-operation and per-frame timings, mirrored by `cpp/src/bench.cpp` |
| `tests/debug.js`, `tests/trace.js` | Replay a single seed and dump transitions or per-tick internals |
| `cpp/` | C++23 port of the simulation core with the same suite and benchmark; see below |

## Verification

Run the stress suite headlessly with Node (no dependencies). It loads the exact browser scripts:

```
node tests/run.js 300 1      # 10 scenarios x 300 seeds from seed 1; exit code 1 on any failure
node tests/bench.js          # per-operation and per-frame timings
```

Scenarios cover four obstacle densities on static worlds, plus four dynamic ones that drop new
obstacles mid-run: 4 drops at 95 px or 45 px ahead of the agent, 8 random drops, and 16 random
drops on the densest setting. A failure is any outcome other than `ARRIVED`, including a mission
still running at 150 s of simulated time. Every failure is checked against an independent
ground-truth oracle that floods a pixel grid at the agent's true radius.

Two more scenarios run with sealing allowed. There, `FAILED` is the correct answer when B really is
cut off, so a run passes if it arrives, or if it ends in `FAILED` and a 1 px oracle confirms no
route leaves at least 2.5 px of clearance on each side. Anything else, such as a stall, a timeout,
or a `FAILED` while a comfortable route exists, counts as a failure.

Latest JavaScript result over four seed ranges (1, 1001, 5001 and 9001, 300 seeds each):

| Scenario | Missions | Arrived | Failed |
| --- | --- | --- | --- |
| Static, densities 12 / 22 / 36 / 40 | 4,800 | 4,800 | 0 |
| 4 drops at 95 px | 1,200 | 1,200 | 0 |
| 4 drops at 45 px | 1,200 | 1,200 | 0 |
| 8 random drops | 1,200 | 1,200 | 0 |
| 16 random drops, density 40 | 1,200 | 1,200 | 0 |
| Sealing allowed, 4 drops at 45 px | 1,200 | 1,014 | 0 |
| Sealing allowed, 16 random drops | 1,200 | 530 | 0 |
| **Total** | **12,000** | | **0** |

With sealing allowed, all 856 missions whose B was sealed off ended in `FAILED`, at most 0.6 s of
simulated time after the sealing drop. In the static scenarios 4,788 of 4,800 missions finished
without a single collision correction. With sealing disallowed, about 16% of attempted drops were
rejected (3,644 of 22,473), mostly in the 16-drop stress test.

In the browser, live missions with mid-run drops ran at 60 fps with zero frames over the 16.7 ms
budget, and a sealing drop with the toggle on ended in `FAILED` with its reason.

**What "zero failures" relies on.** A dropped obstacle can genuinely seal B off, and then no
algorithm can arrive. With sealing disallowed (the default), the app prevents that instead of
solving it: a drop is rejected if it would overlap the agent, cover B, or leave no route the
planner can drive. With sealing allowed, those drops are placed and the agent reports `FAILED`.

**Bugs the suite found and fixed**

* The nearest free cell could be a one-cell island in a pocket corner, so the planner reported
  "no route" while one existed. Searches now seed from every visible free cell nearby.
* The coarse grid missed gaps the agent fits through. A fine grid is now the fallback.
* A follow → replan → follow cycle could reset the progress watchdog forever (a livelock).
  Replans triggered while following no longer reset it.
* Seed cells were checked with a thinner sweep than path validation used, so a fresh path could
  fail its own validation and replan endlessly. The radii are now ordered consistently.
* The seed line-of-sight test could see through gaps the grid would not traverse, which let a
  validated drop seal the agent in. Its sweep is now as strict as the grid when there is room.
* The agent could be pushed into spots no grid cell represents. The pixel-level escape tier now
  gets it back onto the grid.
* A drop beside the path could narrow it without blocking it, leading the agent into a region the
  planner could not leave. Every drop now triggers a replan.
* An escape that allowed less clearance than the grid approved routes the agent could not drive.
  The escape now demands the grid's margin away from the start.
* Sharp corners taken at speed overshot into walls, and whisker steering then pinned the agent.
  This was fixed by visible look-ahead, corner speed limits, pivot turns and clearance-aware
  recovery.
* Quadratic path smoothing and an unsliced escape caused single frames of up to 649 ms. Both are
  now linear or resumable.

## C++23 port

`cpp/src` is a header-only C++23 port of the simulation core: geometry, world, both nav grids, A*,
the pixel escape and the controller state machine. Rendering stays in the browser. It is a
faithful translation so the two can be compared on identical work: random draws are made in the
same order, A* costs use `float` like the JS `Float32Array`, and the broadphase visits polygons in
the same order. It uses `std::expected` for obstacle validation, `std::print`, `std::format` and
ranges, and the broadphase is a template visitor instead of an array per query.

```
cmake -S cpp -B cpp/build && cmake --build cpp/build
./cpp/build/suite 300 1      # same scenarios, pass rules and output as tests/run.js
./cpp/build/bench            # same measurements as tests/bench.js
```

**Parity.** With `PARITY=1` both suites build the fine grid up front, removing the only wall-clock
dependence. Across 300 missions every final state matched between the two languages, and static
worlds matched tick for tick. A few dynamic missions take different but equally valid routes:
traced tick by tick, the first difference is a heading that differs in the 15th significant
digit, from `atan2` in V8 versus the macOS C library, which the steering loop amplifies.

**Correctness.** The C++ suite ran the same 10 scenarios over 12 seed ranges, 36,000 missions in
total, with zero failures.

**Performance** (Apple M4, Node 24.14 vs Apple clang 21 with `-O3`; same seeds and work; JS
measured after JIT warm-up):

| Operation | JS median | C++ median | JS worst | C++ worst | Speed-up (median) |
| --- | --- | --- | --- | --- | --- |
| Generate world (coarse grid) | 4.0 ms | 0.32 ms | 8.8 ms | 0.95 ms | 13× |
| Fine grid full build | 80 ms | 3.7 ms | 106 ms | 10 ms | 21× |
| Validate and apply a drop | 5.5 ms | 0.35 ms | 13 ms | 1.1 ms | 16× |
| Coarse A* with seeding | 0.97 ms | 0.093 ms | 1.9 ms | 0.20 ms | 10× |
| Fine A* with seeding, unsliced | 21 ms | 2.2 ms | 59 ms | 6.5 ms | 10× |

Per-frame cost of the simulation and planning work, with app-like 2 ms planner and build slices:

| Scenario | JS p99 | C++ p99 | JS worst | C++ worst | Frames simulated per second, JS / C++ |
| --- | --- | --- | --- | --- | --- |
| Static, density 36 | 3.1 ms | 0.003 ms | 4.2 ms | 2.1 ms | 4,740 / 98,568 |
| 16 random drops, density 40 | 3.0 ms | 0.17 ms | 7.8 ms | 2.8 ms | 2,870 / 48,812 |

The full 3,000-mission suite takes about 27 s in C++ and about 600 s in JS.

**What the speed-up buys.** One agent already fits comfortably in JS: the worst frame is 7.8 ms of
a 16.7 ms budget, and the 2 ms planner slices dominate the p99. C++ is about 10 to 20 times faster
per operation, which matters for many agents, larger maps, finer grids, or running the planner
without slicing. In C++ an unsliced fine-grid search fits inside one frame, and a whole-map grid
rebuild takes under 4 ms.

**Using it in the browser** would mean compiling the core to WebAssembly with Emscripten, which is
not installed here and was not measured. WebAssembly typically keeps most of the native speed-up,
but calls across the JavaScript boundary and copying state for rendering cost extra.

## ESP32-S3 port

`esp32/` runs the same simulation on an ESP32-S3 N16R8 in single precision. A web page draws what the
chip computes, live over USB. See [`esp32/README.md`](../esp32/README.md).

- **Every step is resumable:**
  - fine-grid seeding;
  - the pixel escape;
  - drop validation, which uses a meet-in-the-middle connectivity test;
  - A* initialisation, which now uses generation stamps.
- **Result:** 0 of 10,065 frames went over 16.7 ms on the chip with obstacles being dropped, and the worst frame was 14.8 ms. The float core passes 144,000 host missions at four slice budgets, and 300 on-chip missions, all with 0 failures.

## Recorded results

Every suite and benchmark above, in all four environments, is recorded with its raw output in
[`results/`](../results/README.md). The latest run,
[`results/runs/20261031T035331Z`](../results/runs/20261031T035331Z/SUMMARY.md), has
**192,300 missions with 0 failures** (JS 12,000; C++ double 36,000; float core 144,000; ESP32-S3 300) and
**0 of 10,065 real-time chip frames over budget**. Screenshots are in its `screenshots/` folder. The run
history there lists what each earlier run found and how it was fixed.
