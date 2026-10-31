# Results: 20261031T035331Z

Recorded 2026-10-31T03:53:31Z at commit `c926144`.

- Host: Mac16,10, Apple M4, 10 cores, 24 GB, macOS 26.6.2 (arm64)
- Tools: Node v24.14.0; Apple clang version 21.0.0 (clang-2100.4.1.1); ESP-IDF v5.5.3; xtensa-esp-elf-gcc (crosstool-NG esp-14.2.0_20251107) 14.2.0
- Board: ESP32-S3 rev 2, 2 cores @ 240 MHz, 16 MB flash, 8 MB PSRAM; core in `float`; budgets per 16.7 ms frame: plan 6 ms, build 6 ms, drop 5 ms

## Correctness: zero-failure suites

Each mission must reach B, or, when sealing drops are allowed, end in FAILED in a world an independent oracle confirms is unwinnable.

| Implementation | Where | Slicing | Missions | Failures | Time |
| --- | --- | --- | ---: | ---: | ---: |
| JavaScript (browser code) | Node on host | 50 ms | 12000 | 0 | 2017.4 s |
| C++23, double | host | 50 ms | 36000 | 0 | 319.5 s |
| C++23, float (MCU core) | host | 0.03 ms (stress) | 36000 | 0 | 207.7 s |
| C++23, float (MCU core) | host | 0.06 ms (≈ chip) | 36000 | 0 | 239.4 s |
| C++23, float (MCU core) | host | 0.1 ms (stress) | 36000 | 0 | 250.3 s |
| C++23, float (MCU core) | host | 50 ms | 36000 | 0 | 262.4 s |
| C++23, float (MCU core) | **ESP32-S3** | 6 ms, as live | 300 | 0 | 472.6 s |

### JavaScript, 12000 missions

| Scenario | Runs | Arrived | Honest FAILED | Fail | Zero-collision runs | Drops placed / rejected | Mean time |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| static d=12 | 1200 | 1200 | 0 | 0 | 1199 | 0 / 0 | 6.3 s |
| static d=22 | 1200 | 1200 | 0 | 0 | 1196 | 0 / 0 | 6.95 s |
| static d=36 | 1200 | 1200 | 0 | 0 | 1196 | 0 / 0 | 7.72 s |
| static d=40 | 1200 | 1200 | 0 | 0 | 1194 | 0 / 0 | 7.75 s |
| 4 drops @95px d=36 | 1200 | 1200 | 0 | 0 | 1036 | 1973 / 168 | 9.2 s |
| 4 drops @45px d=36 | 1200 | 1200 | 0 | 0 | 194 | 1939 / 576 | 10.57 s |
| 8 random drops d=30 | 1200 | 1200 | 0 | 0 | 485 | 5599 / 633 | 11.4 s |
| 16 random drops d=40 | 1200 | 1200 | 0 | 0 | 410 | 9138 / 2276 | 12.07 s |
| SEAL 4 drops @45px d=36 | 1200 | 1001 | 199 | 0 | 236 | 2058 / 346 | 9.68 s |
| SEAL 16 random d=40 | 1200 | 539 | 661 | 0 | 555 | 7424 / 799 | 7.85 s |

### C++23 double, 36000 missions

| Scenario | Runs | Arrived | Honest FAILED | Fail | Zero-collision runs | Drops placed / rejected | Mean time |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| static d=12 | 3600 | 3600 | 0 | 0 | 3597 | 0 / 0 | 6.32 s |
| static d=22 | 3600 | 3600 | 0 | 0 | 3582 | 0 / 0 | 6.99 s |
| static d=36 | 3600 | 3600 | 0 | 0 | 3585 | 0 / 0 | 7.7 s |
| static d=40 | 3600 | 3600 | 0 | 0 | 3581 | 0 / 0 | 7.79 s |
| 4 drops @95px d=36 | 3600 | 3600 | 0 | 0 | 3105 | 5953 / 470 | 9.21 s |
| 4 drops @45px d=36 | 3600 | 3600 | 0 | 0 | 633 | 5784 / 1775 | 10.6 s |
| 8 random drops d=30 | 3600 | 3600 | 0 | 0 | 1517 | 16760 / 1882 | 11.37 s |
| 16 random drops d=40 | 3600 | 3600 | 0 | 0 | 1244 | 27386 / 6923 | 12.17 s |
| SEAL 4 drops @45px d=36 | 3600 | 3015 | 585 | 0 | 748 | 6171 / 1089 | 9.73 s |
| SEAL 16 random d=40 | 3600 | 1593 | 2007 | 0 | 1685 | 22295 / 2460 | 7.92 s |

### Float core at chip-like slicing (0.06 ms), 36000 missions

| Scenario | Runs | Arrived | Honest FAILED | Fail | Zero-collision runs | Drops placed / rejected | Mean time |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| static d=12 | 3600 | 3600 | 0 | 0 | 3597 | 0 / 0 | 6.35 s |
| static d=22 | 3600 | 3600 | 0 | 0 | 3582 | 0 / 0 | 7.03 s |
| static d=36 | 3600 | 3600 | 0 | 0 | 3585 | 0 / 0 | 7.72 s |
| static d=40 | 3600 | 3600 | 0 | 0 | 3581 | 0 / 0 | 7.83 s |
| 4 drops @95px d=36 | 3600 | 3600 | 0 | 0 | 3109 | 5842 / 684 | 9.4 s |
| 4 drops @45px d=36 | 3600 | 3600 | 0 | 0 | 1041 | 4378 / 2595 | 9.63 s |
| 8 random drops d=30 | 3600 | 3600 | 0 | 0 | 1749 | 15789 / 2895 | 11.5 s |
| 16 random drops d=40 | 3600 | 3600 | 0 | 0 | 1547 | 27877 / 8530 | 13.42 s |
| SEAL 4 drops @45px d=36 | 3600 | 3577 | 23 | 0 | 1047 | 4395 / 2608 | 9.63 s |
| SEAL 16 random d=40 | 3600 | 2560 | 1040 | 0 | 1761 | 26469 / 6345 | 11.88 s |

### ESP32-S3, 300 missions

| Scenario | Runs | Arrived | Honest FAILED | Fail |
| --- | ---: | ---: | ---: | ---: |
| static d=12 | 30 | 30 | 0 | 0 |
| static d=22 | 30 | 30 | 0 | 0 |
| static d=36 | 30 | 30 | 0 | 0 |
| static d=40 | 30 | 30 | 0 | 0 |
| 4 drops @95px d=36 | 30 | 30 | 0 | 0 |
| 4 drops @45px d=36 | 30 | 30 | 0 | 0 |
| 8 random drops d=30 | 30 | 30 | 0 | 0 |
| 16 random drops d=40 | 30 | 30 | 0 | 0 |
| SEAL 4 drops @45px d=36 | 30 | 30 | 0 | 0 |
| SEAL 16 random d=40 | 30 | 24 | 6 | 0 |

## Performance: the same operations everywhere (median ms, unsliced total work)

| Operation | JavaScript (Node) | C++ double | C++ float | ESP32-S3 | ESP32 ÷ C++ double |
| --- | ---: | ---: | ---: | ---: | ---: |
| Generate arena + coarse grid | 4.05 | 0.310 | 0.259 | 54.24 | 175× |
| Fine grid full build (480×320) | 80.37 | 3.711 | 2.247 | 886 | 239× |
| Obstacle drop + validation | 5.43 | 0.336 | 0.302 | 50.92 | 151× |
| Coarse A* incl. seeding | 0.94 | 0.087 | 0.092 | 7.32 | 84× |
| Fine A* incl. seeding | 20.94 | 2.061 | 1.927 | 192 | 93× |

On the ESP32-S3, drop validation is the sliced `DropTask` (all slices added up); on the host it is the synchronous `addObstacle`.

## Frame cost (ms per 16.7 ms frame)

| Implementation | Scenario | Median | p99 | Max | Note |
| --- | --- | ---: | ---: | ---: | --- |
| JavaScript (host) | static d=36 | 0.014 | 3.109 | 4.230 | 100 missions, 2 ms slices, 4671 sim frames/s |
| JavaScript (host) | 16 random drops d=40 | 0.032 | 3.093 | 7.493 | 100 missions, 2 ms slices, 3012 sim frames/s |
| C++ double (host) | static d=36 | 0.000 | 0.003 | 2.083 | 100 missions, 2 ms slices, 99220 sim frames/s |
| C++ double (host) | 16 random drops d=40 | 0.001 | 0.162 | 2.077 | 100 missions, 2 ms slices, 52321 sim frames/s |
| C++ float (host) | static d=36 | 0.000 | 0.002 | 2.012 | 100 missions, 2 ms slices, 157693 sim frames/s |
| C++ float (host) | 16 random drops d=40 | 0.001 | 0.344 | 4.008 | 100 missions, 2 ms slices, 54200 sim frames/s |
| **ESP32-S3 (real time)** | 4 static missions | 0.625 | 10.050 | 11.236 | 1921 frames |
| **ESP32-S3 (real time)** | 12 missions with drops | 0.729 | 12.140 | 14.834 | 8144 frames |

## ESP32-S3 real-time missions

16 of 16 missions arrived. **0 of 10065 frames** exceeded 16.7 ms (worst 14.83 ms).
Drops: 32 accepted, 22 rejected (22× would overlap the agent). Validation took a median of 5 frames (max 54) and 54.25 ms of work (max 403).
Memory: PSRAM peak 5.75 MB of 8 MB, internal RAM never below 224 KB free.

| Mission | Density | Drops ahead | Result | Time | Frames > 16.7 ms | Worst frame | Mean frame | Drops ok / rejected | Replans | Contact pushes |
| --- | ---: | ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| static-201 | 36 | – | ARRIVED | 8.57 s | 0 / 514 | 10.10 ms | 1.91 ms | 0 / 0 | 0 | 0 |
| static-202 | 36 | – | ARRIVED | 8.43 s | 0 / 506 | 11.24 ms | 2.26 ms | 0 / 0 | 0 | 0 |
| static-203 | 36 | – | ARRIVED | 7.97 s | 0 / 478 | 10.70 ms | 2.09 ms | 0 / 0 | 0 | 0 |
| static-204 | 36 | – | ARRIVED | 7.05 s | 0 / 423 | 10.71 ms | 2.42 ms | 0 / 0 | 0 | 0 |
| drops-101 | 30 | 45 px | ARRIVED | 9.68 s | 0 / 581 | 12.26 ms | 1.78 ms | 2 / 3 | 2 | 73 |
| drops-102 | 40 | 70 px | ARRIVED | 8.88 s | 0 / 533 | 13.05 ms | 2.86 ms | 3 / 1 | 3 | 0 |
| drops-103 | 30 | 95 px | ARRIVED | 7.15 s | 0 / 429 | 12.56 ms | 2.41 ms | 4 / 0 | 4 | 0 |
| drops-104 | 40 | 45 px | ARRIVED | 8.73 s | 0 / 524 | 12.29 ms | 2.21 ms | 0 / 4 | 0 | 0 |
| drops-105 | 30 | 70 px | ARRIVED | 20.82 s | 0 / 1249 | 14.83 ms | 2.18 ms | 4 / 1 | 4 | 8 |
| drops-106 | 40 | 95 px | ARRIVED | 9.08 s | 0 / 545 | 12.16 ms | 2.93 ms | 3 / 0 | 3 | 0 |
| drops-107 | 30 | 45 px | ARRIVED | 10.07 s | 0 / 604 | 12.71 ms | 1.87 ms | 1 / 4 | 1 | 85 |
| drops-108 | 40 | 70 px | ARRIVED | 14.27 s | 0 / 856 | 12.98 ms | 2.17 ms | 2 / 3 | 2 | 1 |
| drops-109 | 30 | 95 px | ARRIVED | 10.1 s | 0 / 606 | 12.60 ms | 2.40 ms | 5 / 0 | 5 | 1 |
| drops-110 | 40 | 45 px | ARRIVED | 11.2 s | 0 / 672 | 12.17 ms | 2.14 ms | 2 / 2 | 2 | 71 |
| drops-111 | 30 | 70 px | ARRIVED | 10.43 s | 0 / 626 | 12.36 ms | 2.17 ms | 2 / 3 | 2 | 0 |
| drops-112 | 40 | 95 px | ARRIVED | 15.32 s | 0 / 919 | 12.33 ms | 2.96 ms | 4 / 1 | 4 | 10 |

## ESP32-S3: drop-validation phases and search start-up (unsliced, ms)

| Step | Median | p90 | Max |
| --- | ---: | ---: | ---: |
| patch (rasterise the new obstacle) | 5.92 | 14.01 | 18.73 |
| seed B | 4.71 | 21.32 | 30.83 |
| seed agent | 4.58 | 26.16 | 28.08 |
| connect (meet in the middle) | 23.92 | 49.63 | 169 |
| AStar::start, fine grid (first) | 70.37 | 70.44 | 70.47 |
| AStar::start, fine grid (again) | 0.02 | 0.02 | 0.03 |
| EscapeSearch::start | 29.64 | 29.66 | 29.67 |
| commitObstacle | 1.91 | 2.38 | 3.01 |
