# CLAUDE.md

Portfolio repo for MazeRoute: a real-time path-planning and steering demo with a browser app (`index.html`, `js/`), a C++23 port (`cpp/`), and an ESP32-S3 port (`esp32/`). The code lives here and runs from here; there is no build step for the web app.

- `README.md` is the short landing page. Keep it brief; the long write-up is `docs/DESIGN.md`.
- `media/` holds the demo video (`maze_route.mp4`, H.264, no audio), its poster frame and screenshots. The README plays the video from a GitHub `user-attachments` URL (uploaded through the web editor), so re-upload there after replacing the file. `poster.jpg` is shown by the PortfolioHome index via its raw GitHub URL. Keep filenames so links don't break.
- `results/runs/20261031T035331Z` is the recorded run every number in the README comes from. Change a number in the README only after re-running (`results/run_all.sh`) and pointing `results/latest` at the new run.
- Keep the repo about the project itself: no references to where it originated or why it was written.

Checks: `node tests/run.js 10 1` (quick), `node tests/run.js 300 1` (full); C++ via `cmake -S cpp -B cpp/build && cmake --build cpp/build && ./cpp/build/suite 300 1`.
