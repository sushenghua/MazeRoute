// harness.hpp — MCU (float) variant of cpp/src/harness.hpp; see geometry.hpp for the `real` type.
// Original: C++ mirror of tests/harness.js + the scenario table in tests/run.js.
#pragma once
#include <cstdlib>
#include <functional>
#include "drop.hpp"
#include "steering.hpp"

namespace sim {

/**
 * Ground-truth oracle, same answer as tests/harness.js solvable(): is the agent's cell 4-connected to B's on a
 * cs-px grid of discs of radius r + margin? The JS version floods cell by cell with a stack that can hold every
 * cell (2.4 MB at cs = 1); on the chip that growth failed once the heap was fragmented, so this one floods
 * scanline spans (the stack holds span seeds, a few thousand at most), marks cells in place instead of keeping a
 * second "seen" grid, and reuses both buffers between calls.
 */
inline bool solvable(const World& w, Vec2 p, real r, real margin = 0.2f, real cs = 3) {
  const int cols = static_cast<int>(std::floor(w.width / cs)), rows = static_cast<int>(std::floor(w.height / cs));
  enum : std::uint8_t { Blocked, Free, Seen };
  static std::vector<std::uint8_t> cell;  // grow-only scratch, shared by every call (the suite is single-threaded)
  static std::vector<int> stack;
  cell.assign(static_cast<std::size_t>(cols * rows), Blocked);
  for (int y = 0; y < rows; y++) for (int x = 0; x < cols; x++) cell[y * cols + x] = w.discFree((x + 0.5f) * cs, (y + 0.5f) * cs, r + margin) ? Free : Blocked;
  auto near = [&](Vec2 q) {
    int best = -1; real bd = kInf;
    const int cx = static_cast<int>(std::floor(q.x / cs)), cy = static_cast<int>(std::floor(q.y / cs)), R = static_cast<int>(std::ceil(2 * r / cs));
    for (int dy = -R; dy <= R; dy++) for (int dx = -R; dx <= R; dx++) {
      const int x = cx + dx, y = cy + dy;
      if (x < 0 || y < 0 || x >= cols || y >= rows) continue;
      const int i = y * cols + x;
      if (cell[i] != Free) continue;
      const real d = dx * dx + dy * dy;
      if (d < bd) { bd = d; best = i; }
    }
    return best;
  };
  const int s = near(p), g = near(w.goal);
  if (s < 0 || g < 0) return false;
  const int gx = g % cols, gy = g / cols;
  stack.clear(); stack.push_back(s);
  while (!stack.empty()) {
    const int i = stack.back(); stack.pop_back();
    if (cell[i] != Free) continue;
    const int y = i / cols, row = y * cols;
    int xl = i % cols, xr = xl;
    while (xl > 0 && cell[row + xl - 1] == Free) xl--;
    while (xr < cols - 1 && cell[row + xr + 1] == Free) xr++;
    if (y == gy && gx >= xl && gx <= xr) return true;
    for (int x = xl; x <= xr; x++) cell[row + x] = Seen;
    for (const int ny : {y - 1, y + 1}) {  // one seed per free run of the neighbouring row under [xl, xr]
      if (ny < 0 || ny >= rows) continue;
      for (int x = xl, nrow = ny * cols; x <= xr; x++)
        if (cell[nrow + x] == Free && (x == xl || cell[nrow + x - 1] != Free)) stack.push_back(nrow + x);
    }
  }
  return false;
}

struct Drop { real at, dist; };

struct Scenario {
  std::string name;
  int density;
  bool allowSealing = false;
  std::function<std::vector<Drop>(int)> drops = [](int) { return std::vector<Drop>{}; };
};

/** The JS scenario rng() helper is mulberry32 seeded with s * k (as a uint32). */
inline Mulberry32 seededRng(int s, int k) { return Mulberry32(static_cast<std::uint32_t>(static_cast<std::int64_t>(s) * k)); }

inline std::vector<Scenario> scenarios() {
  auto fixed = [](real dist) { return [dist](int) { std::vector<Drop> v; for (real at : {2, 6, 10, 14}) v.push_back({at, dist}); return v; }; };
  auto r8 = [](int s) { auto r = seededRng(s, 977); std::vector<Drop> v; for (int i = 0; i < 8; i++) { const real at = 0.5f + i * 1.5f + r(); const real d = 30 + r() * 150; v.push_back({at, d}); } return v; };
  auto r16 = [](int s) { auto r = seededRng(s, 131); std::vector<Drop> v; for (int i = 0; i < 16; i++) { const real at = 0.3f + i * 0.8f + r() * 0.5f; const real d = 20 + r() * 200; v.push_back({at, d}); } return v; };
  return {
      {"static d=12", 12}, {"static d=22", 22}, {"static d=36", 36}, {"static d=40", 40},
      {"4 drops @95px d=36", 36, false, fixed(95)},
      {"4 drops @45px d=36", 36, false, fixed(45)},
      {"8 random drops d=30", 30, false, r8},
      {"16 random drops d=40", 40, false, r16},
      {"SEAL 4 drops @45px d=36", 36, true, fixed(45)},
      {"SEAL 16 random d=40", 40, true, r16},
  };
}

struct MissionResult {
  int seed = 0; bool gen = true;
  State state = State::Idle; real t = 0;
  int placed = 0, rejected = 0, replans = 0, recov = 0, coll = 0;
  std::string reason;
  std::optional<bool> solvable, solvableAtGuarantee;
  bool sealed = false; std::optional<real> failLatency;
  // per-frame cost of (plan or background build) + tick, in ms
  std::vector<double>* frameCosts = nullptr;
};

inline MissionResult runMission(int seed, int density, const std::vector<Drop>& drops, bool allowSealing, bool checkContract,
                                std::vector<double>* frameCosts = nullptr, double planBudget = 50, double buildBudget = 3,
                                real radius = 9, real maxTime = 150) {
  // SLICE_MS sets every per-frame budget (planner, grid build, drop validation). About 0.06 ms on an
  // Apple M4 slices the work as finely as the 6 ms default does on an ESP32-S3.
  double dropBudget = planBudget;
  if (const char* e = std::getenv("SLICE_MS")) planBudget = buildBudget = dropBudget = std::atof(e);
#ifndef ESP_PLATFORM
  FakeClock::get().beginMission(static_cast<std::uint64_t>(seed) * 1000 + static_cast<std::uint64_t>(density) * 7 + drops.size() * 131 + allowSealing);
#endif
  MissionResult res; res.seed = seed;
  World world(960, 640, 10);
  Agent agent(radius);
  AStar astar;
  if (!world.generate(static_cast<std::uint32_t>(seed), density, radius)) { res.gen = false; return res; }
  if (std::getenv("PARITY")) world.fine->buildStep(kInf);  // see tests/harness.js
  Controller ctl(world, agent, astar);
  ctl.startMission();
  std::size_t di = 0;
  std::optional<real> sealedAt;
  DropTask task;  // drops are validated a slice per frame, exactly as the firmware does it
  while (!ctl.isTerminal() && ctl.simTime < maxTime) {
    const double f0 = frameCosts ? nowMs() : 0, f1 = nowMs();
    const bool plannerIdle = !ctl.isPlanning() && world.fine->ready;  // same budget rule as the firmware
    if (auto out = task.step(dropBudget + (plannerIdle ? planBudget : 0), agent.pos)) {
      if (out->accepted) { res.placed++; if (out->sealed && !sealedAt) sealedAt = ctl.simTime; ctl.onWorldChanged(); }
      else res.rejected++;
    }
    // One work budget per frame, as in the firmware: planner and background build get what the drop slice
    // left, and the background build stands down while a drop (which builds the grid itself) is pending.
    const double workLeft = dropBudget + planBudget - (nowMs() - f1);
    if (ctl.isPlanning()) { if (const double b = std::min(planBudget, workLeft); b > planBudget * 0.08) ctl.plan(b); }
    else if (!world.fine->ready && !task.busy()) { if (const double b = std::min(buildBudget, workLeft); b > buildBudget * 0.08) world.fine->buildStep(b); }
    ctl.tick(1.0f / 60);
    if (frameCosts) frameCosts->push_back(nowMs() - f0);
    if (di < drops.size() && ctl.simTime >= drops[di].at && !task.busy()) {  // a later drop waits for the one in flight
      const Drop d = drops[di++];
      const real px = agent.pos.x + std::cos(agent.heading) * d.dist, py = agent.pos.y + std::sin(agent.heading) * d.dist;
      if (dist({px, py}, world.goal) > 50 && px > 20 && py > 20 && px < world.width - 20 && py < world.height - 20) {
        if (task.start(world, px, py, radius, agent.pos, allowSealing)) res.rejected++;
      }
    }
  }
  res.state = ctl.state; res.t = ctl.simTime;
  res.replans = ctl.stats.replans; res.recov = ctl.stats.recoveries; res.coll = ctl.stats.collisions;
  res.reason = ctl.log.empty() ? "" : ctl.log.front().reason;
  if (ctl.state != State::Arrived) {
    res.solvable = solvable(world, agent.pos, radius);
    if (checkContract) res.solvableAtGuarantee = solvable(world, agent.pos, radius, kFineGuarantee, 1);
  }
  res.sealed = sealedAt.has_value();
  if (sealedAt && ctl.state == State::Failed) res.failLatency = ctl.simTime - *sealedAt;
#ifndef ESP_PLATFORM
  if (ctl.state != State::Arrived && !(allowSealing && res.solvableAtGuarantee == false))
    FakeClock::get().save(std::format("clock_d{}_n{}_s{}_seed{}.bin", density, drops.size(), int(allowSealing), seed));
#endif
  return res;
}

}  // namespace sim
