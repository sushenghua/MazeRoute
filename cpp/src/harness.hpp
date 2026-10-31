// harness.hpp — C++ mirror of tests/harness.js + the scenario table in tests/run.js.
#pragma once
#include <cstdlib>
#include <functional>
#include "steering.hpp"

namespace sim {

/** Ground-truth oracle, identical to tests/harness.js solvable(). */
inline bool solvable(const World& w, Vec2 p, double r, double margin = 0.2, double cs = 3) {
  const int cols = static_cast<int>(std::floor(w.width / cs)), rows = static_cast<int>(std::floor(w.height / cs));
  std::vector<std::uint8_t> freeCell(static_cast<std::size_t>(cols * rows));
  for (int y = 0; y < rows; y++) for (int x = 0; x < cols; x++) freeCell[y * cols + x] = w.discFree((x + 0.5) * cs, (y + 0.5) * cs, r + margin);
  auto near = [&](Vec2 q) {
    int best = -1; double bd = kInf;
    const int cx = static_cast<int>(std::floor(q.x / cs)), cy = static_cast<int>(std::floor(q.y / cs)), R = static_cast<int>(std::ceil(2 * r / cs));
    for (int dy = -R; dy <= R; dy++) for (int dx = -R; dx <= R; dx++) {
      const int x = cx + dx, y = cy + dy;
      if (x < 0 || y < 0 || x >= cols || y >= rows) continue;
      const int i = y * cols + x;
      if (!freeCell[i]) continue;
      const double d = dx * dx + dy * dy;
      if (d < bd) { bd = d; best = i; }
    }
    return best;
  };
  const int s = near(p), g = near(w.goal);
  if (s < 0 || g < 0) return false;
  std::vector<std::uint8_t> seen(freeCell.size(), 0);
  std::vector<int> q{s}; seen[s] = 1;
  while (!q.empty()) {
    const int i = q.back(); q.pop_back();
    if (i == g) return true;
    const int x = i % cols, y = i / cols;
    constexpr int d4[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
    for (auto [dx, dy] : d4) {
      const int nx = x + dx, ny = y + dy;
      if (nx < 0 || ny < 0 || nx >= cols || ny >= rows) continue;
      const int j = ny * cols + nx;
      if (freeCell[j] && !seen[j]) { seen[j] = 1; q.push_back(j); }
    }
  }
  return false;
}

struct Drop { double at, dist; };

struct Scenario {
  std::string name;
  int density;
  bool allowSealing = false;
  std::function<std::vector<Drop>(int)> drops = [](int) { return std::vector<Drop>{}; };
};

/** The JS scenario rng() helper is mulberry32 seeded with s * k (as a uint32). */
inline Mulberry32 seededRng(int s, int k) { return Mulberry32(static_cast<std::uint32_t>(static_cast<std::int64_t>(s) * k)); }

inline std::vector<Scenario> scenarios() {
  auto fixed = [](double dist) { return [dist](int) { std::vector<Drop> v; for (double at : {2, 6, 10, 14}) v.push_back({at, dist}); return v; }; };
  auto r8 = [](int s) { auto r = seededRng(s, 977); std::vector<Drop> v; for (int i = 0; i < 8; i++) { const double at = 0.5 + i * 1.5 + r(); const double d = 30 + r() * 150; v.push_back({at, d}); } return v; };
  auto r16 = [](int s) { auto r = seededRng(s, 131); std::vector<Drop> v; for (int i = 0; i < 16; i++) { const double at = 0.3 + i * 0.8 + r() * 0.5; const double d = 20 + r() * 200; v.push_back({at, d}); } return v; };
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
  State state = State::Idle; double t = 0;
  int placed = 0, rejected = 0, replans = 0, recov = 0, coll = 0;
  std::string reason;
  std::optional<bool> solvable, solvableAtGuarantee;
  bool sealed = false; std::optional<double> failLatency;
  // per-frame cost of (plan or background build) + tick, in ms
  std::vector<double>* frameCosts = nullptr;
};

inline MissionResult runMission(int seed, int density, const std::vector<Drop>& drops, bool allowSealing, bool checkContract,
                                std::vector<double>* frameCosts = nullptr, double planBudget = 50, double buildBudget = 3,
                                double radius = 9, double maxTime = 150) {
  MissionResult res; res.seed = seed;
  World world(960, 640, 10);
  Agent agent(radius);
  AStar astar;
  if (!world.generate(static_cast<std::uint32_t>(seed), density, radius)) { res.gen = false; return res; }
  if (std::getenv("PARITY")) world.fine->buildStep(kInf);  // see tests/harness.js
  Controller ctl(world, agent, astar);
  ctl.startMission();
  std::size_t di = 0;
  std::optional<double> sealedAt;
  while (!ctl.isTerminal() && ctl.simTime < maxTime) {
    const double f0 = frameCosts ? nowMs() : 0;
    if (ctl.isPlanning()) ctl.plan(planBudget);
    else if (!world.fine->ready) world.fine->buildStep(buildBudget);
    ctl.tick(1.0 / 60);
    if (frameCosts) frameCosts->push_back(nowMs() - f0);
    if (di < drops.size() && ctl.simTime >= drops[di].at) {
      const Drop d = drops[di++];
      const double px = agent.pos.x + std::cos(agent.heading) * d.dist, py = agent.pos.y + std::sin(agent.heading) * d.dist;
      if (dist({px, py}, world.goal) > 50 && px > 20 && py > 20 && px < world.width - 20 && py < world.height - 20) {
        auto r = world.addObstacle(px, py, radius, agent.pos, allowSealing);
        if (r) { res.placed++; if (r->sealed && !sealedAt) sealedAt = ctl.simTime; ctl.onWorldChanged(); }
        else res.rejected++;
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
  return res;
}

}  // namespace sim
