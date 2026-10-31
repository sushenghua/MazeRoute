// bench.cpp — C++ counterpart of tests/bench.js. Same seeds, same operations, same statistics.
#include <print>
#include "harness.hpp"

using namespace sim;

static void report(std::string_view label, std::vector<double> v) {
  std::ranges::sort(v);
  auto at = [&](double q) { return v[std::min(v.size() - 1, static_cast<std::size_t>(q * v.size()))]; };
  std::println("{:<34} median {:9.4f} ms   p99 {:9.4f} ms   max {:9.4f} ms   (n={})", label, at(0.5), at(0.99), v.back(), v.size());
}

int main() {
  // 1) Operations (mirrors tests/perf.js; 60 worlds at density 36).
  std::vector<double> gen, fineBuild, add, fineAstar, coarseAstar;
  for (int seed = 1; seed <= 60; seed++) {
    World w(960, 640, 10);
    double t = nowMs(); w.generate(seed, 36, 9); gen.push_back(nowMs() - t);
    t = nowMs(); w.fine->buildStep(kInf); fineBuild.push_back(nowMs() - t);
    for (int k = 0; k < 5; k++) { t = nowMs(); (void)w.addObstacle(100 + k * 170, 320, 9, w.start); add.push_back(nowMs() - t); }
    AStar a;
    t = nowMs(); a.start(*w.nav, w.nav->seedCells(w.start.x, w.start.y, 9), w.nav->seedCells(w.goal.x, w.goal.y, 9)); while (a.step(1e9) == SearchStatus::Running) {} coarseAstar.push_back(nowMs() - t);
    t = nowMs(); a.start(*w.fine, w.fine->seedCells(w.start.x, w.start.y, 9), w.fine->seedCells(w.goal.x, w.goal.y, 9)); while (a.step(1e9) == SearchStatus::Running) {} fineAstar.push_back(nowMs() - t);
  }
  std::println("== operations (60 worlds, density 36)");
  report("generate (coarse grid)", gen);
  report("fine grid full build", fineBuild);
  report("addObstacle + validation", add);
  report("coarse A* incl. seeding (unsliced)", coarseAstar);
  report("fine A* incl. seeding (unsliced)", fineAstar);

  // 2) Per-frame simulation cost: plan/background slice + one controller tick, over whole missions.
  std::println("== per-frame cost with app-like 2 ms slices: plan or build slice + tick (100 missions each)");
  for (int si : {2, 7}) {
    const auto sc = scenarios()[si];
    std::vector<double> frames;
    const double t0 = nowMs();
    for (int s = 1; s <= 100; s++) runMission(s, sc.density, sc.drops(s), false, false, &frames, 2, 2);
    const double total = nowMs() - t0;
    report(sc.name, frames);
    std::println("{:<34} {:.0f} simulated frames per wall-clock second", "", frames.size() / (total / 1000));
  }
}
