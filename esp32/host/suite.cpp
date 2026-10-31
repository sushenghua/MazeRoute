// suite.cpp — copy of cpp/src/suite.cpp run against the MCU (float) core. Usage: suite [seedsPerScenario] [firstSeed] [--dump file]
#include <fstream>
#include <print>
#include "harness.hpp"

int main(int argc, char** argv) {
  using namespace sim;
  const int N = argc > 1 ? std::atoi(argv[1]) : 100;
  const int S0 = argc > 2 ? std::atoi(argv[2]) : 1;
  std::ofstream dump;
  for (int i = 1; i + 1 < argc; i++) if (std::string_view(argv[i]) == "--dump") dump.open(argv[i + 1]);
  int totalFail = 0, totalRuns = 0;
  const double t0 = nowMs();
  for (const auto& sc : scenarios()) {
    std::vector<MissionResult> rows;
    for (int s = S0; s < S0 + N; s++) rows.push_back(runMission(s, sc.density, sc.drops(s), sc.allowSealing, sc.allowSealing));
    int runs = 0, notArrived = 0, fails = 0, zeroColl = 0, recov = 0, replans = 0, placed = 0, rejected = 0, honest = 0, sealedRuns = 0, sealedArrived = 0;
    double sumT = 0;
    std::vector<double> lat;
    std::vector<const MissionResult*> failList;
    for (const auto& r : rows) {
      if (!r.gen) continue;
      runs++; sumT += std::round(r.t * 10) / 10;
      zeroColl += r.coll == 0; recov += r.recov; replans += r.replans; placed += r.placed; rejected += r.rejected;
      if (r.sealed) { sealedRuns++; sealedArrived += r.state == State::Arrived; }
      if (r.state != State::Arrived) {
        notArrived++;
        const bool isHonest = sc.allowSealing && r.state == State::Failed && r.solvableAtGuarantee == false;
        if (isHonest) { honest++; if (r.failLatency) lat.push_back(*r.failLatency); }
        else { fails++; failList.push_back(&r); }
      }
      if (dump.is_open()) dump << std::format("{}|{}|{}|{:.1f}|{}|{}\n", sc.name, r.seed, name(r.state), r.t, r.placed, r.rejected);
    }
    totalFail += fails; totalRuns += runs;
    std::println("{:<22} runs {}  arrived {}  FAIL {}  zeroColl {}  recov {}  replans {}  placed {} rejected {}  avgT {:.1f}",
                 sc.name, runs, runs - notArrived, fails, zeroColl, recov, replans, placed, rejected, sumT / runs);
    for (std::size_t i = 0; i < std::min<std::size_t>(12, failList.size()); i++) {
      const auto& f = *failList[i];
      std::println("    seed {}: {} t={:.1f} solvable={} placed={} recov={} — {}", f.seed, name(f.state), f.t, f.solvable.value_or(true), f.placed, f.recov, f.reason);
    }
    if (sc.allowSealing) {
      std::ranges::sort(lat);
      std::println("{:<22} sealed runs {}  arrived anyway {}  honest FAILED {}  seal->FAILED latency median {:.1f} s  max {:.1f} s", "", sealedRuns,
                   sealedArrived, honest, lat.empty() ? 0.0 : lat[lat.size() / 2], lat.empty() ? 0.0 : lat.back());
    }
  }
  std::println("\nTOTAL runs {}  failures {}  ({:.1f} s)", totalRuns, totalFail, (nowMs() - t0) / 1000);
  return totalFail ? 1 : 0;
}
