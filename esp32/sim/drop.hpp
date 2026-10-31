// drop.hpp — MCU-only: obstacle-drop validation sliced across frames.
//
// World::addObstacle validates a drop in one go: build the fine grid if needed, re-rasterise the region,
// seed the agent and B, flood-fill, and in rare cases run the pixel escape. That is 80-210 ms on an
// ESP32-S3, a dozen frames. DropTask does the same work a budget at a time while the simulation keeps
// running, and replaces the full flood fill (85% of the cost) with a meet-in-the-middle search that stops
// as soon as the agent's and B's sides touch. The live world is never modified until the drop is accepted: during each slice the candidate
// polygon is visible to queries through World::overlay and its blocked cells are patched into the fine grid,
// and both are removed again before the slice returns.
#pragma once
#include "pathfinder.hpp"

namespace sim {

struct DropOutcome {
  bool accepted = false, sealed = false;
  std::string_view reason;  // why it was rejected
  std::size_t index = 0;    // the new polygon, when accepted
  double workMs = 0;        // CPU time over all slices
  int slices = 0;
};

class DropTask {
 public:
  bool busy() const { return phase_ != Phase::Idle; }
  /** Allocate the search buffers for a grid of `cells` now, so no slice pays for it later. */
  void reserve(std::size_t cells) { connect_.reserve(cells); goal_.reserve(cells); escape_.reserve(cells); }
  void cancel() { phase_ = Phase::Idle; }

  /** The checks that are cheap run immediately; returns the reason when the drop is rejected outright. */
  std::optional<std::string_view> start(World& w, real x, real y, real agentR, Vec2 from, bool allowSealing) {
    world_ = &w; r_ = agentR; from_ = from; allowSealing_ = allowSealing;
    poly_ = w.dropPolygon(x, y);
    if (poly_->circlePenetration(from.x, from.y, agentR + 4)) return "would overlap the agent";
    if (poly_->circlePenetration(w.goal.x, w.goal.y, agentR + 4)) return "would cover B";
    newly_.clear();
    workMs_ = 0; slices_ = 0; redo_ = 0; singleSeed_ = false;
    phase_ = Phase::Build;
    return std::nullopt;
  }

  /**
   * Spend up to `budgetMs`. Returns the outcome when finished; an accepted drop is already in the world.
   * `agentNow` is where the agent is at this frame: the drop is re-checked against it before committing.
   */
  std::optional<DropOutcome> step(double budgetMs, Vec2 agentNow) {
    if (!busy()) return std::nullopt;
    const double t0 = nowMs();
    slices_++;
    auto left = [&] { return budgetMs - (nowMs() - t0); };
    World& w = *world_;
    NavGrid& fine = *w.fine;
    Overlay overlay(*this);  // applied once the patch exists; always removed on return
    // The agent keeps moving while this runs; once it touches the candidate there is nothing left to decide.
    if (poly_->circlePenetration(agentNow.x, agentNow.y, r_ + 4)) return finish(t0, overlay, "would overlap the agent");
    for (;;) {
      switch (phase_) {
        case Phase::Idle: return std::nullopt;
        case Phase::Build:
          if (!fine.ready && !fine.buildStep(left())) return pause(t0);
          beginPatch();
          break;
        case Phase::Patch:
          if (!patchStep(t0, budgetMs)) return pause(t0);
          overlay.apply();
          seeder_.start(fine, w.goal.x, w.goal.y, r_);
          phase_ = Phase::GoalSeeds;
          break;
        case Phase::GoalSeeds:
          if (!seeder_.step(left())) return pause(t0);
          goalIds_.clear();
          for (auto& sd : seeder_.result()) goalIds_.push_back(sd.i);
          goalIdsLive_ = false;
          seeder_.start(fine, from_.x, from_.y, r_);
          phase_ = Phase::StartSeeds;
          break;
        case Phase::StartSeeds: {
          if (!seeder_.step(left())) return pause(t0);
          // From the nearest cell first: then every cell the search reaches is connected to B, which the
          // re-check in Commit relies on. Only if that finds no route, from every seed.
          singleSeed_ = !seeder_.result().empty();
          if (singleSeed_) connect_.start(fine, {seeder_.result().front().i}, w.goal, goalIds_, from_);
          else connect_.start(fine, {}, w.goal, goalIds_, from_);
          phase_ = Phase::Connect;
          break;
        }
        case Phase::Connect: {
          const auto res = connect_.step(left());
          if (res == ConnectSearch::Result::Running) return pause(t0);
          if (res == ConnectSearch::Result::Disconnected && singleSeed_ && seeder_.result().size() > 1) {
            std::vector<int> startIds;
            for (auto& sd : seeder_.result()) startIds.push_back(sd.i);
            singleSeed_ = false;
            connect_.start(fine, startIds, w.goal, goalIds_, from_);
            break;
          }
          connected_ = res == ConnectSearch::Result::Connected;
          viaEscape_ = false;
          phase_ = connected_ ? Phase::Commit : Phase::GoalComponent;
          break;
        }
        case Phase::GoalComponent: {  // no grid route: keep B's component for a possible escape check
          if (!connect_.finishSideB(left())) return pause(t0);
          connect_.takeSideB(goal_);
          // Did the agent have a grid route before this drop? If so, the drop removed it, and that seals B
          // even if a pixel squeeze remains (see js/world.js addObstacle). Checked on the live grid, so the
          // candidate is taken out for these phases.
          overlay.remove();
          seeder_.start(fine, w.goal.x, w.goal.y, r_);
          phase_ = Phase::BeforeGoalSeeds;
          break;
        }
        case Phase::BeforeGoalSeeds:
          if (!seeder_.step(left())) return pause(t0);
          goalIds_.clear();
          for (auto& sd : seeder_.result()) goalIds_.push_back(sd.i);
          goalIdsLive_ = true;  // seeded without the candidate: not valid for a re-check with it
          seeder_.start(fine, from_.x, from_.y, r_);
          phase_ = Phase::BeforeStartSeeds;
          break;
        case Phase::BeforeStartSeeds: {
          if (!seeder_.step(left())) return pause(t0);
          std::vector<int> startIds;
          for (auto& sd : seeder_.result()) startIds.push_back(sd.i);
          connect_.start(fine, startIds, w.goal, goalIds_, from_);
          phase_ = Phase::BeforeConnect;
          break;
        }
        case Phase::BeforeConnect: {
          const auto res = connect_.step(left());
          if (res == ConnectSearch::Result::Running) return pause(t0);
          overlay.apply();
          if (res == ConnectSearch::Result::Connected) { connected_ = false; viaEscape_ = false; phase_ = Phase::Commit; break; }
          escape_.start(w, fine, from_, r_, goal_);  // already off the grid before: the planner's last resort decides
          phase_ = Phase::Escape;
          break;
        }
        case Phase::Escape:
          if (!escape_.step(left())) return pause(t0);
          connected_ = escape_.result().has_value();
          viaEscape_ = connected_;
          phase_ = Phase::Commit;
          break;
        case Phase::Commit: {
          // The agent kept moving while this ran. Re-check against where it is now.
          if (poly_->circlePenetration(agentNow.x, agentNow.y, r_ + 4)) return finish(t0, overlay, "would overlap the agent");
          // "Connected" was decided for the position the check started from, and the agent has moved since.
          // It carries over when the agent's cell is known to be connected to B: reached from B's side, or
          // from the agent's side when that side grew from a single cell. (Grown from every seed, it may not
          // be: near a wall the cells visible from the agent can span two components, one of them a pocket the
          // candidate closes, and the agent can step into it while the check runs.) Otherwise (including any
          // answer that needed the pixel escape, which depends on the exact position) check again from here.
          // Retries are bounded; if the answer never settles, reject: conservative, never a sealed B.
          if (connected_ && dist(agentNow, from_) > 0.5f) {
            const int cell = fine.cellIndexAt(agentNow.x, agentNow.y);
            const auto side = fine.blocked[cell] ? 0 : connect_.side(cell);
            const bool carries = !viaEscape_ && (side == 2 || (side == 1 && singleSeed_));
            if (!carries) {
              if (redo_ >= 4) return finish(t0, overlay, "would seal B off from the agent");
              redo_++;
              from_ = agentNow;
              if (goalIdsLive_) { seeder_.start(fine, w.goal.x, w.goal.y, r_); phase_ = Phase::GoalSeeds; break; }
              seeder_.start(fine, from_.x, from_.y, r_);
              phase_ = Phase::StartSeeds;
              break;
            }
          }
          if (!connected_ && !allowSealing_) return finish(t0, overlay, "would seal B off from the agent");
          overlay.remove();
          DropOutcome out{.accepted = true, .sealed = !connected_};
          out.index = w.commitObstacle(std::move(*poly_), newly_);
          return done(t0, out);
        }
      }
      if (left() <= 0) return pause(t0);
    }
  }

 private:
  bool checkingBefore() const {
    return phase_ == Phase::BeforeGoalSeeds || phase_ == Phase::BeforeStartSeeds || phase_ == Phase::BeforeConnect;
  }
  enum class Phase { Idle, Build, Patch, GoalSeeds, StartSeeds, Connect, GoalComponent, BeforeGoalSeeds, BeforeStartSeeds, BeforeConnect, Escape, Commit };

  /** Makes the candidate visible to world queries and the fine grid for the duration of one slice. */
  struct Overlay {
    DropTask& task;
    bool on = false;
    explicit Overlay(DropTask& j) : task(j) { if (j.phase_ > Phase::Patch && !j.checkingBefore()) apply(); }
    ~Overlay() { remove(); }
    void apply() {
      if (on) return;
      task.world_->overlay = &*task.poly_;
      for (int i : task.newly_) task.world_->fine->blocked[i] = 1;
      on = true;
    }
    void remove() {
      if (!on) return;
      for (int i : task.newly_) task.world_->fine->blocked[i] = 0;
      task.world_->overlay = nullptr;
      on = false;
    }
  };

  World* world_ = nullptr;
  std::optional<Polygon> poly_;
  real r_ = 0;
  Vec2 from_{};
  bool allowSealing_ = false, connected_ = false, viaEscape_ = false, singleSeed_ = false, goalIdsLive_ = false;
  Phase phase_ = Phase::Idle;
  int patchRow_ = 0, pr1_ = 0, pc0_ = 0, pc1_ = 0, redo_ = 0, slices_ = 0;
  double workMs_ = 0;
  std::vector<int> newly_;
  std::vector<int> goalIds_;
  SeedSearch seeder_;
  ConnectSearch connect_;
  ComponentFlood goal_;  // passes B's component from connect_ to escape_ (buffers are swapped, never copied)
  EscapeSearch escape_;

  void beginPatch() {
    const NavGrid& g = *world_->fine;
    const real cs = g.cellSize, inf = g.inflation;
    const Polygon& p = *poly_;
    pc0_ = clampi(static_cast<int>(std::floor((p.minX - inf) / cs)), 0, g.cols - 1);
    pc1_ = clampi(static_cast<int>(std::ceil((p.maxX + inf) / cs)), 0, g.cols - 1);
    patchRow_ = clampi(static_cast<int>(std::floor((p.minY - inf) / cs)), 0, g.rows - 1);
    pr1_ = clampi(static_cast<int>(std::ceil((p.maxY + inf) / cs)), 0, g.rows - 1);
    phase_ = Phase::Patch;
  }

  /** Cells the candidate blocks that are free now, one row at a time (same test as NavGrid::updateRegion). */
  bool patchStep(double t0, double budgetMs) {
    const NavGrid& g = *world_->fine;
    const real cs = g.cellSize;
    for (; patchRow_ <= pr1_; patchRow_++) {
      if (nowMs() - t0 > budgetMs) return false;
      const real y = (patchRow_ + 0.5f) * cs;
      for (int c = pc0_; c <= pc1_; c++) {
        const int i = patchRow_ * g.cols + c;
        if (!g.blocked[i] && poly_->circlePenetration((c + 0.5f) * cs, y, g.inflation)) newly_.push_back(i);
      }
    }
    return true;
  }

  std::optional<DropOutcome> pause(double t0) { workMs_ += nowMs() - t0; return std::nullopt; }
  std::optional<DropOutcome> finish(double t0, Overlay& overlay, std::string_view reason) {
    overlay.remove();
    return done(t0, DropOutcome{.reason = reason});
  }
  std::optional<DropOutcome> done(double t0, DropOutcome out) {
    workMs_ += nowMs() - t0;
    out.workMs = workMs_;
    out.slices = slices_;
    phase_ = Phase::Idle;
    poly_.reset();
    return out;
  }
};

}  // namespace sim
