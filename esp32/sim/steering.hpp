// steering.hpp — MCU (float) variant of cpp/src/steering.hpp; see geometry.hpp for the `real` type.
// Original: C++23 port of js/steering.js: directional agent + watchdog state machine.
// The JS table of state objects becomes a switch over an enum; transitions are identical.
#pragma once
#include <format>
#include <string>
#include <utility>
#include "pathfinder.hpp"

namespace sim {

enum class State { Idle, Planning, Following, Recovering, Replanning, Arrived, Failed };

inline std::string_view name(State s) {
  constexpr std::string_view n[] = {"IDLE", "PLANNING", "FOLLOWING", "RECOVERING", "REPLANNING", "ARRIVED", "FAILED"};
  return n[std::to_underlying(s)];
}

struct Agent {
  real radius;
  Vec2 pos{};
  real heading = 0, speed = 0;
  real maxSpeed = 150, maxAccel = 320, maxTurn = 3.6f;
  explicit Agent(real r) : radius(r) {}
  void place(Vec2 p, real h) { pos = p; heading = h; speed = 0; }
};

struct Transition { real t; State from, to; std::string reason; };

struct Stats { int replans = 0, recoveries = 0, collisions = 0, escalation = 0, stuckEvents = 0; long planNodes = 0; double planMs = 0; int planSlices = 0; };

class Controller {
 public:
  static constexpr real kStuckSeconds = 1.2f, kMinProgress = 8;

  World& world;
  Agent& agent;
  AStar& astar;
  State state = State::Idle;
  real timeInState = 0, deadline = kInf, simTime = 0;
  std::vector<Vec2> path, rawPath;
  std::vector<real> cum;
  int segIdx = 0;
  std::optional<Vec2> target, onPath;
  bool targetVisible = false;
  real cornerSpeed = kInf, remaining = 0, bestRemaining = kInf, anchorRemaining = kInf;
  real lastProgressTime = 0, remainingAtRecovery = kInf;
  int stuckStreak = 0, planLevel = 0, frame = 0;
  bool pendingGrid = false, keepWatchdog = false, preferFine = false;
  real lastLateral = 0, brake = 0, recoverSide = 1;
  Vec2 avoidLateral{}, avoidNormal{};
  std::vector<Transition> log;  // newest first, like the JS
  std::vector<Vec2> escapePrefix_;
  Stats stats;

 private:
  enum class PlanPhase { Grid, GoalSeeds, StartSeeds, Escape, Search, Smooth };
  PlanPhase planPhase_ = PlanPhase::Grid;
  SeedSearch seeder_;
  EscapeSearch escape_;
  PathSmoother smoother_;
  std::vector<Seed> goals_;

 public:

  Controller(World& w, Agent& a, AStar& s) : world(w), agent(a), astar(s) { reset(); }

  void reset() {
    state = State::Idle; timeInState = 0; deadline = kInf; simTime = 0;
    path.clear(); rawPath.clear(); cum.clear(); segIdx = 0;
    target.reset(); targetVisible = false; cornerSpeed = kInf; onPath.reset(); remaining = 0; bestRemaining = kInf;
    lastProgressTime = 0; remainingAtRecovery = kInf;
    stuckStreak = 0; pendingGrid = false; keepWatchdog = false; preferFine = false; planLevel = 0; lastLateral = 0; brake = 0; frame = 0;
    avoidLateral = {}; avoidNormal = {};
    log.clear(); stats = {};
    if (world.nav) agent.place(world.start, 0);
  }

  /** Allocate the planner's buffers for the fine grid now (see Labels::reserve). */
  void reserve() {
    const auto cells = static_cast<std::size_t>(world.fine->cols * world.fine->rows);
    astar.reserve(cells);
    escape_.reserve(cells);
  }

  bool isTerminal() const { return state == State::Arrived || state == State::Failed; }
  bool isPlanning() const { return state == State::Planning || state == State::Replanning; }

  void startMission() { if (state != State::Idle) reset(); go(State::Planning, "start requested"); }

  void go(State next, std::string reason) {
    pushLog({simTime, state, next, std::move(reason)});
    state = next;
    timeInState = 0;
    enter(next);
    deadline = maxTime(next);
  }

  void tick(real dt) {
    simTime += dt;
    timeInState += dt;
    const State st = state;
    switch (st) {
      case State::Planning: case State::Replanning: agent.speed = approach(agent.speed, 0, agent.maxAccel * dt); break;
      case State::Following: follow(dt); break;
      case State::Recovering: recoverTick(dt); break;
      default: break;
    }
    if (timeInState > deadline) { timeInState = 0; onTimeout(st); }
  }

  void plan(double budgetMs) { if (isPlanning()) stepPlan(budgetMs); }

  void onWorldChanged() {
    if (state == State::Following) {  // replan on every change; see js/steering.js onWorldChanged
      keepWatchdog = true;
      go(State::Replanning, pathAheadClear(static_cast<int>(path.size())) ? "world changed: replanning" : "world changed: path blocked");
    }
    else if (isPlanning()) beginPlan(preferFine ? 1 : 0);
  }

 private:
  void pushLog(Transition t) { log.insert(log.begin(), std::move(t)); if (log.size() > 14) log.pop_back(); }

  real maxTime(State s) const {
    switch (s) {
      case State::Planning: case State::Replanning: return 20;
      case State::Following: return 6 + pathLength(path) / (agent.maxSpeed * 0.25f);
      case State::Recovering: return 0.6f;
      default: return kInf;
    }
  }

  void enter(State s) {
    switch (s) {
      case State::Planning: beginPlan(preferFine ? 1 : 0); break;
      case State::Following:
        if (!keepWatchdog) lastProgressTime = simTime;
        keepWatchdog = false;
        bestRemaining = kInf; anchorRemaining = kInf; frame = 0;
        break;
      case State::Recovering:
        stats.recoveries++; stuckStreak++;
        remainingAtRecovery = remaining;
        recoverSide = lastLateral >= 0 ? 1 : -1;
        break;
      case State::Replanning: stats.replans++; beginPlan(preferFine ? 1 : 0); break;
      case State::Arrived: agent.speed = 0; break;
      case State::Failed: agent.speed = 0; path.clear(); rawPath.clear(); target.reset(); break;
      default: break;
    }
  }

  void onTimeout(State s) {
    switch (s) {
      case State::Planning: case State::Replanning: go(State::Failed, "planner deadline exceeded"); break;
      case State::Following: go(State::Recovering, "follow deadline exceeded"); break;
      case State::Recovering:
        if (stuckStreak >= 7) { go(State::Failed, "no progress after 7 recoveries"); return; }
        if (stuckStreak >= 3 && !preferFine) { preferFine = true; stats.escalation++; go(State::Replanning, "escalation: plan on the fine grid from now on"); return; }
        go(State::Replanning, "recovery manoeuvre done");
        break;
      default: break;
    }
  }

  void recoverTick(real dt) {
    auto& a = agent;
    const Vec2 back = fromAngle(a.heading + kPi, a.radius + 6);
    const bool rearClear = world.sweep(a.pos.x, a.pos.y, a.pos.x + back.x, a.pos.y + back.y, a.radius * 0.95f) >= 1;
    const Vec2 aim = (onPath && dist(*onPath, a.pos) > 2) ? *onPath - a.pos : fromAngle(a.heading + recoverSide);
    const real dh = wrapAngle(angleOf(aim) - a.heading);
    a.heading = wrapAngle(a.heading + clampd(dh, -a.maxTurn * 0.6f * dt, a.maxTurn * 0.6f * dt));
    move(dt, std::nullopt, rearClear ? -a.maxSpeed * 0.3f : 0);
  }

  /**
   * Planning runs as phases that all resume across frames: (grid build) -> goal seeds -> start seeds or
   * pixel escape -> A* -> smoothing. Starting a plan does no work; stepPlan() spends one frame's budget, moving on to the
   * next phase in the same slice while time remains. On the browser the seeding, escape and smoothing steps take well
   * under a millisecond, so the JS runs them inline; on an ESP32-S3 they take 16-150 ms and must be sliced.
   */
  void beginPlan(int level) {
    planLevel = level;
    planPhase_ = PlanPhase::Grid;
    escapePrefix_.clear();
  }

  const NavGrid& planGrid() const { return planLevel == 0 ? *world.nav : *world.fine; }

  void stepPlan(double budgetMs) {
    const double t0 = nowMs();
    auto left = [&] { return budgetMs - (nowMs() - t0); };
    for (;;) {
      const NavGrid& g = planGrid();
      switch (planPhase_) {
        case PlanPhase::Grid:
          pendingGrid = !g.ready;
          if (pendingGrid && !world.fine->buildStep(left())) return;
          pendingGrid = false;
          seeder_.start(g, world.goal.x, world.goal.y, agent.radius);
          planPhase_ = PlanPhase::GoalSeeds;
          break;
        case PlanPhase::GoalSeeds:
          if (!seeder_.step(left())) return;
          goals_ = seeder_.result();
          if (planLevel == 2) {
            std::vector<int> ids;
            for (auto& sd : goals_) ids.push_back(sd.i);
            escape_.start(world, g, agent.pos, agent.radius, ids);
            planPhase_ = PlanPhase::Escape;
          } else {
            seeder_.start(g, agent.pos.x, agent.pos.y, agent.radius);
            planPhase_ = PlanPhase::StartSeeds;
          }
          break;
        case PlanPhase::StartSeeds:
          if (!seeder_.step(left())) return;
          astar.start(g, seeder_.result(), goals_);
          planPhase_ = PlanPhase::Search;
          break;
        case PlanPhase::Escape:
          if (!escape_.step(left())) return;
          if (const auto& esc = escape_.result()) {
            escapePrefix_ = esc->path;
            astar.start(g, {Seed{esc->cell, esc->length}}, goals_);
          } else {
            astar.start(g, {}, goals_);  // fails at once: no route at the agent's size
          }
          planPhase_ = PlanPhase::Search;
          break;
        case PlanPhase::Search: {
          const auto status = astar.step(std::max(0.0, left()));
          stats.planNodes = astar.expanded; stats.planMs = astar.msSpent; stats.planSlices = astar.slices;
          if (status == SearchStatus::Found) {
            auto raw = astar.path();
            if (!escapePrefix_.empty()) raw.insert(raw.begin(), escapePrefix_.begin(), escapePrefix_.end());
            else raw.insert(raw.begin(), agent.pos);
            raw.push_back(world.goal);
            rawPath = raw;
            smoother_.start(rawPath, world, agent.radius + 3);
            planPhase_ = PlanPhase::Smooth;
            break;
          }
          if (status != SearchStatus::Failed) return;
          if (planLevel == 2) { go(State::Failed, "no route exists at the agent's size"); return; }
          const int next = planLevel + 1;
          constexpr const char* why[] = {"", "coarse grid found no route, retrying on fine grid", "no grid route from here, trying a pixel-level escape"};
          pushLog({simTime, state, state, why[next]});
          beginPlan(next);
          break;
        }
        case PlanPhase::Smooth: {
          if (!smoother_.step(left())) return;
          setPath(smoother_.take());
          constexpr const char* tiers[] = {"coarse", "fine", "fine via pixel escape"};
          go(State::Following, std::format("path found on {} grid: {} nodes in {} slice{}", tiers[planLevel],
                                           astar.expanded, astar.slices, astar.slices > 1 ? "s" : ""));
          return;
        }
      }
      if (left() <= 0) return;
    }
  }

  void setPath(std::vector<Vec2> pts) {
    path = std::move(pts);
    cum.assign(1, 0.0f);
    for (std::size_t i = 1; i < path.size(); i++) cum.push_back(cum[i - 1] + dist(path[i - 1], path[i]));
    segIdx = 0;
    remaining = cum.back();
  }

  Vec2 pointAt(real s, int k0) const {
    const int n = static_cast<int>(path.size()) - 1;
    if (s >= cum[n]) return path[n];
    int k = k0;
    while (k < n - 1 && cum[k + 1] < s) k++;
    const real den = cum[k + 1] - cum[k];
    const real f = clampd((s - cum[k]) / (den != 0 ? den : 1), 0, 1);
    return {path[k].x + (path[k + 1].x - path[k].x) * f, path[k].y + (path[k + 1].y - path[k].y) * f};
  }

  void trackPath() {
    const auto& p = path; auto& a = agent;
    const int n = static_cast<int>(p.size()) - 1;
    PointSeg best{}; bool have = false; int bestK = segIdx;
    for (int k = segIdx; k <= std::min(segIdx + 2, n - 1); k++) {
      const auto r = distPointSeg(a.pos.x, a.pos.y, p[k].x, p[k].y, p[k + 1].x, p[k + 1].y);
      if (!have || r.d < best.d) { best = r; bestK = k; have = true; }
    }
    segIdx = bestK;
    const real along = cum[bestK] + best.t * (cum[bestK + 1] - cum[bestK]);
    const real total = cum[n];
    remaining = total - along + best.d;
    onPath = Vec2{best.qx, best.qy};

    const real lookahead = 34 + std::max(real(0), a.speed) * 0.22f;
    target.reset();
    for (real sweepR : {a.radius + 2.5f, a.radius * 0.95f}) {
      for (real s = std::min(along + lookahead, total); s >= along - 0.01f; s -= 4) {
        const Vec2 pt = pointAt(s, bestK);
        if (world.segmentFree(a.pos.x, a.pos.y, pt.x, pt.y, sweepR)) { target = pt; break; }
      }
      if (target) break;
    }
    targetVisible = target.has_value();
    if (!target) target = onPath;

    cornerSpeed = a.maxSpeed;
    real d = cum[bestK + 1] - along;
    for (int k = bestK + 1; k < n && d < 160; k++) {
      const Vec2 d1 = norm(p[k] - p[k - 1]), d2 = norm(p[k + 1] - p[k]);
      const real turn = std::acos(clampd(dot(d1, d2), -1, 1));
      const real vc = a.maxSpeed * clampd(1 - turn / 2.0f, 0.12f, 1);
      cornerSpeed = std::min(cornerSpeed, std::sqrt(vc * vc + 2 * a.maxAccel * 0.7f * std::max(real(0), d - 4)));
      d += cum[k + 1] - cum[k];
    }
  }

  bool pathAheadClear(int count) const {
    const auto& p = path; const real r = agent.radius;
    for (int k = segIdx; k < std::min(segIdx + count, static_cast<int>(p.size()) - 1); k++) {
      const Vec2 a = k == segIdx ? (onPath ? *onPath : agent.pos) : p[k];
      if (!world.segmentFree(a.x, a.y, p[k + 1].x, p[k + 1].y, r * 0.75f)) return false;
    }
    return true;
  }

  void computeAvoidance() {
    auto& a = agent;
    const real L = 26 + std::max(real(0), a.speed) * 0.3f;
    const Vec2 side = perp(fromAngle(a.heading));
    const real defs[3][2] = {{0, L}, {0.55f, L * 0.7f}, {-0.55f, L * 0.7f}};
    real t[3];
    for (int i = 0; i < 3; i++) {
      const Vec2 dv = fromAngle(a.heading + defs[i][0], defs[i][1]);
      t[i] = world.sweep(a.pos.x, a.pos.y, a.pos.x + dv.x, a.pos.y + dv.y, a.radius * 0.9f);
    }
    const real c = t[0], l = t[1], r = t[2];
    real lat = 0, br = 0;
    if (c < 1) { lat += (l > r ? 1 : -1) * (1 - c) * 1.6f; br += 1 - c; }
    if (l < 1) lat -= (1 - l) * 1.0f;
    if (r < 1) lat += (1 - r) * 1.0f;
    avoidLateral = side * lat;
    avoidNormal = {};
    if (auto ns = world.nearestSurface(a.pos.x, a.pos.y, a.radius + 8)) {
      const real k = (1 - ns->d / (a.radius + 8)) * 0.8f;
      avoidNormal = {ns->nx * k, ns->ny * k};
    }
    lastLateral = lat;
    brake = clampd(br, 0, 0.6f);
  }

  void follow(real dt) {
    auto& a = agent;
    frame++;
    trackPath();
    const real dGoal = dist(a.pos, world.goal);
    if (dGoal < a.radius + 3) { go(State::Arrived, std::format("reached B after {:.1f} s", simTime)); return; }
    computeAvoidance();
    const Vec2 toTarget = norm(*target - a.pos);
    Vec2 desired; real br;
    if (targetVisible) { desired = toTarget + avoidNormal * 0.3f; br = 0; }
    else { desired = toTarget + avoidLateral + avoidNormal; br = brake; }
    const real arrive = clampd(dGoal / 70, 0.2f, 1);
    move(dt, desired, std::min(a.maxSpeed * (1 - br) * arrive, cornerSpeed));

    if (remaining < anchorRemaining - kMinProgress) { anchorRemaining = remaining; lastProgressTime = simTime; }
    if (remaining < remainingAtRecovery - 60) stuckStreak = 0;
    if (simTime - lastProgressTime > kStuckSeconds) { stats.stuckEvents++; go(State::Recovering, "no progress for 1.2f s"); return; }
    if (frame % 10 == 0 && !pathAheadClear(3)) { keepWatchdog = true; go(State::Replanning, "path ahead blocked"); }
  }

  static real approach(real v, real tgt, real maxDelta) { return v + clampd(tgt - v, -maxDelta, maxDelta); }

  void move(real dt, std::optional<Vec2> desired, real speedTarget) {
    auto& a = agent;
    if (desired && (desired->x != 0 || desired->y != 0)) {
      const real dh = wrapAngle(angleOf(*desired) - a.heading);
      a.heading = wrapAngle(a.heading + clampd(dh, -a.maxTurn * dt, a.maxTurn * dt));
      const real align = std::cos(dh);
      speedTarget *= align < 0.5f ? 0 : align * align;
    }
    a.speed = approach(a.speed, speedTarget, a.maxAccel * dt);
    a.pos.x += std::cos(a.heading) * a.speed * dt;
    a.pos.y += std::sin(a.heading) * a.speed * dt;
    const int hits = world.resolveDisc(a.pos, a.radius);
    if (hits) { stats.collisions += hits; a.speed *= 0.6f; }
  }
};

}  // namespace sim
