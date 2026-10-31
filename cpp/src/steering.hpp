// steering.hpp — C++23 port of js/steering.js: directional agent + watchdog state machine.
// The JS table of state objects becomes a switch over an enum; transitions are identical.
#pragma once
#include <format>
#include <string>
#include "pathfinder.hpp"

namespace sim {

enum class State { Idle, Planning, Following, Recovering, Replanning, Arrived, Failed };

inline std::string_view name(State s) {
  constexpr std::string_view n[] = {"IDLE", "PLANNING", "FOLLOWING", "RECOVERING", "REPLANNING", "ARRIVED", "FAILED"};
  return n[std::to_underlying(s)];
}

struct Agent {
  double radius;
  Vec2 pos{};
  double heading = 0, speed = 0;
  double maxSpeed = 150, maxAccel = 320, maxTurn = 3.6;
  explicit Agent(double r) : radius(r) {}
  void place(Vec2 p, double h) { pos = p; heading = h; speed = 0; }
};

struct Transition { double t; State from, to; std::string reason; };

struct Stats { int replans = 0, recoveries = 0, collisions = 0, escalation = 0, stuckEvents = 0; long planNodes = 0; double planMs = 0; int planSlices = 0; };

class Controller {
 public:
  static constexpr double kStuckSeconds = 1.2, kMinProgress = 8;

  World& world;
  Agent& agent;
  AStar& astar;
  State state = State::Idle;
  double timeInState = 0, deadline = kInf, simTime = 0;
  std::vector<Vec2> path, rawPath;
  std::vector<double> cum;
  int segIdx = 0;
  std::optional<Vec2> target, onPath;
  bool targetVisible = false;
  double cornerSpeed = kInf, remaining = 0, bestRemaining = kInf, anchorRemaining = kInf;
  double lastProgressTime = 0, remainingAtRecovery = kInf;
  int stuckStreak = 0, planLevel = 0, frame = 0;
  bool pendingGrid = false, keepWatchdog = false, preferFine = false;
  double lastLateral = 0, brake = 0, recoverSide = 1;
  Vec2 avoidLateral{}, avoidNormal{};
  std::vector<Transition> log;  // newest first, like the JS
  std::vector<Vec2> escapePrefix_;
  Stats stats;

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

  void tick(double dt) {
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

  double maxTime(State s) const {
    switch (s) {
      case State::Planning: case State::Replanning: return 20;
      case State::Following: return 6 + pathLength(path) / (agent.maxSpeed * 0.25);
      case State::Recovering: return 0.6;
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

  void recoverTick(double dt) {
    auto& a = agent;
    const Vec2 back = fromAngle(a.heading + kPi, a.radius + 6);
    const bool rearClear = world.sweep(a.pos.x, a.pos.y, a.pos.x + back.x, a.pos.y + back.y, a.radius * 0.95) >= 1;
    const Vec2 aim = (onPath && dist(*onPath, a.pos) > 2) ? *onPath - a.pos : fromAngle(a.heading + recoverSide);
    const double dh = wrapAngle(angleOf(aim) - a.heading);
    a.heading = wrapAngle(a.heading + clampd(dh, -a.maxTurn * 0.6 * dt, a.maxTurn * 0.6 * dt));
    move(dt, std::nullopt, rearClear ? -a.maxSpeed * 0.3 : 0);
  }

  void beginPlan(int level) {
    planLevel = level;
    const NavGrid& g = level == 0 ? *world.nav : *world.fine;
    if (!g.ready) { pendingGrid = true; return; }
    pendingGrid = false;
    escapePrefix_.clear();
    const auto goals = g.seedCells(world.goal.x, world.goal.y, agent.radius);
    if (level == 2) {
      std::vector<int> ids;
      for (auto& s : goals) ids.push_back(s.i);
      auto esc = escapeRoute(world, g, agent.pos, agent.radius, ids);
      if (!esc) { astar.start(g, {}, goals); return; }
      escapePrefix_ = std::move(esc->path);
      astar.start(g, {Seed{esc->cell, esc->length}}, goals);
      return;
    }
    astar.start(g, g.seedCells(agent.pos.x, agent.pos.y, agent.radius), goals);
  }

  void stepPlan(double budgetMs) {
    if (pendingGrid) {
      if (!world.fine->buildStep(budgetMs)) return;
      beginPlan(planLevel);
      return;
    }
    const auto status = astar.step(budgetMs);
    stats.planNodes = astar.expanded; stats.planMs = astar.msSpent; stats.planSlices = astar.slices;
    if (status == SearchStatus::Found) {
      auto raw = astar.path();
      if (!escapePrefix_.empty()) raw.insert(raw.begin(), escapePrefix_.begin(), escapePrefix_.end());
      else raw.insert(raw.begin(), agent.pos);
      raw.push_back(world.goal);
      rawPath = raw;
      setPath(smoothPath(raw, world, agent.radius + 3));
      constexpr const char* tiers[] = {"coarse", "fine", "fine via pixel escape"};
      go(State::Following, std::format("path found on {} grid: {} nodes in {} slice{}", tiers[planLevel],
                                       astar.expanded, astar.slices, astar.slices > 1 ? "s" : ""));
    } else if (status == SearchStatus::Failed) {
      if (planLevel < 2) {
        const int next = planLevel + 1;
        constexpr const char* why[] = {"", "coarse grid found no route, retrying on fine grid", "no grid route from here, trying a pixel-level escape"};
        pushLog({simTime, state, state, why[next]});
        beginPlan(next);
      } else {
        go(State::Failed, "no route exists at the agent's size");
      }
    }
  }

  void setPath(std::vector<Vec2> pts) {
    path = std::move(pts);
    cum.assign(1, 0.0);
    for (std::size_t i = 1; i < path.size(); i++) cum.push_back(cum[i - 1] + dist(path[i - 1], path[i]));
    segIdx = 0;
    remaining = cum.back();
  }

  Vec2 pointAt(double s, int k0) const {
    const int n = static_cast<int>(path.size()) - 1;
    if (s >= cum[n]) return path[n];
    int k = k0;
    while (k < n - 1 && cum[k + 1] < s) k++;
    const double den = cum[k + 1] - cum[k];
    const double f = clampd((s - cum[k]) / (den != 0 ? den : 1), 0, 1);
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
    const double along = cum[bestK] + best.t * (cum[bestK + 1] - cum[bestK]);
    const double total = cum[n];
    remaining = total - along + best.d;
    onPath = Vec2{best.qx, best.qy};

    const double lookahead = 34 + std::max(0.0, a.speed) * 0.22;
    target.reset();
    for (double sweepR : {a.radius + 2.5, a.radius * 0.95}) {
      for (double s = std::min(along + lookahead, total); s >= along - 0.01; s -= 4) {
        const Vec2 pt = pointAt(s, bestK);
        if (world.segmentFree(a.pos.x, a.pos.y, pt.x, pt.y, sweepR)) { target = pt; break; }
      }
      if (target) break;
    }
    targetVisible = target.has_value();
    if (!target) target = onPath;

    cornerSpeed = a.maxSpeed;
    double d = cum[bestK + 1] - along;
    for (int k = bestK + 1; k < n && d < 160; k++) {
      const Vec2 d1 = norm(p[k] - p[k - 1]), d2 = norm(p[k + 1] - p[k]);
      const double turn = std::acos(clampd(dot(d1, d2), -1, 1));
      const double vc = a.maxSpeed * clampd(1 - turn / 2.0, 0.12, 1);
      cornerSpeed = std::min(cornerSpeed, std::sqrt(vc * vc + 2 * a.maxAccel * 0.7 * std::max(0.0, d - 4)));
      d += cum[k + 1] - cum[k];
    }
  }

  bool pathAheadClear(int count) const {
    const auto& p = path; const double r = agent.radius;
    for (int k = segIdx; k < std::min(segIdx + count, static_cast<int>(p.size()) - 1); k++) {
      const Vec2 a = k == segIdx ? (onPath ? *onPath : agent.pos) : p[k];
      if (!world.segmentFree(a.x, a.y, p[k + 1].x, p[k + 1].y, r * 0.75)) return false;
    }
    return true;
  }

  void computeAvoidance() {
    auto& a = agent;
    const double L = 26 + std::max(0.0, a.speed) * 0.3;
    const Vec2 side = perp(fromAngle(a.heading));
    const double defs[3][2] = {{0, L}, {0.55, L * 0.7}, {-0.55, L * 0.7}};
    double t[3];
    for (int i = 0; i < 3; i++) {
      const Vec2 dv = fromAngle(a.heading + defs[i][0], defs[i][1]);
      t[i] = world.sweep(a.pos.x, a.pos.y, a.pos.x + dv.x, a.pos.y + dv.y, a.radius * 0.9);
    }
    const double c = t[0], l = t[1], r = t[2];
    double lat = 0, br = 0;
    if (c < 1) { lat += (l > r ? 1 : -1) * (1 - c) * 1.6; br += 1 - c; }
    if (l < 1) lat -= (1 - l) * 1.0;
    if (r < 1) lat += (1 - r) * 1.0;
    avoidLateral = side * lat;
    avoidNormal = {};
    if (auto ns = world.nearestSurface(a.pos.x, a.pos.y, a.radius + 8)) {
      const double k = (1 - ns->d / (a.radius + 8)) * 0.8;
      avoidNormal = {ns->nx * k, ns->ny * k};
    }
    lastLateral = lat;
    brake = clampd(br, 0, 0.6);
  }

  void follow(double dt) {
    auto& a = agent;
    frame++;
    trackPath();
    const double dGoal = dist(a.pos, world.goal);
    if (dGoal < a.radius + 3) { go(State::Arrived, std::format("reached B after {:.1f} s", simTime)); return; }
    computeAvoidance();
    const Vec2 toTarget = norm(*target - a.pos);
    Vec2 desired; double br;
    if (targetVisible) { desired = toTarget + avoidNormal * 0.3; br = 0; }
    else { desired = toTarget + avoidLateral + avoidNormal; br = brake; }
    const double arrive = clampd(dGoal / 70, 0.2, 1);
    move(dt, desired, std::min(a.maxSpeed * (1 - br) * arrive, cornerSpeed));

    if (remaining < anchorRemaining - kMinProgress) { anchorRemaining = remaining; lastProgressTime = simTime; }
    if (remaining < remainingAtRecovery - 60) stuckStreak = 0;
    if (simTime - lastProgressTime > kStuckSeconds) { stats.stuckEvents++; go(State::Recovering, "no progress for 1.2 s"); return; }
    if (frame % 10 == 0 && !pathAheadClear(3)) { keepWatchdog = true; go(State::Replanning, "path ahead blocked"); }
  }

  static double approach(double v, double tgt, double maxDelta) { return v + clampd(tgt - v, -maxDelta, maxDelta); }

  void move(double dt, std::optional<Vec2> desired, double speedTarget) {
    auto& a = agent;
    if (desired && (desired->x != 0 || desired->y != 0)) {
      const double dh = wrapAngle(angleOf(*desired) - a.heading);
      a.heading = wrapAngle(a.heading + clampd(dh, -a.maxTurn * dt, a.maxTurn * dt));
      const double align = std::cos(dh);
      speedTarget *= align < 0.5 ? 0 : align * align;
    }
    a.speed = approach(a.speed, speedTarget, a.maxAccel * dt);
    a.pos.x += std::cos(a.heading) * a.speed * dt;
    a.pos.y += std::sin(a.heading) * a.speed * dt;
    const int hits = world.resolveDisc(a.pos, a.radius);
    if (hits) { stats.collisions += hits; a.speed *= 0.6; }
  }
};

}  // namespace sim
